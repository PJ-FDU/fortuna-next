#include "mic_service.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_websocket_client.h"

static const char *TAG = "mic_service";

/* === 配置区 === */
#define MIC_WS_URI "ws://192.168.1.103:8080/audio" // WebSocket 服务端地址
#define MIC_SEND_QUEUE_LEN 16                      // 发送队列最大待发帧
#define MIC_WS_TX_TIMEOUT_MS 2000                  // 单帧发送超时
#define MIC_TX_TASK_STACK_SIZE (8 * 1024)          // 增加栈大小到 8KB
#define SAMPLE_RATE 16000                          // 采样率：16 kHz

/* === WebSocket 客户端句柄 === */
static esp_websocket_client_handle_t s_ws = NULL;
static TaskHandle_t s_tx_task = NULL;
static QueueHandle_t s_tx_queue = NULL;
static volatile bool s_connected = false;
static volatile bool s_running = false;

/* === WebSocket 事件处理回调 === */
static void ws_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *e = (esp_websocket_event_data_t *)event_data;
    switch (event_id)
    {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "WebSocket Connected");
        s_connected = true;
        break;
    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "WebSocket Disconnected");
        s_connected = false;
        break;
    case WEBSOCKET_EVENT_DATA:
        ESP_LOGD(TAG, "Received data: %.*s", e->data_len, (char *)e->data_ptr);
        break;
    case WEBSOCKET_EVENT_ERROR:
        ESP_LOGE(TAG, "WebSocket Error");
        s_connected = false;
        break;
    default:
        break;
    }
}

/* === 发送任务：从队列取帧 -> 发送二进制 === */
static void tx_task(void *arg)
{
    mic_frame_t frame;
    ESP_LOGI(TAG, "TX task started with stack size: %d bytes", MIC_TX_TASK_STACK_SIZE);

    while (s_running)
    {
        // 从队列取数据，超时 100ms
        if (xQueueReceive(s_tx_queue, &frame, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            // 检查连接状态
            if (!s_connected || !s_ws)
            {
                ESP_LOGW(TAG, "WebSocket not connected, dropping frame (%zu bytes)", frame.nbytes);
                continue;
            }

            // 发送二进制数据
            ESP_LOGD(TAG, "Sending frame: %zu bytes", frame.nbytes);
            int ret = esp_websocket_client_send_bin(s_ws, (const char *)frame.data,
                                                    frame.nbytes, pdMS_TO_TICKS(MIC_WS_TX_TIMEOUT_MS));
            if (ret < 0)
            {
                ESP_LOGW(TAG, "send_bin failed (%d), bytes=%zu", ret, frame.nbytes);
            }
            else
            {
                ESP_LOGD(TAG, "Successfully sent %d bytes", ret);
            }
        }
    }

    ESP_LOGI(TAG, "TX task exiting");
    vTaskDelete(NULL);
}

/* === 对外：启动 WebSocket 服务 === */
esp_err_t mic_service_start(void)
{
    if (s_running)
    {
        ESP_LOGW(TAG, "mic_service already running");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Starting mic_service...");

    // 创建发送队列
    s_tx_queue = xQueueCreate(MIC_SEND_QUEUE_LEN, sizeof(mic_frame_t));
    ESP_RETURN_ON_FALSE(s_tx_queue, ESP_ERR_NO_MEM, TAG, "queue create failed");

    // 初始化并连接 WebSocket
    esp_websocket_client_config_t cfg = {
        .uri = MIC_WS_URI,
        .reconnect_timeout_ms = 5000,
        .network_timeout_ms = 10000,
        .buffer_size = 4096,                  // 增加缓冲区大小
        .task_stack = MIC_TX_TASK_STACK_SIZE, // 设置 WebSocket 内部任务栈大小
        .task_prio = 5,
    };

    s_ws = esp_websocket_client_init(&cfg);
    ESP_RETURN_ON_FALSE(s_ws, ESP_FAIL, TAG, "ws init failed");

    // 注册事件处理器 - 修正函数名
    ESP_ERROR_CHECK(esp_websocket_register_events(s_ws, WEBSOCKET_EVENT_ANY, ws_event_handler, NULL));

    // 启动 WebSocket 客户端
    ESP_ERROR_CHECK(esp_websocket_client_start(s_ws));

    s_running = true;

    // 创建发送任务，使用更大的栈
    xTaskCreatePinnedToCore(tx_task, "mic_tx", MIC_TX_TASK_STACK_SIZE, NULL, 5, &s_tx_task, 1);

    ESP_LOGI(TAG, "mic_service started successfully");
    return ESP_OK;
}
/* === 对外：发送一帧 PCM16 数据 === */
esp_err_t mic_service_send_pcm16(const int16_t *pcm, size_t nsamples)
{
    if (!s_running)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_tx_queue)
    {
        return ESP_ERR_INVALID_STATE;
    }

    const size_t need_bytes = nsamples * sizeof(int16_t);
    if (need_bytes > MIC_FRAME_BYTES_MAX)
    {
        ESP_LOGW(TAG, "Frame too large: %zu > %d", need_bytes, MIC_FRAME_BYTES_MAX);
        return ESP_ERR_INVALID_SIZE;
    }

    mic_frame_t frame = {.nbytes = need_bytes};
    memcpy(frame.data, (const uint8_t *)pcm, need_bytes);

    // 非阻塞发送到队列
    if (xQueueSend(s_tx_queue, &frame, 0) != pdTRUE)
    {
        ESP_LOGD(TAG, "Queue full, dropping frame (%zu bytes)", need_bytes);
        return ESP_ERR_TIMEOUT; // 队列满，丢弃数据
    }

    return ESP_OK;
}

/* === 对外：停止服务并释放资源 === */
void mic_service_stop(void)
{
    if (!s_running)
    {
        ESP_LOGI(TAG, "mic_service already stopped");
        return;
    }

    ESP_LOGI(TAG, "Stopping mic_service...");
    s_running = false; // 1. 设置标志，让任务循环退出

    // 2. 停止并销毁 WebSocket 客户端（按官方推荐顺序）
    if (s_ws)
    {
        // a. 停止客户端，这会断开连接并停止内部任务（如重连）
        esp_websocket_client_stop(s_ws);
        // b. 销毁客户端，释放所有内存资源
        esp_websocket_client_destroy(s_ws);
        s_ws = NULL;
    }

    // 3. 等待并删除你自己的发送任务
    if (s_tx_task)
    {
        // 等待一小段时间确保任务已收到 s_running = false 信号并退出循环
        vTaskDelay(pdMS_TO_TICKS(200));
        vTaskDelete(s_tx_task);
        s_tx_task = NULL;
    }

    // 4. 删除队列
    if (s_tx_queue)
    {
        vQueueDelete(s_tx_queue);
        s_tx_queue = NULL;
    }

    s_connected = false;
    ESP_LOGI(TAG, "mic_service stopped");
}