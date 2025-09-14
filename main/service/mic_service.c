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
#define MIC_SEND_QUEUE_LEN 32                      // 增加队列长度
#define MIC_WS_TX_TIMEOUT_MS 5000                  // 增加发送超时时间
#define MIC_TX_TASK_STACK_SIZE (8 * 1024)          // 栈大小
#define MIC_SEND_DELAY_MS 10                       // 发送间隔，避免过快发送
#define MIC_MAX_RECONNECT_ATTEMPTS 3               // 最大重连次数

/* === WebSocket 客户端句柄 === */
static esp_websocket_client_handle_t s_ws = NULL;
static TaskHandle_t s_tx_task = NULL;
static QueueHandle_t s_tx_queue = NULL;
static volatile bool s_connected = false;
static volatile bool s_running = false;
static int s_reconnect_count = 0; // 重连计数器

/* === WebSocket 事件处理回调 === */
static void ws_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_websocket_event_data_t *e = (esp_websocket_event_data_t *)event_data;
    switch (event_id)
    {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "WebSocket Connected (attempt %d)", s_reconnect_count + 1);
        s_connected = true;
        s_reconnect_count = 0; // 重置重连计数器
        break;

    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "WebSocket Disconnected");
        s_connected = false;
        s_reconnect_count++;

        // 如果重连次数过多，停止重连
        if (s_reconnect_count >= MIC_MAX_RECONNECT_ATTEMPTS)
        {
            ESP_LOGE(TAG, "Max reconnection attempts reached, stopping service");
            // 可以选择停止服务或采取其他措施
        }
        break;

    case WEBSOCKET_EVENT_DATA:
        // 打印服务端返回的数据
        if (e->data_len > 0)
        {
            char *response_data = malloc(e->data_len + 1);
            if (response_data)
            {
                memcpy(response_data, e->data_ptr, e->data_len);
                response_data[e->data_len] = '\0';

                if (e->op_code == 0x01)
                { // 文本数据
                    ESP_LOGI(TAG, "🎤 Server Response: %s", response_data);
                }
                else if (e->op_code == 0x02)
                { // 二进制数据
                    ESP_LOGI(TAG, "Server Response (Binary): %d bytes", e->data_len);
                    ESP_LOG_BUFFER_HEX_LEVEL(TAG, e->data_ptr,
                                             e->data_len > 64 ? 64 : e->data_len,
                                             ESP_LOG_INFO);
                }
                else
                {
                    ESP_LOGI(TAG, "Server Response (OpCode: 0x%02x): %s", e->op_code, response_data);
                }

                free(response_data);
            }
            else
            {
                ESP_LOGE(TAG, "Failed to allocate memory for response data");
            }
        }
        break;

    case WEBSOCKET_EVENT_ERROR:
        ESP_LOGE(TAG, "WebSocket Error (reconnect attempt %d/%d)", s_reconnect_count, MIC_MAX_RECONNECT_ATTEMPTS);
        s_connected = false;
        break;

    default:
        ESP_LOGD(TAG, "WebSocket event: %ld", event_id);
        break;
    }
}

/* === 发送任务：从队列取帧 -> 发送二进制 === */
static void tx_task(void *arg)
{
    mic_frame_t frame;
    ESP_LOGI(TAG, "TX task started with stack size: %d bytes", MIC_TX_TASK_STACK_SIZE);

    uint32_t total_frames_sent = 0;
    uint32_t total_bytes_sent = 0;

    while (s_running)
    {
        // 从队列取数据，超时 100ms
        if (xQueueReceive(s_tx_queue, &frame, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            // 检查连接状态
            if (!s_connected || !s_ws)
            {
                ESP_LOGD(TAG, "WebSocket not connected, dropping frame (%zu bytes)", frame.nbytes);
                continue;
            }

            // 发送二进制数据
            int ret = esp_websocket_client_send_bin(s_ws, (const char *)frame.data,
                                                    frame.nbytes, pdMS_TO_TICKS(MIC_WS_TX_TIMEOUT_MS));
            if (ret < 0)
            {
                ESP_LOGW(TAG, "send_bin failed (%d), bytes=%zu, total_sent=%lu", ret, frame.nbytes, total_bytes_sent);
                // 发送失败，可能是连接问题，暂停一下
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            else if (ret != frame.nbytes)
            {
                ESP_LOGW(TAG, "Partial send: expected %zu, sent %d bytes", frame.nbytes, ret);
            }
            else
            {
                total_frames_sent++;
                total_bytes_sent += ret;
                ESP_LOGD(TAG, "✅ Sent frame %lu: %d bytes (total: %lu bytes)",
                         total_frames_sent, ret, total_bytes_sent);
            }

            // 添加小延迟，避免发送过快
            vTaskDelay(pdMS_TO_TICKS(MIC_SEND_DELAY_MS));
        }
    }

    ESP_LOGI(TAG, "TX task exiting. Total sent: %lu frames, %lu bytes", total_frames_sent, total_bytes_sent);
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

    // 初始化并连接 WebSocket - 优化配置
    esp_websocket_client_config_t cfg = {
        .uri = MIC_WS_URI,
        .reconnect_timeout_ms = 5000,
        .network_timeout_ms = 10000,
        .buffer_size = 8192, // 增加缓冲区大小
        .task_stack = MIC_TX_TASK_STACK_SIZE,
        .task_prio = 5,
        .ping_interval_sec = 10,         // 启用心跳
        .disable_auto_reconnect = false, // 启用自动重连
    };

    s_ws = esp_websocket_client_init(&cfg);
    ESP_RETURN_ON_FALSE(s_ws, ESP_FAIL, TAG, "ws init failed");

    // 注册事件处理器
    ESP_ERROR_CHECK(esp_websocket_register_events(s_ws, WEBSOCKET_EVENT_ANY, ws_event_handler, NULL));

    // 启动 WebSocket 客户端
    ESP_ERROR_CHECK(esp_websocket_client_start(s_ws));

    s_running = true;
    s_reconnect_count = 0;

    // 创建发送任务
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

    // 如果连接不稳定，暂时不发送
    if (!s_connected)
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
        return ESP_ERR_TIMEOUT;
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
    s_running = false;

    if (s_ws)
    {
        esp_websocket_client_stop(s_ws);
        esp_websocket_client_destroy(s_ws);
        s_ws = NULL;
    }

    if (s_tx_task)
    {
        vTaskDelay(pdMS_TO_TICKS(200));
        vTaskDelete(s_tx_task);
        s_tx_task = NULL;
    }

    if (s_tx_queue)
    {
        vQueueDelete(s_tx_queue);
        s_tx_queue = NULL;
    }

    s_connected = false;
    s_reconnect_count = 0;
    ESP_LOGI(TAG, "mic_service stopped");
}