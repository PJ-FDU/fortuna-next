#include <errno.h>
#include <string.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_partition.h" // 添加分区API
#include "esp_heap_caps.h" // 添加内存查看
#include "esp_psram.h"
#include "esp_spiffs.h"
#include "dirent.h"
#include "driver/gpio.h"

#include "i2c_driver.h"
#include "io_exp_driver.h"
#include "lcd_disp_driver.h"
#include "lcd_touch_driver.h"
#include "lvgl_driver.h"
#include "sd_card_driver.h"

#include "lvgl.h"
#include "src/misc/cache/instance/lv_image_cache.h"
#include "esp_lvgl_port.h"


static const char *TAG = "main_fortuna";
static lv_obj_t *s_status_label = NULL;

static esp_err_t ensure_spiffs_mounted(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "spiffs",
        .max_files = 5,
        .format_if_mount_failed = false,
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret == ESP_ERR_INVALID_STATE)
    {
        return ESP_OK;
    }
    return ret;
}

static void sd_card_log_directory(const char *dir_path, int depth)
{
    DIR *dir = opendir(dir_path);
    if (dir == NULL)
    {
        ESP_LOGW(TAG, "Failed to open directory %s: %s", dir_path, strerror(errno));
        return;
    }

    struct dirent *entry = NULL;
    while ((entry = readdir(dir)) != NULL)
    {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
        {
            continue;
        }

        char full_path[512];
        int written = snprintf(full_path, sizeof(full_path), "%s/%s", dir_path, entry->d_name);
        if (written <= 0 || written >= (int)sizeof(full_path))
        {
            ESP_LOGW(TAG, "Path too long, skip %s/%s", dir_path, entry->d_name);
            continue;
        }

        struct stat st;
        if (stat(full_path, &st) != 0)
        {
            ESP_LOGW(TAG, "stat failed for %s: %s", full_path, strerror(errno));
            continue;
        }

        if (S_ISDIR(st.st_mode))
        {
            ESP_LOGI(TAG, "%*sDir : %s", depth * 2, "", full_path);
            sd_card_log_directory(full_path, depth + 1);
        }
        else if (S_ISREG(st.st_mode))
        {
            ESP_LOGI(TAG, "%*sFile: %s (%lld bytes)", depth * 2, "", full_path, (long long)st.st_size);
        }
        else
        {
            ESP_LOGI(TAG, "%*sOther: %s (mode: 0%o)", depth * 2, "", full_path, st.st_mode & S_IFMT);
        }
    }

    closedir(dir);
}

static void sd_card_log_contents(void)
{
    if (!sd_card_driver_is_mounted())
    {
        ESP_LOGW(TAG, "SD card not mounted, skip file listing");
        return;
    }

    const char *root_path = sd_card_driver_mount_point();
    if (root_path == NULL)
    {
        ESP_LOGW(TAG, "SD card mount point is NULL");
        return;
    }

    struct stat st;
    if (stat(root_path, &st) != 0)
    {
        ESP_LOGW(TAG, "stat failed for mount point %s: %s", root_path, strerror(errno));
        return;
    }

    if (!S_ISDIR(st.st_mode))
    {
        ESP_LOGI(TAG, "SD card mount point %s is not a directory", root_path);
        return;
    }

    ESP_LOGI(TAG, "Listing SD card files under %s", root_path);
    sd_card_log_directory(root_path, 0);
}

static void create_img_bg(void)
{
    esp_err_t err = ensure_spiffs_mounted();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(err));
        return;
    }

    lv_image_cache_resize(0, true);
    lv_obj_t *scr = lv_scr_act();

    lv_obj_t *img = lv_image_create(scr);
    lv_image_set_src(img, "S:/bg.png");
    const int32_t w = lv_obj_get_self_width(img);
    const int32_t h = lv_obj_get_self_height(img);
    ESP_LOGI(TAG, "Background image size: %" LV_PRId32 " x %" LV_PRId32, w, h);
    lv_obj_set_size(img, LV_PCT(100), LV_PCT(100));
    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
    lv_obj_move_background(img);
}

static const char *gesture_dir_to_text(lv_dir_t dir)
{
    switch (dir)
    {
    case LV_DIR_LEFT:
        return "Gesture: Left";
    case LV_DIR_RIGHT:
        return "Gesture: Right";
    case LV_DIR_TOP:
        return "Gesture: Up";
    case LV_DIR_BOTTOM:
        return "Gesture: Down";
    default:
        return "Gesture: Unknown";
    }
}

static void screen_event_cb(lv_event_t *e)
{
    if (!e)
    {
        return;
    }

    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_SHORT_CLICKED)
    {
        lv_indev_t *indev = lv_event_get_indev(e);
        if (!indev)
        {
            indev = lv_indev_get_act();
        }
        if (!indev || s_status_label == NULL)
        {
            return;
        }

        lv_point_t p;
        lv_indev_get_point(indev, &p);
        lv_label_set_text_fmt(s_status_label, "x:%" LV_PRId32 " y:%" LV_PRId32, p.x, p.y);
    }
    else if (code == LV_EVENT_GESTURE)
    {
        if (s_status_label == NULL)
        {
            return;
        }
        lv_indev_t *indev = lv_event_get_indev(e);
        if (!indev)
        {
            indev = lv_indev_get_act();
        }
        lv_dir_t dir = indev ? lv_indev_get_gesture_dir(indev) : LV_DIR_NONE;
        lv_label_set_text(s_status_label, gesture_dir_to_text(dir));
    }
}

static lv_obj_t *home_create(void)
{
    LV_TRACE_OBJ_CREATE("begin");

    static lv_style_t root;
    static lv_style_t bg_panel;
    static lv_style_t bg_line;

    static bool style_inited = false;

    if (!style_inited)
    {
        lv_style_init(&root);
        lv_style_set_bg_color(&root, lv_color_hex(0x101d23));
        lv_style_set_pad_all(&root, 0);

        lv_style_init(&bg_panel);
        lv_style_set_width(&bg_panel, lv_pct(100));
        lv_style_set_height(&bg_panel, lv_pct(100));
        lv_style_set_bg_color(&bg_panel, lv_color_hex(0xf2e6c4));
        lv_style_set_bg_opa(&bg_panel, 15);
        lv_style_set_radius(&bg_panel, lv_pct(50));
        lv_style_set_border_width(&bg_panel, 0);

        lv_style_init(&bg_line);
        lv_style_set_width(&bg_line, 380);
        lv_style_set_height(&bg_line, 1);
        lv_style_set_bg_color(&bg_line, lv_color_hex(0xF2E6C4));
        lv_style_set_bg_opa(&bg_line, 35);
        lv_style_set_radius(&bg_line, lv_pct(50));
        lv_style_set_border_width(&bg_line, 0);

        style_inited = true;
    }

    lv_obj_t *lv_obj_0 = lv_obj_create(NULL);
    lv_obj_set_flag(lv_obj_0, LV_OBJ_FLAG_SCROLLABLE, false);

    lv_obj_add_style(lv_obj_0, &root, 0);
    lv_obj_t *lv_obj_1 = lv_obj_create(lv_obj_0);
    lv_obj_add_style(lv_obj_1, &bg_panel, 0);

    lv_obj_t *lv_obj_2 = lv_obj_create(lv_obj_0);
    lv_obj_set_align(lv_obj_2, LV_ALIGN_CENTER);
    lv_obj_set_style_transform_rotation(lv_obj_2, 0, 0);
    lv_obj_set_style_transform_pivot_x(lv_obj_2, 190, 0);
    lv_obj_set_style_transform_pivot_y(lv_obj_2, 0, 0);
    lv_obj_add_style(lv_obj_2, &bg_line, 0);

    lv_obj_t *lv_obj_3 = lv_obj_create(lv_obj_0);
    lv_obj_set_align(lv_obj_3, LV_ALIGN_CENTER);
    lv_obj_set_style_transform_rotation(lv_obj_3, 300, 0);
    lv_obj_set_style_transform_pivot_x(lv_obj_3, 190, 0);
    lv_obj_set_style_transform_pivot_y(lv_obj_3, 0, 0);
    lv_obj_add_style(lv_obj_3, &bg_line, 0);

    lv_obj_t *lv_obj_4 = lv_obj_create(lv_obj_0);
    lv_obj_set_align(lv_obj_4, LV_ALIGN_CENTER);
    lv_obj_set_style_transform_rotation(lv_obj_4, 600, 0);
    lv_obj_set_style_transform_pivot_x(lv_obj_4, 190, 0);
    lv_obj_set_style_transform_pivot_y(lv_obj_4, 0, 0);
    lv_obj_add_style(lv_obj_4, &bg_line, 0);

    lv_obj_t *lv_obj_5 = lv_obj_create(lv_obj_0);
    lv_obj_set_align(lv_obj_5, LV_ALIGN_CENTER);
    lv_obj_set_style_transform_rotation(lv_obj_5, 900, 0);
    lv_obj_set_style_transform_pivot_x(lv_obj_5, 190, 0);
    lv_obj_set_style_transform_pivot_y(lv_obj_5, 0, 0);
    lv_obj_add_style(lv_obj_5, &bg_line, 0);

    lv_obj_t *lv_obj_6 = lv_obj_create(lv_obj_0);
    lv_obj_set_align(lv_obj_6, LV_ALIGN_CENTER);
    lv_obj_set_style_transform_rotation(lv_obj_6, 1200, 0);
    lv_obj_set_style_transform_pivot_x(lv_obj_6, 190, 0);
    lv_obj_set_style_transform_pivot_y(lv_obj_6, 0, 0);
    lv_obj_add_style(lv_obj_6, &bg_line, 0);

    lv_obj_t *lv_obj_7 = lv_obj_create(lv_obj_0);
    lv_obj_set_align(lv_obj_7, LV_ALIGN_CENTER);
    lv_obj_set_style_transform_rotation(lv_obj_7, 1500, 0);
    lv_obj_set_style_transform_pivot_x(lv_obj_7, 190, 0);
    lv_obj_set_style_transform_pivot_y(lv_obj_7, 0, 0);
    lv_obj_add_style(lv_obj_7, &bg_line, 0);

    lv_obj_t *lv_obj_8 = lv_obj_create(lv_obj_0);
    lv_obj_set_align(lv_obj_8, LV_ALIGN_CENTER);
    lv_obj_set_style_border_width(lv_obj_8, 2, 0);
    lv_obj_set_style_radius(lv_obj_8, lv_pct(50), 0);
    lv_obj_set_width(lv_obj_8, 332);
    lv_obj_set_height(lv_obj_8, 332);
    lv_obj_set_style_bg_color(lv_obj_8, lv_color_hex(0x191F20), 0);
    lv_obj_set_style_bg_opa(lv_obj_8, 128, 0);
    lv_obj_set_style_border_color(lv_obj_8, lv_color_hex(0xF2E6C4), 0);
    lv_obj_set_style_border_opa(lv_obj_8, 128, 0);

    lv_obj_t *lv_obj_9 = lv_obj_create(lv_obj_0);
    lv_obj_set_align(lv_obj_9, LV_ALIGN_CENTER);
    lv_obj_set_style_border_width(lv_obj_9, 2, 0);
    lv_obj_set_style_radius(lv_obj_9, lv_pct(50), 0);
    lv_obj_set_width(lv_obj_9, 248);
    lv_obj_set_height(lv_obj_9, 248);
    lv_obj_set_style_pad_all(lv_obj_9, 0, 0);
    lv_obj_set_style_clip_corner(lv_obj_9, true, 0);
    lv_obj_set_style_bg_color(lv_obj_9, lv_color_hex(0x191F20), 0);
    lv_obj_set_style_border_color(lv_obj_9, lv_color_hex(0xF2E6C4), 0);
    lv_obj_set_style_border_opa(lv_obj_9, 128, 0);
    lv_obj_set_flex_flow(lv_obj_9, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_flex_main_place(lv_obj_9, LV_FLEX_ALIGN_CENTER, 0);
    lv_obj_set_style_flex_cross_place(lv_obj_9, LV_FLEX_ALIGN_CENTER, 0);
    lv_obj_t *lv_image_0 = lv_image_create(lv_obj_9);
    lv_image_set_src(lv_image_0, "S:/magic_ball.png");
    lv_obj_set_width(lv_image_0, lv_pct(100));
    lv_obj_set_height(lv_image_0, lv_pct(100));

    LV_TRACE_OBJ_CREATE("finished");

    // lv_obj_set_name(lv_obj_0, "home");

    return lv_obj_0;
}

static lv_obj_t * watchface_create(void)
{
    LV_TRACE_OBJ_CREATE("begin");

    static lv_style_t root;
    static lv_style_t bg_img;
    static lv_style_t outer_box;
    static lv_style_t outer_box_spoke;
    static lv_style_t outer_box_text;
    static lv_style_t outer_box_degree_container;
    static lv_style_t outer_box_degree;
    static lv_style_t mid_box;
    static lv_style_t mid_box_spoke;
    static lv_style_t mid_box_icon;
    static lv_style_t mid_box_degree_container;
    static lv_style_t mid_box_degree;
    static lv_style_t inner_box;
    static lv_style_t hour_hand;
    static lv_style_t hour_hand_bar;
    static lv_style_t hour_hand_tail;
    static lv_style_t min_hand;
    static lv_style_t min_hand_bar;
    static lv_style_t min_hand_tail;
    static lv_style_t sec_hand;
    static lv_style_t sec_hand_bar;

    static bool style_inited = false;

    if (!style_inited) {
        lv_style_init(&root);
        lv_style_set_width(&root, 412);
        lv_style_set_height(&root, 412);

        lv_style_init(&bg_img);
        lv_style_set_width(&bg_img, 412);
        lv_style_set_height(&bg_img, 412);

        lv_style_init(&outer_box);
        lv_style_set_width(&outer_box, 412);
        lv_style_set_height(&outer_box, 412);
        lv_style_set_radius(&outer_box, 206);
        lv_style_set_bg_opa(&outer_box, 0);
        lv_style_set_pad_all(&outer_box, 0);
        lv_style_set_align(&outer_box, LV_ALIGN_CENTER);
        lv_style_set_border_width(&outer_box, 0);

        lv_style_init(&outer_box_spoke);
        lv_style_set_width(&outer_box_spoke, 44);
        lv_style_set_height(&outer_box_spoke, 206);
        lv_style_set_align(&outer_box_spoke, LV_ALIGN_TOP_MID);
        lv_style_set_transform_pivot_x(&outer_box_spoke, 22);
        lv_style_set_transform_pivot_y(&outer_box_spoke, 206);
        lv_style_set_pad_all(&outer_box_spoke, 0);
        lv_style_set_bg_opa(&outer_box_spoke, 0);
        lv_style_set_border_width(&outer_box_spoke, 0);

        lv_style_init(&outer_box_text);
        lv_style_set_text_font(&outer_box_text, &lv_font_montserrat_40);
        lv_style_set_align(&outer_box_text, LV_ALIGN_TOP_MID);
        lv_style_set_text_color(&outer_box_text, lv_color_hex(0xF2E6C4));

        lv_style_init(&outer_box_degree_container);
        lv_style_set_width(&outer_box_degree_container, 6);
        lv_style_set_height(&outer_box_degree_container, 324);
        lv_style_set_pad_all(&outer_box_degree_container, 0);
        lv_style_set_border_width(&outer_box_degree_container, 0);
        lv_style_set_bg_opa(&outer_box_degree_container, 0);
        lv_style_set_align(&outer_box_degree_container, LV_ALIGN_CENTER);
        lv_style_set_transform_pivot_x(&outer_box_degree_container, 3);
        lv_style_set_transform_pivot_y(&outer_box_degree_container, 162);

        lv_style_init(&outer_box_degree);
        lv_style_set_width(&outer_box_degree, 6);
        lv_style_set_height(&outer_box_degree, 6);
        lv_style_set_radius(&outer_box_degree, lv_pct(50));
        lv_style_set_bg_color(&outer_box_degree, lv_color_hex(0xF2E6C4));

        lv_style_init(&mid_box);
        lv_style_set_width(&mid_box, 300);
        lv_style_set_height(&mid_box, 300);
        lv_style_set_radius(&mid_box, lv_pct(50));
        lv_style_set_bg_opa(&mid_box, 0);
        lv_style_set_pad_all(&mid_box, 0);
        lv_style_set_align(&mid_box, LV_ALIGN_CENTER);
        lv_style_set_border_width(&mid_box, 2);
        lv_style_set_border_color(&mid_box, lv_color_hex(0xF2E6C4));

        lv_style_init(&mid_box_spoke);
        lv_style_set_width(&mid_box_spoke, 64);
        lv_style_set_height(&mid_box_spoke, 150);
        lv_style_set_transform_pivot_x(&mid_box_spoke, 32);
        lv_style_set_transform_pivot_y(&mid_box_spoke, 150);
        lv_style_set_bg_opa(&mid_box_spoke, 0);
        lv_style_set_pad_all(&mid_box_spoke, 0);
        lv_style_set_align(&mid_box_spoke, LV_ALIGN_TOP_MID);
        lv_style_set_border_width(&mid_box_spoke, 0);

        lv_style_init(&mid_box_icon);
        lv_style_set_width(&mid_box_icon, 48);
        lv_style_set_height(&mid_box_icon, 48);
        lv_style_set_align(&mid_box_icon, LV_ALIGN_TOP_MID);

        lv_style_init(&mid_box_degree_container);
        lv_style_set_width(&mid_box_degree_container, 2);
        lv_style_set_height(&mid_box_degree_container, 296);
        lv_style_set_pad_all(&mid_box_degree_container, 0);
        lv_style_set_border_width(&mid_box_degree_container, 0);
        lv_style_set_bg_opa(&mid_box_degree_container, 0);
        lv_style_set_align(&mid_box_degree_container, LV_ALIGN_CENTER);
        lv_style_set_transform_pivot_x(&mid_box_degree_container, 1);
        lv_style_set_transform_pivot_y(&mid_box_degree_container, 148);

        lv_style_init(&mid_box_degree);
        lv_style_set_width(&mid_box_degree, 2);
        lv_style_set_height(&mid_box_degree, 48);
        lv_style_set_radius(&mid_box_degree, lv_pct(50));
        lv_style_set_bg_color(&mid_box_degree, lv_color_hex(0xF2E6C4));

        lv_style_init(&inner_box);
        lv_style_set_width(&inner_box, 204);
        lv_style_set_height(&inner_box, 204);
        lv_style_set_radius(&inner_box, lv_pct(50));
        lv_style_set_bg_opa(&inner_box, 0);
        lv_style_set_pad_all(&inner_box, 0);
        lv_style_set_align(&inner_box, LV_ALIGN_CENTER);
        lv_style_set_border_width(&inner_box, 2);
        lv_style_set_border_color(&inner_box, lv_color_hex(0xF2E6C4));

        lv_style_init(&hour_hand);
        lv_style_set_border_width(&hour_hand, 0);
        lv_style_set_width(&hour_hand, 12);
        lv_style_set_height(&hour_hand, 80);
        lv_style_set_transform_pivot_x(&hour_hand, 6);
        lv_style_set_transform_pivot_y(&hour_hand, 6);
        lv_style_set_bg_opa(&hour_hand, 0);
        lv_style_set_pad_all(&hour_hand, 0);

        lv_style_init(&hour_hand_bar);
        lv_style_set_border_width(&hour_hand_bar, 0);
        lv_style_set_width(&hour_hand_bar, 4);
        lv_style_set_height(&hour_hand_bar, lv_pct(100));
        lv_style_set_radius(&hour_hand_bar, lv_pct(50));
        lv_style_set_align(&hour_hand_bar, LV_ALIGN_TOP_MID);

        lv_style_init(&hour_hand_tail);
        lv_style_set_border_width(&hour_hand_tail, 0);
        lv_style_set_width(&hour_hand_tail, 8);
        lv_style_set_height(&hour_hand_tail, 48);
        lv_style_set_radius(&hour_hand_tail, lv_pct(50));
        lv_style_set_align(&hour_hand_tail, LV_ALIGN_BOTTOM_MID);

        lv_style_init(&min_hand);
        lv_style_set_border_width(&min_hand, 0);
        lv_style_set_width(&min_hand, 12);
        lv_style_set_height(&min_hand, 96);
        lv_style_set_transform_pivot_x(&min_hand, 6);
        lv_style_set_transform_pivot_y(&min_hand, 6);
        lv_style_set_bg_opa(&min_hand, 0);
        lv_style_set_pad_all(&min_hand, 0);

        lv_style_init(&min_hand_bar);
        lv_style_set_border_width(&min_hand_bar, 0);
        lv_style_set_width(&min_hand_bar, 4);
        lv_style_set_height(&min_hand_bar, lv_pct(100));
        lv_style_set_radius(&min_hand_bar, lv_pct(50));
        lv_style_set_align(&min_hand_bar, LV_ALIGN_TOP_MID);

        lv_style_init(&min_hand_tail);
        lv_style_set_border_width(&min_hand_tail, 0);
        lv_style_set_width(&min_hand_tail, 8);
        lv_style_set_height(&min_hand_tail, 64);
        lv_style_set_radius(&min_hand_tail, lv_pct(50));
        lv_style_set_align(&min_hand_tail, LV_ALIGN_BOTTOM_MID);

        lv_style_init(&sec_hand);
        lv_style_set_border_width(&sec_hand, 0);
        lv_style_set_width(&sec_hand, 4);
        lv_style_set_height(&sec_hand, 144);
        lv_style_set_transform_pivot_x(&sec_hand, 2);
        lv_style_set_transform_pivot_y(&sec_hand, 6);
        lv_style_set_bg_opa(&sec_hand, 0);
        lv_style_set_pad_all(&sec_hand, 0);

        lv_style_init(&sec_hand_bar);
        lv_style_set_border_width(&sec_hand_bar, 0);
        lv_style_set_width(&sec_hand_bar, 4);
        lv_style_set_height(&sec_hand_bar, lv_pct(100));
        lv_style_set_radius(&sec_hand_bar, lv_pct(50));
        lv_style_set_align(&sec_hand_bar, LV_ALIGN_TOP_MID);
        lv_style_set_bg_color(&sec_hand_bar, lv_color_hex(0x8d2d2d));

        style_inited = true;
    }

    lv_obj_t * lv_obj_0 = lv_obj_create(NULL);

    lv_obj_add_style(lv_obj_0, &root, 0);
    lv_obj_t * lv_image_0 = lv_image_create(lv_obj_0);
    lv_image_set_src(lv_image_0, "S:/bg.png");
    lv_obj_add_style(lv_image_0, &bg_img, 0);
    
    lv_obj_t * lv_obj_1 = lv_obj_create(lv_obj_0);
    lv_obj_add_style(lv_obj_1, &outer_box, 0);
    lv_obj_t * lv_obj_2 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_2, 0, 0);
    lv_obj_add_style(lv_obj_2, &outer_box_spoke, 0);
    lv_obj_t * lv_label_0 = lv_label_create(lv_obj_2);
    lv_label_set_text(lv_label_0, "12");
    lv_obj_add_style(lv_label_0, &outer_box_text, 0);
    
    lv_obj_t * lv_obj_3 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_3, 300, 0);
    lv_obj_add_style(lv_obj_3, &outer_box_spoke, 0);
    lv_obj_t * lv_label_1 = lv_label_create(lv_obj_3);
    lv_label_set_text(lv_label_1, "1");
    lv_obj_add_style(lv_label_1, &outer_box_text, 0);
    
    lv_obj_t * lv_obj_4 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_4, 600, 0);
    lv_obj_add_style(lv_obj_4, &outer_box_spoke, 0);
    lv_obj_t * lv_label_2 = lv_label_create(lv_obj_4);
    lv_label_set_text(lv_label_2, "2");
    lv_obj_add_style(lv_label_2, &outer_box_text, 0);
    
    lv_obj_t * lv_obj_5 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_5, 900, 0);
    lv_obj_add_style(lv_obj_5, &outer_box_spoke, 0);
    lv_obj_t * lv_label_3 = lv_label_create(lv_obj_5);
    lv_label_set_text(lv_label_3, "3");
    lv_obj_add_style(lv_label_3, &outer_box_text, 0);
    
    lv_obj_t * lv_obj_6 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_6, 1200, 0);
    lv_obj_add_style(lv_obj_6, &outer_box_spoke, 0);
    lv_obj_t * lv_label_4 = lv_label_create(lv_obj_6);
    lv_label_set_text(lv_label_4, "4");
    lv_obj_add_style(lv_label_4, &outer_box_text, 0);
    
    lv_obj_t * lv_obj_7 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_7, 1500, 0);
    lv_obj_add_style(lv_obj_7, &outer_box_spoke, 0);
    lv_obj_t * lv_label_5 = lv_label_create(lv_obj_7);
    lv_label_set_text(lv_label_5, "5");
    lv_obj_add_style(lv_label_5, &outer_box_text, 0);
    
    lv_obj_t * lv_obj_8 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_8, 1800, 0);
    lv_obj_add_style(lv_obj_8, &outer_box_spoke, 0);
    lv_obj_t * lv_label_6 = lv_label_create(lv_obj_8);
    lv_label_set_text(lv_label_6, "6");
    lv_obj_add_style(lv_label_6, &outer_box_text, 0);
    
    lv_obj_t * lv_obj_9 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_9, 2100, 0);
    lv_obj_add_style(lv_obj_9, &outer_box_spoke, 0);
    lv_obj_t * lv_label_7 = lv_label_create(lv_obj_9);
    lv_label_set_text(lv_label_7, "7");
    lv_obj_add_style(lv_label_7, &outer_box_text, 0);
    
    lv_obj_t * lv_obj_10 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_10, 2400, 0);
    lv_obj_add_style(lv_obj_10, &outer_box_spoke, 0);
    lv_obj_t * lv_label_8 = lv_label_create(lv_obj_10);
    lv_label_set_text(lv_label_8, "8");
    lv_obj_add_style(lv_label_8, &outer_box_text, 0);
    
    lv_obj_t * lv_obj_11 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_11, 2700, 0);
    lv_obj_add_style(lv_obj_11, &outer_box_spoke, 0);
    lv_obj_t * lv_label_9 = lv_label_create(lv_obj_11);
    lv_label_set_text(lv_label_9, "9");
    lv_obj_add_style(lv_label_9, &outer_box_text, 0);
    
    lv_obj_t * lv_obj_12 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_12, 3000, 0);
    lv_obj_add_style(lv_obj_12, &outer_box_spoke, 0);
    lv_obj_t * lv_label_10 = lv_label_create(lv_obj_12);
    lv_label_set_text(lv_label_10, "10");
    lv_obj_add_style(lv_label_10, &outer_box_text, 0);
    
    lv_obj_t * lv_obj_13 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_13, 3300, 0);
    lv_obj_add_style(lv_obj_13, &outer_box_spoke, 0);
    lv_obj_t * lv_label_11 = lv_label_create(lv_obj_13);
    lv_label_set_text(lv_label_11, "11");
    lv_obj_add_style(lv_label_11, &outer_box_text, 0);
    
    lv_obj_t * lv_obj_14 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_14, 0, 0);
    lv_obj_add_style(lv_obj_14, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_15 = lv_obj_create(lv_obj_14);
    lv_obj_set_style_align(lv_obj_15, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_15, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_16 = lv_obj_create(lv_obj_14);
    lv_obj_set_style_align(lv_obj_16, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_16, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_17 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_17, 60, 0);
    lv_obj_add_style(lv_obj_17, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_18 = lv_obj_create(lv_obj_17);
    lv_obj_set_style_align(lv_obj_18, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_18, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_19 = lv_obj_create(lv_obj_17);
    lv_obj_set_style_align(lv_obj_19, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_19, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_20 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_20, 120, 0);
    lv_obj_add_style(lv_obj_20, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_21 = lv_obj_create(lv_obj_20);
    lv_obj_set_style_align(lv_obj_21, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_21, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_22 = lv_obj_create(lv_obj_20);
    lv_obj_set_style_align(lv_obj_22, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_22, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_23 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_23, 180, 0);
    lv_obj_add_style(lv_obj_23, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_24 = lv_obj_create(lv_obj_23);
    lv_obj_set_style_align(lv_obj_24, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_24, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_25 = lv_obj_create(lv_obj_23);
    lv_obj_set_style_align(lv_obj_25, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_25, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_26 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_26, 240, 0);
    lv_obj_add_style(lv_obj_26, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_27 = lv_obj_create(lv_obj_26);
    lv_obj_set_style_align(lv_obj_27, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_27, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_28 = lv_obj_create(lv_obj_26);
    lv_obj_set_style_align(lv_obj_28, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_28, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_29 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_29, 300, 0);
    lv_obj_add_style(lv_obj_29, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_30 = lv_obj_create(lv_obj_29);
    lv_obj_set_style_align(lv_obj_30, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_30, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_31 = lv_obj_create(lv_obj_29);
    lv_obj_set_style_align(lv_obj_31, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_31, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_32 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_32, 360, 0);
    lv_obj_add_style(lv_obj_32, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_33 = lv_obj_create(lv_obj_32);
    lv_obj_set_style_align(lv_obj_33, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_33, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_34 = lv_obj_create(lv_obj_32);
    lv_obj_set_style_align(lv_obj_34, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_34, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_35 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_35, 420, 0);
    lv_obj_add_style(lv_obj_35, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_36 = lv_obj_create(lv_obj_35);
    lv_obj_set_style_align(lv_obj_36, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_36, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_37 = lv_obj_create(lv_obj_35);
    lv_obj_set_style_align(lv_obj_37, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_37, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_38 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_38, 480, 0);
    lv_obj_add_style(lv_obj_38, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_39 = lv_obj_create(lv_obj_38);
    lv_obj_set_style_align(lv_obj_39, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_39, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_40 = lv_obj_create(lv_obj_38);
    lv_obj_set_style_align(lv_obj_40, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_40, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_41 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_41, 540, 0);
    lv_obj_add_style(lv_obj_41, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_42 = lv_obj_create(lv_obj_41);
    lv_obj_set_style_align(lv_obj_42, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_42, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_43 = lv_obj_create(lv_obj_41);
    lv_obj_set_style_align(lv_obj_43, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_43, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_44 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_44, 600, 0);
    lv_obj_add_style(lv_obj_44, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_45 = lv_obj_create(lv_obj_44);
    lv_obj_set_style_align(lv_obj_45, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_45, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_46 = lv_obj_create(lv_obj_44);
    lv_obj_set_style_align(lv_obj_46, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_46, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_47 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_47, 660, 0);
    lv_obj_add_style(lv_obj_47, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_48 = lv_obj_create(lv_obj_47);
    lv_obj_set_style_align(lv_obj_48, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_48, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_49 = lv_obj_create(lv_obj_47);
    lv_obj_set_style_align(lv_obj_49, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_49, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_50 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_50, 720, 0);
    lv_obj_add_style(lv_obj_50, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_51 = lv_obj_create(lv_obj_50);
    lv_obj_set_style_align(lv_obj_51, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_51, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_52 = lv_obj_create(lv_obj_50);
    lv_obj_set_style_align(lv_obj_52, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_52, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_53 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_53, 780, 0);
    lv_obj_add_style(lv_obj_53, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_54 = lv_obj_create(lv_obj_53);
    lv_obj_set_style_align(lv_obj_54, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_54, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_55 = lv_obj_create(lv_obj_53);
    lv_obj_set_style_align(lv_obj_55, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_55, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_56 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_56, 840, 0);
    lv_obj_add_style(lv_obj_56, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_57 = lv_obj_create(lv_obj_56);
    lv_obj_set_style_align(lv_obj_57, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_57, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_58 = lv_obj_create(lv_obj_56);
    lv_obj_set_style_align(lv_obj_58, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_58, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_59 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_59, 900, 0);
    lv_obj_add_style(lv_obj_59, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_60 = lv_obj_create(lv_obj_59);
    lv_obj_set_style_align(lv_obj_60, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_60, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_61 = lv_obj_create(lv_obj_59);
    lv_obj_set_style_align(lv_obj_61, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_61, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_62 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_62, 960, 0);
    lv_obj_add_style(lv_obj_62, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_63 = lv_obj_create(lv_obj_62);
    lv_obj_set_style_align(lv_obj_63, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_63, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_64 = lv_obj_create(lv_obj_62);
    lv_obj_set_style_align(lv_obj_64, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_64, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_65 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_65, 1020, 0);
    lv_obj_add_style(lv_obj_65, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_66 = lv_obj_create(lv_obj_65);
    lv_obj_set_style_align(lv_obj_66, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_66, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_67 = lv_obj_create(lv_obj_65);
    lv_obj_set_style_align(lv_obj_67, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_67, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_68 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_68, 1080, 0);
    lv_obj_add_style(lv_obj_68, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_69 = lv_obj_create(lv_obj_68);
    lv_obj_set_style_align(lv_obj_69, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_69, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_70 = lv_obj_create(lv_obj_68);
    lv_obj_set_style_align(lv_obj_70, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_70, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_71 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_71, 1140, 0);
    lv_obj_add_style(lv_obj_71, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_72 = lv_obj_create(lv_obj_71);
    lv_obj_set_style_align(lv_obj_72, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_72, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_73 = lv_obj_create(lv_obj_71);
    lv_obj_set_style_align(lv_obj_73, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_73, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_74 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_74, 1200, 0);
    lv_obj_add_style(lv_obj_74, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_75 = lv_obj_create(lv_obj_74);
    lv_obj_set_style_align(lv_obj_75, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_75, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_76 = lv_obj_create(lv_obj_74);
    lv_obj_set_style_align(lv_obj_76, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_76, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_77 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_77, 1260, 0);
    lv_obj_add_style(lv_obj_77, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_78 = lv_obj_create(lv_obj_77);
    lv_obj_set_style_align(lv_obj_78, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_78, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_79 = lv_obj_create(lv_obj_77);
    lv_obj_set_style_align(lv_obj_79, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_79, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_80 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_80, 1320, 0);
    lv_obj_add_style(lv_obj_80, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_81 = lv_obj_create(lv_obj_80);
    lv_obj_set_style_align(lv_obj_81, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_81, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_82 = lv_obj_create(lv_obj_80);
    lv_obj_set_style_align(lv_obj_82, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_82, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_83 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_83, 1380, 0);
    lv_obj_add_style(lv_obj_83, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_84 = lv_obj_create(lv_obj_83);
    lv_obj_set_style_align(lv_obj_84, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_84, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_85 = lv_obj_create(lv_obj_83);
    lv_obj_set_style_align(lv_obj_85, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_85, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_86 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_86, 1440, 0);
    lv_obj_add_style(lv_obj_86, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_87 = lv_obj_create(lv_obj_86);
    lv_obj_set_style_align(lv_obj_87, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_87, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_88 = lv_obj_create(lv_obj_86);
    lv_obj_set_style_align(lv_obj_88, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_88, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_89 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_89, 1500, 0);
    lv_obj_add_style(lv_obj_89, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_90 = lv_obj_create(lv_obj_89);
    lv_obj_set_style_align(lv_obj_90, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_90, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_91 = lv_obj_create(lv_obj_89);
    lv_obj_set_style_align(lv_obj_91, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_91, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_92 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_92, 1560, 0);
    lv_obj_add_style(lv_obj_92, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_93 = lv_obj_create(lv_obj_92);
    lv_obj_set_style_align(lv_obj_93, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_93, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_94 = lv_obj_create(lv_obj_92);
    lv_obj_set_style_align(lv_obj_94, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_94, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_95 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_95, 1620, 0);
    lv_obj_add_style(lv_obj_95, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_96 = lv_obj_create(lv_obj_95);
    lv_obj_set_style_align(lv_obj_96, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_96, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_97 = lv_obj_create(lv_obj_95);
    lv_obj_set_style_align(lv_obj_97, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_97, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_98 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_98, 1680, 0);
    lv_obj_add_style(lv_obj_98, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_99 = lv_obj_create(lv_obj_98);
    lv_obj_set_style_align(lv_obj_99, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_99, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_100 = lv_obj_create(lv_obj_98);
    lv_obj_set_style_align(lv_obj_100, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_100, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_101 = lv_obj_create(lv_obj_1);
    lv_obj_set_style_transform_rotation(lv_obj_101, 1740, 0);
    lv_obj_add_style(lv_obj_101, &outer_box_degree_container, 0);
    lv_obj_t * lv_obj_102 = lv_obj_create(lv_obj_101);
    lv_obj_set_style_align(lv_obj_102, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_102, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_103 = lv_obj_create(lv_obj_101);
    lv_obj_set_style_align(lv_obj_103, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_103, &outer_box_degree, 0);
    
    lv_obj_t * lv_obj_104 = lv_obj_create(lv_obj_0);
    lv_obj_add_style(lv_obj_104, &mid_box, 0);
    lv_obj_t * lv_obj_105 = lv_obj_create(lv_obj_104);
    lv_obj_set_style_transform_rotation(lv_obj_105, 0, 0);
    lv_obj_add_style(lv_obj_105, &mid_box_spoke, 0);
    lv_obj_t * lv_image_1 = lv_image_create(lv_obj_105);
    lv_image_set_src(lv_image_1, "S:/icon_aries.png");
    lv_image_set_inner_align(lv_image_1, LV_IMAGE_ALIGN_STRETCH);
    lv_obj_add_style(lv_image_1, &mid_box_icon, 0);
    
    lv_obj_t * lv_obj_106 = lv_obj_create(lv_obj_104);
    lv_obj_set_style_transform_rotation(lv_obj_106, 300, 0);
    lv_obj_add_style(lv_obj_106, &mid_box_spoke, 0);
    lv_obj_t * lv_image_2 = lv_image_create(lv_obj_106);
    lv_image_set_src(lv_image_2, "S:/icon_aquarius.png");
    lv_image_set_inner_align(lv_image_2, LV_IMAGE_ALIGN_STRETCH);
    lv_obj_add_style(lv_image_2, &mid_box_icon, 0);
    
    lv_obj_t * lv_obj_107 = lv_obj_create(lv_obj_104);
    lv_obj_set_style_transform_rotation(lv_obj_107, 600, 0);
    lv_obj_add_style(lv_obj_107, &mid_box_spoke, 0);
    lv_obj_t * lv_image_3 = lv_image_create(lv_obj_107);
    lv_image_set_src(lv_image_3, "S:/icon_cancer.png");
    lv_image_set_inner_align(lv_image_3, LV_IMAGE_ALIGN_STRETCH);
    lv_obj_add_style(lv_image_3, &mid_box_icon, 0);
    
    lv_obj_t * lv_obj_108 = lv_obj_create(lv_obj_104);
    lv_obj_set_style_transform_rotation(lv_obj_108, 900, 0);
    lv_obj_add_style(lv_obj_108, &mid_box_spoke, 0);
    lv_obj_t * lv_image_4 = lv_image_create(lv_obj_108);
    lv_image_set_src(lv_image_4, "S:/icon_capricorn.png");
    lv_image_set_inner_align(lv_image_4, LV_IMAGE_ALIGN_STRETCH);
    lv_obj_add_style(lv_image_4, &mid_box_icon, 0);
    
    lv_obj_t * lv_obj_109 = lv_obj_create(lv_obj_104);
    lv_obj_set_style_transform_rotation(lv_obj_109, 1200, 0);
    lv_obj_add_style(lv_obj_109, &mid_box_spoke, 0);
    lv_obj_t * lv_image_5 = lv_image_create(lv_obj_109);
    lv_image_set_src(lv_image_5, "S:/icon_gemini.png");
    lv_image_set_inner_align(lv_image_5, LV_IMAGE_ALIGN_STRETCH);
    lv_obj_add_style(lv_image_5, &mid_box_icon, 0);
    
    lv_obj_t * lv_obj_110 = lv_obj_create(lv_obj_104);
    lv_obj_set_style_transform_rotation(lv_obj_110, 1500, 0);
    lv_obj_add_style(lv_obj_110, &mid_box_spoke, 0);
    lv_obj_t * lv_image_6 = lv_image_create(lv_obj_110);
    lv_image_set_src(lv_image_6, "S:/icon_leo.png");
    lv_image_set_inner_align(lv_image_6, LV_IMAGE_ALIGN_STRETCH);
    lv_obj_add_style(lv_image_6, &mid_box_icon, 0);
    
    lv_obj_t * lv_obj_111 = lv_obj_create(lv_obj_104);
    lv_obj_set_style_transform_rotation(lv_obj_111, 1800, 0);
    lv_obj_add_style(lv_obj_111, &mid_box_spoke, 0);
    lv_obj_t * lv_image_7 = lv_image_create(lv_obj_111);
    lv_image_set_src(lv_image_7, "S:/icon_libra.png");
    lv_image_set_inner_align(lv_image_7, LV_IMAGE_ALIGN_STRETCH);
    lv_obj_add_style(lv_image_7, &mid_box_icon, 0);
    
    lv_obj_t * lv_obj_112 = lv_obj_create(lv_obj_104);
    lv_obj_set_style_transform_rotation(lv_obj_112, 2100, 0);
    lv_obj_add_style(lv_obj_112, &mid_box_spoke, 0);
    lv_obj_t * lv_image_8 = lv_image_create(lv_obj_112);
    lv_image_set_src(lv_image_8, "S:/icon_pisces.png");
    lv_image_set_inner_align(lv_image_8, LV_IMAGE_ALIGN_STRETCH);
    lv_obj_add_style(lv_image_8, &mid_box_icon, 0);
    
    lv_obj_t * lv_obj_113 = lv_obj_create(lv_obj_104);
    lv_obj_set_style_transform_rotation(lv_obj_113, 2400, 0);
    lv_obj_add_style(lv_obj_113, &mid_box_spoke, 0);
    lv_obj_t * lv_image_9 = lv_image_create(lv_obj_113);
    lv_image_set_src(lv_image_9, "S:/icon_sagittarius.png");
    lv_image_set_inner_align(lv_image_9, LV_IMAGE_ALIGN_STRETCH);
    lv_obj_add_style(lv_image_9, &mid_box_icon, 0);
    
    lv_obj_t * lv_obj_114 = lv_obj_create(lv_obj_104);
    lv_obj_set_style_transform_rotation(lv_obj_114, 2700, 0);
    lv_obj_add_style(lv_obj_114, &mid_box_spoke, 0);
    lv_obj_t * lv_image_10 = lv_image_create(lv_obj_114);
    lv_image_set_src(lv_image_10, "S:/icon_taurus.png");
    lv_image_set_inner_align(lv_image_10, LV_IMAGE_ALIGN_STRETCH);
    lv_obj_add_style(lv_image_10, &mid_box_icon, 0);
    
    lv_obj_t * lv_obj_115 = lv_obj_create(lv_obj_104);
    lv_obj_set_style_transform_rotation(lv_obj_115, 3000, 0);
    lv_obj_add_style(lv_obj_115, &mid_box_spoke, 0);
    lv_obj_t * lv_image_11 = lv_image_create(lv_obj_115);
    lv_image_set_src(lv_image_11, "S:/icon_scorpio.png");
    lv_image_set_inner_align(lv_image_11, LV_IMAGE_ALIGN_STRETCH);
    lv_obj_add_style(lv_image_11, &mid_box_icon, 0);
    
    lv_obj_t * lv_obj_116 = lv_obj_create(lv_obj_104);
    lv_obj_set_style_transform_rotation(lv_obj_116, 3300, 0);
    lv_obj_add_style(lv_obj_116, &mid_box_spoke, 0);
    lv_obj_t * lv_image_12 = lv_image_create(lv_obj_116);
    lv_image_set_src(lv_image_12, "S:/icon_virgo.png");
    lv_image_set_inner_align(lv_image_12, LV_IMAGE_ALIGN_STRETCH);
    lv_obj_add_style(lv_image_12, &mid_box_icon, 0);
    
    lv_obj_t * lv_obj_117 = lv_obj_create(lv_obj_104);
    lv_obj_set_style_transform_rotation(lv_obj_117, 150, 0);
    lv_obj_add_style(lv_obj_117, &mid_box_degree_container, 0);
    lv_obj_t * lv_obj_118 = lv_obj_create(lv_obj_117);
    lv_obj_set_style_align(lv_obj_118, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_118, &mid_box_degree, 0);
    
    lv_obj_t * lv_obj_119 = lv_obj_create(lv_obj_117);
    lv_obj_set_style_align(lv_obj_119, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_119, &mid_box_degree, 0);
    
    lv_obj_t * lv_obj_120 = lv_obj_create(lv_obj_104);
    lv_obj_set_style_transform_rotation(lv_obj_120, 450, 0);
    lv_obj_add_style(lv_obj_120, &mid_box_degree_container, 0);
    lv_obj_t * lv_obj_121 = lv_obj_create(lv_obj_120);
    lv_obj_set_style_align(lv_obj_121, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_121, &mid_box_degree, 0);
    
    lv_obj_t * lv_obj_122 = lv_obj_create(lv_obj_120);
    lv_obj_set_style_align(lv_obj_122, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_122, &mid_box_degree, 0);
    
    lv_obj_t * lv_obj_123 = lv_obj_create(lv_obj_104);
    lv_obj_set_style_transform_rotation(lv_obj_123, 750, 0);
    lv_obj_add_style(lv_obj_123, &mid_box_degree_container, 0);
    lv_obj_t * lv_obj_124 = lv_obj_create(lv_obj_123);
    lv_obj_set_style_align(lv_obj_124, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_124, &mid_box_degree, 0);
    
    lv_obj_t * lv_obj_125 = lv_obj_create(lv_obj_123);
    lv_obj_set_style_align(lv_obj_125, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_125, &mid_box_degree, 0);
    
    lv_obj_t * lv_obj_126 = lv_obj_create(lv_obj_104);
    lv_obj_set_style_transform_rotation(lv_obj_126, 1050, 0);
    lv_obj_add_style(lv_obj_126, &mid_box_degree_container, 0);
    lv_obj_t * lv_obj_127 = lv_obj_create(lv_obj_126);
    lv_obj_set_style_align(lv_obj_127, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_127, &mid_box_degree, 0);
    
    lv_obj_t * lv_obj_128 = lv_obj_create(lv_obj_126);
    lv_obj_set_style_align(lv_obj_128, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_128, &mid_box_degree, 0);
    
    lv_obj_t * lv_obj_129 = lv_obj_create(lv_obj_104);
    lv_obj_set_style_transform_rotation(lv_obj_129, 1350, 0);
    lv_obj_add_style(lv_obj_129, &mid_box_degree_container, 0);
    lv_obj_t * lv_obj_130 = lv_obj_create(lv_obj_129);
    lv_obj_set_style_align(lv_obj_130, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_130, &mid_box_degree, 0);
    
    lv_obj_t * lv_obj_131 = lv_obj_create(lv_obj_129);
    lv_obj_set_style_align(lv_obj_131, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_131, &mid_box_degree, 0);
    
    lv_obj_t * lv_obj_132 = lv_obj_create(lv_obj_104);
    lv_obj_set_style_transform_rotation(lv_obj_132, 1650, 0);
    lv_obj_add_style(lv_obj_132, &mid_box_degree_container, 0);
    lv_obj_t * lv_obj_133 = lv_obj_create(lv_obj_132);
    lv_obj_set_style_align(lv_obj_133, LV_ALIGN_TOP_MID, 0);
    lv_obj_add_style(lv_obj_133, &mid_box_degree, 0);
    
    lv_obj_t * lv_obj_134 = lv_obj_create(lv_obj_132);
    lv_obj_set_style_align(lv_obj_134, LV_ALIGN_BOTTOM_MID, 0);
    lv_obj_add_style(lv_obj_134, &mid_box_degree, 0);
    
    lv_obj_t * lv_obj_135 = lv_obj_create(lv_obj_0);
    lv_obj_add_style(lv_obj_135, &inner_box, 0);
    
    lv_obj_t * lv_obj_136 = lv_obj_create(lv_obj_0);
    lv_obj_set_x(lv_obj_136, 200);
    lv_obj_set_y(lv_obj_136, 200);
    lv_obj_add_style(lv_obj_136, &hour_hand, 0);
    lv_obj_t * lv_obj_137 = lv_obj_create(lv_obj_136);
    lv_obj_add_style(lv_obj_137, &hour_hand_bar, 0);
    
    lv_obj_t * lv_obj_138 = lv_obj_create(lv_obj_136);
    lv_obj_add_style(lv_obj_138, &hour_hand_tail, 0);
    
    lv_obj_t * lv_obj_139 = lv_obj_create(lv_obj_0);
    lv_obj_set_x(lv_obj_139, 200);
    lv_obj_set_y(lv_obj_139, 200);
    lv_obj_set_style_transform_rotation(lv_obj_139, 450, 0);
    lv_obj_add_style(lv_obj_139, &min_hand, 0);
    lv_obj_t * lv_obj_140 = lv_obj_create(lv_obj_139);
    lv_obj_add_style(lv_obj_140, &min_hand_bar, 0);
    
    lv_obj_t * lv_obj_141 = lv_obj_create(lv_obj_139);
    lv_obj_add_style(lv_obj_141, &min_hand_tail, 0);
    
    lv_obj_t * lv_obj_142 = lv_obj_create(lv_obj_0);
    lv_obj_set_x(lv_obj_142, 204);
    lv_obj_set_y(lv_obj_142, 200);
    lv_obj_set_style_transform_rotation(lv_obj_142, 1350, 0);
    lv_obj_add_style(lv_obj_142, &sec_hand, 0);
    lv_obj_t * lv_obj_143 = lv_obj_create(lv_obj_142);
    lv_obj_add_style(lv_obj_143, &sec_hand_bar, 0);

    LV_TRACE_OBJ_CREATE("finished");

    // lv_obj_set_name(lv_obj_0, "watchface");

    return lv_obj_0;
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== Fortuna System Starting ===");

    ESP_ERROR_CHECK(i2c_driver_init());
    ESP_ERROR_CHECK(io_exp_driver_init());
    ESP_ERROR_CHECK(lcd_disp_driver_init());
    ESP_ERROR_CHECK(lcd_touch_driver_init());
    ESP_ERROR_CHECK(lvgl_driver_init());

    esp_err_t sd_err = sd_card_driver_init();
    if (sd_err != ESP_OK)
    {
        ESP_LOGW(TAG, "SD card driver init failed: %s", esp_err_to_name(sd_err));
    }
    else
    {
        sd_card_log_contents();
    }

    // 写个最简单的示例
    if (lvgl_port_lock(0))
    {
        create_img_bg();

        s_status_label = lv_label_create(lv_scr_act());
        lv_label_set_text(s_status_label, "Hello, Fortuna!");
        lv_obj_align(s_status_label, LV_ALIGN_CENTER, 0, 12);
        lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xffffff), LV_PART_MAIN);
        lv_obj_set_style_text_font(s_status_label, lv_theme_get_font_large(lv_scr_act()), LV_PART_MAIN);
        lv_obj_set_style_text_align(s_status_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);

        lv_obj_add_flag(lv_scr_act(), LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(lv_scr_act(), screen_event_cb, LV_EVENT_ALL, NULL);

        // esp_err_t err = ensure_spiffs_mounted();
        // if (err != ESP_OK)
        // {
        //     ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(err));
        //     return;
        // }
        // 在屏幕上绘制home_create
        // lv_obj_t *home = home_create();
        // lv_scr_load(home);

        // lv_obj_t *watchface = watchface_create();
        // lv_scr_load(watchface);

        lvgl_port_unlock();
    }
    else
    {
        ESP_LOGE(TAG, "Failed to lock LVGL port for UI init");
    }

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
