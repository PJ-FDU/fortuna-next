#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_spiffs.h"
#include <dirent.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>

#include "i2c_service.h"
#include "io_expander_service.h"
#include "lcd_service.h"
#include "lcd_touch_service.h"
#include "lvgl_service.h"

#include "lvgl.h"
#include "esp_lvgl_port.h"
#include <math.h>
// #include "astro_chart.h"   // 不再需要

static const char *TAG = "FORTUNA";

/*===========================*
 *      星盘：内部状态
 *===========================*/
typedef struct
{
    lv_obj_t *obj;
    float angle_deg;
    uint32_t last_ms; // 新增：上次时间戳（ms）
} astro_state_t;

static astro_state_t g_ast = {0};
static lv_timer_t *g_ast_timer = NULL;

/*===========================*
 *      背景样式（屏幕渐变）
 *===========================*/
static lv_style_t style_scr_bg;

static void style_scr_bg_init(void)
{
    lv_style_init(&style_scr_bg);
    lv_style_set_bg_opa(&style_scr_bg, LV_OPA_COVER);
    /* 竖向线性渐变（上浅下深）；可按需改色 */
    lv_style_set_bg_color(&style_scr_bg, lv_color_hex(0x5A6FFF));
    lv_style_set_bg_grad_color(&style_scr_bg, lv_color_hex(0x000000));
    lv_style_set_bg_grad_dir(&style_scr_bg, LV_GRAD_DIR_VER);
}

/*===========================*
 *   工具：极坐标 → 直角坐标
 *===========================*/
static inline void polar_to_xy(float ang_deg, float r, lv_coord_t cx, lv_coord_t cy,
                               lv_point_t *out)
{
    /* 我们定义 0°=12点方向，因此要 -90° 才能对应 cos/sin 的 0° 在 3点钟 */
    float rad = (ang_deg - 90.0f) * (float)M_PI / 180.0f;
    out->x = cx + (lv_coord_t)lrintf(r * cosf(rad));
    out->y = cy + (lv_coord_t)lrintf(r * sinf(rad));
}

/*===========================*
 *   工具：画圆环（整圆）
 *===========================*/
static void draw_circle_ring(lv_layer_t *layer, lv_coord_t cx, lv_coord_t cy,
                             uint16_t radius, uint16_t width, lv_color_t color, lv_opa_t opa, bool rounded)
{
    lv_draw_arc_dsc_t dsc;
    lv_draw_arc_dsc_init(&dsc);
    dsc.color = color;
    dsc.opa = opa;
    dsc.width = width;
    dsc.rounded = rounded ? 1 : 0;
    dsc.center.x = cx;
    dsc.center.y = cy;
    dsc.radius = radius;
    dsc.start_angle = 0; /* lv_draw_arc 的角度：0°在3点钟方向，顺时针递增 */
    dsc.end_angle = 360;
    lv_draw_arc(layer, &dsc); /* v9.3：通过描述子提交绘制任务 */
}

/*===========================*
 *   工具：画一条极坐标射线
 *===========================*/
static void draw_radial_line(lv_layer_t *layer, lv_coord_t cx, lv_coord_t cy,
                             float ang_deg, float r0, float r1,
                             lv_color_t color, uint16_t width, lv_opa_t opa)
{
    float rad = (ang_deg - 90.0f) * (float)M_PI / 180.0f;
    float x0 = cx + r0 * cosf(rad);
    float y0 = cy + r0 * sinf(rad);
    float x1 = cx + r1 * cosf(rad);
    float y1 = cy + r1 * sinf(rad);

    lv_draw_line_dsc_t ld;
    lv_draw_line_dsc_init(&ld);
    ld.color = color;
    ld.width = width;
    ld.opa = opa;

    /* v9: 端点写入 dsc，再调用 2 参版本 */
    ld.p1.x = x0;
    ld.p1.y = y0;
    ld.p2.x = x1;
    ld.p2.y = y1;
    lv_draw_line(layer, &ld);
}

/* ================= 角度工具 / 行星数据（NEW） ================= */

static inline float norm360f(float a){ 
    float r = fmodf(a, 360.0f); 
    return (r < 0) ? r + 360.0f : r;
}
static inline float ang_diff_min(float a, float b){
    float d = fabsf(norm360f(a) - norm360f(b));
    return d > 180.0f ? 360.0f - d : d;
}

/* 行星：用缩写（不画“R”标记） */
typedef struct { const char *abbr; float Ldeg; float speed; } body_t;
static const body_t BODIES[] = {
    { "Su",170.0f, +1.00f },
    { "Mo",285.0f, -0.20f },
    { "Me",162.0f, +1.10f },
    { "Ve",205.0f, +1.20f },
    { "Ma",312.0f, +0.50f },
    { "Ju", 78.0f, +0.08f },
    { "Sa",221.0f, +0.03f },
    { "Ur", 32.0f, +0.02f },
    { "Ne",355.0f, +0.02f },
    { "Pl",255.0f, -0.01f },
};
#define N_BODIES  (int)(sizeof(BODIES)/sizeof(BODIES[0]))

/* 小圆点（实心） */
static void draw_dot(lv_layer_t *layer, lv_coord_t x, lv_coord_t y, lv_coord_t r, lv_color_t col)
{
    lv_draw_rect_dsc_t rd;
    lv_draw_rect_dsc_init(&rd);
    rd.bg_color = col;
    rd.bg_opa   = LV_OPA_COVER;
    rd.border_width = 0;
    rd.radius = LV_RADIUS_CIRCLE;

    lv_area_t a = { x - r, y - r, x + r, y + r };
    lv_draw_rect(layer, &rd, &a);
}

/* 任意两点连线（支持虚线） */
static void draw_line_pts(lv_layer_t *layer, lv_coord_t x1, lv_coord_t y1,
                          lv_coord_t x2, lv_coord_t y2,
                          lv_color_t color, uint16_t width, lv_opa_t opa,
                          uint16_t dash_w, uint16_t dash_gap)
{
    lv_draw_line_dsc_t ld;
    lv_draw_line_dsc_init(&ld);
    ld.color = color;
    ld.width = width;
    ld.opa   = opa;
    ld.p1.x = x1; ld.p1.y = y1;
    ld.p2.x = x2; ld.p2.y = y2;
    if(dash_w && dash_gap){ ld.dash_width = dash_w; ld.dash_gap = dash_gap; }
    lv_draw_line(layer, &ld);
}


/*===========================*
 *   工具：极坐标放置文本（居中）
 *===========================*/
static void draw_text_polar(lv_layer_t *layer, lv_coord_t cx, lv_coord_t cy,
                            float ang_deg, float r, const char *txt,
                            lv_color_t color, const lv_font_t *font)
{
    float rad = (ang_deg - 90.0f) * (float)M_PI / 180.0f;
    lv_coord_t x = cx + (lv_coord_t)lrintf(r * cosf(rad));
    lv_coord_t y = cy + (lv_coord_t)lrintf(r * sinf(rad));

    lv_draw_label_dsc_t td;
    lv_draw_label_dsc_init(&td);
    td.color = color;
    td.opa = LV_OPA_COVER;
    td.font = font;
    td.align = LV_TEXT_ALIGN_CENTER;
    td.text = txt; /* v9: 文本写在 dsc.text */

    lv_area_t a = {x - 14, y - 9, x + 14, y + 9};
    lv_draw_label(layer, &td, &a); /* v9: 3 参版本 */
}

/*===========================*
 *   自绘对象：DRAW 回调
 *===========================*/
static void astro_draw_cb(lv_event_t *e)
{
    if(lv_event_get_code(e) != LV_EVENT_DRAW_MAIN) return;

    lv_layer_t *layer = lv_event_get_layer(e);
    lv_obj_t   *obj   = lv_event_get_target(e);

    /* 内容区域与圆心 */
    lv_area_t cont; 
    lv_obj_get_content_coords(obj, &cont);
    lv_coord_t w = lv_area_get_width(&cont);
    lv_coord_t h = lv_area_get_height(&cont);
    lv_coord_t cx = cont.x1 + w/2;
    lv_coord_t cy = cont.y1 + h/2;

    /* 几何半径 */
    const lv_coord_t R_outer        = LV_MIN(w, h)/2 - 4;     /* 外圈（~204） */
    const lv_coord_t R_inner        = (lv_coord_t)(R_outer * 0.72f); /* 内环 */
    const lv_coord_t R_tick_short_i = R_outer - 10;           /* 5°刻度内端 */
    const lv_coord_t R_tick_long_i  = R_outer - 20;           /* 30°刻度内端 */
    const lv_coord_t R_text_sign    = R_outer - 10;           /* 星座缩写半径 */
    const lv_coord_t R_text_house   = R_inner - 12;           /* 数字(1..12) */
    const lv_coord_t R_planet       = R_inner - 20;           /* 行星点半径 */
    const lv_coord_t R_aspect       = R_inner - 30;           /* 相位连线半径 */

    /* 颜色 */
    const lv_color_t COL_RING   = lv_color_hex(0xE0E0E0);
    const lv_color_t COL_MAJOR  = lv_color_hex(0xD8D8D8);
    const lv_color_t COL_MINOR  = lv_color_hex(0x666A73);
    const lv_color_t COL_TEXT   = lv_color_hex(0xF0F0F0);
    const lv_color_t COL_PNTS[] = { /* 行星点配色（循环使用） */
        lv_color_hex(0xFFD166), lv_color_hex(0xEF476F), lv_color_hex(0x06D6A0),
        lv_color_hex(0x118AB2), lv_color_hex(0x83C5BE), lv_color_hex(0xE5989B),
        lv_color_hex(0x8ECAE6), lv_color_hex(0xA3B18A), lv_color_hex(0xCDB4DB),
        lv_color_hex(0xF2CC8F)
    };

    /* 旋转基准角（0°=12点方向） */
    astro_state_t *st = (astro_state_t *)lv_event_get_user_data(e);
    const float base = st ? st->angle_deg : 0.0f;

    /* 1) 外圈双环 */
    draw_circle_ring(layer, cx, cy, R_outer, 2, COL_RING, LV_OPA_90, true);
    draw_circle_ring(layer, cx, cy, R_inner, 2, COL_RING, LV_OPA_60, true);

    /* 2) 刻度：30°主刻度 + 5°细分刻度 */
    for(int i = 0; i < 72; i++){
        float ang = base + i * 5.0f;
        if(i % 6 == 0){
            /* 主刻度（每30°） */
            draw_radial_line(layer, cx, cy, ang, R_tick_long_i, R_outer, COL_MAJOR, 2, LV_OPA_80);
        }else{
            /* 细刻度（每5°） */
            draw_radial_line(layer, cx, cy, ang, R_tick_short_i, R_outer, COL_MINOR, 1, LV_OPA_80);
        }
    }

    /* 3) 星座缩写（AR/TA/...）摆在每个30°区间的中点 */
    static const char *Z_ABBR[12] = { "AR","TA","GE","CN","LE","VI","LI","SC","SG","CP","AQ","PI" };
    const lv_font_t *font = lv_font_get_default();
    for(int i = 0; i < 12; i++){
        float mid = base + i * 30.0f + 15.0f;  /* 区间中点 */
        draw_text_polar(layer, cx, cy, mid, (float)R_text_sign, Z_ABBR[i], COL_TEXT, font);
    }

    /* 4) 宫位数字（1..12），同样放中点，但靠内圈 */
    char num[3];
    for(int i = 0; i < 12; i++){
        float mid = base + i * 30.0f + 15.0f;
        lv_snprintf(num, sizeof num, "%d", i+1);
        draw_text_polar(layer, cx, cy, mid, (float)R_text_house, num, COL_TEXT, font);
    }

    /* 5) 行星点（内圈），并记录“相位连线”投影点 */
    lv_point_t p_aspect[N_BODIES];
    for(int i = 0; i < N_BODIES; i++){
        /* 简易避让：与前一颗小于 6° 时上下错位 */
        static float lastL = -1000.0f; static bool flip = false;
        float L = norm360f(BODIES[i].Ldeg);
        float r = (fabsf(L - lastL) < 6.0f) ? (R_planet + (flip ? -7 : +7)) : R_planet;
        flip = !flip; lastL = L;

        float ang = base + L;                 /* 绝对黄经 + 盘面旋转 */
        lv_point_t p;
        polar_to_xy(ang, r, cx, cy, &p);
        draw_dot(layer, p.x, p.y, 4, COL_PNTS[i % (int)(sizeof(COL_PNTS)/sizeof(COL_PNTS[0]))]);

        /* 相位连线使用固定半径 R_aspect 的投影点 */
        polar_to_xy(ang, (float)R_aspect, cx, cy, &p_aspect[i]);
    }

    /* 6) 相位连线（合0/冲180/刑90/拱120/六合60） */
    const struct { float deg; float orb; lv_color_t col; uint16_t w; uint16_t dash_w; uint16_t dash_g; }
    ASPECTS[] = {
        {   0.0f,  8.0f, lv_color_hex(0xA0A7B4), 2, 0, 0 },   /* 合相：实线 */
        { 180.0f,  8.0f, lv_color_hex(0xFF6B6B), 2, 0, 0 },   /* 对冲：实线红 */
        {  90.0f,  7.0f, lv_color_hex(0xFF7B7B), 2, 5, 4 },   /* 刑：虚线 */
        { 120.0f,  7.0f, lv_color_hex(0x36C691), 2, 2, 3 },   /* 拱：短虚线 */
        {  60.0f,  6.0f, lv_color_hex(0x4AA3FF), 2, 1, 4 },   /* 六合：点划 */
    };

    for(int i = 0; i < N_BODIES; i++){
        for(int j = i+1; j < N_BODIES; j++){
            float L1 = norm360f(BODIES[i].Ldeg);
            float L2 = norm360f(BODIES[j].Ldeg);
            float d  = ang_diff_min(L1, L2);

            int hit = -1;
            for(int k=0; k< (int)(sizeof(ASPECTS)/sizeof(ASPECTS[0])); k++){
                if(fabsf(d - ASPECTS[k].deg) <= ASPECTS[k].orb){ hit = k; break; }
            }
            if(hit < 0) continue;

            draw_line_pts(layer,
                          p_aspect[i].x, p_aspect[i].y,
                          p_aspect[j].x, p_aspect[j].y,
                          ASPECTS[hit].col, ASPECTS[hit].w, LV_OPA_90,
                          ASPECTS[hit].dash_w, ASPECTS[hit].dash_g);
        }
    }
}


/*===========================*
 *   定时器：以 1°/秒 旋转
 *===========================*/
static void astro_timer_cb(lv_timer_t *t)
{
    astro_state_t *st = (astro_state_t *)lv_timer_get_user_data(t);
    if (!st || !st->obj)
        return;

    uint32_t now = lv_tick_get();
    uint32_t dt = lv_tick_elaps(st->last_ms); // ms
    st->last_ms = now;

    st->angle_deg += (dt * 1.0f) / 1000.0f; // 1°/秒
    if (st->angle_deg >= 360.0f)
        st->angle_deg -= 360.0f;

    lv_obj_invalidate(st->obj);
}

/*===========================*
 *   创建星盘对象（412x412）
 *===========================*/
static lv_obj_t *astro_create(lv_obj_t *parent)
{
    lv_obj_t *obj = lv_obj_create(parent);
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(obj, 412, 412);
    lv_obj_center(obj);

    /* 对象自身透明，靠屏幕样式给背景 */
    lv_obj_set_style_bg_opa(obj, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(obj, 0, 0);
    lv_obj_set_style_clip_corner(obj, true, 0);
    lv_obj_set_style_radius(obj, LV_RADIUS_CIRCLE, 0);

    g_ast.obj = obj;
    g_ast.angle_deg = 0.0f;
    g_ast.last_ms = lv_tick_get(); // 新增

    /* 注册绘制事件（主阶段） */
    lv_obj_add_event_cb(obj, astro_draw_cb, LV_EVENT_DRAW_MAIN, &g_ast);

    /* 定时器：16ms 周期 ≈ 60FPS，1°/秒 */
    g_ast_timer = lv_timer_create(astro_timer_cb, 16, &g_ast);

    return obj;
}

/*===========================*
 *           app_main
 *===========================*/
void app_main(void)
{
    ESP_LOGI(TAG, "System initialization started");

    /* 1) I2C 总线 & IO 扩展（TCA9554） */
    ESP_ERROR_CHECK(esp_i2c_service_init());
    ESP_ERROR_CHECK(esp_io_expander_service_init());

    /* 2) LCD（QSPI + SPD2010 面板） */
    ESP_ERROR_CHECK(lcd_service_init());
    ESP_LOGI(TAG, "LCD ready");

    /* 3) 触摸（如果需要） */
    // ESP_ERROR_CHECK(lcd_touch_service_init());
    // lcd_touch_service_debug_once();

    /* 4) LVGL 服务 */
    ESP_ERROR_CHECK(lvgl_service_init(
        lcd_service_get_panel(),
        lcd_service_get_panel_io(),
        NULL));

    /* ========== 创建 UI ========== */
    lvgl_port_lock(portMAX_DELAY);

    /* 屏幕渐变背景样式 */
    style_scr_bg_init();
    lv_obj_add_style(lv_screen_active(), &style_scr_bg, 0);

    /* 星盘主体（412×412，自绘） */
    astro_create(lv_screen_active());

    lvgl_port_unlock();

    ESP_LOGI(TAG, "System initialization completed");
}
