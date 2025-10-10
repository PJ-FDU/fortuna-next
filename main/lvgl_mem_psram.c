#include "lvgl.h"
#include "esp_heap_caps.h"

#if LV_USE_STDLIB_MALLOC == LV_STDLIB_CUSTOM

#define PSRAM_CAPS (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
#define INTERNAL_CAPS MALLOC_CAP_8BIT

void lv_mem_init(void) {}

void lv_mem_deinit(void) {}

lv_mem_pool_t lv_mem_add_pool(void *mem, size_t bytes)
{
    LV_UNUSED(mem);
    LV_UNUSED(bytes);
    return NULL;
}

void lv_mem_remove_pool(lv_mem_pool_t pool)
{
    LV_UNUSED(pool);
}

static void *alloc_from_caps(size_t size, uint32_t caps)
{
    if (size == 0)
    {
        return NULL;
    }
    return heap_caps_malloc(size, caps);
}

void *lv_malloc_core(size_t size)
{
    void *ptr = alloc_from_caps(size, PSRAM_CAPS);
    if (ptr)
    {
        return ptr;
    }
    return alloc_from_caps(size, INTERNAL_CAPS);
}

void *lv_realloc_core(void *p, size_t new_size)
{
    if (new_size == 0)
    {
        heap_caps_free(p);
        return NULL;
    }

    void *ptr = heap_caps_realloc(p, new_size, PSRAM_CAPS);
    if (ptr)
    {
        return ptr;
    }
    return heap_caps_realloc(p, new_size, INTERNAL_CAPS);
}

void lv_free_core(void *p)
{
    if (p)
    {
        heap_caps_free(p);
    }
}

void lv_mem_monitor_core(lv_mem_monitor_t *mon_p)
{
    if (mon_p == NULL)
    {
        return;
    }

    multi_heap_info_t info_psram = {0};
    heap_caps_get_info(&info_psram, PSRAM_CAPS);

    multi_heap_info_t info_internal = {0};
    heap_caps_get_info(&info_internal, INTERNAL_CAPS);

    size_t total_size = info_psram.total_allocated_bytes + info_psram.total_free_bytes +
                        info_internal.total_allocated_bytes + info_internal.total_free_bytes;
    size_t free_size = info_psram.total_free_bytes + info_internal.total_free_bytes;
    size_t used_size = total_size - free_size;

    mon_p->total_size = total_size;
    mon_p->free_size = free_size;
    mon_p->used_cnt = 0;
    mon_p->free_cnt = 0;
    mon_p->free_biggest_size = info_psram.largest_free_block > info_internal.largest_free_block ? info_psram.largest_free_block : info_internal.largest_free_block;
    mon_p->max_used = used_size;
    mon_p->used_pct = (total_size == 0) ? 0 : (uint8_t)((used_size * 100) / total_size);
    mon_p->frag_pct = mon_p->free_biggest_size == 0 || free_size == 0 ? 0 : (uint8_t)(100 - (mon_p->free_biggest_size * 100) / free_size);
}

lv_result_t lv_mem_test_core(void)
{
    return LV_RESULT_OK;
}

#endif /* LV_USE_STDLIB_MALLOC == LV_STDLIB_CUSTOM */
