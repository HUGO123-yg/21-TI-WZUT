#include "zf_common_headfile.h"
#include "gnss_cache.h"

// 上次保存时的 sys_times（避免频繁擦写 Flash）
static uint32 last_save_tick = 0;
static gnss_cache_struct cache_buffer;

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：从 Flash Page 0 读取 GPS 缓存，判断是否有效
// 返回值：  true — 缓存有效（上次定位成功过），false — 无有效缓存
//-------------------------------------------------------------------------------------------------------------------
bool gnss_cache_load(gnss_cache_struct *cache)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(0, GNSS_CACHE_PAGE, sizeof(gnss_cache_struct));

    // 从缓冲区复制到结构体（逐字节，因为 flash_union_buffer 是 union 类型）
    uint8 *src = (uint8 *)flash_union_buffer;
    uint8 *dst = (uint8 *)cache;
    for (uint32 i = 0; i < sizeof(gnss_cache_struct); i++)
    {
        dst[i] = src[i];
    }

    flash_buffer_clear();

    if (cache->magic != GNSS_CACHE_MAGIC)
        return false;
    if (cache->save_count == 0)
        return false;

    return true;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：将当前 GPS 定位数据保存到 Flash Page 0
// 注意事项：自动做 5 分钟间隔判断，避免频繁擦写 Flash（寿命 ~100k 次擦除）
//-------------------------------------------------------------------------------------------------------------------
void gnss_cache_save(const gnss_cache_struct *cache)
{
    // 5 分钟保存间隔（sys_times 单位为 ms）
    if (last_save_tick != 0 && (sys_times - last_save_tick) < GNSS_CACHE_SAVE_INTERVAL)
        return;
    last_save_tick = sys_times;

    memcpy(&cache_buffer, cache, sizeof(gnss_cache_struct));

    flash_buffer_clear();

    // 复制结构体到 Flash 缓冲区
    uint8 *src = (uint8 *)&cache_buffer;
    uint8 *dst = (uint8 *)flash_union_buffer;
    for (uint32 i = 0; i < sizeof(gnss_cache_struct); i++)
    {
        dst[i] = src[i];
    }

    flash_erase_page(0, GNSS_CACHE_PAGE);
    flash_write_page_from_buffer(0, GNSS_CACHE_PAGE, sizeof(gnss_cache_struct));
    flash_buffer_clear();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：发送 GPS 启动模式命令（Allystar CFG-SIMPLERST）
// 输入参数：start_mode — GNSS_START_COLD(1) / GNSS_START_WARM(2) / GNSS_START_HOT(3)
// 协议格式：F1 D9 06 40 01 00 [mode] [CK_A] [CK_B]
//-------------------------------------------------------------------------------------------------------------------
void gnss_send_startup_mode(uint8 start_mode)
{
    // 预计算 3 种模式对应的校验和（CK_A, CK_B）
    // Cold:  F1 D9 06 40 01 00 01 → CK=0x2248
    // Warm:  F1 D9 06 40 01 00 02 → CK=0x2349
    // Hot:   F1 D9 06 40 01 00 03 → CK=0x244A
    const uint8 cold_start[] = {0xF1, 0xD9, 0x06, 0x40, 0x01, 0x00, 0x01, 0x48, 0x22};
    const uint8 warm_start[] = {0xF1, 0xD9, 0x06, 0x40, 0x01, 0x00, 0x02, 0x49, 0x23};
    const uint8 hot_start[]  = {0xF1, 0xD9, 0x06, 0x40, 0x01, 0x00, 0x03, 0x4A, 0x24};

    const uint8 *cmd;
    switch (start_mode)
    {
        case GNSS_START_HOT:  cmd = hot_start;  break;
        case GNSS_START_WARM: cmd = warm_start; break;
        default:              cmd = cold_start; break;
    }

    uart_write_buffer(GNSS_UART, (uint8 *)cmd, 9);
    system_delay_ms(50);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：GPS 热启动智能调度
//          根据 Flash 缓存状态自动选择：
//            - 有缓存 → 热启动（后备电池保持星历/历书有效）
//            - 无缓存 → 冷启动（首次上电）
// 注意事项：需在 gnss_init() 配置完成、UART 已初始化后调用
//-------------------------------------------------------------------------------------------------------------------
void gnss_auto_start(void)
{
    gnss_cache_struct cache;
    if (gnss_cache_load(&cache))
    {
        gnss_send_startup_mode(GNSS_START_HOT);
    }
    // 无缓存时保持默认冷启动，不发送额外命令
}
