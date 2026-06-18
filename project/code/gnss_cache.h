/*********************************************************************************************************************
 * 文件名称          gnss_cache
 * 功能描述          GPS 热启动缓存管理 — 利用 Flash Page 0 存储上次定位数据，配合后备电池实现快速热启动
 * 注意事项          缓存数据约 40 字节，占用 Flash Page 0（2KB 中极小部分）
 *                  写入以 5 分钟为间隔，避免频繁擦写 Flash
 ********************************************************************************************************************/

#ifndef _gnss_cache_h_
#define _gnss_cache_h_

#include "zf_common_headfile.h"

// ============== Flash 缓存配置 ==============
#define GNSS_CACHE_PAGE         0           // Flash Page 0（空闲页）
#define GNSS_CACHE_MAGIC        0x47505348  // "GPSH" — 缓存有效标志
#define GNSS_CACHE_SAVE_INTERVAL 300000     // 缓存保存间隔：5 分钟（单位 ms）

// ============== Allystar CFG-SIMPLERST 启动模式命令 ==============
// 协议格式: F1 D9 06 40 [len] [mode] [CK_A] [CK_B]
// 参见 DATAGNSS Allystar Binary Protocol Specification
#define GNSS_START_COLD         0x01        // 冷启动 — 清除全部缓存
#define GNSS_START_WARM         0x02        // 暖启动 — 保留历书/时间，丢弃星历
#define GNSS_START_HOT          0x03        // 热启动 — 使用全部缓存数据

// ============== GPS 缓存数据结构（32 bytes，Flash 友好） ==============
typedef struct
{
    uint32  magic;                          // 0x47505348 — 有效缓存标志
    int32   latitude_e7;                    // 纬度 × 1e7（例：39.9042° → 399042000）
    int32   longitude_e7;                   // 经度 × 1e7（例：116.4074° → 1164074000）
    int32   altitude_cm;                    // 高度 × 100（单位 cm）
    int32   ground_speed_cm;                // 地面速度 × 100（单位 cm/s）
    int16   heading_deg;                    // 航向角 × 100（单位 0.01°）
    uint16  year;                           // 年份（例：2026）
    uint8   month;                          // 月份（1-12）
    uint8   day;                            // 日期（1-31）
    uint8   hour;                           // 小时（0-23，北京时间）
    uint8   minute;                         // 分钟（0-59）
    uint8   second;                         // 秒（0-59）
    uint8   satellite_used;                 // 定位卫星数
    uint8   save_count;                     // 缓存保存次数（判断是否有过成功定位）
    uint8   reserved[5];                    // 对齐保留
} gnss_cache_struct;

// ============== API ==============
// 函数功能：从 Flash Page 0 读取 GPS 缓存，判断是否有效
// 返回值：  true — 缓存有效（上次定位成功过），false — 无有效缓存
// 注意事项：如果缓存有效，结构体中包含上次定位的完整数据
bool gnss_cache_load(gnss_cache_struct *cache);

// 函数功能：将当前 GPS 定位数据保存到 Flash Page 0
// 注意事项：会自动做间隔判断（5 分钟内不重复写），避免频繁擦写 Flash
void gnss_cache_save(const gnss_cache_struct *cache);

// 函数功能：发送 GPS 启动模式命令（冷/暖/热启动）
// 输入参数：start_mode — GNSS_START_COLD / GNSS_START_WARM / GNSS_START_HOT
// 注意事项：需在 gnss_init() 配置完成后调用
void gnss_send_startup_mode(uint8 start_mode);

// 函数功能：GPS 热启动智能调度 — 根据缓存状态自动选择最佳启动模式
// 注意事项：需在 gnss_init() 配置完成后调用
void gnss_auto_start(void);

#endif /* _gnss_cache_h_ */
