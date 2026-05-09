/*********************************************************************************************************************
 * 文件名称          ipc_protocol
 * 功能说明          CM7_1→CM7_0 IPC vision data transfer protocol — shared header
 *                   Defines message types, float↔uint32 safe conversion, and vision data packing/unpacking macros
 * 版本信息          Multicore Motion Vision — Wave 1
 * 开发平台          IAR 9.40.1
 * 硬件平台          CYT4BB
 *
 * 修改记录
 * 日期              作者              备注
 * 2026-5-9         Seekfree          first version — Wave 1 IPC protocol definition
 ********************************************************************************************************************/

#ifndef _ipc_protocol_h_
#define _ipc_protocol_h_

#include "zf_common_headfile.h"

//===== IPC 消息类型定义 =====

#define IPC_MSG_TYPE_VISION          (0x01)  // vision data: image_error + quality packed into uint32
#define IPC_MSG_TYPE_HEARTBEAT       (0x02)  // heartbeat / keep-alive message

//===== float↔uint32 安全转换（严格别名规则安全） =====

typedef union
{
    float   f_value;
    uint32  u_value;
} ipc_float_converter_t;

//===== 视觉数据打包/解包宏 =====
//
// 数据格式 (uint32):
//   [31:24] reserved (0)
//   [23:16] quality        (uint8,  0-255)
//   [15:0]  image_error    (int16,  fixed-point ×1000, range ±32.767 pixels, resolution 0.001 pixel)

#define IPC_VISION_PACK(error_float, quality) \
    ((uint32)((uint8)((quality)) << 16) | ((uint32)((int16)((error_float) * 1000.0f)) & 0xFFFF))

#define IPC_VISION_UNPACK_ERROR(packed) \
    ((float)((int16)((packed) & 0xFFFF)) / 1000.0f)

#define IPC_VISION_UNPACK_QUALITY(packed) \
    ((uint8)(((packed) >> 16) & 0xFF))

#endif
