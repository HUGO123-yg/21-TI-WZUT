/*********************************************************************************************************************
 * zf_device_config.c — GCC stub for precompiled IAR library
 *
 * This file provides placeholder implementations for symbols originally in
 * zf_device_config.a (IAR precompiled blob, no source available).
 *
 * WARNING: These are ZERO-FILLED stubs. The real calibration/config data
 * must be obtained from SeekFree or extracted from the original .a binary.
 * Using these stubs will result in uncalibrated sensor behavior.
 *
 * Functions provided:
 *   - imu660ra_config_file[8192]  — IMU660RA/RB/RC init register values
 *   - image_frame_header[4]       — Camera frame header magic bytes
 *   - dl1b_config_file[135]       — DL1B ToF sensor config
 *   - mt9v03x_sccb_* functions    — Camera SCCB register access stubs
 ********************************************************************************************************************/

#include "zf_common_headfile.h"

/* --------------------------------------------------------------------------
 * Configuration Data Arrays (ZERO-FILLED — REPLACE WITH REAL DATA)
 * -------------------------------------------------------------------------- */

/** IMU660RA/RB/RC initialization register configuration (8192 bytes) */
const unsigned char imu660ra_config_file[8192] = {
    0x00  /* FIXME: Replace with actual IMU calibration data from SeekFree SDK */
};

/** Image frame header magic bytes (4 bytes) */
const unsigned char image_frame_header[4] = {
    0x00, 0x00, 0x00, 0x00  /* FIXME: Replace with actual camera frame header */
};

/** DL1B ToF sensor configuration data (135 bytes) */
const unsigned char dl1b_config_file[135] = {
    0x00  /* FIXME: Replace with actual DL1B config data */
};

/* --------------------------------------------------------------------------
 * MT9V03X Camera SCCB Functions (STUBS — NOT FUNCTIONAL)
 * These are only needed if the MT9V03X camera module is used.
 * Not called in the default balance-car configuration.
 * -------------------------------------------------------------------------- */

unsigned char mt9v03x_sccb_check_id(void *soft_iic_obj)
{
    (void)soft_iic_obj;
    return 0;  /* STUB: camera not present */
}

unsigned char mt9v03x_sccb_set_config(const short int buff[10][2])
{
    (void)buff;
    return 0;  /* STUB: no-op */
}

unsigned char mt9v03x_sccb_set_exposure_time(unsigned short int light)
{
    (void)light;
    return 0;  /* STUB: no-op */
}

unsigned char mt9v03x_sccb_set_reg(unsigned char addr, unsigned short int data)
{
    (void)addr;
    (void)data;
    return 0;  /* STUB: no-op */
}
