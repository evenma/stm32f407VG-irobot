/**
 * @file packet_reports.h
 * @author Leo.ma
 * @brief 所有串口通信数据回报
 * @version 0.1
 * @date 2026-05-3
 *
 * @copyright Copyright (c) 2026
 *
 */
#include <stdint.h>

#pragma pack(1)
typedef union  {
    float array[4];
    struct {
        float w;
        float x;
        float y;
        float z;
    } element;
} PacketReportIMU_Quat_TypeDef;

typedef struct {
	uint8_t sub_cmd;
	uint16_t voltage;
}PacketReportBatteryVoltageTypeDef;

typedef union  {
    struct {
		float accel_array[3];
		float gyro_array[3];
	}array;
    struct {
        struct {
            float x;
            float y;
            float z;
        } accel;
        struct {
            float x;
            float y;
            float z;
        } gyro;
    } element;
} PacketReportIMU_Raw_TypeDef;

typedef struct  {
    uint8_t key_id;
    uint8_t event;
} PacketReportKeyEventTypeDef;

#pragma pack()
