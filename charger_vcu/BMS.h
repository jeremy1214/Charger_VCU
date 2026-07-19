#ifndef BMS_H
#define BMS_H
#include <cstdint>
#include <driver/twai.h>
#include <Arduino.h>

using namespace std;

#define NUM_IC 6
#define ACTIVE_CELLS_PER_IC 17
#define NTC_PER_IC 4
#define NUM_FRAMES 5

const uint16_t CAN_ID_CELL_BASE = 0x1100;
const uint16_t CAN_ID_TEMP_BASE = 0x1200;
const uint16_t CAN_ID_STATUS_BASE = 0x1300;

const uint16_t CELL_OK_MIN_CODE = 30000;
const uint16_t CELL_OK_MAX_CODE = 33500;

const int16_t TEMP_LIMIT_DECI_C = 600;

const uint32_t BMS_SIGNAL_THRESHOLD = 30;

const float CODE_TO_VOLT = 0.0001f;
const int CODE_TO_TEMP_DECI_C = 10; // 0.1°C per code

typedef struct
{
    uint16_t voltages[ACTIVE_CELLS_PER_IC]; // Cell voltages in volts

} Battery_Volt_Info_t;

typedef struct
{
    int16_t temperatures[NTC_PER_IC]; // Temperatures in °0.1C
} Battery_Temp_Info_t;

typedef enum {
    SIGNAL_LOST,
    OVER_TEMPERATURE,
    VOLTAGE_OUT_OF_RANGE,
    SENSOR_FAULT,
    NORMAL
} FAULT_STATES;

typedef struct
{
    uint8_t fault_bits;
    FAULT_STATES fault_state;
    uint32_t balance_mask; // Bitmask indicating which cells are being balanced
    uint16_t min_voltage;
    uint16_t max_voltage;
} Battery_Status_Info_t;

typedef struct
{
    Battery_Volt_Info_t volt_info;
    Battery_Temp_Info_t temp_info;
    Battery_Status_Info_t status_info;
    uint32_t CAN_signal_lost_count;
} BMS_IC_info_t;

typedef enum
{
    BMS_FAULT,
    BMS_NORMAL,
    BMS_SENSOR_FAULT
} BMS_STATES;

typedef enum{
    CHARGING_INIT,
    CHARGING_TEST,
    CHARGING_READY,
    CHARGING_START,
    CHARGING_FAIL
}CHARGING_STATES;

typedef struct{
    BMS_STATES bms_states;
    CHARGING_STATES charging_states;
    uint32_t total_voltage_mV;
    bool over_voltage;
    bool under_voltage;
    bool over_temperature;
    bool signal_lost;
    bool bms_fault;
}BMS_t;

#endif

void BMS_Init();
BMS_STATES BMS_Check_Fault();
void BMS_Update_Volt();
void BMS_Update_State();
void BMS_Get_CAN_Message(const twai_message_t *message);
void Get_BMS_IC_Info(uint8_t ic);
void BMS_Update_Data();
// 在 BMS.h 的末尾附近
void BMS_Print_Diagnostics();
void BMS_Print_Cell_Voltages();
