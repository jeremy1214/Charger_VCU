#ifndef BMS_h
#define BMS_h

#include <stdbool.h>
#include <driver/twai.h>  // ESP32 TWAI (CAN) driver

#define BMS_BOARD_ID 1
#define NUM_BMS_CV_MSG_ID 6
#define NUM_BMS_NTC_MSG_ID 4
#define MAX_NUM_CELL_PER_CMU 18
#define MAX_NUM_NTC_PER_CMU 12


#define NUM_CMU_MODULE 10
#define NUM_CELL_PER_CMU 12
#define NUM_NTC_PER_CMU 5

#define BMS_INIT_CV_mV 0
#define BMS_INIT_NTC_deg_c 0.0
#define BMS_OV_FAULT_mV 3650
#define BMS_UV_FAULT_mV 2500    
#define BMS_OT_FAULT_deg_c 60.0 

#define BMS_FAULT_CNT_THRESHOLD 30
#define BMS_CAN_SIGNAL_LOST_CNT_THRESHOLD 50

// In BMS.h, add this enum near the top (after the includes)
typedef enum {
    CHG_INITIAL,  // System is initializing
    CHG_TEST,     // Running self-tests
    CHG_READY,    // System ready but not charging
    CHG_START,    // Charging in progress
    CHG_FAIL      // Charging failed
} ChargingState;

typedef struct {
  uint16_t cell_voltage_mV[MAX_NUM_CELL_PER_CMU];
  float NTC_deg_c[MAX_NUM_NTC_PER_CMU];
  uint32_t cell_voltage_fault;
  uint16_t NTC_fault;
} CMU_message_t;

typedef enum{
    BMS_NORMAL,
    BMS_FAULT,
    BMS_SIGNAL_LOST
}BMS_state;

typedef struct VCUBMSComm
{
    BMS_state state;
    ChargingState charging_state;  // Add this line
    uint32_t total_voltage_mV;
    uint16_t cell_voltage_mV_highest;
    uint16_t cell_voltage_mV_lowest;
    uint32_t CAN_signal_lost_cnt;
    float temp_deg_c_highest;
    float temp_deg_c_lowest;
    float temp_deg_c_avg;
    bool over_voltage;
    bool under_voltage;
    bool over_temperature;
    bool signal_lost;

} BMS_t;
#endif

void BMS_CAN_handler(twai_message_t *message);
void BMS_init();
bool BMS_check_fault();
void BMS_update_data();
void BMS_state_update();
void print_cmu_messages();

extern BMS_t Comm_dev;

