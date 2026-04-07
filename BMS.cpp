#include <cstdint>
#include <driver/twai.h>

#include "BMS.h"

using namespace std;

BMS_IC_info_t bms_ic_info[NUM_IC];



BMS_t bms_t;

void BMS_Init()
{
    // Placeholder for BMS initialization logic
    bms_t.bms_states = BMS_NORMAL;
    bms_t.charging_states = CHARGING_INIT;
    bms_t.total_voltage_mV = 0;
    bms_t.over_voltage = false;
    bms_t.under_voltage = false;
    bms_t.over_temperature = false;
    bms_t.signal_lost = false;
    for(int i=0; i<NUM_IC; i++){
        bms_ic_info[i].CAN_signal_lost_count = 0;
    }
}

BMS_STATES BMS_Check_Fault()
{
    // Placeholder for fault checking logic
    if(bms_t.over_voltage || bms_t.under_voltage || bms_t.over_temperature){
        return BMS_SENSOR_FAULT;
    }else if(bms_t.signal_lost){
        return BMS_FAULT;
    }else{
        return BMS_NORMAL;
    }
}

void BMS_Update_Volt(){
    bms_t.total_voltage_mV = 0;
    for(uint8_t ic = 0; ic < NUM_IC; ic++){
        for(uint8_t cell = 0; cell < ACTIVE_CELLS_PER_IC; cell++){
            bms_t.total_voltage_mV += bms_ic_info[ic].volt_info.voltages[cell];
        }
        if(bms_ic_info[ic].status_info.max_voltage>CELL_OK_MAX_CODE){
            bms_t.over_voltage = true;
        }
        if(bms_ic_info[ic].status_info.min_voltage<CELL_OK_MIN_CODE){
            bms_t.under_voltage = true;
        }
    }
}

void BMS_Update_State(){
    for(uint8_t ic = 0; ic < NUM_IC; ic++){
        if(bms_ic_info[ic].CAN_signal_lost_count < INT32_MAX){
            bms_ic_info[ic].CAN_signal_lost_count++;  
        }
        uint8_t fault_bits = bms_ic_info[ic].status_info.fault_bits;
        switch (fault_bits)
        {
        case 0x01:
            bms_ic_info[ic].status_info.fault_state = SIGNAL_LOST;
            break;
        case 0x02:
            bms_ic_info[ic].status_info.fault_state = SENSOR_FAULT;
            break;
        case 0x04:
            bms_ic_info[ic].status_info.fault_state = OVER_TEMPERATURE;
            break;
        case 0x08:
            bms_ic_info[ic].status_info.fault_state = VOLTAGE_OUT_OF_RANGE;
            break;
        default:
            bms_ic_info[ic].status_info.fault_state = NORMAL;
            break;
        }

        if(bms_ic_info[ic].status_info.fault_state == NORMAL){
            bms_t.signal_lost = bms_ic_info[ic].CAN_signal_lost_count > BMS_SIGNAL_THRESHOLD;
        }
    }

    bms_t.bms_states = BMS_Check_Fault();
    for(uint8_t ic = 0; ic < NUM_IC; ic++){
        
    }

    if(bms_t.bms_states == NORMAL){
        bool signal_lost = 0 > BMS_SIGNAL_THRESHOLD;
        bms_t.signal_lost = signal_lost;
    }
}

void Get_BMS_IC_Info(uint8_t ic){
    bms_ic_info[ic].CAN_signal_lost_count = 0;
}

void BMS_Get_CAN_Message(const twai_message_t *message)
{
    // Placeholder for CAN message retrieval logic
    uint16_t id = static_cast<uint16_t>(message->identifier);

    if (id < 0x100 || id > 0x2FF) return;

    uint8_t *data = message->data;

    if ((id & 0xF00) == CAN_ID_CELL_BASE)
    {
        // Process cell voltage message for IC ic
        // Convert data to voltages using CODE_TO_VOLT
        uint8_t ic = static_cast<uint8_t>((id >> 4) & 0x0F);
        Get_BMS_IC_Info(ic);
        uint8_t frame = static_cast<uint8_t>(id & 0x00F); // Extract frame number from ID
        uint8_t cell_index = frame * 4;                   // Each frame contains 4 cells
        for (uint8_t i = cell_index; i < cell_index + 4 && i < ACTIVE_CELLS_PER_IC; i++)
        {
            uint16_t code = (data[(i - cell_index) * 2 + 1] << 8) | data[(i - cell_index) * 2];
            bms_ic_info[ic].volt_info.voltages[i] = code;
        }
    }
    else if ((id & 0xFF0) == CAN_ID_TEMP_BASE)
    {
        // Process temperature message for IC ic
        // Convert data to temperatures using CODE_TO_TEMP_DECI_C
        uint8_t ic = static_cast<uint8_t>(id & 0x00F); // Extract IC number from ID
        Get_BMS_IC_Info(ic);
        for (uint8_t i = 0; i < NTC_PER_IC; i++)
        {
            uint16_t code = (data[i * 2 + 1] << 8) | data[i * 2];
            int16_t temp_deci_c = static_cast<int16_t>(code); // Assuming code is signed
            bms_ic_info[ic].temp_info.temperatures[i] = temp_deci_c;
        }
    }
    else if ((id & 0xFF0) == CAN_ID_STATUS_BASE)
    {
        // Process status message for IC ic
        // Extract status information from data bytes
        uint8_t ic = static_cast<uint8_t>(id & 0x00F);
        Get_BMS_IC_Info(ic);                         // Extract IC number from ID
        bms_ic_info[ic].status_info.fault_bits = data[0]; // get whether cell discharge or not
        bms_ic_info[ic].status_info.balance_mask = (data[3] << 16) | (data[2] << 8) | data[1];
        uint16_t min_code = (data[5] << 8) | data[4];
        bms_ic_info[ic].status_info.min_voltage = min_code;
        uint16_t max_code = (data[7] << 8) | data[6];
        bms_ic_info[ic].status_info.max_voltage = max_code;
    }
}

void BMS_Update_Data(){
    BMS_Update_Volt();
    BMS_Update_State();
}