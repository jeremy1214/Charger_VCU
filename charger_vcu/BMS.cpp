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
    bms_t.over_voltage = false;  // Reset flags before checking
    bms_t.under_voltage = false;
    
    for(uint8_t ic = 0; ic < NUM_IC; ic++){
        for(uint8_t cell = 0; cell < ACTIVE_CELLS_PER_IC; cell++){
            // Convert code to voltage: code * CODE_TO_VOLT = voltage in V
            // Then multiply by 1000 to get millivolts
            uint32_t cell_voltage_mv = (uint32_t)(bms_ic_info[ic].volt_info.voltages[cell] * CODE_TO_VOLT * 1000);
            bms_t.total_voltage_mV += cell_voltage_mv;
        }
        if(bms_ic_info[ic].status_info.max_voltage > CELL_OK_MAX_CODE){
            bms_t.over_voltage = true;
        }
        if(bms_ic_info[ic].status_info.min_voltage < CELL_OK_MIN_CODE){
            bms_t.under_voltage = true;
        }
    }
}

void BMS_Update_State(){
    bms_t.signal_lost = false;  // Reset signal_lost flag before checking
    bms_t.over_temperature = false;  // Reset temperature flag
    
    for(uint8_t ic = 0; ic < NUM_IC; ic++){
        // Increment signal lost counter if no message received
        if(bms_ic_info[ic].CAN_signal_lost_count < UINT32_MAX){
            bms_ic_info[ic].CAN_signal_lost_count++;  
        }
        
        // Check if signal is lost for this IC
        if(bms_ic_info[ic].CAN_signal_lost_count > BMS_SIGNAL_THRESHOLD){
            bms_t.signal_lost = true;
        }
        
        // Decode fault bits
        uint8_t fault_bits = bms_ic_info[ic].status_info.fault_bits;
        switch (fault_bits)
        {
        case 0x01:
            bms_ic_info[ic].status_info.fault_state = SIGNAL_LOST;
            bms_t.signal_lost = true;
            break;
        case 0x02:
            bms_ic_info[ic].status_info.fault_state = SENSOR_FAULT;
            break;
        case 0x04:
            bms_ic_info[ic].status_info.fault_state = OVER_TEMPERATURE;
            bms_t.over_temperature = true;
            break;
        case 0x08:
            bms_ic_info[ic].status_info.fault_state = VOLTAGE_OUT_OF_RANGE;
            break;
        default:
            bms_ic_info[ic].status_info.fault_state = NORMAL;
            break;
        }
    }

    // Update overall BMS state based on all fault conditions
    bms_t.bms_states = BMS_Check_Fault();
}

void Get_BMS_IC_Info(uint8_t ic){
    bms_ic_info[ic].CAN_signal_lost_count = 0;
}

void BMS_Get_CAN_Message(const twai_message_t *message)
{
    // Placeholder for CAN message retrieval logic
    uint16_t id = static_cast<uint16_t>(message->identifier);

    if (id < 0x100 || id > 0x2FF) return;

    const uint8_t *data = message->data; // 在前面加上 const

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
            uint16_t byte_offset = (i - cell_index) * 2;
            uint16_t code = (data[byte_offset + 1] << 8) | data[byte_offset];
            bms_ic_info[ic].volt_info.voltages[i] = code;  // Store raw code value
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

void BMS_Print_Diagnostics() {
    Serial.println("--- BMS Diagnostic Report ---");
    
    if (bms_t.over_voltage) {
        Serial.println("[!] ERROR: Over Voltage Detected!");
    }
    if (bms_t.under_voltage) {
        Serial.println("[!] ERROR: Under Voltage Detected!");
    }
    if (bms_t.over_temperature) {
        Serial.println("[!] ERROR: Over Temperature Detected!");
    }
    if (bms_t.signal_lost) {
        Serial.println("[!] WARNING: CAN Signal Lost from some ICs.");
    }

    // 逐一檢查每個 IC 的具體數值
    for (int i = 0; i < NUM_IC; i++) {
        bool ic_has_issue = false;
        
        // 檢查該 IC 是否斷聯
        if (bms_ic_info[i].CAN_signal_lost_count > BMS_SIGNAL_THRESHOLD) {
            Serial.printf("  - IC %d: OFFLINE (Signal Lost)\n", i);
            continue; 
        }

        // 檢查電壓範圍 (範例邏輯)
        for (int j = 0; j < ACTIVE_CELLS_PER_IC; j++) {
            uint16_t v = bms_ic_info[i].volt_info.voltages[j];
            if (v > CELL_OK_MAX_CODE || v < CELL_OK_MIN_CODE) {
                Serial.printf("  - IC %d Cell %d: Voltage Abnormal (%d)\n", i, j, v);
                ic_has_issue = true;
            }
        }

        // 檢查溫度
        for (int j = 0; j < NTC_PER_IC; j++) {
            if (bms_ic_info[i].temp_info.temperatures[j] > TEMP_LIMIT_DECI_C) {
                Serial.printf("  - IC %d NTC %d: Over Temp (%d C)\n", i, j, bms_ic_info[i].temp_info.temperatures[j]/10);
                ic_has_issue = true;
            }
        }
    }
    Serial.println("-----------------------------");
}

void BMS_Print_Cell_Voltages() {
    Serial.println("=== Battery Cell Voltage Report ===");
    
    // 計算最大、最小和總電壓
    uint16_t max_voltage_mv = 0;
    uint16_t min_voltage_mv = UINT16_MAX;
    uint32_t total_voltage_mv = 0;
    uint8_t valid_cell_count = 0;
    
    // 輸出每個電池單元的電壓
    for (uint8_t ic = 0; ic < NUM_IC; ic++) {
        Serial.printf("IC %d:\n", ic);
        
        for (uint8_t cell = 0; cell < ACTIVE_CELLS_PER_IC; cell++) {
            // 將原始代碼轉換為電壓 (mV)
            uint32_t cell_voltage_mv = (uint32_t)(bms_ic_info[ic].volt_info.voltages[cell] * CODE_TO_VOLT * 1000);
            
            // 只統計有效的電壓值 (非零值)
            if (cell_voltage_mv > 0) {
                total_voltage_mv += cell_voltage_mv;
                valid_cell_count++;
                
                if (cell_voltage_mv > max_voltage_mv) {
                    max_voltage_mv = cell_voltage_mv;
                }
                if (cell_voltage_mv < min_voltage_mv) {
                    min_voltage_mv = cell_voltage_mv;
                }
            }
            
            // 輸出格式: Cell X: XXXX mV
            Serial.printf("  Cell %2d: %4d mV", cell, cell_voltage_mv);
            
            // 每行輸出4個電池單元
            if ((cell + 1) % 4 == 0) {
                Serial.println();
            } else {
                Serial.print(" | ");
            }
        }
        
        // 如果最後一行沒有滿4個，補上換行
        if (ACTIVE_CELLS_PER_IC % 4 != 0) {
            Serial.println();
        }
        Serial.println();
    }
    
    // 輸出統計信息
    Serial.println("=== Voltage Statistics ===");
    Serial.printf("Total Cells: %d\n", NUM_IC * ACTIVE_CELLS_PER_IC);
    Serial.printf("Valid Cells: %d\n", valid_cell_count);
    
    if (valid_cell_count > 0) {
        Serial.printf("Max Voltage: %d mV (%.3f V)\n", max_voltage_mv, max_voltage_mv / 1000.0f);
        Serial.printf("Min Voltage: %d mV (%.3f V)\n", min_voltage_mv, min_voltage_mv / 1000.0f);
        Serial.printf("Total Voltage: %d mV (%.3f V)\n", total_voltage_mv, total_voltage_mv / 1000.0f);
        Serial.printf("Average Voltage: %.1f mV (%.4f V)\n", 
                     (float)total_voltage_mv / valid_cell_count, 
                     (float)total_voltage_mv / valid_cell_count / 1000.0f);
        
        // 計算電壓差異
        uint16_t voltage_diff_mv = max_voltage_mv - min_voltage_mv;
        Serial.printf("Voltage Difference: %d mV (%.3f V)\n", voltage_diff_mv, voltage_diff_mv / 1000.0f);
    } else {
        Serial.println("No valid voltage data available");
    }
    
    Serial.println("============================");
}