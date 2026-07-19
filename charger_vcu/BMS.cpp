#include <cstdint>
#include <driver/twai.h>

#include "BMS.h"

using namespace std;

BMS_IC_info_t bms_ic_info[NUM_IC];
BMS_t bms_t;

/**
 * @brief Initialize BMS system and all IC structures
 * Reset all counters, flags, and states to default values
 */
void BMS_Init()
{
    // Initialize BMS overall state
    bms_t.bms_states = BMS_NORMAL;
    bms_t.charging_states = CHARGING_INIT;
    bms_t.total_voltage_mV = 0;
    bms_t.over_voltage = false;
    bms_t.under_voltage = false;
    bms_t.over_temperature = false;
    bms_t.signal_lost = false;
    bms_t.bms_fault = false;
    
    // Initialize each IC's data and counters
    for(uint8_t ic = 0; ic < NUM_IC; ic++){
        bms_ic_info[ic].CAN_signal_lost_count = 0;
        
        // Initialize voltage data
        for(uint8_t cell = 0; cell < ACTIVE_CELLS_PER_IC; cell++){
            bms_ic_info[ic].volt_info.voltages[cell] = 0;
        }
        
        // Initialize temperature data
        for(uint8_t ntc = 0; ntc < NTC_PER_IC; ntc++){
            bms_ic_info[ic].temp_info.temperatures[ntc] = 0;
        }
        
        // Initialize status data
        bms_ic_info[ic].status_info.fault_bits = 0;
        bms_ic_info[ic].status_info.fault_state = NORMAL;
        bms_ic_info[ic].status_info.balance_mask = 0;
        bms_ic_info[ic].status_info.min_voltage = 0;
        bms_ic_info[ic].status_info.max_voltage = 0;
    }
}

/**
 * @brief Check BMS fault conditions based on voltage, temperature, and signal status
 * @return BMS_STATES Current BMS state (NORMAL, SENSOR_FAULT, or FAULT)
 */
BMS_STATES BMS_Check_Fault()
{
    // Priority 1: Sensor faults (voltage/temperature issues)
    for(uint8_t ic = 0; ic < NUM_IC; ic++){
        if(bms_ic_info[ic].status_info.fault_state == SENSOR_FAULT ||
           bms_ic_info[ic].status_info.fault_state == OVER_TEMPERATURE ||
           bms_ic_info[ic].status_info.fault_state == VOLTAGE_OUT_OF_RANGE){
            return BMS_SENSOR_FAULT;
        }
    }
    
    // Priority 2: Communication faults (signal lost)
    if(bms_t.signal_lost){
        return BMS_FAULT;
    }
    
    // No faults detected
    return BMS_NORMAL;
}

void BMS_Update_Volt(){
    bms_t.total_voltage_mV = 0;
    bms_t.over_voltage = false;  // Reset flags before checking
    bms_t.under_voltage = false;
    
    for(uint8_t ic = 0; ic < NUM_IC; ic++){
        // Skip offline ICs when calculating voltage statistics
        if(bms_ic_info[ic].CAN_signal_lost_count > BMS_SIGNAL_THRESHOLD){
            continue;
        }
        
        // Sum up all cell voltages for this IC
        for(uint8_t cell = 0; cell < ACTIVE_CELLS_PER_IC; cell++){
            // Safely convert raw code to voltage in millivolts
            // code * CODE_TO_VOLT = voltage in V, then * 1000 = mV
            uint32_t cell_voltage_mv = (uint32_t)(bms_ic_info[ic].volt_info.voltages[cell] * CODE_TO_VOLT * 1000);
            bms_t.total_voltage_mV += cell_voltage_mv;
        }
    }
}

void BMS_Update_State(){
    bms_t.signal_lost = false;  // Reset signal_lost flag before checking
    bms_t.over_temperature = false;  // Reset temperature flag
    bms_t.bms_fault = false;  // Reset overall fault flag
    
    for(uint8_t ic = 0; ic < NUM_IC; ic++){
        // Increment signal lost counter ONLY if this IC has not received messages
        // The counter is reset to 0 when a message is received (via BMS_Reset_Signal_Lost_Count)
        if(bms_ic_info[ic].CAN_signal_lost_count < UINT32_MAX){
            bms_ic_info[ic].CAN_signal_lost_count++;
        }
        
        // Check if signal is lost for this IC based on counter threshold
        bool ic_signal_lost = (bms_ic_info[ic].CAN_signal_lost_count > BMS_SIGNAL_THRESHOLD);
        
        // Decode fault bits according to the BMS status definition:
        // Bit 0: !BMS_OK -> general fault
        // Bit 1: Temp Range ERR -> sensor fault
        // Bit 2: Over Temp -> over temperature
        // Bit 3: Cell range ERR -> voltage out of range
        // Bit 4: Open Wire -> sensor fault
        uint8_t fault_bits = bms_ic_info[ic].status_info.fault_bits;
        
        // Determine fault state for this IC based on CAN loss first, then fault bits
        FAULT_STATES ic_fault_state = NORMAL;
        if(ic_signal_lost){
            ic_fault_state = SIGNAL_LOST;
        }
        else if(fault_bits & 0x01){  // !BMS_OK
            ic_fault_state = SENSOR_FAULT;
        }
        else if(fault_bits & 0x02){  // Temp Range ERR
            ic_fault_state = SENSOR_FAULT;
        }
        else if(fault_bits & 0x04){  // Over Temp
            ic_fault_state = OVER_TEMPERATURE;
        }
        else if(fault_bits & 0x08){  // Cell range ERR
            ic_fault_state = VOLTAGE_OUT_OF_RANGE;
        }
        else if(fault_bits & 0x10){  // Open Wire
            ic_fault_state = SENSOR_FAULT;
        }
        
        // Update IC fault state
        bms_ic_info[ic].status_info.fault_state = ic_fault_state;
        
        // Update overall BMS flags based on fault bits / state only
        if(ic_fault_state == SIGNAL_LOST){
            bms_t.signal_lost = true;
        }
        if(ic_fault_state == OVER_TEMPERATURE){
            bms_t.over_temperature = true;
        }
        if(ic_fault_state != NORMAL){
            bms_t.bms_fault = true;
        }
    }

    // Update overall BMS state based on fault state rather than computed voltages/temps
    bms_t.bms_states = BMS_Check_Fault();
}

/**
 * @brief Reset CAN signal lost counter when a valid message is received
 * @param ic IC index to reset the counter for
 */
void BMS_Reset_Signal_Lost_Count(uint8_t ic)
{
    if(ic < NUM_IC){
        bms_ic_info[ic].CAN_signal_lost_count = 0;
    }
}

void BMS_Get_CAN_Message(const twai_message_t *message)
{
    // Validate input pointer
    if(message == nullptr) return;
    
    uint16_t id = static_cast<uint16_t>(message->identifier);

    // Only process messages in valid CAN ID range
    if (id < 0x100 || id > 0x2FF) return;

    const uint8_t *data = message->data;

    if ((id & 0xFF00) == CAN_ID_CELL_BASE)
    {
        // Process cell voltage message for IC ic
        // CAN ID format: 0x1XY where X = IC number, Y = frame number
        uint8_t ic = static_cast<uint8_t>((id >> 4) & 0x00F);
        
        // Validate IC index
        if(ic >= NUM_IC) return;
        
        // Reset signal lost counter since we received a message from this IC
        BMS_Reset_Signal_Lost_Count(ic);
        
        uint8_t frame = static_cast<uint8_t>(id & 0x000F); // Extract frame number from ID
        uint8_t cell_index = frame * 4;                   // Each frame contains 4 cells
        
        // Process up to 4 cell voltages from this frame
        for (uint8_t i = cell_index; i < cell_index + 4 && i < ACTIVE_CELLS_PER_IC; i++)
        {
            uint16_t byte_offset = (i - cell_index) * 2;
            uint16_t code = (data[byte_offset + 1] << 8) | data[byte_offset];
            bms_ic_info[ic].volt_info.voltages[i] = code;  // Store raw code value
        }
    }
    else if ((id & 0xFFF0) == CAN_ID_TEMP_BASE)
    {
        // Process temperature message for IC
        // CAN ID format: 0x2XY where X = reserved, Y = IC number
        uint8_t ic = static_cast<uint8_t>(id & 0x000F); // Extract IC number from ID
        
        // Validate IC index
        if(ic >= NUM_IC) return;
        
        // Reset signal lost counter since we received a message from this IC
        BMS_Reset_Signal_Lost_Count(ic);
        
        // Process temperature data for each NTC sensor
        for (uint8_t i = 0; i < NTC_PER_IC; i++)
        {
            uint16_t code = (data[i * 2 + 1] << 8) | data[i * 2];
            int16_t temp_deci_c = static_cast<int16_t>(code); // Assuming code is signed
            bms_ic_info[ic].temp_info.temperatures[i] = temp_deci_c;
        }
    }
    else if ((id & 0xFFF0) == CAN_ID_STATUS_BASE)
    {
        // Process status message for IC
        // CAN ID format: 0x24Y where Y = IC number
        uint8_t ic = static_cast<uint8_t>(id & 0x000F);
        
        // Validate IC index
        if(ic >= NUM_IC) return;
        
        // Reset signal lost counter since we received a message from this IC
        BMS_Reset_Signal_Lost_Count(ic);
        
        // Extract status information from data bytes
        bms_ic_info[ic].status_info.fault_bits = data[0];
        bms_ic_info[ic].status_info.balance_mask = ((uint32_t)data[3] << 16) |
                                                    ((uint32_t)data[2] << 8) |
                                                    data[1];
        bms_ic_info[ic].status_info.balance_mask &= 0x0001FFFFu; // 17-bit balance mask
        
        // Extract and convert min/max voltages
        uint16_t min_code = (data[5] << 8) | data[4];
        bms_ic_info[ic].status_info.min_voltage = static_cast<uint32_t>(min_code) * CODE_TO_VOLT * 1000; // Store as mV
        
        uint16_t max_code = (data[7] << 8) | data[6];
        bms_ic_info[ic].status_info.max_voltage = static_cast<uint32_t>(max_code) * CODE_TO_VOLT * 1000; // Store as mV
    }
}

/**
 * @brief Update all BMS data including voltage and system state
 * Call this periodically (e.g., in main loop) to update BMS status
 */
void BMS_Update_Data(){
    BMS_Update_Volt();      // Update voltage measurements and check voltage ranges
    BMS_Update_State();     // Update fault states and system status
}

void BMS_Print_Diagnostics() {
    Serial.println("\n=== BMS Diagnostic Report ===");
    
    // Print overall BMS fault status based on fault bits and signal status only
    Serial.println("[Overall Status]");
    if (bms_t.signal_lost) {
        Serial.println("  [!] WARNING: CAN Signal Lost from some ICs.");
    }
    if (bms_t.bms_states == BMS_SENSOR_FAULT) {
        Serial.println("  [!] ERROR: Sensor fault detected from fault bits.");
    }
    if (bms_t.bms_states == BMS_FAULT && !bms_t.signal_lost) {
        Serial.println("  [!] ERROR: Fault detected from fault bits.");
    }
    if (!bms_t.signal_lost && bms_t.bms_states == BMS_NORMAL) {
        Serial.println("  [OK] System operating normally.");
    }

    // Print per-IC status
    Serial.println("\n[Per-IC Status]");
    for (uint8_t ic = 0; ic < NUM_IC; ic++) {
        Serial.printf("IC %d: ", ic);
        
        // Check if signal is lost for this IC
        bool ic_offline = (bms_ic_info[ic].CAN_signal_lost_count > BMS_SIGNAL_THRESHOLD);
        if (ic_offline) {
            Serial.printf("OFFLINE (Signal Lost - Count: %lu)\n", 
                         bms_ic_info[ic].CAN_signal_lost_count);
            continue; 
        }
        
        // IC is online, check for specific faults
        bool ic_has_issue = false;
        FAULT_STATES fault_state = bms_ic_info[ic].status_info.fault_state;
        
        switch(fault_state) {
            case SIGNAL_LOST:
                Serial.println("SIGNAL LOST");
                ic_has_issue = true;
                break;
            case SENSOR_FAULT:
                Serial.println("SENSOR FAULT");
                ic_has_issue = true;
                break;
            case OVER_TEMPERATURE:
                Serial.println("OVER TEMPERATURE");
                ic_has_issue = true;
                break;
            case VOLTAGE_OUT_OF_RANGE:
                Serial.println("VOLTAGE OUT OF RANGE");
                ic_has_issue = true;
                break;
            case NORMAL:
                Serial.println("NORMAL");
                break;
        }
        
        if (!ic_has_issue) {
            // Additional checks for online and normally-reporting ICs
            bool voltage_issue = false;
            for (uint8_t j = 0; j < ACTIVE_CELLS_PER_IC; j++) {
                uint16_t v = bms_ic_info[ic].volt_info.voltages[j];
                if (v > CELL_OK_MAX_CODE || v < CELL_OK_MIN_CODE) {
                    if (!voltage_issue) {
                        Serial.printf("  [!] Voltage Abnormal - Cell %d: %d\n", j, v);
                        voltage_issue = true;
                    }
                }
            }
            
            // Check temperatures
            for (uint8_t j = 0; j < NTC_PER_IC; j++) {
                if (bms_ic_info[ic].temp_info.temperatures[j] > TEMP_LIMIT_DECI_C) {
                    Serial.printf("  [!] Over Temperature - NTC %d: %d (%.1f°C)\n", 
                                 j, bms_ic_info[ic].temp_info.temperatures[j],
                                 bms_ic_info[ic].temp_info.temperatures[j] / 10.0f);
                }
            }
        }
    }
    Serial.println("=============================\n");
}

void BMS_Print_Cell_Voltages() {
    Serial.println("\n=== Battery Cell Voltage Report ===\n");
    
    // Statistics tracking
    uint16_t max_voltage_mv = 0;
    uint16_t min_voltage_mv = UINT16_MAX;
    uint32_t total_voltage_mv = 0;
    uint8_t valid_cell_count = 0;
    
    // Output voltage for each cell in each IC
    for (uint8_t ic = 0; ic < NUM_IC; ic++) {
        // Check if this IC is offline
        if (bms_ic_info[ic].CAN_signal_lost_count > BMS_SIGNAL_THRESHOLD) {
            Serial.printf("IC %d: [OFFLINE - No signal]\n\n", ic);
            continue;
        }
        
        Serial.printf("IC %d:\n", ic);
        
        // Display all cells for this IC
        for (uint8_t cell = 0; cell < ACTIVE_CELLS_PER_IC; cell++) {
            // Convert raw code to voltage (mV)
            uint32_t cell_voltage_mv = (uint32_t)(bms_ic_info[ic].volt_info.voltages[cell] * CODE_TO_VOLT * 1000);
            
            // Track statistics (only for non-zero values)
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
            
            // Print cell voltage with formatting (4 cells per line)
            Serial.printf("  C %2d: %5d mV", cell, cell_voltage_mv);
            Serial.print(" | ");
        }
        
        // Ensure newline if last line incomplete
        Serial.println();
    }
    
    // Print voltage statistics
    Serial.println("=== Voltage Statistics ===");
    Serial.printf("Total IC Cells: %d\n", NUM_IC * ACTIVE_CELLS_PER_IC);
    Serial.printf("Valid Cells: %d\n", valid_cell_count);
    
    if (valid_cell_count > 0) {
        // Calculate and display voltage metrics
        float max_voltage_v = max_voltage_mv / 1000.0f;
        float min_voltage_v = min_voltage_mv / 1000.0f;
        float total_voltage_v = total_voltage_mv / 1000.0f;
        float avg_voltage_mv = (float)total_voltage_mv / valid_cell_count;
        float avg_voltage_v = avg_voltage_mv / 1000.0f;
        
        Serial.printf("Max Voltage:   %5d mV = %.3f V\n", max_voltage_mv, max_voltage_v);
        Serial.printf("Min Voltage:   %5d mV = %.3f V\n", min_voltage_mv, min_voltage_v);
        Serial.printf("Total Voltage: %5d mV = %.3f V\n", total_voltage_mv, total_voltage_v);
        Serial.printf("Avg Voltage:   %5.1f mV = %.4f V\n", avg_voltage_mv, avg_voltage_v);
        
        // Calculate and display voltage difference
        uint16_t voltage_diff_mv = max_voltage_mv - min_voltage_mv;
        float voltage_diff_v = voltage_diff_mv / 1000.0f;
        Serial.printf("Voltage Diff:  %5d mV = %.3f V\n", voltage_diff_mv, voltage_diff_v);
    } else {
        Serial.println("ERROR: No valid voltage data available");
    }
    
    Serial.println("============================\n");
}

void BMS_Print_Temperature() {
    Serial.println("\n=== Temperature Report ===\n");
    
    for (uint8_t ic = 0; ic < NUM_IC; ic++) {
        if (bms_ic_info[ic].CAN_signal_lost_count > BMS_SIGNAL_THRESHOLD) {
            Serial.printf("IC %d: [OFFLINE - No signal]\n\n", ic);
            continue;
        }
        
        Serial.printf("IC %d:\n", ic);
        
        for (uint8_t j = 0; j < NTC_PER_IC; j++) {
            Serial.printf("  NTC %d: %d (%.1f°C)\n", j, bms_ic_info[ic].temp_info.temperatures[j], bms_ic_info[ic].temp_info.temperatures[j] / 10.0f);
        }
    }
    
    Serial.println("============================\n");
}