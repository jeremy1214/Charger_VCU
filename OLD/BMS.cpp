#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>

#include "stdlib.h"

//#include "ff.h"
#include "BMS.h"

#define rxmsg (*pBMSRxmsg)

BMS_t BMS_dev;

static bool bms_initialized = false;
CMU_message_t cmu_messages[NUM_CMU_MODULE];
uint16_t BMS_fault_cnt = 0;
uint16_t BMS_fault_cnt_maximum = 0;


const uint32_t BMS_CV_ADDRESS[NUM_BMS_CV_MSG_ID] = {
  0x12905300 + BMS_BOARD_ID,
  0x12905380 + BMS_BOARD_ID,
  0x12905400 + BMS_BOARD_ID,
  0x12905480 + BMS_BOARD_ID,
  0x12905500 + BMS_BOARD_ID,
  0x12905580 + BMS_BOARD_ID
};

const uint32_t BMS_NTC_ADDRESS[NUM_BMS_NTC_MSG_ID] = {
  0x12905600 + BMS_BOARD_ID,
  0x12905680 + BMS_BOARD_ID,
  0x12905700 + BMS_BOARD_ID,
  0x12905780 + BMS_BOARD_ID,
};

/*
Specify each CMU's NTC pin. 
If a CMU's NTC number is less than NUM_NTC_PER_CMU
just duplicate the pin idx
e.g. {4, 5, 6, 6, 6}
*/
/*const uint32_t CMU_NTC_idx[NUM_CMU_MODULE][NUM_NTC_PER_CMU] = {
    {3, 4, 5, 5, 5},
    {3, 4, 5, 5, 5},
    {3, 4, 5, 5, 5},
    {3, 4, 5, 5, 5},
    {0, 1, 2, 3, 4},
    {3, 4, 5, 5, 5},
    {0, 1, 2, 3, 4},
    {0, 1, 2, 3, 4},
    {3, 4, 5, 5, 5},
    {3, 4, 5, 5, 5}
};*/

const uint32_t CMU_NTC_idx[NUM_CMU_MODULE][NUM_NTC_PER_CMU] = {
    {1, 2, 3, 4, 5},
    {1, 2, 3, 4, 5},
    {1, 2, 3, 4, 5},
    {1, 2, 3, 4, 5},
    {1, 2, 3, 4, 5},
    {1, 2, 3, 4, 5},
    {1, 2, 3, 4, 5},
    {1, 2, 3, 4, 5},
    {1, 2, 3, 4, 5},
    {1, 2, 3, 4, 5}
};

/*
The format of the BMS_CV message is as follows with LSB
| byte 0    | byte 1  | byte 2 | byte 3 | byte 4 | byte 5 | byte 6 | byte 7 |
| module id | balance | cell a | cell a | cell b | cell b | cell c | cell c |
*/
void BMS_CAN_handler(twai_message_t *message){
    uint32_t EID = message->identifier;
    uint8_t *data8 = message->data;
    // Check if is cell voltage message
    for(uint32_t i = 0; i < NUM_BMS_CV_MSG_ID; i++) {
        if (EID == BMS_CV_ADDRESS[i]) {
            BMS_dev.CAN_signal_lost_cnt = 0;
            uint8_t module_id = data8[0];
            // Check if the received module_id is valid
            if (module_id >= NUM_CMU_MODULE) break;

            CMU_message_t *cmu_message = &cmu_messages[module_id];
            // Calculate the cell idx 
            uint8_t cell_a_idx = i * 3;
            uint8_t cell_b_idx = i * 3 + 1;
            uint8_t cell_c_idx = i * 3 + 2;
            uint16_t cell_a_mV = (data8[3] << 8) | data8[2];
            uint16_t cell_b_mV = (data8[5] << 8) | data8[4];
            uint16_t cell_c_mV = (data8[7] << 8) | data8[6];

            // Check if cell idx is smaller than maximum number of cell per cmu
            if(cell_a_idx < MAX_NUM_CELL_PER_CMU) cmu_message->cell_voltage_mV[cell_a_idx] = cell_a_mV;
            if(cell_b_idx < MAX_NUM_CELL_PER_CMU) cmu_message->cell_voltage_mV[cell_b_idx] = cell_b_mV;
            if(cell_c_idx < MAX_NUM_CELL_PER_CMU) cmu_message->cell_voltage_mV[cell_c_idx] = cell_c_mV;

            break;
        }
    }
    // Check if is NTC message
    for(uint32_t i = 0; i < NUM_BMS_NTC_MSG_ID; i++) {
        if (EID == BMS_NTC_ADDRESS[i]) {
            uint8_t module_id = data8[0];
            // Check if the received module_id is valid
            if (module_id >= NUM_CMU_MODULE) break;

            CMU_message_t *cmu_message = &cmu_messages[module_id];
            // Calculate the cell idx 
            uint8_t NTC_a_idx = i * 3;
            uint8_t NTC_b_idx = i * 3 + 1;
            uint8_t NTC_c_idx = i * 3 + 2;
            // Convert 0.1 kelvin to celsius
            float NTC_a_deg_c = 0.1 * (float)((data8[3] << 8) | data8[2]) - 273.15;
            float NTC_b_deg_c = 0.1 * (float)((data8[5] << 8) | data8[4]) - 273.15;
            float NTC_c_deg_c = 0.1 * (float)((data8[7] << 8) | data8[6]) - 273.15;

            // Check if cell idx is smaller than maximum number of NTC per cmu
            if(NTC_a_idx < MAX_NUM_NTC_PER_CMU) cmu_message->NTC_deg_c[NTC_a_idx] = NTC_a_deg_c;
            if(NTC_b_idx < MAX_NUM_NTC_PER_CMU) cmu_message->NTC_deg_c[NTC_b_idx] = NTC_b_deg_c;
            if(NTC_c_idx < MAX_NUM_NTC_PER_CMU) cmu_message->NTC_deg_c[NTC_c_idx] = NTC_c_deg_c;

            break;
        }
    }
}

void BMS_init(){
    if (!bms_initialized) {
        // Initialize BMS
        BMS_dev.state = BMS_NORMAL;
        BMS_dev.charging_state = CHG_INITIAL;
        BMS_dev.over_voltage = false;
        BMS_dev.under_voltage = false;
        BMS_dev.over_temperature = false;

        BMS_fault_cnt = 0;

        // Initialize CMU
        for (int i = 0; i < NUM_CMU_MODULE; i++) {
            for (int j = 0; j < MAX_NUM_CELL_PER_CMU; j++) {
                cmu_messages[i].cell_voltage_mV[j] = BMS_INIT_CV_mV;  // Initialize cell_voltage_mV to 0
            }
            for (int j = 0; j < MAX_NUM_NTC_PER_CMU; j++) {
                cmu_messages[i].NTC_deg_c[j] = BMS_INIT_NTC_deg_c;  // Initialize NTC to 0.0
            }
            cmu_messages[i].cell_voltage_fault = 0;
            cmu_messages[i].NTC_fault = 0;
        }
        bms_initialized = true;
    } 
}

bool BMS_check_fault() {
    bool fault = false;
    BMS_dev.over_voltage = false;
    BMS_dev.under_voltage = false;
    BMS_dev.over_temperature = false;
    for (int i = 0; i < NUM_CMU_MODULE; i++) {
        CMU_message_t *cmu = &cmu_messages[i];
        cmu->cell_voltage_fault = 0;
        cmu->NTC_fault = 0;
        for (int j = 0; j < NUM_CELL_PER_CMU; j++) {
            if (cmu->cell_voltage_mV[j] <= BMS_UV_FAULT_mV) {
                if (j == 11 && cmu->cell_voltage_mV[j] == 0) continue;
                fault = true;
                cmu->cell_voltage_fault |= (1 << j);
                BMS_dev.under_voltage = true;
            }
            if (cmu->cell_voltage_mV[j] >= BMS_OV_FAULT_mV) {
                fault = true;
                cmu->cell_voltage_fault |= (1 << j);
                BMS_dev.over_voltage = true;
            }
        }
        for (int j = 0; j < NUM_NTC_PER_CMU; j++) {
            uint8_t NTC_idx = CMU_NTC_idx[i][j];
            if (NTC_idx > MAX_NUM_NTC_PER_CMU) continue;

            if(cmu->NTC_deg_c[NTC_idx] >= BMS_OT_FAULT_deg_c) {
                fault = true;
                cmu->NTC_fault |= (1 << NTC_idx);
                BMS_dev.over_temperature = true;
            }
        }
    }

    return fault;
}

void BMS_update_data() {
    // Aggregrate data from CMU
    uint32_t total_voltage = 0;
    uint32_t highest_voltage = 0;
    uint32_t lowest_voltage = UINT32_MAX;
    float total_temperature = 0;
    float highest_temperature = 0;
    float lowest_temperature = 1000;
    for (int i = 0; i < NUM_CMU_MODULE; i++) {
        CMU_message_t *cmu = &cmu_messages[i];
        for (int j = 0; j < NUM_CELL_PER_CMU; j++) {
            uint16_t cell_voltage = cmu->cell_voltage_mV[j];
            total_voltage += cell_voltage;
            if (cell_voltage < lowest_voltage) lowest_voltage = cell_voltage;
            if (cell_voltage > highest_voltage) highest_voltage = cell_voltage;
        }
        for (int j = 0; j < NUM_NTC_PER_CMU; j++) {
            uint8_t NTC_idx = CMU_NTC_idx[i][j];
            if (NTC_idx > MAX_NUM_NTC_PER_CMU) continue;
            float NTC = cmu->NTC_deg_c[NTC_idx];
            total_temperature += NTC;
            if (NTC < lowest_temperature) lowest_temperature = NTC;
            if (NTC > highest_temperature) highest_temperature = NTC;
        }
    }

    float avg_temperature = total_temperature / (NUM_CMU_MODULE * NUM_NTC_PER_CMU);

    BMS_dev.total_voltage_mV = total_voltage;
    BMS_dev.cell_voltage_mV_highest = highest_voltage;
    BMS_dev.cell_voltage_mV_lowest = lowest_voltage;
    BMS_dev.temp_deg_c_avg = avg_temperature;
    BMS_dev.temp_deg_c_highest = highest_temperature;
    BMS_dev.temp_deg_c_lowest = lowest_temperature;
}

void BMS_state_update(){
    if (BMS_dev.CAN_signal_lost_cnt < INT_MAX)
        BMS_dev.CAN_signal_lost_cnt++;

    if (BMS_dev.state == BMS_NORMAL) {
        if(BMS_check_fault() && BMS_fault_cnt < INT16_MAX) BMS_fault_cnt++;
        else if (BMS_fault_cnt > 0) BMS_fault_cnt--;

        if (BMS_fault_cnt > BMS_fault_cnt_maximum)
            BMS_fault_cnt_maximum = BMS_fault_cnt;
        
        bool fault = BMS_fault_cnt > BMS_FAULT_CNT_THRESHOLD;
        bool signal_lost = BMS_dev.CAN_signal_lost_cnt > BMS_CAN_SIGNAL_LOST_CNT_THRESHOLD;
        BMS_dev.signal_lost = signal_lost;
        /*
        Send signal to HFL
        HIGH: normal condition
        LOW: fault occurs
        */
        //if (fault) dio_write(IDO_BMS_HFL_SIGNAL_CH, 0);
        //else dio_write(IDO_BMS_HFL_SIGNAL_CH, 1);

        // State transition
        if (fault) BMS_dev.state = BMS_FAULT;
        if (signal_lost) BMS_dev.state = BMS_SIGNAL_LOST;
    }
    if (BMS_dev.state == BMS_FAULT) {
        // DO NOTHING FOR NOW
        //dio_write(IDO_BMS_HFL_SIGNAL_CH, 0);
    }
    if (BMS_dev.state == BMS_SIGNAL_LOST) {
        // DO NOTHING FOR NOW
        //dio_write(IDO_BMS_HFL_SIGNAL_CH, 0);
    }

}

void print_cmu_messages() {
    for (int i = 0; i < NUM_CMU_MODULE; i++) {
        printf("CMU Module %d:\n", i);
        
        // Print cell voltages
        printf("  Cell Voltages (mV): ");
        for (int j = 0; j < MAX_NUM_CELL_PER_CMU; j++) {
            printf("%d ", cmu_messages[i].cell_voltage_mV[j]);
        }
        printf("\n");
        
        // Print temperatures
        printf("  Temperatures (°C): ");
        for (int j = 0; j < MAX_NUM_CELL_PER_CMU; j++) {
            printf("%.1f ", cmu_messages[i].NTC_deg_c[j]);
        }
        printf("\n");
        
        // Print faults
        printf("  Cell Voltage Faults: 0x%08X\n", cmu_messages[i].cell_voltage_fault);
        printf("  NTC Faults: 0x%04X\n", cmu_messages[i].NTC_fault);
        printf("\n");
    }
}

