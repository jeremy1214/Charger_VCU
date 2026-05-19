/*
Update: 
FaultPin -> SDCPin, pullup -> pulldown, active low -> active high: ctrl+F "kSDCPin" to check the logic
line 424: Handle OBC CAN messages id 0x218, 0x216.
Check difference between the two OBC states is that the first one is for the charger handle state, and the second one is for the charger state.
line 679, line 728: Add the additional condition of status 
line 775: To tell whether the charging is successful or not.
*/


#include <driver/twai.h>      // ESP32 TWAI (CAN) driver
#include <freertos/FreeRTOS.h> // Use FreeRTOS includes for task functions
#include <freertos/task.h>
#include <Arduino.h>          // Include Arduino core for functions like millis(), pinMode, etc.
#include "BMS.h"              // Include the BMS header file

// Extern declarations for BMS data structures
extern CMU_message_t cmu_messages[];

// ----------------- Pin Configuration -----------------
constexpr gpio_num_t kCanTxPin       = GPIO_NUM_2;    // CAN Transmit (TWAI_TX)
constexpr gpio_num_t kCanRxPin       = GPIO_NUM_15;   // CAN Receive (TWAI_RX)
constexpr gpio_num_t kSDCPin         = GPIO_NUM_17;   // SDC input (active-high)
constexpr gpio_num_t kFaultLedPin    = GPIO_NUM_27;    // Fault LED (D5)
constexpr gpio_num_t kStartButtonPin = GPIO_NUM_18;   // Start charging button
constexpr gpio_num_t kChargingLedPin = GPIO_NUM_5;   // Charging indicator LED (D27)
constexpr gpio_num_t kBatteryHealthPin = GPIO_NUM_14; // Battery health indicator output
constexpr gpio_num_t kBMSFaultPin    = GPIO_NUM_21;   // signal to BMS


// ----------------- Constants -----------------
// constexpr uint32_t kCanBaudRate = 500000; // Defined by TWAI_TIMING_CONFIG_500KBITS()
constexpr uint32_t kHealthCheckIntervalMs = 30000; // System health check interval (ms)
constexpr uint32_t kBmsCheckIntervalMs    = 100;   // BMS check interval (ms)
constexpr uint32_t kTestStateTimeoutMs = 5000;
constexpr float kInputVoltageThresholdV = 380.0;   // Minimum input voltage threshold (V)
constexpr float kOvervoltageThresholdV = 450.0;    // Maximum input voltage threshold (V)
constexpr uint32_t kOvercurrentTimeoutMs = 500;    // Time to wait before declaring overcurrent fault (ms)
constexpr TickType_t kCanTimeoutTicks     = pdMS_TO_TICKS(10); // CAN TX/RX timeout in FreeRTOS ticks
constexpr TickType_t kTaskDelayTicks      = pdMS_TO_TICKS(1);  // Basic task yield delay

// ----------------- LED Constants -----------------
constexpr uint32_t kTestLedIntervalMs  = 250;  // TEST state flash interval (ms)
constexpr uint32_t kReadyLedIntervalMs = 1000; // READY state flash interval (ms)
constexpr uint32_t kStateChangeMinIntervalMs = 500; // Minimum time between state changes (reduced)

// LED state variables
static uint32_t lastLedUpdateMs = 0;

// OBC varialbles
static int OBC_STATE_READY = 2;
static int OBC_STATE_FAULT = 3;
static int successfully_charged_mV = 3650;
static int unsuccessfully_charged_mV = 2000;


// -------- BMS Global Variable (Defined in BMS.cpp) --------
extern BMS_t BMS_dev; 
bool isBMSNormal;

// ----------------- Message Definitions -----------------
typedef struct {
    uint32_t id;            // CAN message ID
    uint8_t length;         // Data length
    uint8_t data[8];        // CAN data bytes (template/default)
    uint32_t interval_ms;   // Send interval in milliseconds
    uint32_t lastSentMs;    // Last time this message was sent (using ms suffix for clarity)
} CanMessageConfig_t;

// Define HV Battery Parameters (Consider making these configurable or const)
#define HV_BATT_U_LIM_VOLTS 438   // Default value in volts
#define HV_BATT_U_DC_VOLTS  300   // Default value in volts
#define HV_BATT_CHRG_I_LIM_AMPS 1 // Default value in amps

// --- Compile-time calculation of CAN data bytes ---
// Helper function (using constexpr for compile-time evaluation if possible, else inline)
// Note: std::min might not be constexpr in all toolchains, relying on macro/inline
template <typename T>
inline constexpr T constrain_val(T val, T max) {
    return (val > max) ? max : val;
}

// Convert voltage/current to the bit value needed for CAN message
constexpr uint16_t hvBattULimTo11bit(uint16_t voltage) {
    // Scale: 0.25V/bit, Offset: 0V. Range: 0 to 511.75V (0x7FF)
    return constrain_val(static_cast<uint16_t>(voltage * 4), static_cast<uint16_t>(0x7FF));
}

constexpr uint16_t hvBattUDcTo11bit(uint16_t voltage) {
    // Scale: 0.25V/bit, Offset: 0V. Range: 0 to 511.75V (0x7FF)
    return constrain_val(static_cast<uint16_t>(voltage * 4), static_cast<uint16_t>(0x7FF));
}

constexpr uint16_t hvBattChrgnILimTo13bit(uint16_t amps) {
    // Scale: 0.1A/bit, Offset: 0A. Range: 0 to 819.1A (0x1FFF)
    return constrain_val(static_cast<uint16_t>(amps * 10), static_cast<uint16_t>(0x1FFF));
}

// Pre-calculate parts of the message data where possible
constexpr uint16_t kHvBattULimBits = hvBattULimTo11bit(HV_BATT_U_LIM_VOLTS);
constexpr uint16_t kHvBattUDcBits = hvBattUDcTo11bit(HV_BATT_U_DC_VOLTS);
constexpr uint16_t kHvBattChrgnILimBits = hvBattChrgnILimTo13bit(HV_BATT_CHRG_I_LIM_AMPS);

// --- CAN Message Configuration ---
// Define fixed data arrays for enable/disable messages
const uint8_t kEnableOutputData[8] = {0x40, 0x00, 0x08, 0xFF, 0xA0, 0x00, 0xC8, 0x00};
const uint8_t kDisableOutputData[8] = {0x40, 0x00, 0x08, 0xFF, 0xA0, 0x00, 0xC0, 0x00};

CanMessageConfig_t messageConfigs[] = {
    // ID, Length, Data (Template), Interval (ms), LastSentMs
    {0x51A, 8, {0x1A, 0x40, 0x43, 0xEF, 0x00, 0x00, 0x00, 0x00}, 100, 0}, // Wake up
    {0x141, 8, {0x20, 0x00, static_cast<uint8_t>((0b01101 << 3) | (kHvBattUDcBits >> 8)), static_cast<uint8_t>(kHvBattUDcBits & 0xFF), 0x48, 0x00, 0x00, 0x01}, 20, 0}, // Parameters: HvBattUDc
    {0x178, 8, {0x60, 0x00, 0x00, 0x00, 0x28, 0xC3, static_cast<uint8_t>((0b00011 << 3) | (kHvBattULimBits >> 8)), static_cast<uint8_t>(kHvBattULimBits & 0xFF)}, 70, 0}, // Parameters: HvBattULim
    {0x084, 8, {0x40, 0x00, 0x08, 0xFF, 0xA0, 0x00, 0xC0, 0x00}, 25, 0}, // Output Control (default disable)
    {0x056, 8, {0x00, 0x02, 0x00, 0x00, 0x01, 0x61, 0x00, 0x00}, 15, 0}, // Vehicle Sim
    {0x289, 8, {0x00, 0x01, 0x04, 0x00, static_cast<uint8_t>((0b000 << 5) | (kHvBattChrgnILimBits >> 8)), static_cast<uint8_t>(kHvBattChrgnILimBits & 0xFF), 0xE0, 0x00}, 100, 0}, // Parameters: HvBattChrgnILim
    {0x345, 8, {0x00, 0x00, 0x01, 0x01, 0xFE, 0x32, 0x32, 0x10}, 1000, 0}  // Parameters: HvBattPreHeatgReq (No Preheat)
};

const size_t kNumMessages = sizeof(messageConfigs) / sizeof(CanMessageConfig_t);

// Global state variable (optional, consider passing as arg if preferred)
// static bool chargingEnabled = false; // Redundant? BMS_dev.charging_state holds this info

// ----------------- Function Prototypes -----------------
bool initCAN();
void canSenderTask(void *pvParameters);
void canReceiverTask(void *pvParameters);
void systemHealthCheckTask(void *pvParameters); // Changed loop content to a task
void bmsMonitorTask(void *pvParameters);      // Changed loop content to a task
void checkBMSStatus();
void processBmsMessage(const twai_message_t *message);
bool isSDCClosed();
bool isBatteryHealthy();
void updateChargingState();
void updateChargingLed();
void printSystemStatus(); // Combined print functions

// ----------------- Main Functions: setup() and loop() -----------------
void setup() {
    Serial.begin(115200);
    // Wait for Serial port to connect needed for native USB
    // while (!Serial) {
    //     delay(10); // Use standard delay before FreeRTOS scheduler starts
    // }
    Serial.println("\n--- ESP32 CAN BMS Charger Controller ---");

    // --- Initialize GPIO ---
    pinMode(kSDCPin, INPUT_PULLUP);
    pinMode(kFaultLedPin, OUTPUT);
    digitalWrite(kFaultLedPin, LOW); // Start with LED off

    pinMode(kStartButtonPin, INPUT_PULLUP);

    pinMode(kChargingLedPin, OUTPUT);
    digitalWrite(kChargingLedPin, LOW);
    
    pinMode(kBatteryHealthPin, OUTPUT);
    digitalWrite(kBatteryHealthPin, LOW); // Start with battery health indicator off

    pinMode(kBMSFaultPin, OUTPUT);
    digitalWrite(kBMSFaultPin, HIGH); // Start with BMS normal

    // --- Initialize CAN ---
    if (!initCAN()) {
        Serial.println("FATAL: CAN initialization failed. Halting.");
        // Indicate fatal error (e.g., rapid LED blink)
        while (true) {
             digitalWrite(kFaultLedPin, !digitalRead(kFaultLedPin));
             delay(100);
        }
    }

    // --- Initialize BMS Data (from BMS.cpp/BMS.h) ---
    BMS_init(); // Initializes BMS_dev structure and state
    Serial.println("BMS Data Initialized.");

    // --- Create FreeRTOS Tasks ---
    // Core affinity can be adjusted based on performance testing (tskNO_AFFINITY allows scheduler to choose)
    // Priority 2 is higher than idle (0) and default loop (1)
    xTaskCreatePinnedToCore(
        canSenderTask,
        "CAN_TX",        // Task name
        4096,            // Stack size (words) - Monitor needed stack size
        NULL,            // Parameters
        3,               // Priority (higher than monitor tasks)
        NULL,            // Task handle
        0                // Core ID (Core 0)
    );

    xTaskCreatePinnedToCore(
        canReceiverTask,
        "CAN_RX",        // Task name
        4096,            // Stack size
        NULL,            // Parameters
        3,               // Priority (higher than monitor tasks)
        NULL,            // Task handle
        0                // Core ID (Core 0)
    );

     xTaskCreatePinnedToCore(
        bmsMonitorTask,
        "BMS_Mon",       // Task name
        3072,            // Stack size (adjust based on printf usage)
        NULL,            // Parameters
        2,               // Priority
        NULL,            // Task handle
        0                // Core ID (Core 0 - separate from CAN)
    );

    xTaskCreatePinnedToCore(
        systemHealthCheckTask,
        "Sys_Health",    // Task name
        2048,            // Stack size
        NULL,            // Parameters
        1,               // Priority (lower)
        NULL,            // Task handle
        0                // Core ID (Core 0 - separate from CAN)
    );


    Serial.println("System Setup Complete. Tasks Running.");
    Serial.printf("Configured to send %d different CAN messages.\n", kNumMessages);

    // Print message configuration once at startup
    for (size_t i = 0; i < kNumMessages; i++) {
        Serial.printf("  Msg %d: ID 0x%03X, Interval %4lums, Data: ",
                      i + 1, messageConfigs[i].id, messageConfigs[i].interval_ms);
        for (int j = 0; j < messageConfigs[i].length; j++) {
            Serial.printf("%02X ", messageConfigs[i].data[j]);
        }
        Serial.println();
    }
    Serial.println("----------------------------------------");

    // Delete the setup and loop task if running Arduino as a component
    // vTaskDelete(NULL); // No loop() task needed anymore
}

// loop() is not used when tasks handle all functionality
void loop() {
    // This task will be starved by higher priority tasks or can be deleted in setup()
    vTaskDelay(portMAX_DELAY); // Effectively sleep forever
}

// ----------------- CAN Initialization -----------------
bool initCAN() {
    // Standard TWAI configurations for 500 kbit/s
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(kCanTxPin, kCanRxPin, TWAI_MODE_NORMAL);
    // Adjust queue lengths if needed
    g_config.tx_queue_len = 10; // Increased queue size slightly
    g_config.rx_queue_len = 10;
    // Disable alerts unless specific error handling is needed
    g_config.alerts_enabled = TWAI_ALERT_NONE;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // Accept all messages

    // Install TWAI driver
    esp_err_t result = twai_driver_install(&g_config, &t_config, &f_config);
    if (result != ESP_OK) {
        Serial.printf("Failed to install TWAI driver: %s\n", esp_err_to_name(result));
        return false;
    }
    Serial.println("TWAI driver installed.");

    // Start TWAI driver
    result = twai_start();
    if (result != ESP_OK) {
        Serial.printf("Failed to start TWAI driver: %s\n", esp_err_to_name(result));
        twai_driver_uninstall(); // Clean up if start fails
        return false;
    }
    Serial.println("TWAI driver started successfully at 500 kbit/s.");
    return true;
}

// ----------------- Task Functions -----------------

void canSenderTask(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t taskFrequency = pdMS_TO_TICKS(5); // Check messages every 5ms

    twai_message_t message;
    message.extd = 0; // Standard ID format
    message.rtr = 0;  // Not a remote frame

    while (true) {
        uint32_t currentTimeMs = millis(); // Use millis for message interval timing

        for (size_t i = 0; i < kNumMessages; i++) {
            // Check if interval has passed
            if (currentTimeMs - messageConfigs[i].lastSentMs >= messageConfigs[i].interval_ms) {
                message.identifier = messageConfigs[i].id;
                message.data_length_code = messageConfigs[i].length;

                bool sendMessage = false;
                const uint8_t* dataToSend = messageConfigs[i].data; // Default data

                // --- Message Specific Logic ---
                switch (messageConfigs[i].id) {
                    case 0x51A: // Wake-up Message
                        // Send unless in FAIL state
                        /*if (BMS_dev.charging_state != CHG_FAIL) { //
                            sendMessage = true;
                        }*/
                        sendMessage = true;
                        break;

                    case 0x084: // Output Control Message
                        // Data depends on charging state
                        if (BMS_dev.charging_state == CHG_START) { //
                            dataToSend = kEnableOutputData;
                        } else {
                            dataToSend = kDisableOutputData;
                        }
                        // Always send this control message periodically when not failed
                        /*if (BMS_dev.charging_state != CHG_FAIL) {
                            sendMessage = true;
                        }*/
                        sendMessage = true;
                        break;

                    default: // Other periodic messages
                        // Send only when actively charging
                        if (BMS_dev.charging_state == CHG_START) { //
                            sendMessage = true;
                        }
                        break;
                }

                // --- Transmit Logic ---
                if (sendMessage) {
                    memcpy(message.data, dataToSend, messageConfigs[i].length);
                    esp_err_t result = twai_transmit(&message, kCanTimeoutTicks);
                    if (result == ESP_OK) {
                        messageConfigs[i].lastSentMs = currentTimeMs; // Update last sent time on success
                    } else {
                        // Log error infrequently to avoid spamming
                        static uint32_t lastTxErrorLog = 0;
                        if (currentTimeMs - lastTxErrorLog > 1000) {
                             Serial.printf("WARN: Failed to queue CAN message 0x%lX: %s\n", message.identifier, esp_err_to_name(result));
                             lastTxErrorLog = currentTimeMs;
                        }
                        // Optional: Implement retry or error handling logic here
                    }
                } else {
                     // If message shouldn't be sent now, ensure its timer resets
                     // only when conditions are met again. We can update lastSentMs
                     // here to prevent immediate re-attempt if state flips quickly,
                     // or leave it to wait for the full interval.
                     // Let's reset it to prevent spamming attempts if state toggles.
                     // messageConfigs[i].lastSentMs = currentTimeMs;
                     // -- Alternative: Don't reset, wait for state + interval --
                }
            } // end interval check
        } // end for loop

        // Delay until next check cycle
        vTaskDelayUntil(&lastWakeTime, taskFrequency);
    }
}


void canReceiverTask(void *pvParameters) {
    twai_message_t message;
    while (true) {
        // Wait indefinitely for a message (or use a timeout)
        esp_err_t result = twai_receive(&message, portMAX_DELAY);

        if (result == ESP_OK) {
            // Process BMS messages as before
            processBmsMessage(&message);
            
            // Process OBC messages to monitor values
            processOBCMessage(&message);
        }
    }
}

// Add these global variables to store OBC monitoring values
struct OBC_Monitor {
    float onBdChrgrT;
    int   onBdChrgrHndlSt;
    int   onBdChrgrSt;
    float onBdChrgrIAct;      
    float onBdChrgrUDc;       // Input DC voltage (V)
    float onBdChrgrIDc;       // Input DC current (A)
    float onBdChrgrUAct;      // Output voltage (V)
    uint32_t lastUpdateMs;    // Last time OBC data was updated
};

// Global instance
OBC_Monitor obc = {
    .onBdChrgrT = 0.0,
    .onBdChrgrHndlSt = 0,
    .onBdChrgrSt = 0,
    .onBdChrgrIAct = 0.0,
    .onBdChrgrUDc = 0.0,
    .onBdChrgrIDc = 0.0,
    .onBdChrgrUAct = 0.0,
    .lastUpdateMs = 0
};

// Function to handle OBC messages and extract the needed values
void processOBCMessage(const twai_message_t *message) {
    // Check if this is the OBC status message (0x12A from your screenshot)
    if (message->identifier == 0x12A) {
        uint32_t nowMs = millis();
        
        // Extract OnBdChrgrUDc - Correct
        uint16_t raw_udc = 0;
        raw_udc = ((uint16_t)(message->data[5])) | (((uint16_t)(message->data[4] & 0x1F)) << 8);
        obc.onBdChrgrUDc = raw_udc * 0.2;
        
        uint16_t raw_idc = 0;
        raw_idc = ((uint16_t)(message->data[1] & 0b11111000) >> 3) | ((uint16_t)(message->data[0] & 0b01111111) << 5);
        obc.onBdChrgrIDc = (raw_idc * 0.1) - 200.0; // Apply scaling and offset
        
        // Extract OnBdChrgrUAct - Correct
        uint16_t raw_uact = 0;
        raw_uact = ((uint16_t)message->data[7]) | (((uint16_t)(message->data[6] & 0x07)) << 8);
        obc.onBdChrgrUAct = raw_uact * 0.25;
        
        obc.lastUpdateMs = nowMs;
        
        // Log changes
        static uint32_t lastLoggedMs = 0;
        if (nowMs - lastLoggedMs > 2000) { // Log every 2 seconds
            //Serial.printf("OBC Monitor - Input: %.1fV/%.1fA, Output: %.1fV\n", obc.onBdChrgrUDc, obc.onBdChrgrIDc, obc.onBdChrgrUAct);
            lastLoggedMs = nowMs;
        }
    }
    else if(message->identifier == 0x218) {
        uint8_t raw_HndlSt = message->data[2] & 0x0F; // Extract state from first byte
        obc.onBdChrgrHndlSt = raw_HndlSt; // Store the state

        uint8_t raw_temp = message->data[3] & 0xFF;
        obc.onBdChrgrT = raw_temp;
    }
    else if(message->identifier == 0x216) {
        uint16_t raw_iact = message->data[0] & 0x0007;
        raw_iact = (raw_iact << 8) | message->data[1];
        raw_iact = (raw_iact << 1) | ((message->data[2] >> 7) & 0x01);
        obc.onBdChrgrIAct = raw_iact * 0.1; // Store the raw current value

        uint8_t raw_OBCSt = (message->data[2] >> 3) & 0x0F; // Extract state from second byte
        obc.onBdChrgrSt = raw_OBCSt; // Store the state
    }
}

void bmsMonitorTask(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t taskFrequency = pdMS_TO_TICKS(kBmsCheckIntervalMs);

    while(true) {
         isBMSNormal = true;
        // Check BMS library reported faults
        if (BMS_dev.signal_lost ||
            BMS_dev.over_voltage ||
            BMS_dev.under_voltage ||
            BMS_dev.over_temperature ||
            BMS_dev.state != BMS_NORMAL) { //
            isBMSNormal = false;
        }
    
        // Update the battery health indicator pin
        digitalWrite(kBMSFaultPin, isBMSNormal ? HIGH : LOW);
        //Serial.printf("isBMSNormal: %d", isBMSNormal);
        // Wait for the next cycle
        vTaskDelayUntil(&lastWakeTime, taskFrequency);

        // Perform BMS status check and state updates
        checkBMSStatus();

        // Update LED based on current state
        updateChargingLed();

        // Print status periodically (can be throttled further if needed)
        static uint32_t lastPrintTimeMs = 0;
        uint32_t nowMs = millis();
        if (nowMs - lastPrintTimeMs > 1000) // Print every second
        {
            printSystemStatus();
            lastPrintTimeMs = nowMs;
        }
    }
}


void systemHealthCheckTask(void *pvParameters) {
     TickType_t lastWakeTime = xTaskGetTickCount();
     const TickType_t taskFrequency = pdMS_TO_TICKS(kHealthCheckIntervalMs);

    while(true) {
         // Wait for the next cycle
        vTaskDelayUntil(&lastWakeTime, taskFrequency);

        // Check CAN controller status
        twai_status_info_t status_info;
        esp_err_t result = twai_get_status_info(&status_info);

        if (result == ESP_OK) {
            // Optional: Log detailed status less frequently if needed
            // Serial.printf("DEBUG: CAN Status: State=%d, TXerr=%d, RXerr=%d, TXq=%d, RXq=%d, Rcvd=%d, Sent=%d\n",
            //       status_info.state, status_info.tx_error_counter, status_info.rx_error_counter,
            //       status_info.msgs_to_tx, status_info.msgs_to_rx,
            //       status_info.rx_msg_count, status_info.tx_msg_count);

            if (status_info.state == TWAI_STATE_BUS_OFF) {
                Serial.println("ERROR: CAN bus-off detected! Attempting recovery...");
                result = twai_initiate_recovery(); // Attempt recovery
                Serial.printf("INFO: CAN recovery attempt result: %s\n", esp_err_to_name(result));
                // Consider more robust recovery (e.g., re-init after repeated failures)
            } else if (status_info.state == TWAI_STATE_RECOVERING) {
                Serial.println("INFO: CAN bus is recovering...");
            } else if (status_info.tx_error_counter > 127 || status_info.rx_error_counter > 127) {
                 Serial.printf("WARN: High CAN error count (TX:%d, RX:%d). State: %d\n",
                       status_info.tx_error_counter, status_info.rx_error_counter, status_info.state);
                 // Potentially trigger a warning state or investigation
            }
        } else {
             Serial.printf("ERROR: Failed to get TWAI status: %s\n", esp_err_to_name(result));
        }

        // Add other health checks here (e.g., stack high water mark, temperature sensor)
        // UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL); // Check own stack
        // Serial.printf("DEBUG: SysHealth Task Stack HWM: %u words\n", stackHighWater);
    }
}


// ----------------- BMS Functions -----------------

void processBmsMessage(const twai_message_t *message) {
    // Call the handler from BMS library
    // This function likely updates BMS_dev based on the message content
    BMS_CAN_handler((twai_message_t*)message); // Cast away const if BMS_CAN_handler doesn't use const

    // Update last message timestamp if needed (or rely on BMS_dev.CAN_signal_lost_cnt)
    // lastBmsMessageTimeMs = millis();
}

void checkBMSStatus() {
    // Functions from BMS library
    BMS_update_data();    // Aggregate latest readings into BMS_dev
    BMS_state_update();   // Update BMS_dev.state based on faults/timeouts

    // Update the charger's state machine based on BMS status
    updateChargingState();
}

// Combined status printing function
void printSystemStatus() {
    const char* BMS_state_str[] = {"NORMAL", "FAULT", "SIGNAL_LOST"}; //
    const char* Charging_state_str[] = {"INITIAL", "TEST", "READY", "START", "FAIL"}; //

    // Use local copies to prevent race conditions during printing if tasks modify concurrently
    BMS_state current_bms_state = BMS_dev.state; //
    ChargingState current_charging_state = BMS_dev.charging_state; //
    bool healthy = isBatteryHealthy(); // Check health just before printing

    Serial.println("--- System Status ---");
    Serial.printf(" BMS State:    %s\n",
                  (current_bms_state >= 0 && current_bms_state < sizeof(BMS_state_str)/sizeof(char*)) ? BMS_state_str[current_bms_state] : "INVALID");              
    Serial.printf(" System State:    %s (GPIO14: %s)\n",
                   healthy ? "Healthy" : "UNHEALTHY",
                   digitalRead(kBatteryHealthPin) ? "HIGH" : "LOW");
    Serial.printf(" Charger State: %s\n",
                  (current_charging_state >= 0 && current_charging_state < sizeof(Charging_state_str)/sizeof(char*)) ? Charging_state_str[current_charging_state] : "INVALID");
                  
    Serial.printf(" BMSStatetoLatch: %s\n", 
                  isBMSNormal ? "Normal" : "Abnormal");
    Serial.printf(" OBCChrgrSt: %d\n", obc.onBdChrgrHndlSt);
    Serial.printf(" OBCSt: %d\n", obc.onBdChrgrSt);
    // Print OBC input voltage status with threshold check
    if (obc.onBdChrgrUDc >= kInputVoltageThresholdV) {
        if (obc.onBdChrgrUDc <= kOvervoltageThresholdV) {
            Serial.printf(" OBC Input: %.1fV (OK)\n", obc.onBdChrgrUDc);
        } else {
            Serial.printf(" OBC Input: %.1fV (OVERVOLTAGE!)\n", obc.onBdChrgrUDc);
        }
    } else {
        Serial.printf(" OBC Input: %.1fV (WAITING FOR >= %.1fV)\n", obc.onBdChrgrUDc, (float)kInputVoltageThresholdV);
    }
                 
    // Print OBC current status with threshold check if charging
    Serial.printf(" OBC Current: %.1fA %s\n",
                 obc.onBdChrgrIDc,
                 (obc.onBdChrgrIDc <= HV_BATT_CHRG_I_LIM_AMPS) ? "(OK)" : 
                   "(OVERCURRENT!)");

    // Print details only if BMS state is valid
     if (current_bms_state != BMS_SIGNAL_LOST) { //
         Serial.printf(" Voltage (mV): Total=%lu, Max Cell=%u, Min Cell=%u\n",
                       BMS_dev.total_voltage_mV, //
                       BMS_dev.cell_voltage_mV_highest, //
                       BMS_dev.cell_voltage_mV_lowest); //
         Serial.printf(" Temperature (C): Avg=%.1f, Max=%.1f, Min=%.1f\n",
                       BMS_dev.temp_deg_c_avg, //
                       BMS_dev.temp_deg_c_highest, //
                       BMS_dev.temp_deg_c_lowest); //
         
         // Print all individual cell voltages
         Serial.println(" Individual Cell Voltages (mV):");
         for (int i = 0; i < NUM_CMU_MODULE; i++) {
             Serial.printf("  CMU %d: ", i);
             for (int j = 0; j < NUM_CELL_PER_CMU; j++) {
                 Serial.printf("%d ", cmu_messages[i].cell_voltage_mV[j]);
                 // Print in groups of 6 cells for readability
                 if ((j + 1) % 6 == 0 && j < NUM_CELL_PER_CMU - 1) {
                     Serial.printf("\n         ");
                 }
             }
             Serial.println();
         }
    } else {
         Serial.println(" Voltage/Temp: N/A (Signal Lost)");
    }


    // Print active faults clearly
    Serial.print(" Faults Active: ");
    bool anyFault = false;
    if (digitalRead(kSDCPin) == LOW) { // Active HIGH
        Serial.print("[SDC] ");
        anyFault = true;
    }
    if (BMS_dev.signal_lost) { Serial.print("[BMS Signal Loss] "); anyFault = true; } //
    if (BMS_dev.over_voltage) { Serial.print("[BMS Over Voltage] "); anyFault = true; } //
    if (BMS_dev.under_voltage) { Serial.print("[BMS Under Voltage] "); anyFault = true; } //
    if (BMS_dev.over_temperature) { Serial.print("[BMS Over Temp] "); anyFault = true; } //
    if (!anyFault) {
        Serial.print("None");
    }
    Serial.println("\n---------------------");

    // For detailed debugging of all CMU data (uncomment if needed)
    // print_cmu_messages();
}

bool isSDCClosed() {

    bool SDCclosed = true;
    if (digitalRead(kSDCPin) == LOW) { // Active HIGH fault
        // Log only on change or infrequently to avoid spam
        static bool lastSDCFaultState = false;
        if (!lastSDCFaultState) {
            // Serial.println("DEBUG: IMD Fault Detected (External)");
            lastSDCFaultState = true;
        }
        SDCclosed = false;
    } else {
         static bool lastSDCFaultState = true; // Reset logging state
         if (lastSDCFaultState) lastSDCFaultState = false;
    }

    return SDCclosed;
}



bool isBatteryHealthy() {
    bool isHealthy = true; // Default to healthy
    
    // Check BMS library reported faults
    if (BMS_dev.signal_lost ||
        BMS_dev.over_voltage ||
        BMS_dev.under_voltage ||
        BMS_dev.over_temperature ||
        BMS_dev.state != BMS_NORMAL) { //
        isHealthy = false;
    }
    
    // Update the battery health indicator pin
    digitalWrite(kBatteryHealthPin, isHealthy ? HIGH : LOW);
    
    return 1; // Return the health status
}

// ----------------- State Machine and LED Logic -----------------

void updateChargingState() {
    static uint32_t lastStateChangeMs = 0;
    static bool lastButtonState = HIGH; // Assuming INPUT_PULLUP
    static uint32_t testStateEntryTimeMs = 0; // *** NEW: Timestamp for entering TEST state ***
    uint32_t nowMs = millis();

    // Check button state (simple debounce check)
    bool currentButtonState = digitalRead(kStartButtonPin);
    bool buttonPressed = (currentButtonState == LOW && lastButtonState == HIGH);
    lastButtonState = currentButtonState; // Update for next cycle

    // Check battery health
    bool batteryOk = isBatteryHealthy();
    
    //************************************************************************
    //bool batteryOk = true; // For testing, assume battery is always healthy
    //************************************************************************

    // Prevent rapid state changes, but allow immediate transition *to* FAIL
    ChargingState currentState = BMS_dev.charging_state; // Read current state once
    if (currentState != CHG_FAIL && (nowMs - lastStateChangeMs < kStateChangeMinIntervalMs)) {
       // If not failing and minimum interval hasn't passed, return
       // (This prevents flickering between non-fail states)
       return;
    }

    ChargingState previousState = currentState; // Store state before switch

    switch (currentState) {
        case CHG_INITIAL:
            // Transition to TEST once BMS is initialized and normal
            if (BMS_dev.state == BMS_NORMAL) {
                BMS_dev.charging_state = CHG_TEST;
                testStateEntryTimeMs = nowMs; // *** Record entry time ***
                Serial.println("STATE: -> TEST");
            }
            // Stay initial if BMS not ready yet
            break;

        case CHG_TEST:
             // Transition to READY if battery is healthy AND input voltage is above threshold
            if (batteryOk && obc.onBdChrgrUDc >= kInputVoltageThresholdV && obc.onBdChrgrHndlSt == OBC_STATE_READY) {
                BMS_dev.charging_state = CHG_READY;
                Serial.println("chrgrHndlSt:");
                Serial.println(obc.onBdChrgrHndlSt);
                Serial.println("STATE: -> READY (Test Passed, Input Voltage OK)");
            }
            // Only fail if battery health check times out
            else if (!batteryOk && (nowMs - testStateEntryTimeMs > kTestStateTimeoutMs)) {
                 // Battery unhealthy and timeout exceeded -> FAIL
                 BMS_dev.charging_state = CHG_FAIL;
                 Serial.println("STATE: -> FAIL (Test Timeout: Battery Unhealthy)");
            }
            // else: Either waiting for battery to be healthy or for input voltage to reach 380V -> Remain in TEST
            else {
                // Add periodic status update when in TEST state
                static uint32_t lastTestStateUpdateMs = 0;
                if (nowMs - lastTestStateUpdateMs > 5000) { // Update every 5 seconds
                    if (!batteryOk) {
                        Serial.printf("STATE: TEST - Waiting for battery health (timeout in %.1fs)\n", 
                                     (kTestStateTimeoutMs - (nowMs - testStateEntryTimeMs)) / 1000.0);
                    } 
                    else if (obc.onBdChrgrUDc < kInputVoltageThresholdV) {
                        Serial.printf("STATE: TEST - Waiting for input voltage (%.1fV) to reach %.1fV (no timeout)\n", 
                                     obc.onBdChrgrUDc, (float)kInputVoltageThresholdV);
                    }
                    lastTestStateUpdateMs = nowMs;
                }
            }
            break;

        case CHG_READY:
             // Transition to START if button pressed AND battery healthy
            if (buttonPressed) {
                 if (digitalRead(kSDCPin)) {
                    BMS_dev.charging_state = CHG_START;
                    Serial.println("STATE: -> START (Button Pressed)");
                 } else {
                     // Button pressed but battery not healthy -> Go to FAIL
                     BMS_dev.charging_state = CHG_FAIL;
                     Serial.println("STATE: -> FAIL (Button pressed, but SDC open)");
                 }
            }
            // Transition back to FAIL if battery becomes unhealthy while waiting
            if (!digitalRead(kSDCPin)) {
                BMS_dev.charging_state = CHG_FAIL;
                Serial.println("STATE: -> FAIL (SDC open during READY)");
            }
            break;

        case CHG_START:
            // Check for immediate fault conditions first (overvoltage)
            if (obc.onBdChrgrUDc > kOvervoltageThresholdV || obc.onBdChrgrSt == OBC_STATE_FAULT) {
                BMS_dev.charging_state = CHG_FAIL;
                Serial.printf("STATE: -> FAIL (Overvoltage detected: %.1fV > %.1fV)\n", 
                             obc.onBdChrgrUDc, kOvervoltageThresholdV);
                break;
            }
            
            // Check for timed fault conditions (overcurrent)
            static uint32_t overcurrentStartTimeMs = 0;
            if (obc.onBdChrgrIDc > (HV_BATT_CHRG_I_LIM_AMPS + 0.5)) {
                // Start timing if this is the beginning of an overcurrent condition
                if (overcurrentStartTimeMs == 0) {
                    overcurrentStartTimeMs = nowMs;
                }
                // Check if overcurrent has persisted long enough to trigger fault
                else if (nowMs - overcurrentStartTimeMs > kOvercurrentTimeoutMs) {
                    BMS_dev.charging_state = CHG_FAIL;
                    Serial.printf("STATE: -> FAIL (Overcurrent persisted for >%dms: %.1fA > %dA)\n", 
                                 kOvercurrentTimeoutMs, obc.onBdChrgrIDc, HV_BATT_CHRG_I_LIM_AMPS);
                    overcurrentStartTimeMs = 0; // Reset timer
                    break;
                }
                // Still in overcurrent but timeout not reached yet
                else {
                    // Optional: Add warning log
                    static uint32_t lastOvercurrentWarningMs = 0;
                    if (nowMs - lastOvercurrentWarningMs > 200) { // Limit warning frequency
                        Serial.printf("WARNING: Overcurrent condition: %.1fA > %dA (for %dms)\n", 
                                     obc.onBdChrgrIDc, HV_BATT_CHRG_I_LIM_AMPS, 
                                     nowMs - overcurrentStartTimeMs);
                        lastOvercurrentWarningMs = nowMs;
                    }
                }
            } else {
                // Reset overcurrent timer if current is back to normal
                overcurrentStartTimeMs = 0;
            }
            
            // Transition to FAIL if battery becomes unhealthy during charging
            if (!batteryOk || !digitalRead(kSDCPin)) {
                BMS_dev.charging_state = CHG_FAIL;
                Serial.println("STATE: -> FAIL (Unhealthy during START)");
            }
            // Add conditions for stopping charge (e.g., BMS reports full, button pressed again?)
            // if (buttonPressed) { // Example: Allow stopping with button
            //    BMS_dev.charging_state = CHG_READY;
            //    Serial.println("STATE: -> READY (Charge stopped by button)");
            // }
            if (BMS_dev.cell_voltage_mV_highest > successfully_charged_mV){
                BMS_dev.charging_state = CHG_READY;
                Serial.println("STATE: -> READY (Charge completed successfully)");
            } else if (BMS_dev.cell_voltage_mV_lowest < unsuccessfully_charged_mV) {
                BMS_dev.charging_state = CHG_FAIL;
                Serial.println("STATE: -> FAIL (Charge failed)");
            } else {
                // Continue charging, keep state as START
                // Optional: Add logic to monitor charge progress or time
            }
            break;

        case CHG_FAIL:
            // Stay in FAIL state. Requires manual reset or condition clear.
            // Optional: Add logic to attempt recovery or transition back if health returns
            // if (batteryOk && (nowMs - lastStateChangeMs > 10000)) { // Example: Auto-retry after 10s if healthy
            //    BMS_dev.charging_state = CHG_READY;
            //    Serial.println("STATE: -> READY (Recovered from FAIL)");
            // }

            // Ensure fault LED is ON
            digitalWrite(kFaultLedPin, HIGH);
            break;

         default:
             Serial.printf("WARN: Unknown charging state: %d\n", BMS_dev.charging_state);
             BMS_dev.charging_state = CHG_FAIL; // Default to fail on unknown state
             break;

    } // end switch

    // If state changed, record the time
    if (BMS_dev.charging_state != previousState) {
        lastStateChangeMs = nowMs;
        // Logging moved to printSystemStatus for periodic updates
        // Serial.printf("DEBUG: Charging state changed to %d\n", BMS_dev.charging_state);

        // Reset LED timer on state change for clean flashing start
        lastLedUpdateMs = nowMs;
        // Ensure Fault LED is OFF unless in FAIL state
         if (BMS_dev.charging_state != CHG_FAIL) {
             digitalWrite(kFaultLedPin, LOW);
         }
    }

} // end updateChargingState


void updateChargingLed() {
    uint32_t nowMs = millis();

    switch (BMS_dev.charging_state) {
        case CHG_TEST: // Fast flash
            if (nowMs - lastLedUpdateMs > kTestLedIntervalMs) {
                digitalWrite(kChargingLedPin, !digitalRead(kChargingLedPin));
                lastLedUpdateMs = nowMs;
            }
            break;

        case CHG_READY: // Slow flash
            if (nowMs - lastLedUpdateMs > kReadyLedIntervalMs) {
                digitalWrite(kChargingLedPin, !digitalRead(kChargingLedPin));
                lastLedUpdateMs = nowMs;
            }
            break;

        case CHG_START: // Solid ON
            digitalWrite(kChargingLedPin, HIGH);
            break;

        case CHG_INITIAL: // Off
        case CHG_FAIL:    // Off (Fault LED handles fail indication)
        default:          // Off
            digitalWrite(kChargingLedPin, LOW);
            break;
    }
}