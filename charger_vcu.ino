#include<driver/twai.h>
#include<Arduino.h>
#include <freertos/FreeRTOS.h> // Use FreeRTOS includes for task functions
#include <freertos/task.h>

#include "BMS.h"

// GPIO pin definitions
#define CAN_TX_PIN GPIO_NUM_2
#define CAN_RX_PIN GPIO_NUM_15
#define SDC_PIN GPIO_NUM_17
#define FAULT_LED_PIN GPIO_NUM_27
#define START_BUTTON_PIN GPIO_NUM_18
#define CHARGING_LED_PIN GPIO_NUM_5
#define BATTERY_HEALTH_PIN GPIO_NUM_14
#define BMS_FAULT_PIN GPIO_NUM_21

// Define HV Battery Parameters
#define HV_BATT_U_LIM_VOLTS 438   // Default value in volts
#define HV_BATT_U_DC_VOLTS  300   // Default value in volts
#define HV_BATT_CHRG_I_LIM_AMPS 1 // current limit in amps

// Define TWAI Queue Size
#define TWAI_TX_QUEUE_SIZE 10
#define TWAI_RX_QUEUE_SIZE 10

// ----------------- Constants -----------------
constexpr TickType_t kCanTimeoutTicks     = pdMS_TO_TICKS(10); // CAN TX/RX timeout in FreeRTOS ticks
constexpr TickType_t kTaskDelayTicks      = pdMS_TO_TICKS(1);  // Basic task yield delay
constexpr uint32_t kHealthCheckIntervalMs = 30000; // System health check interval (ms)
constexpr uint32_t kBmsCheckIntervalMs    = 100;   // BMS check interval (ms)
constexpr uint32_t kTestStateTimeoutMs = 5000;
constexpr float kMinVoltageThresholdV = 380.0f;   // Minimum input voltage threshold (V)
constexpr float kOvervoltageThresholdV = 450.0f;    // Maximum input voltage threshold (V)
constexpr uint32_t kOvercurrentTimeoutMs = 500;    // Time to wait before declaring overcurrent fault (ms)

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

// ----------------- Pin Configuration -----------------
constexpr gpio_num_t kCanTxPin       = CAN_TX_PIN;   // CAN Transmit (TWAI_TX)
constexpr gpio_num_t kCanRxPin       = CAN_RX_PIN;   // CAN Receive (TWAI_RX)
constexpr gpio_num_t kSDCPin         = SDC_PIN;   // SDC input (active-high)
constexpr gpio_num_t kFaultLedPin    = FAULT_LED_PIN;    // Fault LED (D5)
constexpr gpio_num_t kStartButtonPin = START_BUTTON_PIN;   // Start charging button
constexpr gpio_num_t kChargingLedPin = CHARGING_LED_PIN;   // Charging indicator LED (D27)
constexpr gpio_num_t kBatteryHealthPin = BATTERY_HEALTH_PIN; // Battery health indicator output
constexpr gpio_num_t kBMSFaultPin    = BMS_FAULT_PIN;   // signal to BMS

// ----------------- Message Definitions -----------------
typedef struct {
    uint32_t id;            // CAN message ID
    uint8_t length;         // Data length
    uint8_t data[8];        // CAN data bytes (template/default)
    uint32_t interval_ms;   // Send interval in milliseconds
    uint32_t lastSentMs;    // Last time this message was sent (using ms suffix for clarity)
} CANMessageConfig_t;

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

// --- CAN Message Send to OBC Configuration ---
// Define fixed data arrays for enable/disable messages
const uint8_t kEnableOutputData[8] = {0x40, 0x00, 0x08, 0xFF, 0xA0, 0x00, 0xC8, 0x00};
const uint8_t kDisableOutputData[8] = {0x40, 0x00, 0x08, 0xFF, 0xA0, 0x00, 0xC0, 0x00};

CANMessageConfig_t messageConfigs[] = {
    // ID, Length, Data (Template), Interval (ms), LastSentMs
    {0x51A, 8, {0x1A, 0x40, 0x43, 0xEF, 0x00, 0x00, 0x00, 0x00}, 100, 0}, // Wake up
    {0x141, 8, {0x20, 0x00, static_cast<uint8_t>((0b01101 << 3) | (kHvBattUDcBits >> 8)), static_cast<uint8_t>(kHvBattUDcBits & 0xFF), 0x48, 0x00, 0x00, 0x01}, 20, 0}, // Parameters: HvBattUDc
    {0x178, 8, {0x60, 0x00, 0x00, 0x00, 0x28, 0xC3, static_cast<uint8_t>((0b00011 << 3) | (kHvBattULimBits >> 8)), static_cast<uint8_t>(kHvBattULimBits & 0xFF)}, 70, 0}, // Parameters: HvBattULim
    {0x084, 8, {0x40, 0x00, 0x08, 0xFF, 0xA0, 0x00, 0xC0, 0x00}, 25, 0}, // Output Control (default disable)
    {0x056, 8, {0x00, 0x02, 0x00, 0x00, 0x01, 0x61, 0x00, 0x00}, 15, 0}, // Vehicle Sim
    {0x289, 8, {0x00, 0x01, 0x04, 0x00, static_cast<uint8_t>((0b000 << 5) | (kHvBattChrgnILimBits >> 8)), static_cast<uint8_t>(kHvBattChrgnILimBits & 0xFF), 0xE0, 0x00}, 100, 0}, // Parameters: HvBattChrgnILim
    {0x345, 8, {0x00, 0x00, 0x01, 0x01, 0xFE, 0x32, 0x32, 0x10}, 1000, 0}  // Parameters: HvBattPreHeatgReq (No Preheat)
};

// Calculate the number of messages in the configuration array
const size_t kNumMessages = sizeof(messageConfigs) / sizeof(CANMessageConfig_t);

extern BMS_t bms_t;
bool bms_normal;

// ----------------- Function Prototypes -----------------
bool Init_CAN();
void CAN_Sender_Task(void *pvParameters);
void CAN_Receiver_Task(void *pvParameters);
void System_Health_Check_Task(void *pvParameters); // Changed loop content to a task
void BMS_Monitor_Task(void *pvParameters);      // Changed loop content to a task
void checkBMSStatus();
void Process_BMS_Message(const twai_message_t *message);
void updateChargingState();
void updateChargingLed();
void printSystemStatus(); // Combined print functions
void Get_Bd_Chrgr(const uint8_t *data);


void setup() {
    // Initialize your setup code here
    Serial.begin(115200);

    Serial.println("\n--- ESP32 CAN BMS Charger Controller ---");

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

    if(!Init_CAN()) {
        Serial.println("Failed to initialize CAN");
        while(true){
            digitalWrite(kFaultLedPin, !digitalRead(kFaultLedPin));
            delay(250);
        }
    }

    BMS_Init();
    Serial.println("BMS initialized.");

    // --- Create FreeRTOS tasks ---
    
}

void loop() {
    // Add your main code here, to run repeatedly
}

bool Init_CAN() {
    // Standard TWAI configurations for 500 kbit/s
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(kCanTxPin, kCanRxPin, TWAI_MODE_NORMAL);
    // Adjust queue lengths if needed
    g_config.tx_queue_len = TWAI_TX_QUEUE_SIZE; // Increased queue size slightly
    g_config.rx_queue_len = TWAI_RX_QUEUE_SIZE;
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
    // Add your CAN initialization code here
}

void CAN_Sender_Task(void *pvParameters) {
    // Add your CAN sender code here
    TickType xLastWakeTime = xTaskGetTickCount(); // Initialize xLastWakeTime
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // Adjust the frequency as needed

    twai_message_t txMessage;
    txMessage.extd = 0;
    txMessage.rtr = 0;

    while(true){
        uint32_t currentTime = millis();

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
                        // Always send this message periodically
                        sendMessage = true;
                        break;

                    case 0x084: // Output Control Message
                        // Data depends on charging state
                        if (BMS_dev.charging_state == CHG_START) { //
                            dataToSend = kEnableOutputData;
                        } else {
                            dataToSend = kDisableOutputData;
                        }
                        // Always send this control message periodically
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
                }
            } // end interval check
        } // end for loop

        // Delay until next check cycle
        vTaskDelayUntil(&lastWakeTime, taskFrequency);
    }
}

void CAN_Receiver_Task(void *pvParameters){
    twai_message_t message;
    while (true) {
        // Wait indefinitely for a message (or use a timeout)
        esp_err_t result = twai_receive(&message, portMAX_DELAY);

        if (result == ESP_OK) {
            // Process BMS messages as before
            Process_BMS_Message(&message);
            
            // Process OBC messages to monitor values
            processOBCMessage(&message);
        }
    }
}

void Process_BMS_Message(const twai_message_t *message) {
    // Process BMS messages as before
    BMS_Get_CAN_Message(message);
}

typedef struct 
{
    float onBdChrgrT;
    int   onBdChrgrHndlSt;
    int   onBdChrgrSt;
    float onBdChrgrIAct;      
    float onBdChrgrUDc;       // Input DC voltage (V)
    float onBdChrgrIDc;       // Input DC current (A)
    float onBdChrgrUAct;      // Output voltage (V)
    uint32_t lastUpdateMs;    // Last time OBC data was updated
} OBC_Monitor;

OBC_Monitor OBC_dev = {
    .onBdChrgrT = 0.0,
    .onBdChrgrHndlSt = 0,
    .onBdChrgrSt = 0,
    .onBdChrgrIAct = 0.0,
    .onBdChrgrUDc = 0.0,
    .onBdChrgrIDc = 0.0,
    .onBdChrgrUAct = 0.0,
    .lastUpdateMs = 0
};

void Get_Bd_Chrgr(const uint8_t *data) {
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

void Get_Bd_State(const uint8_t *data) {
    uint8_t raw_HndlSt = message->data[2] & 0x0F; // Extract state from first byte
    obc.onBdChrgrHndlSt = raw_HndlSt; // Store the state

    uint8_t raw_temp = message->data[3] & 0xFF;
    obc.onBdChrgrT = raw_temp;
}

void Get_Current_State(const uint8_t *data) {
    uint16_t raw_iact = message->data[0] & 0x0007;
    raw_iact = (raw_iact << 8) | message->data[1];
    raw_iact = (raw_iact << 1) | ((message->data[2] >> 7) & 0x01);
    obc.onBdChrgrIAct = raw_iact * 0.1; // Store the raw current value

    uint8_t raw_OBCSt = (message->data[2] >> 3) & 0x0F; // Extract state from second byte
    obc.onBdChrgrSt = raw_OBCSt; // Store the state
}

void processOBCMessage(const twai_message_t *message) {
    const uint16_t id = static_cast<uint16_t>(message->identifier);
    const uint8_t *data = message->data;

    switch (id){
        case 0x12A:
            Get_Bd_Chrgr(data);
            break;
        case 0x218:
            Get_Bd_State(data);
            break;
        case 0x216:
            Get_Current_State(data);
            break;
        default:
            break;
    }
}

void System_Health_Check_Task(void *pvParameters) {
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

void BMS_Monitor_Task(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t taskFrequency = pdMS_TO_TICKS(kBMSMonitorIntervalMs);
    
    while (true) {
        bms_normal = bms_t.bms_states == BMS_NORMAL;

        // LED lighting is bms normal
        digitalWrite(kBMSFaultPin, bms_normal ? HIGH : LOW);

        vTaskDelayUntil(&lastWakeTime, taskFrequency);

        checkBMSStatus();

        updateChargingLed();

        static uint32_t last_print_time_Ms = 0;
        uint32_t now_Ms = millis();
        if(now_Ms - last_print_time_Ms > 1000) {
            printSystemStatus();
            last_print_time_Ms = now_Ms;
        }
    }
}

void checkBMSStatus() {
    BMS_Update_Data();
    updateChargingState();
}

void updateChargingLed() {
    uint32_t now_Ms = millis();

    switch (bms_t.charging_states) {
        case CHARGING_OFF:
            digitalWrite(kChargingLedPin, LOW);
            break;
        case CHARGING_ON:
            digitalWrite(kChargingLedPin, now_Ms % 1000 > 500);
            break;
        default:
            break;
    }
}






