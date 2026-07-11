#include <driver/twai.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h> // Use FreeRTOS includes for task functions
#include <freertos/task.h>

#include "BMS.h"

// GPIO pin definitions
#include <cstdint>

#include <Adafruit_NeoPixel.h>

// 定義 RGB LED 的接腳，nanoESP-C6 通常為 GPIO 8
#define LED_PIN 8
// 板子上通常只有 1 顆 LED
#define NUMPIXELS 1

Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

#define CAN_TX_PIN GPIO_NUM_2
#define CAN_RX_PIN GPIO_NUM_15
#define FAULT_LED_PIN GPIO_NUM_8
#define START_BUTTON_PIN GPIO_NUM_18
#define CHARGING_LED_PIN GPIO_NUM_5
#define BATTERY_HEALTH_PIN GPIO_NUM_10
#define BMS_FAULT_PIN GPIO_NUM_21
#define PRECHARGE_PIN GPIO_NUM_6
#define HFL_PIN GPIO_NUM_4

// Define HV Battery Parameters
#define HV_BATT_CHRG_I_LIM_AMPS 7.0 // current limit in amps 
#define HV_BATT_U_LIM_VOLTS 420.0          // Maximum battery voltage limit (V)  // 443.7
#define HV_BATT_U_DC_VOLTS 300           // DC voltage setting (V)  // 300

// Define TWAI Queue Size
#define TWAI_TX_QUEUE_SIZE 10
#define TWAI_RX_QUEUE_SIZE 10

// ----------------- Constants -----------------
constexpr TickType_t kCanTimeoutTicks = pdMS_TO_TICKS(10); // CAN TX/RX timeout in FreeRTOS ticks
constexpr TickType_t kTaskDelayTicks = pdMS_TO_TICKS(1);   // Basic task yield delay
constexpr uint32_t kHealthCheckIntervalMs = 30000;         // System health check interval (ms)
constexpr uint32_t kBmsCheckIntervalMs = 100;              // BMS check interval (ms)
constexpr uint32_t kTestStateTimeoutMs = 5000;
constexpr float kInputVoltageThresholdV = 8.0f;   // Minimum input voltage threshold (V)
constexpr float kOvervoltageThresholdV = 396.0f;  // Maximum input voltage threshold (V) //450
constexpr uint32_t kOvercurrentTimeoutMs = 500;   // Time to wait before declaring overcurrent fault (ms)
constexpr uint8_t overTemperatureLimit = 60; // Maximum allowed temperature in Celsius

// ----------------- LED Constants -----------------
constexpr uint32_t kTestLedIntervalMs = 250;        // TEST state flash interval (ms)
constexpr uint32_t kReadyLedIntervalMs = 1000;      // READY state flash interval (ms)
constexpr uint32_t kStateChangeMinIntervalMs = 500; // Minimum time between state changes (reduced)
// Grace period for transient precharge loss before declaring FAIL
constexpr uint32_t kPrechargeWarningTimeoutMs = 1000; // ms


// LED state variables
static uint32_t lastLedUpdateMs = 0;

// OBC varialbles
static int OBC_STATE_READY = 2;
static int OBC_STATE_FAULT = 3;

// ----------------- Pin Configuration -----------------
constexpr gpio_num_t kCanTxPin = CAN_TX_PIN; // CAN Transmit (TWAI_TX)
constexpr gpio_num_t kCanRxPin = CAN_RX_PIN; // CAN Receive (TWAI_RX)
constexpr gpio_num_t kFaultLedPin = FAULT_LED_PIN;           // Fault LED (D5)
constexpr gpio_num_t kStartButtonPin = START_BUTTON_PIN;     // Start charging button
constexpr gpio_num_t kChargingLedPin = CHARGING_LED_PIN;     // Charging indicator LED (D27)
constexpr gpio_num_t kBatteryHealthPin = BATTERY_HEALTH_PIN; // Battery health indicator output
constexpr gpio_num_t kBMSFaultPin = BMS_FAULT_PIN;           // signal to BMS
constexpr gpio_num_t kPreChargePin = PRECHARGE_PIN;          // PRECHARGE LED (D26)
constexpr gpio_num_t kHFLPin = HFL_PIN;

// ----------------- Message Definitions -----------------
typedef struct
{
    uint32_t id;          // CAN message ID
    uint8_t length;       // Data length
    uint8_t data[8];      // CAN data bytes (template/default)
    uint32_t interval_ms; // Send interval in milliseconds
    uint32_t lastSentMs;  // Last time this message was sent (using ms suffix for clarity)
} CANMessageConfig_t;

// --- Compile-time calculation of CAN data bytes ---
// Helper function (using constexpr for compile-time evaluation if possible, else inline)
// Note: std::min might not be constexpr in all toolchains, relying on macro/inline
template <typename T>
inline constexpr T constrain_val(T val, T max)
{
    return (val > max) ? max : val;
}

// Convert voltage/current to the bit value needed for CAN message
constexpr uint16_t hvBattULimTo11bit(uint16_t voltage)
{
    // Scale: 0.25V/bit, Offset: 0V. Range: 0 to 511.75V (0x7FF)
    return constrain_val(static_cast<uint16_t>(voltage * 4), static_cast<uint16_t>(0x7FF));
}

constexpr uint16_t hvBattUDcTo11bit(uint16_t voltage)
{
    // Scale: 0.25V/bit, Offset: 0V. Range: 0 to 511.75V (0x7FF)
    return constrain_val(static_cast<uint16_t>(voltage * 4), static_cast<uint16_t>(0x7FF));
}

constexpr uint16_t hvBattChrgnILimTo13bit(uint16_t amps)
{
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

const uint32_t Max_mv = 443700;
static float currentLimit = HV_BATT_CHRG_I_LIM_AMPS;

CANMessageConfig_t messageConfigs[] = {
    // ID, Length, Data (Template), Interval (ms), LastSentMs
    {0x51A, 8, {0x1A, 0x40, 0x43, 0xEF, 0x00, 0x00, 0x00, 0x00}, 100, 0},                                                                                                                    // Wake up
    {0x141, 8, {0x20, 0x00, static_cast<uint8_t>((0b01101 << 3) | (kHvBattUDcBits >> 8)), static_cast<uint8_t>(kHvBattUDcBits & 0xFF), 0x48, 0x00, 0x00, 0x01}, 20, 0},                      // Parameters: HvBattUDc
    {0x178, 8, {0x60, 0x00, 0x00, 0x00, 0x28, 0xC3, static_cast<uint8_t>((0b00011 << 3) | (kHvBattULimBits >> 8)), static_cast<uint8_t>(kHvBattULimBits & 0xFF)}, 70, 0},                    // Parameters: HvBattULim
    {0x084, 8, {0x40, 0x00, 0x08, 0xFF, 0xA0, 0x00, 0xC0, 0x00}, 25, 0},                                                                                                                     // Output Control (default disable)
    {0x056, 8, {0x00, 0x02, 0x00, 0x00, 0x01, 0x61, 0x00, 0x00}, 15, 0},                                                                                                                     // Vehicle Sim
    {0x289, 8, {0x00, 0x01, 0x04, 0x00, static_cast<uint8_t>((0b000 << 5) | (kHvBattChrgnILimBits >> 8)), static_cast<uint8_t>(kHvBattChrgnILimBits & 0xFF), 0xE0, 0x00}, 100, 0}, // Parameters: HvBattChrgnILim (default High)
    {0x345, 8, {0x00, 0x00, 0x01, 0x01, 0xFE, 0x32, 0x32, 0x10}, 1000, 0}                                                                                                                    // Parameters: HvBattPreHeatgReq (No Preheat)
};

// Calculate the number of messages in the configuration array
const size_t kNumMessages = sizeof(messageConfigs) / sizeof(CANMessageConfig_t);

extern BMS_t bms_t;
extern BMS_IC_info_t bms_ic_info[NUM_IC];
bool bms_normal;
// Flag set when Serial receives the literal "start" (case-insensitive)
static volatile bool serialStartRequested = false;
static volatile bool serialResetRequested = false;

// ----------------- Function Prototypes -----------------
bool Init_CAN();
void CAN_Sender_Task(void *pvParameters);
void CAN_Receiver_Task(void *pvParameters);
void System_Health_Check_Task(void *pvParameters); // Changed loop content to a task
void BMS_Monitor_Task(void *pvParameters);         // Changed loop content to a task
void Check_BMS_Status();
void Process_BMS_Message(const twai_message_t *message);
void Process_OBC_Message(const twai_message_t *message);
void Update_Charging_State();
void Update_Charging_Led();
// bool isBatteryHealthy();
void Print_System_Status(); // Combined print functions
void Get_Bd_Chrgr(const uint8_t *data);
void Control_HFL();
bool Read_Precharge_Signal();

void setup()
{
    // Initialize your setup code here
    Serial.begin(115200);

    pixels.begin(); // 初始化 NeoPixel 物件

    Serial.println("\n--- ESP32 CAN BMS Charger Controller ---");

    pinMode(kFaultLedPin, OUTPUT);
    digitalWrite(kFaultLedPin, LOW); // Start with LED off

    pinMode(kStartButtonPin, INPUT_PULLDOWN);

    pinMode(kChargingLedPin, OUTPUT);
    digitalWrite(kChargingLedPin, LOW);

    pinMode(kBatteryHealthPin, OUTPUT);
    digitalWrite(kBatteryHealthPin, LOW); // Start with battery health indicator off

    pinMode(kBMSFaultPin, OUTPUT);
    digitalWrite(kBMSFaultPin, HIGH); // Start with BMS normal

    pinMode(kPreChargePin, INPUT);

    pinMode(kHFLPin, OUTPUT);
    digitalWrite(kHFLPin, HIGH);

    if (!Init_CAN())
    {
        Serial.println("Failed to initialize CAN");
        while (true)
        {
            digitalWrite(kFaultLedPin, !digitalRead(kFaultLedPin));
            delay(250);
        }
    }

    BMS_Init();
    Serial.println("BMS initialized.");

    // --- Create FreeRTOS tasks ---
    xTaskCreatePinnedToCore(
        CAN_Sender_Task,
        "CAN_TX", // Task name
        4096,     // Stack size (words) - Monitor needed stack size
        NULL,     // Parameters
        3,        // Priority (higher than monitor tasks)
        NULL,     // Task handle
        0         // Core ID (Core 0)
    );

    xTaskCreatePinnedToCore(
        CAN_Receiver_Task,
        "CAN_RX", // Task name
        4096,     // Stack size
        NULL,     // Parameters
        3,        // Priority (higher than monitor tasks)
        NULL,     // Task handle
        0         // Core ID (Core 0)
    );

    xTaskCreatePinnedToCore(
        BMS_Monitor_Task,
        "BMS_Mon", // Task name
        3072,      // Stack size (adjust based on printf usage)
        NULL,      // Parameters
        2,         // Priority
        NULL,      // Task handle
        0          // Core ID (Core 0 - separate from CAN)
    );

    xTaskCreatePinnedToCore(
        System_Health_Check_Task,
        "Sys_Health", // Task name
        2048,         // Stack size
        NULL,         // Parameters
        1,            // Priority (lower)
        NULL,         // Task handle
        0             // Core ID (Core 0)
    );

    Serial.println("System Setup Complete. Tasks Running.");
    Serial.printf("Configured to send %d different CAN messages.\n", kNumMessages);

    // Print message configuration once at startup
    for (size_t i = 0; i < kNumMessages; i++)
    {
        Serial.printf("  Msg %d: ID 0x%03X, Interval %4lums, Data: ",
                      i + 1, messageConfigs[i].id, messageConfigs[i].interval_ms);
        for (int j = 0; j < messageConfigs[i].length; j++)
        {
            Serial.printf("%02X ", messageConfigs[i].data[j]);
        }
        Serial.println();
    }
    Serial.println("----------------------------------------");
}

void loop()
{
    pixels.clear(); // 清除顏色
    // // 設定顏色為綠色 (R, G, B) - 數值範圍 0-255
    pixels.setPixelColor(0, pixels.Color(0, 150, 0));
    pixels.show(); // 更新 LED 顯示
    delay(500);

    // This task will be starved by higher priority tasks or can be deleted in setup()
    vTaskDelay(portMAX_DELAY); // Effectively sleep forever
}

bool Init_CAN()
{
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
    if (result != ESP_OK)
    {
        Serial.printf("Failed to install TWAI driver: %s\n", esp_err_to_name(result));
        return false;
    }
    Serial.println("TWAI driver installed.");

    // Start TWAI driver
    result = twai_start();
    if (result != ESP_OK)
    {
        Serial.printf("Failed to start TWAI driver: %s\n", esp_err_to_name(result));
        twai_driver_uninstall(); // Clean up if start fails
        return false;
    }
    Serial.println("TWAI driver started successfully at 500 kbit/s.");
    return true;
    // Add your CAN initialization code here
}

void CAN_Sender_Task(void *pvParameters)
{
    // Add your CAN sender code here
    TickType_t xLastWakeTime = xTaskGetTickCount(); // Initialize xLastWakeTime
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // Adjust the frequency as needed

    twai_message_t txMessage;
    txMessage.extd = 0;
    txMessage.rtr = 0;

    while (true)
    {
        uint32_t currentTimeMs = millis();

        for (size_t i = 0; i < kNumMessages; i++)
        {
            // Check if interval has passed
            if (currentTimeMs - messageConfigs[i].lastSentMs >= messageConfigs[i].interval_ms)
            {
                txMessage.identifier = messageConfigs[i].id;
                txMessage.data_length_code = messageConfigs[i].length;

                bool sendMessage = false;
                const uint8_t *dataToSend = messageConfigs[i].data; // Default data

                // --- Message Specific Logic ---
                switch (messageConfigs[i].id)
                {
                case 0x51A: // Wake-up Message
                    // Always send this message periodically
                    sendMessage = true;
                    break;

                case 0x084: // Output Control Message
                    // Data depends on charging state
                    if (bms_t.charging_states == CHARGING_START)
                    { //
                        dataToSend = kEnableOutputData;
                    }
                    else
                    {
                        dataToSend = kDisableOutputData;
                    }
                    // Always send this control message periodically
                    sendMessage = true;
                    break;

                default: // Other periodic messages
                    // Send only when actively charging
                    if (bms_t.charging_states == CHARGING_START)
                    { //
                        sendMessage = true;
                    }
                    break;
                }

                // --- Transmit Logic ---
                if (sendMessage)
                {
                    memcpy(txMessage.data, dataToSend, messageConfigs[i].length);
                    esp_err_t result = twai_transmit(&txMessage, kCanTimeoutTicks);
                    if (result == ESP_OK)
                    {
                        messageConfigs[i].lastSentMs = currentTimeMs; // Update last sent time on success
                    }
                    else
                    {
                        // Log error infrequently to avoid spamming
                        static uint32_t lastTxErrorLog = 0;
                        if (currentTimeMs - lastTxErrorLog > 1000)
                        {
                            Serial.printf("WARN: Failed to queue CAN message 0x%lX: %s\n", txMessage.identifier, esp_err_to_name(result));
                            lastTxErrorLog = currentTimeMs;
                        }
                        // Optional: Implement retry or error handling logic here
                    }
                }
            } // end interval check
        } // end for loop

        // Delay until next check cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void CAN_Receiver_Task(void *pvParameters)
{
    twai_message_t message;
    while (true)
    {
        // Wait indefinitely for a message (or use a timeout)
        esp_err_t result = twai_receive(&message, portMAX_DELAY);

        if (result == ESP_OK)
        {
            // Process BMS messages as before
            Process_BMS_Message(&message);

            // Process OBC messages to monitor values
            Process_OBC_Message(&message);
        }
    }
}

void Process_BMS_Message(const twai_message_t *message)
{
    // Process BMS messages as before
    BMS_Get_CAN_Message(message);
}

typedef struct
{
    float onBdChrgrT;
    int onBdChrgrHndlSt;
    int onBdChrgrSt;
    float onBdChrgrIAct;
    float onBdChrgrUDc;    // Input DC voltage (V)
    float onBdChrgrIDc;    // Input DC current (A)
    float onBdChrgrUAct;   // Output voltage (V)
    uint32_t lastUpdateMs; // Last time OBC data was updated
    uint8_t lastRaw[8];    // Last raw CAN data bytes
    uint8_t lastRawLen;    // Length of last raw data
} OBC_Monitor;

OBC_Monitor obc = {
    .onBdChrgrT = 0.0,
    .onBdChrgrHndlSt = 0,
    .onBdChrgrSt = 0,
    .onBdChrgrIAct = 0.0,
    .onBdChrgrUDc = 0.0,
    .onBdChrgrIDc = 0.0,
    .onBdChrgrUAct = 0.0,
    .lastUpdateMs = 0,
    .lastRaw = {0},
    .lastRawLen = 0,
};

void Get_Bd_Chrgr(const uint8_t *data)
{
    obc.lastUpdateMs = millis();

    // Extract OnBdChrgrUDc - Correct
    uint16_t raw_udc = 0;
    raw_udc = ((uint16_t)(data[5])) | (((uint16_t)(data[4] & 0x1F)) << 8);
    obc.onBdChrgrUDc = raw_udc * 0.2;

    uint16_t raw_idc = 0;
    raw_idc = ((uint16_t)(data[1] & 0b11111000) >> 3) | ((uint16_t)(data[0] & 0b01111111) << 5);
    obc.onBdChrgrIDc = (raw_idc * 0.1) - 200.0; // Apply scaling and offset

    // Extract OnBdChrgrUAct - Correct
    uint16_t raw_uact = 0;
    raw_uact = ((uint16_t)data[7]) | (((uint16_t)(data[6] & 0x07)) << 8);
    obc.onBdChrgrUAct = raw_uact * 0.25;
}

void Get_Bd_State(const uint8_t *data)
{
    obc.lastUpdateMs = millis();

    uint8_t raw_HndlSt = data[2] & 0x0F; // Extract state from first byte
    obc.onBdChrgrHndlSt = raw_HndlSt;    // Store the state

    uint8_t raw_temp = data[3] & 0xFF;
    obc.onBdChrgrT = raw_temp;
}

void Get_Current_State(const uint8_t *data)
{
    obc.lastUpdateMs = millis();

    uint16_t raw_iact = data[0] & 0x0007;
    raw_iact = (raw_iact << 8) | data[1];
    raw_iact = (raw_iact << 1) | ((data[2] >> 7) & 0x01);
    obc.onBdChrgrIAct = raw_iact * 0.1 - 200; // Store the raw current value

    uint8_t raw_OBCSt = (data[2] >> 3) & 0x0F; // Extract state from second byte
    obc.onBdChrgrSt = raw_OBCSt;               // Store the state
}

void Process_OBC_Message(const twai_message_t *message)
{
    const uint16_t id = static_cast<uint16_t>(message->identifier);
    const uint8_t *data = message->data;
    // Store raw data and parse, but do NOT print here. Print_System_Status() will output the info.
    switch (id)
    {
    case 0x084: // wake-up obc message
                // Store raw data for Output Control message (for debugging)
        obc.lastRawLen = message->data_length_code;
        memcpy(obc.lastRaw, data, obc.lastRawLen);
        obc.lastUpdateMs = millis();
        break;

    case 0x12A:
        obc.lastRawLen = message->data_length_code;
        memcpy(obc.lastRaw, data, obc.lastRawLen);
        Get_Bd_Chrgr(data);
        break;

    case 0x218:
        obc.lastRawLen = message->data_length_code;
        memcpy(obc.lastRaw, data, obc.lastRawLen);
        Get_Bd_State(data);
        break;

    case 0x216:
        obc.lastRawLen = message->data_length_code;
        memcpy(obc.lastRaw, data, obc.lastRawLen);
        Get_Current_State(data);
        break;

    default:
        break;
    }
}

void System_Health_Check_Task(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t taskFrequency = pdMS_TO_TICKS(kHealthCheckIntervalMs);

    while (true)
    {
        // Wait for the next cycle
        vTaskDelayUntil(&lastWakeTime, taskFrequency);

        // Check CAN controller status
        twai_status_info_t status_info;
        esp_err_t result = twai_get_status_info(&status_info);

        if (result == ESP_OK)
        {
            // Optional: Log detailed status less frequently if needed
            // Serial.printf("DEBUG: CAN Status: State=%d, TXerr=%d, RXerr=%d, TXq=%d, RXq=%d, Rcvd=%d, Sent=%d\n",
            //       status_info.state, status_info.tx_error_counter, status_info.rx_error_counter,
            //       status_info.msgs_to_tx, status_info.msgs_to_rx,
            //       status_info.rx_msg_count, status_info.tx_msg_count);

            if (status_info.state == TWAI_STATE_BUS_OFF)
            {
                Serial.println("ERROR: CAN bus-off detected! Attempting recovery...");
                result = twai_initiate_recovery(); // Attempt recovery
                Serial.printf("INFO: CAN recovery attempt result: %s\n", esp_err_to_name(result));
                // Consider more robust recovery (e.g., re-init after repeated failures)
            }
            else if (status_info.state == TWAI_STATE_RECOVERING)
            {
                Serial.println("INFO: CAN bus is recovering...");
            }
            else if (status_info.tx_error_counter > 127 || status_info.rx_error_counter > 127)
            {
                Serial.printf("WARN: High CAN error count (TX:%d, RX:%d). State: %d\n",
                              status_info.tx_error_counter, status_info.rx_error_counter, status_info.state);
                // Potentially trigger a warning state or investigation
            }
        }
        else
        {
            Serial.printf("ERROR: Failed to get TWAI status: %s\n", esp_err_to_name(result));
        }

        // Add other health checks here (e.g., stack high water mark, temperature sensor)
        // UBaseType_t stackHighWater = uxTaskGetStackHighWaterMark(NULL); // Check own stack
        // Serial.printf("DEBUG: SysHealth Task Stack HWM: %u words\n", stackHighWater);
    }
}

void BMS_Monitor_Task(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t taskFrequency = pdMS_TO_TICKS(kBmsCheckIntervalMs);

    while (true)
    {
        // --- Poll Serial for a single-line "start" command ---
        while (Serial.available())
        {
            static String _serialBuf = "";
            char c = static_cast<char>(Serial.read());

            if (c == '\r' || c == '\n')
            {
                if (_serialBuf.length() > 0)
                {
                    String cmd = _serialBuf;
                    cmd.toLowerCase();
                    cmd.trim();
                    if (cmd == "start")
                    {
                        serialStartRequested = true;
                        Serial.println("Serial: 'start' received - ready to start when button pressed.");
                    }
                    else if (cmd == "e" || cmd == "stop" || cmd == "exit")
                    {
                        serialStartRequested = false;
                        Serial.println("Serial: stop command received - stopping charging.");
                    }
                    else if (cmd == "reset")
                    {
                        // Reset from FAIL state back to READY (if in FAIL state)
                        if (bms_t.charging_states == CHARGING_FAIL)
                        {
                            serialResetRequested = true;
                            serialStartRequested = false;
                            Serial.println("Serial: reset command received - transitioning from FAIL to READY.");
                        }
                        else
                        {
                            Serial.printf("Serial: reset command ignored (not in FAIL state, current: %d)\n", bms_t.charging_states);
                        }
                    }
                    _serialBuf = "";
                }
            }
            else
            {
                _serialBuf += c;
                // Prevent runaway buffer
                if (_serialBuf.length() > 64)
                    _serialBuf = _serialBuf.substring(_serialBuf.length() - 64);
            }
        }

        bms_normal = bms_t.bms_states == BMS_NORMAL;

        // LED lighting is bms normal
        digitalWrite(kBMSFaultPin, bms_normal ? HIGH : LOW);

        vTaskDelayUntil(&lastWakeTime, taskFrequency);

        Check_BMS_Status();

        Update_Charging_Led();

        Update_Charging_State();

        Control_HFL();

        static uint32_t last_print_time_Ms = 0;
        uint32_t now_Ms = millis();
        if (now_Ms - last_print_time_Ms > 1000)
        {
            Print_System_Status();
            last_print_time_Ms = now_Ms;
        }
    }
}

bool Read_Precharge_Signal(){
    if(analogRead(kPreChargePin) > 2000) return 1;
    return digitalRead(kPreChargePin); 
}

void Check_BMS_Status()
{
    BMS_Update_Data();
}

void Control_HFL()
{
    uint32_t now_Ms = millis();
    if (now_Ms - obc.lastUpdateMs > 500) // If we have recent OBC data
    {
        digitalWrite(kHFLPin, LOW); // Active LOW for READY
        // Serial.println("Error: OBC disconnected. Shutting down HFL.");
    }
}

void Update_Charging_Led()
{
    uint32_t now_Ms = millis();

    switch (bms_t.charging_states)
    {
    case CHARGING_INIT: // Off - initializing
        digitalWrite(kChargingLedPin, LOW);
        break;

    case CHARGING_READY: // Slow flash - ready for charging
        if (now_Ms - lastLedUpdateMs > kReadyLedIntervalMs)
        {
            digitalWrite(kChargingLedPin, !digitalRead(kChargingLedPin));
            lastLedUpdateMs = now_Ms;
        }
        break;

    case CHARGING_TEST: // Fast flash (not used, but keep for compatibility)
        if (now_Ms - lastLedUpdateMs > kTestLedIntervalMs)
        {
            digitalWrite(kChargingLedPin, !digitalRead(kChargingLedPin));
            lastLedUpdateMs = now_Ms;
        }
        break;

    case CHARGING_START: // Solid ON - actively charging
        digitalWrite(kChargingLedPin, HIGH);
        break;

    case CHARGING_FAIL: // Off (Fault LED handles fail indication)
    default:            // Off
        digitalWrite(kChargingLedPin, LOW);
        break;
    }
}

// bool isBatteryHealthy()
// {
//     bool isHealthy = true; // Default to healthy

//     // Check BMS library reported faults
//     if (bms_t.bms_states != BMS_NORMAL)
//     { //
//         isHealthy = false;
//     }

//     // Update the battery health indicator pin
//     digitalWrite(kBatteryHealthPin, isHealthy ? LOW : HIGH);

//     // return isHealthy; // Return the health status
//     return true; // Return the health status
// }

void Update_Charging_State()
{
    static uint32_t lastStateChangeMs = 0;
    static bool lastButtonState = HIGH; // Assuming INPUT_PULLUP
    uint32_t nowMs = millis();

    // Check button state (simple debounce check)
    bool currentButtonState = digitalRead(kStartButtonPin);
    bool buttonPressed = (currentButtonState == HIGH && lastButtonState == LOW);
    lastButtonState = currentButtonState; // Update for next cycle
    buttonPressed = true;

    // Check if BMS is normal and Precharge is valid (read live signal)
    // bool bmsReady = isBatteryHealthy();
    bool prechargeValid = Read_Precharge_Signal();

    // Prevent rapid state changes, but allow immediate transition *to* FAIL
    CHARGING_STATES currentState = bms_t.charging_states; // Read current state once
    if (currentState != CHARGING_FAIL && (nowMs - lastStateChangeMs < kStateChangeMinIntervalMs))
    {
        // If not failing and minimum interval hasn't passed, return
        // (This prevents flickering between non-fail states)
        return;
    }

    CHARGING_STATES previousState = currentState; // Store state before switch

    switch (currentState)
    {
    case CHARGING_INIT:
        // Transition to READY once BMS is initialized and normal
        if(obc.onBdChrgrUDc < kInputVoltageThresholdV){
            bms_t.charging_states = CHARGING_TEST;
            Serial.println("WARN: Obc is not connected or initial input voltage is too low");
            break;
        } 
        bms_t.charging_states = CHARGING_READY;
        Serial.println("STATE: -> READY");
        break;

    case CHARGING_TEST:
        // Not used in new logic - skip directly to READY
        if(obc.onBdChrgrUDc > kInputVoltageThresholdV && obc.onBdChrgrHndlSt == OBC_STATE_READY){
            bms_t.charging_states = CHARGING_READY;
            Serial.println("chrgrHndlSt:");
            Serial.println(obc.onBdChrgrHndlSt);
            Serial.println("STATE: -> READY");
            break;
        }
        // else: Either waiting for battery to be healthy or for input voltage to reach 380V -> Remain in TEST
        else {
            // Add periodic status update when in TEST state
            static uint32_t lastTestStateUpdateMs = 0;
            if (nowMs - lastTestStateUpdateMs > 5000) { // Update every 5 seconds
                if (obc.onBdChrgrUDc < kInputVoltageThresholdV) {
                    Serial.printf("STATE: TEST - Waiting for input voltage (%.1fV) to reach %.1fV (no timeout)\n", 
                                    obc.onBdChrgrUDc, (float)kInputVoltageThresholdV);
                }
                lastTestStateUpdateMs = nowMs;
            }
        }
        break;

    case CHARGING_READY:
        // Transition to START if:
        // 1. BMS is normal AND Precharge is valid AND button pressed
        // Additionally require a prior Serial "start" command
        if (prechargeValid && buttonPressed && serialStartRequested)
        {
            bms_t.charging_states = CHARGING_START;
            Serial.println("STATE: -> START (Button Pressed & Serial 'start' received, Conditions Met)");
            // consume the serial start request once used
            serialStartRequested = false;
        }
        break;

    case CHARGING_START:
        // Check Precharge state — treat transient loss as a warning first
        static uint32_t prechargeLostStartMs = 0;
        if (!prechargeValid)
        {
            if (prechargeLostStartMs == 0)
            {
                prechargeLostStartMs = nowMs; // start grace timer
                Serial.println("WARN: Precharge lost (transient) — monitoring before fail.");
            }
            else if (nowMs - prechargeLostStartMs > kPrechargeWarningTimeoutMs)
            {
                bms_t.charging_states = CHARGING_FAIL;
                Serial.println("STATE: -> FAIL (Precharge disconnected during CHARGING)");
                prechargeLostStartMs = 0;
                break;
            }
            // else: still within grace period, do not fail yet
        }
        else
        {
            prechargeLostStartMs = 0; // reset timer when precharge returns
        }

        // Check for overvoltage condition
        if (obc.onBdChrgrUDc > kOvervoltageThresholdV)
        {
            bms_t.charging_states = CHARGING_FAIL;
            Serial.printf("STATE: -> FAIL (Overvoltage detected: %.1fV > %.1fV)\n",
                          obc.onBdChrgrUDc, kOvervoltageThresholdV);
            break;
        }

        // Check for OBC fault status
        if (obc.onBdChrgrSt == OBC_STATE_FAULT)
        {
            bms_t.charging_states = CHARGING_FAIL;
            Serial.println("STATE: -> FAIL (OBC Fault detected)");
            break;
        }

        // Check for overcurrent condition
        static uint32_t overcurrentStartTimeMs = 0;
        if (obc.onBdChrgrIDc > (currentLimit + 0.5))
        {
            if (overcurrentStartTimeMs == 0)
            {
                overcurrentStartTimeMs = nowMs;
            }
            else if (nowMs - overcurrentStartTimeMs > kOvercurrentTimeoutMs)
            {
                bms_t.charging_states = CHARGING_FAIL;
                Serial.printf("STATE: -> FAIL (Overcurrent persisted for >%dms: %.1fA > %.1fA)\n",
                              kOvercurrentTimeoutMs, obc.onBdChrgrIDc, currentLimit);
                overcurrentStartTimeMs = 0;
                break;
            }
        }
        else
        {
            overcurrentStartTimeMs = 0; // Reset timer if current is normal
        }

        if (obc.onBdChrgrUDc >= bms_t.total_voltage_mV / 1000.0f){
            bms_t.charging_states = CHARGING_READY;
            Serial.println("STATE: -> READY (Charge completed successfully)");
        }
        break;

    case CHARGING_FAIL:
        // Check if BMS recovers - don't auto-recover, require manual button press
        // User must press button again to attempt restart
        if (prechargeValid && serialResetRequested)
        {
            // Attempt to recover by going back to READY
            bms_t.charging_states = CHARGING_READY;
            Serial.println("STATE: -> READY (Manual Recovery via Button)");
            serialResetRequested = false;
        }
        // Ensure fault LED is ON
        digitalWrite(kFaultLedPin, HIGH);
        break;

    default:
        Serial.printf("WARN: Unknown charging state: %d\n", bms_t.charging_states);
        bms_t.charging_states = CHARGING_FAIL; // Default to fail on unknown state
        break;

    } // end switch

    // If state changed, record the time
    if (bms_t.charging_states != previousState)
    {
        lastStateChangeMs = nowMs;
        Serial.printf("STATE CHANGE: %d -> %d at %lu ms\n", previousState, bms_t.charging_states, nowMs);

        // Reset LED timer on state change for clean flashing start
        lastLedUpdateMs = nowMs;
        // Ensure Fault LED is OFF unless in FAIL state
        if (bms_t.charging_states != CHARGING_FAIL)
        {
            digitalWrite(kFaultLedPin, LOW);
        }
    }

} // end Update_Charging_State

void Print_System_Status()
{
    Serial.println("\n========== System Status ==========");

    // 1. Charging State
    Serial.print("Charging State: ");
    switch (bms_t.charging_states)
    {
    case CHARGING_INIT:
        Serial.println("INITIALIZING");
        break;
    case CHARGING_TEST:
        Serial.println("TEST");
        break;
    case CHARGING_READY:
        Serial.println("READY");
        break;
    case CHARGING_START:
        Serial.println("ACTIVE");
        break;
    case CHARGING_FAIL:
        Serial.println("FAILED");
        break;
    default:
        Serial.println("UNKNOWN");
        break;
    }

    // 2. Battery Voltage Status
    Serial.printf("Battery Voltage: %.2f V (%.1f%%)\n", 
                  bms_t.total_voltage_mV / 1000.0,
                  (bms_t.total_voltage_mV / (float)Max_mv) * 100.0);

    // 3. OBC Status (merged from multiple print locations)
    if (obc.lastRawLen > 0)
    {
        // Consolidated OBC information
        Serial.printf("OBC DC Output Voltage: %.1fV, Current: %.1fA, AC Input: %.1fV, Current: %.1fA, Temperature: %.1f°C\n",
                      obc.onBdChrgrUDc, obc.onBdChrgrIDc, obc.onBdChrgrUAct, obc.onBdChrgrIAct, obc.onBdChrgrT);
    }

    // 4. System Conditions
    Serial.println("System Conditions:");
    Serial.printf("  BMS Status: %s\n", (bms_t.bms_states == BMS_NORMAL) ? "NORMAL" : "ERROR");
    Serial.printf("  Precharge: %s\n", Read_Precharge_Signal() ? "COMPLETE" : "INCOMPLETE");
    Serial.printf("  Button: %s | Serial 'start': %s\n",
                  digitalRead(kStartButtonPin) ? "PRESSED" : "NOT",
                  serialStartRequested ? "YES" : "NO");

    // 5. Fault Warnings (only show if present)
    if (bms_t.over_voltage || bms_t.under_voltage || bms_t.over_temperature)
    {
        Serial.print("  WARNINGS: ");
        if (bms_t.over_voltage) Serial.print("OverV ");
        if (bms_t.under_voltage) Serial.print("UnderV ");
        if (bms_t.over_temperature) Serial.print("OverT ");
        Serial.println();
    }

    Serial.println("====================================\n");

    // 6. Detailed diagnostics (only if needed for troubleshooting)
    // Uncomment these lines for verbose debugging:
    BMS_Print_Cell_Voltages();
    BMS_Print_Temperature();
    // if (bms_t.bms_states != BMS_NORMAL) {
    //     BMS_Print_Diagnostics();
    // }
}
