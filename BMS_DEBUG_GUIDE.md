# BMS.cpp 使用指南 & 除錯協助

## 🎯 核心工作流

### 初始化
```cpp
void setup() {
    BMS_Init();  // 初始化所有數據結構
}
```

### 主循環
```cpp
void loop() {
    // 定期更新 BMS 狀態（每 10-100ms 調用一次）
    BMS_Update_Data();
    
    // 可選：打印診斷信息
    if(millis() % 1000 == 0) {  // 每秒打印一次
        BMS_Print_Diagnostics();
    }
}
```

### CAN 消息接收
```cpp
// CAN 中斷處理或消息輪詢
void handle_can_message(const twai_message_t *msg) {
    BMS_Get_CAN_Message(msg);  // 自動重置計數器
}
```

---

## 🔍 除錯技巧

### 1. 檢查信號遺失狀態
```cpp
// 在診斷函數中查看
BMS_Print_Diagnostics();

// 或直接檢查
for(int i = 0; i < NUM_IC; i++) {
    if(bms_ic_info[i].CAN_signal_lost_count > BMS_SIGNAL_THRESHOLD) {
        Serial.printf("IC %d is OFFLINE\n", i);
    }
}
```

### 2. 監測電壓
```cpp
// 完整的電壓報告
BMS_Print_Cell_Voltages();

// 或個別檢查
Serial.printf("Total Voltage: %d mV\n", bms_t.total_voltage_mV);
Serial.printf("Over Voltage: %s\n", bms_t.over_voltage ? "YES" : "NO");
Serial.printf("Under Voltage: %s\n", bms_t.under_voltage ? "YES" : "NO");
```

### 3. 監測溫度
```cpp
for(int ic = 0; ic < NUM_IC; ic++) {
    for(int ntc = 0; ntc < NTC_PER_IC; ntc++) {
        int16_t temp_deci_c = bms_ic_info[ic].temp_info.temperatures[ntc];
        Serial.printf("IC%d NTC%d: %.1f°C\n", ic, ntc, temp_deci_c / 10.0f);
    }
}
```

### 4. 檢查故障位
```cpp
for(int ic = 0; ic < NUM_IC; ic++) {
    uint8_t fault_bits = bms_ic_info[ic].status_info.fault_bits;
    Serial.printf("IC %d fault bits: 0x%02X\n", ic, fault_bits);
    
    // 解釋故障位
    if(fault_bits & 0x01) Serial.println("  - Signal Lost");
    if(fault_bits & 0x02) Serial.println("  - Sensor Fault");
    if(fault_bits & 0x04) Serial.println("  - Over Temperature");
    if(fault_bits & 0x08) Serial.println("  - Voltage Out of Range");
}
```

---

## ⚠️ 常見問題解決

### 問題 1: 一直報告 IC 離線
**症狀：** `CAN_signal_lost_count` 持續增加，IC 被標記為離線

**可能原因：**
1. CAN 消息根本沒有發送
2. CAN ID 設置錯誤
3. CAN 硬件連接問題
4. 消息 ID 不在 0x100-0x2FF 範圍內

**調試步驟：**
```cpp
// 在 BMS_Get_CAN_Message 中添加調試代碼
void BMS_Get_CAN_Message(const twai_message_t *message) {
    if(message == nullptr) return;
    
    uint16_t id = static_cast<uint16_t>(message->identifier);
    Serial.printf("Received CAN ID: 0x%03X\n", id);  // 檢查 ID
    
    // ... 原始代碼 ...
}
```

### 問題 2: 電壓異常或為 0
**症狀：** `total_voltage_mV` 始終為 0 或異常值

**可能原因：**
1. 所有 IC 都被標記為離線（見問題 1）
2. 電壓數據未正確解析
3. `CODE_TO_VOLT` 常數不正確

**調試步驟：**
```cpp
// 檢查原始代碼值
for(int ic = 0; ic < NUM_IC; ic++) {
    for(int cell = 0; cell < ACTIVE_CELLS_PER_IC; cell++) {
        uint16_t raw_code = bms_ic_info[ic].volt_info.voltages[cell];
        Serial.printf("IC%d Cell%d raw: %d\n", ic, cell, raw_code);
    }
}

// 檢查轉換
float v = 30000 * 0.0001f * 1000;
Serial.printf("30000 code -> %.2f mV\n", v);  // 應該是 3000 mV
```

### 問題 3: 溫度讀數奇怪
**症狀：** 溫度值過大或為負數

**可能原因：**
1. 溫度代碼被誤解為無符號數
2. 字節序錯誤

**調試步驟：**
```cpp
// 檢查原始溫度代碼
int16_t temp_raw = bms_ic_info[0].temp_info.temperatures[0];
Serial.printf("Raw temp code: %d (0x%04X)\n", temp_raw, (uint16_t)temp_raw);
Serial.printf("Temperature: %.1f°C\n", temp_raw / 10.0f);
```

### 問題 4: 故障檢測誤報
**症狀：** 系統不斷報告故障，但數據看起來正常

**可能原因：**
1. 閾值設置不當
2. 故障位未正確重置
3. 狀態機邏輯有問題

**調試步驟：**
```cpp
// 打印完整的故障狀態
Serial.printf("BMS State: %d\n", bms_t.bms_states);
Serial.printf("Signal Lost: %s\n", bms_t.signal_lost ? "YES" : "NO");
Serial.printf("Over Voltage: %s\n", bms_t.over_voltage ? "YES" : "NO");
Serial.printf("Under Voltage: %s\n", bms_t.under_voltage ? "YES" : "NO");
Serial.printf("Over Temperature: %s\n", bms_t.over_temperature ? "YES" : "NO");

// 檢查每個 IC 的狀態
for(int ic = 0; ic < NUM_IC; ic++) {
    Serial.printf("IC%d: fault_state=%d, signal_lost_count=%lu\n", 
                 ic, 
                 bms_ic_info[ic].status_info.fault_state,
                 bms_ic_info[ic].CAN_signal_lost_count);
}
```

---

## 📊 診斷輸出範例

### 正常狀態
```
=== BMS Diagnostic Report ===
[Overall Status]
  [OK] System operating normally.

[Per-IC Status]
IC 0: NORMAL
IC 1: NORMAL
IC 2: NORMAL
IC 3: NORMAL
IC 4: NORMAL
IC 5: NORMAL
=============================
```

### 有故障的狀態
```
=== BMS Diagnostic Report ===
[Overall Status]
  [!] ERROR: Over Voltage Detected!
  [!] WARNING: CAN Signal Lost from some ICs.

[Per-IC Status]
IC 0: NORMAL
IC 1: OFFLINE (Signal Lost - Count: 45)
IC 2: OVER TEMPERATURE
  [!] Over Temperature - NTC 0: 68 (68.0°C)
IC 3: NORMAL
IC 4: VOLTAGE OUT OF RANGE
IC 5: NORMAL
=============================
```

---

## 🧪 快速測試清單

- [ ] 系統初始化成功
- [ ] CAN 消息被正確接收
- [ ] 計數器在收到消息時重置
- [ ] 計數器在沒有消息時遞增
- [ ] 電壓計算正確
- [ ] 溫度轉換正確
- [ ] 故障檢測準確
- [ ] 診斷輸出清晰且有用

---

## 📞 快速參考

| 函數 | 用途 | 調用頻率 |
|------|------|---------|
| `BMS_Init()` | 系統初始化 | 開機一次 |
| `BMS_Get_CAN_Message()` | 處理 CAN 消息 | 每接收到消息調用 |
| `BMS_Update_Data()` | 更新 BMS 狀態 | 10-100ms 調用一次 |
| `BMS_Print_Diagnostics()` | 打印診斷信息 | 1-10 秒調用一次 |
| `BMS_Print_Cell_Voltages()` | 打印詳細電壓 | 調試時按需調用 |

---

## 💡 性能建議

1. **信號遺失閾值**
   - 當前設置: `BMS_SIGNAL_THRESHOLD = 30`
   - 如果在邊界環境中頻繁離線，增加到 50-100
   - 如果需要快速檢測，減少到 10-20

2. **更新頻率**
   - 推薦: 10-20ms 調用一次 `BMS_Update_Data()`
   - 更快會浪費 CPU 資源
   - 更慢會延遲故障檢測

3. **診斷輸出**
   - 調試時: 每秒打印一次
   - 生產環境: 每 5-10 秒打印一次或按需打印

