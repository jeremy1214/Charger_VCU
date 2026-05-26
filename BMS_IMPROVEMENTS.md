# BMS.cpp 程式碼改進文檔

## 改進概述
已完成 BMS.cpp 的全面重構，修復了關鍵 Bug，增強了程式碼可讀性和維護性。

---

## 🔴 重大 Bug 修復

### 1. **Signal Lost 計數邏輯錯誤** ⚠️ CRITICAL
**問題：**
- 原代碼在 `BMS_Update_State()` 中，每次調用都會無條件遞增 `CAN_signal_lost_count`
- 即使收到 CAN 消息也會持續累加
- 導致計數器永遠會超過閾值，造成誤報

**修復：**
- 計數器只在沒有收到消息時遞增（在 `BMS_Update_State()` 中）
- 當收到 CAN 消息時，調用新函數 `BMS_Reset_Signal_Lost_Count(ic)` 重置為 0
- 增加了清晰的註解說明邏輯

**影響：** 防止了虛假的「信號遺失」告警

---

### 2. **重複的 Signal Lost 檢查**
**問題：**
- 在 fault_bits 中使用 switch-case 檢查 0x01 標誌
- 又在計數器中單獨檢查信號遺失
- 邏輯混亂且容易出錯

**修復：**
- 統一邏輯：先用計數器判斷是否離線，再用 fault_bits 確認錯誤類型
- 使用更清晰的 if-else-if 結構替代 switch-case
- 優先級明確：計數器 > fault_bits

---

### 3. **缺少指針驗證**
**問題：**
- `BMS_Get_CAN_Message()` 沒有驗證 `message` 指針

**修復：**
```cpp
if(message == nullptr) return;
```

---

## 📝 功能改進

### 1. **函數命名改進**
| 舊名稱 | 新名稱 | 原因 |
|--------|--------|------|
| `Get_BMS_IC_Info(ic)` | `BMS_Reset_Signal_Lost_Count(ic)` | 名字清楚表達實際功能 |

### 2. **增強的邊界檢查**
- CAN 消息處理中添加了 IC 索引驗證
- 防止數組越界訪問

```cpp
if(ic >= NUM_IC) return;
```

### 3. **改進的 BMS_Init()**
- 完全初始化所有 IC 結構體
- 初始化所有數組欄位（不只是計數器）
- 初始化 `bms_fault` 標誌

---

## 🎯 工作流改進

### 1. **CAN 消息處理流程**
```
收到 CAN 消息
    ↓
驗證指針 & IC 索引
    ↓
重置信號遺失計數器 (BMS_Reset_Signal_Lost_Count)
    ↓
解析並更新相應數據
```

### 2. **狀態更新流程**
```
BMS_Update_Data()
    ↓
BMS_Update_Volt()          → 檢查電壓範圍
    ↓
BMS_Update_State()         → 檢查信號和故障位
    ↓
BMS_Check_Fault()          → 綜合判斷系統狀態
```

### 3. **故障優先級**
1. **Sensor Fault**（高優先級）：電壓/溫度異常
2. **Comm Fault**（中優先級）：信號遺失
3. **Normal**（低優先級）：無故障

---

## 📊 診斷函數改進

### BMS_Print_Diagnostics()
**改進內容：**
- 分為「整體狀態」和「逐 IC 狀態」兩部分
- 離線 IC 單獨標記，不重複檢查
- 使用 `FAULT_STATES` 枚舉顯示故障類型
- 更清晰的輸出格式

**輸出範例：**
```
[Overall Status]
  [!] WARNING: CAN Signal Lost from some ICs.

[Per-IC Status]
IC 0: NORMAL
IC 1: OFFLINE (Signal Lost - Count: 35)
IC 2: OVER TEMPERATURE
  [!] Over Temperature - NTC 0: 65 (65.0°C)
```

### BMS_Print_Cell_Voltages()
**改進內容：**
- 檢查 IC 是否離線，離線則跳過
- 統計數據計算改進（float 精度）
- 更清晰的格式化輸出
- 添加了電壓差異計算

---

## 🛡️ 類型安全改進

### 隱式類型轉換
**改進前：**
```cpp
if(bms_ic_info[ic].status_info.max_voltage > CELL_OK_MAX_CODE * CODE_TO_VOLT * 1000)
```

**改進後：**
```cpp
uint32_t max_voltage_mv = (uint32_t)(CELL_OK_MAX_CODE * CODE_TO_VOLT * 1000);
if(bms_ic_info[ic].status_info.max_voltage > max_voltage_mv)
```

- 避免重複計算
- 類型轉換明確
- 計算結果存儲在臨時變量中

---

## 📚 代碼文檔改進

添加了 Doxygen 風格的註解：
```cpp
/**
 * @brief 函數簡短說明
 * @param 參數說明
 * @return 返回值說明
 */
```

---

## ✅ 測試建議

1. **驗證 Signal Lost 邏輯**
   - 模擬 CAN 消息丟失，驗證計數器遞增
   - 恢復 CAN 消息，驗證計數器重置

2. **驗證故障檢測**
   - 測試各種故障組合
   - 驗證優先級正確

3. **驗證診斷輸出**
   - 確認離線 IC 正確標記
   - 驗證統計數據準確性

---

## 📋 總結

| 項目 | 修改數量 | 影響 |
|------|---------|------|
| 關鍵 Bug 修復 | 3 | 防止虛假告警和數組越界 |
| 功能改進 | 5 | 提升代碼清晰度 |
| 邊界檢查 | 4 | 增強系統穩定性 |
| 文檔改進 | 8+ | 便於理解和維護 |

**主要成果：** 代碼從容易出錯轉變為清晰、安全、易於維護的版本 ✨
