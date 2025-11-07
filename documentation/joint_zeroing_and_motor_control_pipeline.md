# 關節歸零與馬達控制流程教學

## 目錄
1. [概述](#概述)
2. [關節歸零（Leg Calibration）](#關節歸零leg-calibration)
3. [馬達控制流程](#馬達控制流程)
4. [關鍵組件說明](#關鍵組件說明)
5. [實際操作步驟](#實際操作步驟)

---

## 概述

本文件說明 MiLAB 四足機器人的關節歸零（腿部校正）流程，以及從高階控制器到低階馬達控制板的完整控制流程。系統採用分層架構，從用戶控制器到最終的馬達驅動器，每一層都有其特定的職責。

---

## 關節歸零（Leg Calibration）

### 什麼是關節歸零？

關節歸零是將機器人關節編碼器的零點位置設定為已知參考位置的過程。這對於確保機器人的運動學計算和控制精度至關重要。每個關節的零位置定義了該關節的「零度」參考點。

### 為什麼需要關節歸零？

1. **編碼器初始位置不確定**：馬達編碼器在啟動時可能處於任意位置
2. **機械組裝誤差**：實際機械零點與理論零點可能存在偏差
3. **控制精度要求**：準確的零點位置是實現精確運動控制的基礎

### 關節歸零的實現方式

#### 1. Spine Board 層級的歸零（硬體層）

在 `spine/main.cpp` 中實現了硬體層的歸零功能：

```cpp
void Zero(CANMessage * msg)
{
    msg->data[0] = 0xFF;
    msg->data[1] = 0xFF;
    msg->data[2] = 0xFF;
    msg->data[3] = 0xFF;
    msg->data[4] = 0xFF;
    msg->data[5] = 0xFF;
    msg->data[6] = 0xFF;
    msg->data[7] = 0xFE;  // 特殊歸零命令標記
    WriteAll();
}
```

這個函數通過發送特殊的 CAN 訊息（最後一個位元組為 `0xFE`）來觸發馬達控制器的歸零操作。當通過串列埠輸入 `'z'` 字元時，會對所有關節執行歸零：

```cpp
case('z'):
    printf("\n\r zeroing \n\r");
    Zero(&a1_can);  // Ab/Ad joint 1
    Zero(&a2_can);  // Ab/Ad joint 2
    Zero(&h1_can);  // Hip joint 1
    Zero(&h2_can);  // Hip joint 2
    Zero(&k1_can);  // Knee joint 1
    Zero(&k2_can);  // Knee joint 2
    break;
```

#### 2. 軟體層級的歸零標記

在 `LegController` 類別中，有一個 `_zeroEncoders` 標記用於控制歸零行為：

```cpp
// common/include/Controllers/LegController.h
bool _zeroEncoders = false;
```

這個標記可以通過控制器設定：

```cpp
// user/MiLAB_JPos_Controller/JPos_Controller.cpp
if (userParameters.zero > 0.5) {
    _legController->_zeroEncoders = true;
} else {
    _legController->_zeroEncoders = false;
}
```

當 `_zeroEncoders` 設為 `true` 時，在更新 TI Board 命令時會設定歸零標記：

```cpp
// common/src/Controllers/LegController.cpp
tiBoardCommand[leg].zero = _zeroEncoders ? 1 : 0;
if(_zeroEncoders) {
    tiBoardCommand[leg].enable = 0;  // 歸零時禁用馬達
}
```

#### 3. 關節偏移量（Joint Offsets）

系統使用關節偏移量來補償實際馬達零點與模擬器零點之間的差異：

```cpp
// robot/src/rt/rt_spi.cpp
// Zero Position for actual motor!! Offsets are calculated from actual zero to sim zero.
const float abad_offset[4] = {0.f, 0.f, 0.f, 0.f};
const float hip_offset[4]  = {-HIP_OFFSET_POS,  HIP_OFFSET_POS,  -HIP_OFFSET_POS,  HIP_OFFSET_POS};
const float knee_offset[4] = {KNEE_OFFSET_POS, -KNEE_OFFSET_POS, KNEE_OFFSET_POS, -KNEE_OFFSET_POS};

#define HIP_OFFSET_POS 4.189f
#define KNEE_OFFSET_POS 2.897f
```

這些偏移量確保實際機器人的關節角度能正確轉換為模擬器使用的座標系統。

---

## 馬達控制流程

### 完整控制流程圖

```
┌─────────────────────────────────────────────────────────────┐
│                    User Controller Layer                     │
│  (e.g., MiLAB_Controller, JPos_Controller, etc.)           │
│                                                              │
│  - 計算期望的關節位置 (qDes)                                │
│  - 設定 PD 控制增益 (kpJoint, kdJoint)                      │
│  - 計算前饋力矩 (tauFeedForward)                             │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│                    LegController Layer                       │
│  (common/src/Controllers/LegController.cpp)                 │
│                                                              │
│  - 接收用戶控制器的命令                                      │
│  - 轉換為低階控制板格式                                      │
│  - 計算估計力矩                                              │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│                  HardwareBridge Layer                        │
│  (robot/src/HardwareBridge.cpp)                             │
│                                                              │
│  - 管理硬體通訊 (SPI/CAN)                                    │
│  - 週期性發送/接收資料 (500Hz for SPI)                       │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│                    SPI Communication                         │
│  (robot/src/rt/rt_spi.cpp)                                  │
│                                                              │
│  - 透過 SPI 介面與 Spine Board 通訊                          │
│  - 應用關節偏移量轉換                                        │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│                    Spine Board (STM32F4)                     │
│  (spine/main.cpp)                                           │
│                                                              │
│  - 接收 SPI 命令                                             │
│  - 透過 CAN 匯流排與馬達控制器通訊                           │
│  - 執行關節限制檢查                                          │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│                  Motor Controllers                           │
│  (CAN Bus, 40kHz control loop)                              │
│                                                              │
│  - 執行關節 PD 控制                                          │
│  - 回傳關節位置、速度、電流                                  │
└─────────────────────────────────────────────────────────────┘
```

### 詳細流程說明

#### 步驟 1: 用戶控制器產生命令

用戶控制器（如 `MiLAB_Controller` 或 `JPos_Controller`）在 `runController()` 函數中計算期望的關節位置：

```cpp
// user/MiLAB_JPos_Controller/JPos_Controller.cpp
void JPos_Controller::leg_JointPD(int leg, Vec3<float> qDes, Vec3<float> qdDes) {
    _legController->commands[leg].kpJoint = kpMat;
    _legController->commands[leg].kdJoint = kdMat;
    _legController->commands[leg].qDes = qDes;      // 期望關節位置
    _legController->commands[leg].qdDes = qdDes;     // 期望關節速度
}
```

#### 步驟 2: LegController 轉換命令

`LegController::updateCommand()` 函數將高階命令轉換為低階控制板格式：

```cpp
// common/src/Controllers/LegController.cpp
void LegController<T>::updateCommand(SpiCommand* spiCommand) {
    for (int leg = 0; leg < 4; leg++) {
        // 設定關節 PD 增益
        spiCommand->kp_abad[leg] = commands[leg].kpJoint(0, 0);
        spiCommand->kp_hip[leg] = commands[leg].kpJoint(1, 1);
        spiCommand->kp_knee[leg] = commands[leg].kpJoint(2, 2);
        
        spiCommand->kd_abad[leg] = commands[leg].kdJoint(0, 0);
        spiCommand->kd_hip[leg] = commands[leg].kdJoint(1, 1);
        spiCommand->kd_knee[leg] = commands[leg].kdJoint(2, 2);
        
        // 設定期望關節位置和速度
        spiCommand->q_des_abad[leg] = commands[leg].qDes(0);
        spiCommand->q_des_hip[leg] = commands[leg].qDes(1);
        spiCommand->q_des_knee[leg] = commands[leg].qDes(2);
        
        spiCommand->qd_des_abad[leg] = commands[leg].qdDes(0);
        spiCommand->qd_des_hip[leg] = commands[leg].qdDes(1);
        spiCommand->qd_des_knee[leg] = commands[leg].qdDes(2);
        
        // 設定前饋力矩
        spiCommand->tau_abad_ff[leg] = legTorque(0);
        spiCommand->tau_hip_ff[leg] = legTorque(1);
        spiCommand->tau_knee_ff[leg] = legTorque(2);
        
        // 設定啟用標記
        spiCommand->flags[leg] = _legsEnabled ? 1 : 0;
    }
}
```

#### 步驟 3: HardwareBridge 發送 SPI 命令

`HardwareBridge::runSpi()` 函數以 500Hz（每 2ms）的頻率執行：

```cpp
// robot/src/HardwareBridge.cpp
void MilabHardwareBridge::runSpi() {
    spi_command_t* cmd = get_spi_command();
    spi_data_t* data = get_spi_data();
    
    // 複製命令到 SPI 驅動緩衝區
    memcpy(cmd, &_spiCommand, sizeof(spi_command_t));
    
    // 執行 SPI 通訊（發送命令，接收資料）
    spi_driver_run();
    
    // 複製接收到的資料
    memcpy(&_spiData, data, sizeof(spi_data_t));
    
    // 發布 LCM 訊息（用於除錯）
    _spiLcm.publish("spi_data", data);
    _spiLcm.publish("spi_command", cmd);
}
```

#### 步驟 4: Spine Board 處理命令

Spine Board（STM32F4）透過 SPI 接收命令，然後透過 CAN 匯流排發送給馬達控制器：

```cpp
// spine/main.cpp
void control() {
    // 從 SPI 命令中提取控制參數
    l1_control.a.p_des = spi_command.q_des_abad[0];
    l1_control.a.v_des = spi_command.qd_des_abad[0];
    l1_control.a.kp = spi_command.kp_abad[0];
    l1_control.a.kd = spi_command.kd_abad[0];
    l1_control.a.t_ff = spi_command.tau_abad_ff[0];
    // ... 其他關節類似
    
    // 打包 CAN 訊息
    PackAll();
    
    // 透過 CAN 匯流排發送
    WriteAll();
}
```

CAN 訊息打包函數將浮點數轉換為整數並打包成 8 位元組的 CAN 訊息：

```cpp
void pack_cmd(CANMessage * msg, joint_control joint) {
    // 限制數值範圍
    float p_des = fminf(fmaxf(P_MIN, joint.p_des), P_MAX);
    float v_des = fminf(fmaxf(V_MIN, joint.v_des), V_MAX);
    float kp = fminf(fmaxf(KP_MIN, joint.kp), KP_MAX);
    float kd = fminf(fmaxf(KD_MIN, joint.kd), KD_MAX);
    float t_ff = fminf(fmaxf(T_MIN, joint.t_ff), T_MAX);
    
    // 轉換為整數
    uint16_t p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    uint16_t v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    // ... 其他參數類似
    
    // 打包到 CAN 訊息
    msg->data[0] = p_int>>8;
    msg->data[1] = p_int&0xFF;
    // ... 其他位元組
}
```

#### 步驟 5: 馬達控制器執行控制

馬達控制器以 40kHz 的頻率執行關節 PD 控制：

```
實際力矩 = kp * (qDes - qActual) + kd * (qdDes - qdActual) + tauFeedForward
```

馬達控制器透過 CAN 匯流排回傳關節位置、速度和電流資訊。

#### 步驟 6: 資料回傳流程

資料回傳流程與命令發送相反：

1. **馬達控制器** → CAN 訊息 → **Spine Board**
2. **Spine Board** → SPI 資料 → **HardwareBridge**
3. **HardwareBridge** → **LegController::updateData()**
4. **LegController** → **User Controller**

```cpp
// common/src/Controllers/LegController.cpp
void LegController<T>::updateData(const SpiData* spiData) {
    for (int leg = 0; leg < 4; leg++) {
        // 更新關節位置
        datas[leg].q(0) = spiData->q_abad[leg];
        datas[leg].q(1) = spiData->q_hip[leg];
        datas[leg].q(2) = spiData->q_knee[leg];
        
        // 更新關節速度
        datas[leg].qd(0) = spiData->qd_abad[leg];
        datas[leg].qd(1) = spiData->qd_hip[leg];
        datas[leg].qd(2) = spiData->qd_knee[leg];
        
        // 計算正向運動學（足端位置和速度）
        computeLegJacobianAndPosition<T>(_quadruped, datas[leg].q, 
                                        &(datas[leg].J), &(datas[leg].p), leg);
        datas[leg].v = datas[leg].J * datas[leg].qd;
    }
}
```

---

## 關鍵組件說明

### 1. LegController

**位置**: `common/src/Controllers/LegController.cpp`

**職責**:
- 提供統一的腿部控制介面
- 轉換高階命令為低階格式
- 計算正向運動學（足端位置、速度、雅可比矩陣）
- 估計關節力矩

**關鍵成員變數**:
- `commands[4]`: 四條腿的控制命令
- `datas[4]`: 四條腿的狀態資料
- `_legsEnabled`: 是否啟用腿部控制
- `_zeroEncoders`: 是否執行編碼器歸零

**關鍵方法**:
- `updateCommand()`: 更新低階控制命令
- `updateData()`: 更新腿部狀態資料
- `zeroCommand()`: 清零所有命令

### 2. HardwareBridge

**位置**: `robot/src/HardwareBridge.cpp`

**職責**:
- 初始化硬體（SPI、IMU、RC 等）
- 管理硬體通訊週期
- 處理 LCM 訊息
- 載入控制參數

**關鍵方法**:
- `runSpi()`: 執行 SPI 通訊（500Hz）
- `initHardware()`: 初始化硬體
- `runMicrostrain()`: 讀取 IMU 資料

### 3. Spine Board (STM32F4)

**位置**: `spine/main.cpp`

**職責**:
- 作為 UP Board 與馬達控制器之間的中介
- 透過 SPI 與 UP Board 通訊
- 透過 CAN 匯流排與馬達控制器通訊
- 執行關節限制檢查和軟停止

**關鍵功能**:
- SPI 中斷處理
- CAN 訊息打包/解包
- 關節限制保護
- 馬達啟用/禁用控制

### 4. RobotRunner

**位置**: `robot/src/RobotRunner.cpp`

**職責**:
- 協調整個控制系統的執行
- 管理控制迴圈（1kHz）
- 初始化狀態估計器和控制器
- 處理緊急停止

**關鍵方法**:
- `run()`: 主控制迴圈
- `setupStep()`: 每個控制週期開始時的設定
- `finalizeStep()`: 每個控制週期結束時的處理

---

## 實際操作步驟

### 關節歸零操作

#### 方法 1: 透過 Spine Board 串列埠（硬體層歸零）

1. 連接 Spine Board 的串列埠到電腦
2. 開啟串列埠終端（波特率通常為 115200）
3. 發送 `'z'` 字元觸發歸零
4. 等待所有關節完成歸零（通常需要幾秒鐘）

#### 方法 2: 透過軟體參數（軟體層歸零）

1. 編輯配置文件 `config/jpos-user-parameters.yaml`:
   ```yaml
   zero: 1.0  # 設為大於 0.5 的值以啟用歸零
   ```

2. 執行控制器:
   ```bash
   ./user/MiLAB_JPos_Controller/milab_ctrl i r
   ```

3. 控制器會自動執行歸零流程

#### 方法 3: 使用 JPosInitializer（自動初始化）

系統在啟動時會自動使用 `JPosInitializer` 將關節移動到安全位置：

```cpp
// robot/src/RobotRunner.cpp
_jpos_initializer = new JPosInitializer<float>(3., controlParameters->controller_dt);
```

這會在 3 秒內平滑地將關節從當前位置移動到目標位置（定義在 `config/initial_jpos_ctrl.yaml`）。

### 驗證歸零是否成功

系統會在 `RobotRunner::debugPrint()` 中檢查關節位置是否在預期範圍內：

```cpp
// robot/src/RobotRunner.cpp
if (std::abs(legs->datas[i].q[0]) > 0.2 || std::abs(legs->datas[i].q[0]) < 0.02)
    motorError = true;
if (legs->datas[i].q[1] > -1.2 || legs->datas[i].q[1] < -1.4)
    motorError = true;
if (legs->datas[i].q[2] > 2.99 || legs->datas[i].q[2] < 2.80)
    motorError = true;
```

如果歸零失敗，系統會顯示錯誤訊息：

```
****** FATAL ERROR ******
Motor Zero Initialization Is Wrong!
Restart Motors And Controller!
```

### 發送關節位置命令

#### 使用 JPos Controller

1. 編輯 `config/jpos-user-parameters.yaml` 設定控制參數
2. 執行控制器並觀察關節運動

#### 使用 Lowlevel Controller（Python 介面）

1. 使用提供的 Python 範例：
   ```python
   # user/MiLAB_Lowlevel_Controller/python_ctrl_eaxmple.py
   cmd = lowlevel_cmd()
   cmd.q_des[0] = 0.0  # Ab/Ad joint
   cmd.q_des[1] = -0.8  # Hip joint
   cmd.q_des[2] = 1.6   # Knee joint
   cmd.kp_joint[0] = 5.0
   cmd.kd_joint[0] = 0.1
   # ... 設定其他參數
   lc.publish("low_level_cmds", cmd.encode())
   ```

2. 執行低階控制器：
   ```bash
   ./user/MiLAB_Lowlevel_Controller/low_ctrl i r
   ```

---

## 總結

本文件說明了 MiLAB 四足機器人的關節歸零流程和完整的馬達控制流程。關鍵要點：

1. **關節歸零**可以在硬體層（Spine Board）或軟體層（LegController）執行
2. **控制流程**採用分層架構，從用戶控制器到馬達控制器，每一層都有明確的職責
3. **SPI 通訊**以 500Hz 頻率在 UP Board 和 Spine Board 之間傳輸資料
4. **CAN 匯流排**用於 Spine Board 與馬達控制器之間的通訊
5. **關節偏移量**確保實際機器人與模擬器之間的座標系統一致性

理解這個流程對於開發新的控制器、除錯控制問題以及優化系統性能都非常重要。
