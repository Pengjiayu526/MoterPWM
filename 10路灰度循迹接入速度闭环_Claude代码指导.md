# 天猛星两轮差速小车：10 路灰度循迹接入速度闭环代码指导

> 用途：将本文件直接交给 Claude，让其基于当前 MSPM0G3507 工程完成代码编写。
> 当前阶段只完成：**10 路数字灰度 → 线位置 → 循迹 PD → 左右目标 pps → 已有速度 PI**。
> 暂不加入陀螺仪、路口状态机、环岛识别和编码器航向同步。

---

## 1. 当前工程条件

当前工程已完成：

1. 两轮电机 PWM 驱动。
2. 左右编码器 GPIO 中断计数。
3. 20 ms 编码器测速。
4. 左右轮独立 PI 速度闭环。
5. 速度目标接口：

```c
void SpeedControl_SetTargetPPS(
    int32_t leftTargetPPS,
    int32_t rightTargetPPS);
```

6. 编码器速度查询接口：

```c
uint32_t Encoder_GetSpeed(Encoder_Select encoder);
```

7. 串口调试使用：

```c
snprintf();
USART_SendString();
```

8. 不使用 CCS CIO 作为实际串口输出。
9. 速度 PI 在 800、1000、1200 pps 下均已验证可用。
10. 速度 PI 当前周期为 20 ms。

---

## 2. 10 路灰度传感器已确认信息

### 2.1 物理顺序

```text
最左                                                  最右
CH0  CH1  CH2  CH3  CH4  CH5  CH6  CH7  CH8  CH9
```

必须保持：

```text
CH0 = 最左
CH9 = 最右
```

### 2.2 黑白逻辑

已经实测：

```text
黑线 = 1
白底 = 0
```

因此不需要反相：

```c
line_detected = ((sensor_bitmap >> i) & 0x01U);
```

### 2.3 现有灰度接口

现有 `grayscale.c/.h` 已实现：

```c
#define GRAYSCALE_CHANNEL_COUNT 10

uint8_t  Grayscale_ReadChannel(uint8_t channel);
uint16_t Grayscale_ReadAll(void);
void     Grayscale_Print(void);
```

`Grayscale_ReadAll()` 返回：

```text
bit0 → CH0
bit1 → CH1
...
bit9 → CH9
```

每个循迹周期只读取一次：

```c
uint16_t sensors = Grayscale_ReadAll();
```

后续所有判断都基于这次快照。

---

## 3. 旧 12 路循迹代码可保留的思路

用户提供的旧 `pid.c/.h` 可以参考：

1. 传感器加权位置计算。
2. 位置 EMA 滤波。
3. 微分项 EMA 滤波。
4. PID 输出限幅。
5. 丢线处理。
6. 状态查询接口。
7. 低频串口遥测。

但不能直接复制，必须修改以下内容。

### 3.1 通道数改为 10

旧代码：

```c
#define PID_SENSOR_COUNT 12U
```

新代码应直接使用：

```c
GRAYSCALE_CHANNEL_COUNT
```

不要留下 12 路数组、循环或位图判断。

### 3.2 权重改为 10 路

推荐：

```c
static const float gLineWeights[GRAYSCALE_CHANNEL_COUNT] = {
    -4.5f, -3.5f, -2.5f, -1.5f, -0.5f,
     0.5f,  1.5f,  2.5f,  3.5f,  4.5f
};
```

含义：

```text
负位置 = 黑线位于左侧
正位置 = 黑线位于右侧
0      = 黑线位于 CH4、CH5 中间
```

### 3.3 禁止直接调用 Motor_SetSpeed()

旧代码直接输出 PWM：

```c
Motor_SetSpeed(left_speed, right_speed);
```

新工程已有速度 PI，因此循迹层必须输出左右目标速度：

```c
leftTargetPPS  = basePPS - turnPPS;
rightTargetPPS = basePPS + turnPPS;

SpeedControl_SetTargetPPS(
    leftTargetPPS,
    rightTargetPPS);
```

控制层次必须是：

```text
灰度传感器
    ↓
线位置与循迹 PD
    ↓
左右目标 pps
    ↓
左右轮速度 PI
    ↓
PWM
```

### 3.4 模块改名

为避免与速度 PI 混淆，不再沿用泛化的 `pid.c/.h`。

建议新增：

```text
line_follow.c
line_follow.h
```

---

## 4. 本次代码任务

新增独立灰度循迹模块，实现：

1. 读取 10 路灰度位图。
2. 计算黑线加权位置。
3. 计算线位置误差。
4. 对位置进行 EMA 滤波。
5. 使用 P/PD 产生 `turnPPS`。
6. 将 `basePPS`、`turnPPS` 转成左右目标速度。
7. 调用现有 `SpeedControl_SetTargetPPS()`。
8. 检测丢线。
9. 检测全黑和宽线。
10. 提供状态查询接口。
11. 不在循迹函数中发送串口。
12. 不直接控制 PWM。
13. 不修改现有速度 PI、编码器中断和电机底层。

---

## 5. 推荐文件结构

新增：

```text
line_follow.c
line_follow.h
```

主要修改：

```text
main.c
```

不应大改：

```text
grayscale.c/.h
speed_control.c/.h
encoder.c/.h
motor.c/.h
```

---

## 6. line_follow.h 建议接口

```c
#ifndef LINE_FOLLOW_H_
#define LINE_FOLLOW_H_

#include <stdint.h>

typedef enum
{
    LINE_STATE_NORMAL = 0,
    LINE_STATE_LOST,
    LINE_STATE_ALL_BLACK,
    LINE_STATE_WIDE
} LineFollow_State;

typedef struct
{
    float kp;
    float ki;
    float kd;

    float position_alpha;
    float derivative_alpha;

    int32_t base_pps;
    int32_t max_turn_pps;
    int32_t max_target_pps;

} LineFollow_Config;

void LineFollow_Init(const LineFollow_Config *config);
void LineFollow_Start(void);
void LineFollow_Stop(void);
void LineFollow_Reset(void);
void LineFollow_Update(void);

void LineFollow_SetBasePPS(int32_t basePPS);
void LineFollow_SetPD(float kp, float kd);
void LineFollow_SetPID(float kp, float ki, float kd);
void LineFollow_SetMaxTurnPPS(int32_t maxTurnPPS);

uint16_t LineFollow_GetSensorBitmap(void);
uint8_t  LineFollow_GetActiveCount(void);

float LineFollow_GetRawPosition(void);
float LineFollow_GetFilteredPosition(void);
float LineFollow_GetError(void);
float LineFollow_GetOutput(void);
float LineFollow_GetIntegral(void);

int32_t LineFollow_GetTurnPPS(void);
int32_t LineFollow_GetLeftTargetPPS(void);
int32_t LineFollow_GetRightTargetPPS(void);

uint8_t LineFollow_IsEnabled(void);
LineFollow_State LineFollow_GetState(void);

#endif
```

名称可以按工程风格微调，但必须保留初始化、更新、启停、参数设置和状态查询功能。

---

## 7. 加权位置计算

推荐权重：

```c
static const float gLineWeights[GRAYSCALE_CHANNEL_COUNT] = {
    -4.5f, -3.5f, -2.5f, -1.5f, -0.5f,
     0.5f,  1.5f,  2.5f,  3.5f,  4.5f
};
```

计算：

```c
weighted_sum = 0.0f;
active_count = 0U;

for (i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++)
{
    if (((sensor_bitmap >> i) & 0x01U) != 0U)
    {
        weighted_sum += gLineWeights[i];
        active_count++;
    }
}
```

正常情况下：

```c
position = weighted_sum / (float)active_count;
```

示例：

```text
CH4、CH5 为黑：position = 0
CH1、CH2 为黑：position = -3.0
CH7、CH8 为黑：position = +3.0
```

误差定义：

```c
error = 0.0f - filtered_position;
```

因此：

```text
黑线在左 → position < 0 → error > 0
黑线在右 → position > 0 → error < 0
```

配合：

```c
leftTarget  = basePPS - turnPPS;
rightTarget = basePPS + turnPPS;
```

正 `turnPPS` 通常会使小车左转。如果实车转向相反，只交换左右加减号，不要改传感器权重定义。

---

## 8. 丢线与特殊图案

### 8.1 全白：丢线

由于白底为 0：

```c
active_count == 0U
```

表示丢线。

第一版安全策略：

```c
state = LINE_STATE_LOST;
LineFollow_ResetControllerState();
SpeedControl_SetTargetPPS(0, 0);
```

暂不实现高速找线。

### 8.2 全黑

```c
active_count == GRAYSCALE_CHANNEL_COUNT
```

可能代表十字路口、黑块、起终点线或传感器阈值异常。

不能把它当作普通“居中”。必须单独标记：

```c
state = LINE_STATE_ALL_BLACK;
```

第一版建议：

```c
turnPPS = 0;
左右轮以较低基础速度直行；
```

或者由主循环状态机决定。不要直接沿用普通 PID 输出。

### 8.3 宽线

例如：

```c
active_count >= 6U
```

但未全黑，可标记：

```c
state = LINE_STATE_WIDE;
```

仍可暂时使用加权位置，但保留状态供后续路口识别。

---

## 9. 位置 EMA 滤波

```c
filtered =
    alpha * raw +
    (1.0f - alpha) * previous_filtered;
```

推荐初值：

```c
position_alpha = 0.4f;
```

建议范围：

```text
0.3 ~ 0.7
```

首次获得有效位置时直接：

```c
filtered_position = raw_position;
```

丢线后重新找到线时，重置滤波器，避免沿用旧状态。

---

## 10. 第一版使用 PD，不使用 I

底层电机已经有速度 PI。循迹层主要负责快速转向和抑制摆动。

第一版初始化：

```c
Ki = 0.0f;
```

推荐离散 PD：

```c
error = 0.0f - filtered_position;

p_term = kp * error;

derivative_raw = error - previous_error;

derivative_filtered =
    derivative_alpha * derivative_raw +
    (1.0f - derivative_alpha) * previous_derivative;

d_term = kd * derivative_filtered;

output = p_term + d_term;
```

这里 10 ms 周期固定，`dt` 吸收到 `Kd` 中。代码注释必须说明：更新周期变化后需要重新调 Kd。

推荐：

```c
derivative_alpha = 0.3f;
```

建议范围：

```text
0.2 ~ 0.6
```

---

## 11. 输出单位必须是 pps

循迹输出：

```text
turnPPS
```

不是 PWM。

例如：

```text
basePPS = 500
turnPPS = 150
```

得到：

```text
leftTarget  = 350 pps
rightTarget = 650 pps
```

最终调用：

```c
SpeedControl_SetTargetPPS(leftTarget, rightTarget);
```

---

## 12. 限幅要求

### 12.1 转向限幅

初始：

```c
max_turn_pps = 400;
```

```c
turnPPS = clamp(turnPPS, -400, 400);
```

### 12.2 基础速度

初次循迹：

```text
400 ~ 600 pps
```

推荐：

```c
basePPS = 500;
```

### 12.3 单轮最大目标

```c
max_target_pps = 1800;
```

### 12.4 第一版禁止单轮反转

普通循迹第一版应限制：

```c
leftTarget  = clamp(leftTarget,  0, max_target_pps);
rightTarget = clamp(rightTarget, 0, max_target_pps);
```

急弯时一侧最多停转，不要突然反转。

---

## 13. 推荐初始参数

```c
LineFollow_Config config = {
    .kp = 100.0f,
    .ki = 0.0f,
    .kd = 40.0f,

    .position_alpha   = 0.4f,
    .derivative_alpha = 0.3f,

    .base_pps       = 500,
    .max_turn_pps   = 400,
    .max_target_pps = 1800
};
```

这些只是安全起点，不是最终参数。

---

## 14. LineFollow_Update 推荐流程

```c
void LineFollow_Update(void)
{
    uint16_t sensors;
    uint8_t activeCount;
    float rawPosition;
    float filteredPosition;
    float error;
    float output;
    int32_t turnPPS;
    int32_t leftTarget;
    int32_t rightTarget;

    if (gEnabled == 0U)
    {
        return;
    }

    sensors = Grayscale_ReadAll();
    activeCount = LineFollow_CountActive(sensors);

    if (activeCount == 0U)
    {
        gState = LINE_STATE_LOST;
        LineFollow_ResetControllerState();
        SpeedControl_SetTargetPPS(0, 0);
        return;
    }

    if (activeCount == GRAYSCALE_CHANNEL_COUNT)
    {
        gState = LINE_STATE_ALL_BLACK;
        gTurnPPS = 0;
        gLeftTargetPPS  = gBasePPS;
        gRightTargetPPS = gBasePPS;
        SpeedControl_SetTargetPPS(gLeftTargetPPS, gRightTargetPPS);
        return;
    }

    gState = (activeCount >= 6U) ?
        LINE_STATE_WIDE : LINE_STATE_NORMAL;

    rawPosition = LineFollow_CalculatePosition(
        sensors,
        activeCount);

    filteredPosition =
        LineFollow_FilterPosition(rawPosition);

    error = 0.0f - filteredPosition;

    output = LineFollow_ComputePD(error);

    turnPPS = LineFollow_FloatToInt(output);
    turnPPS = ClampInt32(
        turnPPS,
        -gMaxTurnPPS,
         gMaxTurnPPS);

    leftTarget  = gBasePPS - turnPPS;
    rightTarget = gBasePPS + turnPPS;

    leftTarget = ClampInt32(
        leftTarget,
        0,
        gMaxTargetPPS);

    rightTarget = ClampInt32(
        rightTarget,
        0,
        gMaxTargetPPS);

    gTurnPPS        = turnPPS;
    gLeftTargetPPS  = leftTarget;
    gRightTargetPPS = rightTarget;

    SpeedControl_SetTargetPPS(
        leftTarget,
        rightTarget);
}
```

如果转向相反，只交换左右加减号。

---

## 15. 更新周期

推荐：

```text
灰度循迹：10 ms，100 Hz
速度 PI：20 ms，50 Hz
```

新增 10 ms 定时器，ISR 只置标志：

```c
volatile uint8_t g_line_update_flag = 0U;

void LINE_TIMER_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(LINE_TIMER_INST))
    {
    case DL_TIMER_IIDX_ZERO:
        g_line_update_flag = 1U;
        break;

    default:
        break;
    }
}
```

主循环：

```c
while (1)
{
    if (g_line_update_flag != 0U)
    {
        g_line_update_flag = 0U;
        LineFollow_Update();
    }

    /* 其他非实时任务 */
}
```

禁止在 ISR 中：

- 串口发送；
- `snprintf()`；
- 延时；
- 阻塞 I/O；
- 大量浮点格式化。

---

## 16. 初始化顺序

```c
int main(void)
{
    __disable_irq();

    SYSCFG_DL_init();

    Motor_Init();
    USART_Init();

    SpeedControl_Init();
    Encoder_Init();

    LineFollow_Config lineConfig = {
        .kp = 100.0f,
        .ki = 0.0f,
        .kd = 40.0f,
        .position_alpha = 0.4f,
        .derivative_alpha = 0.3f,
        .base_pps = 500,
        .max_turn_pps = 400,
        .max_target_pps = 1800
    };

    LineFollow_Init(&lineConfig);

    /* 清除 10 ms 定时器中断标志和 NVIC pending */
    LineFollow_TimerInit();

    __enable_irq();

    LineFollow_Start();

    while (1)
    {
        if (g_line_update_flag != 0U)
        {
            g_line_update_flag = 0U;
            LineFollow_Update();
        }

        /* 低频串口遥测 */
    }
}
```

---

## 17. 串口遥测

每 200 ms 打印一次，不要每 10 ms 打印。

建议内容：

```text
GRAY ACTIVE POSITION ERROR TURN TARGET_L/R SPEED_L/R PWM_L/R STATE
```

避免 `%f`，将浮点乘 1000 转整数：

```c
int32_t pos_milli =
    (int32_t)(LineFollow_GetFilteredPosition() * 1000.0f);

int32_t error_milli =
    (int32_t)(LineFollow_GetError() * 1000.0f);

snprintf(
    txBuf,
    sizeof(txBuf),
    "GRAY=0x%03X ACTIVE=%u POS=%ld ERR=%ld TURN=%ld "
    "TARGET=%ld/%ld SPEED=%lu/%lu PWM=%d/%d STATE=%u\r\n",
    (unsigned)LineFollow_GetSensorBitmap(),
    (unsigned)LineFollow_GetActiveCount(),
    (long)pos_milli,
    (long)error_milli,
    (long)LineFollow_GetTurnPPS(),
    (long)LineFollow_GetLeftTargetPPS(),
    (long)LineFollow_GetRightTargetPPS(),
    (unsigned long)Encoder_GetSpeed(ENCODER_LEFT),
    (unsigned long)Encoder_GetSpeed(ENCODER_RIGHT),
    (int)SpeedControl_GetLeftPWM(),
    (int)SpeedControl_GetRightPWM(),
    (unsigned)LineFollow_GetState());

USART_SendString(txBuf);
```

---

## 18. 调参顺序

### 18.1 先只调 P

初始：

```c
Kp = 80.0f;
Ki = 0.0f;
Kd = 0.0f;
basePPS = 400;
```

逐步测试：

```text
Kp: 80 → 100 → 120 → 150
```

判断：

- 转弯反应弱、冲出线：增大 Kp；
- 左右频繁摆动：Kp 过大，或需要 D；
- 转向方向错误：交换左右目标的加减号。

### 18.2 再加入 D

```text
Kd: 10 → 20 → 40 → 60
```

判断：

- 直线左右摆动：增加 Kd；
- 转弯反应迟钝：Kd 过大或滤波过强；
- 出弯恢复慢：降低 Kd；
- 输出尖峰：适当增加微分滤波。

### 18.3 暂不加入 I

第一版：

```c
Ki = 0.0f;
```

只有确认存在长期固定位置偏差且机械和安装问题已排除后，才考虑很小的 I。

---

## 19. 静态验证

电机不运行时检查：

### 白底

```text
GRAY=0x000
ACTIVE=0
STATE=LOST
```

### 黑线中心

CH4、CH5 为 1：

```text
POSITION ≈ 0
ERROR ≈ 0
TURN ≈ 0
```

### 黑线最左

CH0 为 1：

```text
POSITION ≈ -4.5
ERROR ≈ +4.5
TURN > 0
```

### 黑线最右

CH9 为 1：

```text
POSITION ≈ +4.5
ERROR ≈ -4.5
TURN < 0
```

确认符号正确后再上地测试。

---

## 20. 动态测试顺序

1. 抬起驱动轮，基础速度 300~400 pps。
2. 手动移动黑线，观察左右目标速度变化。
3. 确认转向方向。
4. 放到地面，使用宽缓弯赛道。
5. 基础速度 400 pps。
6. 只使用 P。
7. 调到能跟线但有轻微摆动。
8. 加 D 抑制摆动。
9. 增加到 500、600 pps。
10. 最后测试连续弯和窄弯。

---

## 21. 安全与鲁棒性要求

Claude 生成代码必须满足：

1. 丢线第一版立即设左右目标速度为 0。
2. 循迹模块不直接控制 PWM。
3. 不在任何 ISR 中发送串口。
4. 不在 ISR 中延时。
5. 不修改编码器中断。
6. 不修改速度 PI 核心。
7. 左右目标速度必须限幅。
8. 第一版禁止转向导致单轮反转。
9. `LineFollow_Stop()` 必须：
   - 禁用循迹；
   - 清除控制器状态；
   - 调用 `SpeedControl_SetTargetPPS(0, 0)`。
10. 丢线、全黑、重新找到线时合理重置滤波和微分状态。
11. ISR 与主循环共享标志必须使用 `volatile`。
12. 不使用动态内存。
13. 不使用 RTOS。
14. 兼容 TI Arm Clang 和 MSPM0 DriverLib。
15. 不依赖未提供的外部库。

---

## 22. 建议内部状态变量

```c
static LineFollow_Config gConfig;

static uint8_t gEnabled;

static uint16_t gSensorBitmap;
static uint8_t gActiveCount;
static LineFollow_State gState;

static float gRawPosition;
static float gFilteredPosition;
static float gLastValidPosition;

static float gError;
static float gPreviousError;
static float gIntegral;
static float gDerivativeFiltered;
static float gOutput;

static uint8_t gFilterInitialized;

static int32_t gTurnPPS;
static int32_t gLeftTargetPPS;
static int32_t gRightTargetPPS;
```

若只在主循环访问，不需要全部设为 `volatile`。

定时器标志必须：

```c
volatile uint8_t g_line_update_flag;
```

---

## 23. Claude 应交付的内容

### 23.1 完整新增文件

```text
line_follow.h
line_follow.c
```

必须可直接加入工程。

### 23.2 main.c 集成代码

给出：

- 初始化顺序；
- 10 ms 调度；
- 200 ms 串口遥测；
- 循迹启停示例。

### 23.3 SysConfig 定时器要求

说明新增 10 ms 定时器：

```text
模式：Periodic
周期：10 ms
中断源：ZERO
优先级：低于编码器 GPIO
```

必须使用 SysConfig 自动生成宏：

```c
LINE_TIMER_INST
LINE_TIMER_INST_INT_IRQN
LINE_TIMER_INST_IRQHandler
```

不要硬编码具体 TIMA/TIMG 实例。

### 23.4 参数位置

明确指出：

```text
kp
ki
kd
position_alpha
derivative_alpha
base_pps
max_turn_pps
max_target_pps
```

在哪里修改。

### 23.5 完整代码而非伪代码

必须包含：

- include；
- 结构体；
- static 辅助函数；
- 限幅；
- 初始化；
- 更新；
- 状态查询；
- 类型转换；
- 注释；
- 异常处理。

---

## 24. 第一版验收标准

1. 白底时识别丢线并停车。
2. 黑线居中时左右目标速度接近相同。
3. 黑线左移时正确向左修正。
4. 黑线右移时正确向右修正。
5. 400~600 pps 下能跟随简单闭合赛道。
6. 直线段不持续大幅摆动。
7. 弯道不会明显冲出赛道。
8. 速度 PI 保持正常。
9. 循迹模块不直接控制 PWM。
10. 串口调试不阻塞 10 ms 更新。
11. 全黑状态单独识别。
12. 不存在 12 路遗留数组越界。
13. CH0 始终是最左，CH9 始终是最右。
14. 黑线始终按 1、白底按 0 处理。

---

## 25. 本次暂不实现

- ICM42688 航向闭环；
- 灰度与陀螺仪融合；
- 十字路口决策；
- 环岛识别；
- 起终点识别；
- 丢线搜索；
- 编码器航向同步；
- 动态基础速度；
- 模糊 PID；
- RTOS。

先完成可靠的 10 路灰度 PD 循迹，并接入现有左右轮速度 PI。

---

## 26. 给 Claude 的最终任务描述

请基于当前 MSPM0G3507 天猛星两轮差速小车工程，编写 10 路数字灰度循迹模块。

已知：

1. CH0 最左，CH9 最右。
2. 黑线输出 1，白底输出 0。
3. `Grayscale_ReadAll()` 的 bit0~bit9 对应 CH0~CH9。
4. 已有左右轮速度 PI。
5. 使用 `SpeedControl_SetTargetPPS(left, right)` 设置左右目标速度。
6. 循迹模块禁止直接调用 `Motor_SetSpeed()`。
7. 灰度更新周期 10 ms。
8. 速度 PI 周期 20 ms。
9. 第一版使用 PD，Ki=0。
10. 使用 10 路对称加权位置。
11. 实现位置 EMA 和微分 EMA。
12. 输出 `turnPPS` 并转换为左右目标 pps。
13. 初始参数：
    - Kp = 100
    - Ki = 0
    - Kd = 40
    - basePPS = 500
    - maxTurnPPS = 400
    - positionAlpha = 0.4
    - derivativeAlpha = 0.3
14. 全白丢线停车。
15. 全黑状态单独标记。
16. 第一版禁止单轮反转。
17. 提供完整 `line_follow.c/.h` 和 `main.c` 集成代码。
18. 不在 ISR 中打印串口。
19. 提供状态查询接口用于低频串口调试。
20. 兼容 TI Arm Clang 和 MSPM0 DriverLib。
