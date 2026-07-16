# 融合工程 `empty.c` 规范化与 IMU 就绪后启动循迹——Claude 修改指导

> 适用工程：MSPM0G3507 两轮差速小车，已经完成电机、编码器速度 PI、10 路灰度循迹和 ICM42688 融合。  
> 本次只规范 `empty.c`，并加入“等待陀螺仪校准完成后才启动循迹”的逻辑。  
> 要求：**尽可能小改，不重写各功能模块，不修改 SysConfig，不新增复杂状态机。**

---

# 1. 本次修改目标

请只围绕以下内容修改 `empty.c`：

1. 整理文件头注释，使其反映当前完整架构；
2. 整理 `#include` 顺序；
3. 删除已经无效、重复或描述历史修改阶段的注释；
4. 删除未使用的变量和宏；
5. 规范初始化顺序；
6. 上电后保持电机目标速度为 0；
7. 主循环持续执行 20 ms IMU 更新，让零偏校准能够完成；
8. `IMU_IsReady()` 返回 1 后，只调用一次 `LineFollow_Start()`；
9. IMU 未就绪时，循迹保持禁用；
10. 不使用阻塞式 `while (!IMU_IsReady())` 等待；
11. 保留现有速度 PI、循迹 PD、IMU任务和定时器结构。

---

# 2. 当前工程控制结构

当前工程包含三个周期任务：

```text
编码器与速度 PI：20 ms
    TIMER_TICK ISR
    → 编码器测速
    → SpeedControl_Update()
    → Motor_SetSpeed()

灰度循迹：10 ms
    LINE_TIMER ISR
    → 只设置 g_line_update_flag
    → 主循环调用 LineFollow_Update()

ICM42688：20 ms
    TIMER_0 ISR
    → 只设置 g_imu_update_flag
    → 主循环调用 IMU_getYawPitchRoll()
```

完整控制层次：

```text
灰度传感器
    ↓
循迹 PD
    ↓
左右轮目标 pps
    ↓
左右轮速度 PI
    ↓
PWM 与电机方向
```

ICM42688当前主要用于：

```text
上电静止校准
姿态角采集
后续航向环反馈
```

本次不建立航向环，只把 IMU作为循迹启动条件。

---

# 3. 为什么不能用阻塞等待

不要写成：

```c
while (IMU_IsReady() == 0U)
{
}
```

也不要写成：

```c
while (IMU_IsReady() == 0U)
{
    delay_ms(10);
}
```

原因是当前陀螺仪校准不是在 `IMU_init()` 内一次性完成的，而是在主循环每 20 ms调用：

```c
IMU_getYawPitchRoll(ypr);
```

时逐步采集约100组数据并完成零偏计算。

如果主程序在进入正常任务循环前只等待：

```c
IMU_IsReady()
```

却没有继续调用 `IMU_getYawPitchRoll()`，校准状态将永远无法从：

```text
IMU_STATE_CALIBRATING
```

变成：

```text
IMU_STATE_READY
```

正确方法是非阻塞门控：

```text
全局中断开启
    ↓
IMU定时器每20 ms置标志
    ↓
主循环执行IMU更新和校准
    ↓
IMU_IsReady()变为1
    ↓
只调用一次LineFollow_Start()
```

---

# 4. 修改范围

原则上只修改：

```text
empty.c
```

不要修改：

```text
motor.c/.h
encoder.c/.h
speed_control.c/.h
grayscale.c/.h
line_follow.c/.h
IMU.c/.h
icm42688.c/.h
I2C_communication.c/.h
delay.c/.h
empty.syscfg
```

本次不重命名定时器实例，不改变周期和中断优先级。

---

# 5. 当前代码中需要清理的内容

## 5.1 文件头注释已经不完整

当前文件头主要描述：

```text
灰度循迹
速度 PI
```

但没有完整描述：

```text
ICM42688 20 ms任务
IMU校准完成后启动循迹
```

应替换为反映当前架构的简洁注释，不要继续保留“第三阶段”“第五阶段”等开发历史说明。

文件注释应描述当前代码做什么，而不是记录过去如何修改。

---

## 5.2 删除没有实际使用的变量

当前 `main()` 中存在：

```c
uint32_t telemetryTick = 0U;
char txBuf[256];
```

如果后续代码没有使用，应删除。

当前还存在：

```c
#define TELEMETRY_INTERVAL 20U
```

但实际打印由：

```c
printDivider
```

控制。

应只保留一套遥测分频方式，避免注释和代码不一致。

建议改成：

```c
#define IMU_TELEMETRY_DIVIDER 10U
```

因为：

```text
10 × 20 ms = 200 ms
```

然后用：

```c
if (telemetryDivider >= IMU_TELEMETRY_DIVIDER)
```

控制打印。

---

## 5.3 删除历史阶段注释

以下类型注释应删除或改写：

```text
修改说明（第三阶段）
第五阶段
原代码……
现改为……
```

保留当前行为说明即可，例如：

```c
/*
 * IMU 20 ms定时器ISR：
 * 只发布更新请求，I2C读取和姿态计算在主循环完成。
 */
```

不要在主文件中保留大段修改历史。

---

# 6. 推荐文件结构

规范后的 `empty.c` 建议按以下顺序组织：

```text
1. 文件头说明
2. include
3. 常量宏
4. 全局任务标志和调试计数
5. UART printf重定向
6. IMU定时器初始化函数
7. IMU定时器ISR
8. main()
```

不要为了本次整理新建大量辅助文件或复杂类结构。

---

# 7. include 顺序

建议整理为：

```c
#include "ti_msp_dl_config.h"

#include "motor.h"
#include "encoder.h"
#include "usart.h"
#include "grayscale.h"
#include "speed_control.h"
#include "line_follow.h"
#include "IMU.h"
#include "I2C_communication.h"

#include <stdint.h>
#include <stdio.h>
```

当前 `empty.c` 没有直接调用 `icm42688.h` 中的接口，可以删除：

```c
#include "icm42688.h"
```

如果 Claude发现工程因间接依赖而出现编译问题，可以暂时保留，但不要随意增加新的头文件。

当前主文件不使用 `delay_ms()`，因此不需要包含：

```c
#include "delay.h"
```

---

# 8. 文件头注释参考

建议把旧文件头替换成：

```c
/**
 * @file  empty.c
 * @brief 两轮小车主程序：速度闭环、灰度循迹与ICM42688任务调度
 *
 * 控制结构：
 *   1. 编码器测速与速度PI：
 *      TIMER_TICK每20 ms在ISR中完成测速和速度PI更新。
 *
 *   2. 灰度循迹：
 *      LINE_TIMER每10 ms只置标志，
 *      主循环调用LineFollow_Update()。
 *
 *   3. ICM42688：
 *      TIMER_0每20 ms只置标志，
 *      主循环完成I2C读取、校准和姿态解算。
 *
 * 启动逻辑：
 *   上电后电机保持停止；
 *   主循环持续更新IMU；
 *   IMU_IsReady()成立后只启动一次灰度循迹。
 *
 * 约束：
 *   - ISR中不打印、不延时、不执行阻塞式I2C；
 *   - 循迹模块不直接控制PWM；
 *   - IMU校准完成前不允许小车起步。
 */
```

---

# 9. 全局变量规范

保留现有 IMU调度变量：

```c
static float g_ypr[3] = {0.0f, 0.0f, 0.0f};

volatile uint8_t  g_imu_update_flag  = 0U;
volatile uint32_t g_imu_timer_count  = 0U;
volatile uint32_t g_imu_missed_count = 0U;

static uint32_t g_imu_update_count  = 0U;
static uint32_t g_imu_success_count = 0U;
static uint32_t g_imu_error_count   = 0U;
```

原来的：

```c
float ypr[3];
```

建议改成：

```c
static float g_ypr[3] = {0.0f, 0.0f, 0.0f};
```

原因：

- 只在 `empty.c` 使用；
- 明确属于主文件内部状态；
- 初始化为0；
- 命名与其他全局变量一致。

如果改名会造成不必要的修改，也可以继续使用：

```c
static float ypr[3] = {0.0f, 0.0f, 0.0f};
```

重点是增加 `static` 和显式初始化。

---

# 10. 新增循迹启动状态

在 `main()` 局部变量中增加：

```c
uint8_t lineFollowStarted = 0U;
```

它只负责保证：

```c
LineFollow_Start();
```

最多执行一次。

不要新增复杂车辆状态机。

---

# 11. 初始化顺序

## 11.1 推荐顺序

```c
__disable_irq();

SYSCFG_DL_init();

Motor_Init();
USART_Init();

SpeedControl_Init();
LineFollow_Init(&lineConfig);

IMU_init();

Encoder_Init();
LineFollow_TimerInit();
TimeA1_Init();

LineFollow_Stop();

__enable_irq();
```

## 11.2 顺序说明

### 第一步：关闭全局中断

```c
__disable_irq();
```

避免模块尚未初始化完成时进入 ISR。

### 第二步：初始化 SysConfig资源

```c
SYSCFG_DL_init();
```

完成时钟、GPIO、UART、I²C和定时器的底层配置。

当前 SysConfig中的周期定时器设置为自动启动，因此后续必须在开启全局中断前完成各模块中断侧初始化。

### 第三步：初始化安全输出和串口

```c
Motor_Init();
USART_Init();
```

`Motor_Init()` 应让PWM处于停止输出状态。

串口必须在 `IMU_init()` 之前完成，因为 IMU初始化过程可能打印调试信息。

### 第四步：初始化软件控制器

```c
SpeedControl_Init();
LineFollow_Init(&lineConfig);
```

这两个函数主要初始化软件状态：

- 速度目标默认为0；
- 循迹默认处于禁用状态。

### 第五步：初始化 IMU

```c
IMU_init();
```

该函数完成 ICM42688寄存器配置，并进入：

```text
IMU_STATE_CALIBRATING
```

注意：这一步只是开始校准流程，真正的100点采集仍在主循环中完成。

### 第六步：初始化各中断入口

```c
Encoder_Init();
LineFollow_TimerInit();
TimeA1_Init();
```

这些函数清理 pending并使能相应 NVIC通道。

把它们放在可能包含延时的 `IMU_init()` 之后，可减少初始化期间积累的无意义 pending。

### 第七步：显式保持循迹关闭

```c
LineFollow_Stop();
```

虽然 `LineFollow_Init()` 已把循迹设为禁用，仍建议保留这一行作为安全声明：

```text
上电后左右轮目标速度明确为0
```

不要在此处调用：

```c
LineFollow_Start();
```

### 第八步：开启全局中断

```c
__enable_irq();
```

此后：

- 编码器速度任务开始运行；
- 灰度定时器开始发布更新标志；
- IMU定时器开始发布更新标志；
- 电机目标仍为0；
- 循迹仍处于禁用状态。

---

# 12. 规范 TimeA1_Init()

保持函数名称不变，避免本次大范围重命名。

建议注释改为：

```c
/**
 * @brief 初始化IMU 20 ms定时器的中断侧。
 *
 * SysConfig已经配置并自动启动TIMER_0，
 * 本函数只负责清理pending并使能NVIC。
 */
void TimeA1_Init(void)
{
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}
```

如果当前 DriverLib版本支持并且 Claude能够确认正确的清除接口，可以在清理 NVIC pending前增加 TIMER_0 外设 ZERO标志清理。

不要猜测不存在的 DriverLib函数；应参考当前可正常编译的：

```c
LineFollow_TimerInit()
```

及生成的 DriverLib头文件。

本次不强制修改该函数的API调用，只规范注释即可。

---

# 13. 规范 IMU ISR

保持现有逻辑，只简化注释：

```c
/**
 * @brief IMU 20 ms定时器中断。
 *
 * ISR只发布任务，不执行I2C读取和姿态解算。
 */
void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST))
    {
    case DL_TIMER_IIDX_ZERO:
        g_imu_timer_count++;

        if (g_imu_update_flag != 0U)
        {
            g_imu_missed_count++;
        }

        g_imu_update_flag = 1U;
        break;

    default:
        break;
    }
}
```

不要改变当前已经测试通过的：

```c
DL_TimerG_getPendingInterrupt()
```

不要把 `IMU_getYawPitchRoll()` 放回 ISR。

---

# 14. 主循环正确的任务顺序

建议主循环按以下顺序执行：

```text
1. 处理IMU 20 ms任务
2. 检查IMU是否首次进入READY
3. READY后只启动一次循迹
4. 处理灰度10 ms任务
5. 低频打印调试信息
```

原因是：

- IMU校准必须持续运行；
- 当本次 IMU更新使状态变为 READY时，可以立即打开循迹；
- 灰度标志即使在校准期间持续产生，也只清除、不执行循迹计算。

---

# 15. IMU任务逻辑

保留当前错误统计：

```c
if (g_imu_update_flag != 0U)
{
    g_imu_update_flag = 0U;

    if (IMU_getYawPitchRoll(g_ypr) == 0)
    {
        g_imu_update_count++;
        g_imu_success_count++;
    }
    else
    {
        g_imu_error_count++;
    }
}
```

注意：

- 必须先清标志再调用阻塞式 IMU函数；
- 读取失败时保留上一帧姿态；
- 不因单次错误启动循迹；
- `IMU_STATE_READY` 只能由成功的校准流程产生。

---

# 16. IMU就绪后只启动一次循迹

在 IMU任务之后增加：

```c
if ((lineFollowStarted == 0U) &&
    (IMU_IsReady() != 0U))
{
    /*
     * 清除校准期间积累的循迹调度标志，
     * 并重置循迹动态状态后再启动。
     */
    g_line_update_flag = 0U;
    LineFollow_Reset();
    LineFollow_Start();

    lineFollowStarted = 1U;
}
```

这段代码实现：

```text
IMU未就绪
    → 不启动循迹

IMU第一次就绪
    → 清除旧循迹标志
    → 重置循迹滤波、微分和误差状态
    → 启动循迹
    → 记录已启动

后续循环
    → 不重复调用LineFollow_Start()
```

为什么调用：

```c
LineFollow_Reset();
```

因为在校准等待阶段，LINE_TIMER已经运行了一段时间。虽然循迹尚未启用，但启动前显式重置动态状态更清晰，也不会改变配置参数。

---

# 17. 灰度任务门控

灰度更新建议写成：

```c
if (g_line_update_flag != 0U)
{
    g_line_update_flag = 0U;

    if (lineFollowStarted != 0U)
    {
        LineFollow_Update();
    }
}
```

这样：

- IMU校准期间，灰度标志仍会被及时清除；
- 不会积累一个长期未处理的旧请求；
- 未启动前不会计算循迹，也不会设置非零速度目标；
- 启动后保持原来的10 ms循迹逻辑。

虽然 `LineFollow_Update()` 内部已经检查 `gEnabled`，仍建议在主循环明确做一次门控，让启动逻辑更直观。

不要在校准期间反复调用：

```c
LineFollow_Stop();
```

因为没有必要每10 ms重复清控制器状态和目标速度。

---

# 18. 遥测打印规范

建议把打印频率统一为200 ms：

```c
#define IMU_TELEMETRY_DIVIDER 10U
```

局部变量：

```c
uint8_t telemetryDivider = 0U;
```

每次 IMU任务处理后：

```c
telemetryDivider++;

if (telemetryDivider >= IMU_TELEMETRY_DIVIDER)
{
    telemetryDivider = 0U;

    printf(
        "YPR=%.2f/%.2f/%.2f "
        "IMU_STATE=%u READY=%u "
        "LINE=%u "
        "OFFSET=%.4f/%.4f/%.4f "
        "OK=%lu ERR=%lu MISS=%lu I2C=%d\r\n",
        g_ypr[0],
        g_ypr[1],
        g_ypr[2],
        (unsigned)IMU_GetState(),
        (unsigned)IMU_IsReady(),
        (unsigned)lineFollowStarted,
        IMU_GetGyroOffsetX(),
        IMU_GetGyroOffsetY(),
        IMU_GetGyroOffsetZ(),
        (unsigned long)g_imu_success_count,
        (unsigned long)g_imu_error_count,
        (unsigned long)g_imu_missed_count,
        (int)I2C_GetLastResult());
}
```

这里的：

```text
LINE=0
```

表示等待校准；

```text
LINE=1
```

表示循迹已经启动。

不必继续打印同时含义重复的：

```text
UPDATE
OK
```

如果二者在当前逻辑中总是一起增加，可以保留一个即可。

为了改动小，Claude也可以暂时保留现有全部计数，只需把注释和分频统一。

---

# 19. 推荐的 main()核心参考

下面代码用于说明修改目标。Claude应结合当前文件完整内容输出可编译的 `empty.c`，不要机械复制未使用变量。

```c
int main(void)
{
    LineFollow_Config lineConfig = {
        .kp               = 100.0f,
        .ki               = 0.0f,
        .kd               = 40.0f,
        .position_alpha   = 0.4f,
        .derivative_alpha = 0.3f,
        .base_pps         = 800,
        .max_turn_pps     = 400,
        .max_target_pps   = 1800
    };

    uint8_t lineFollowStarted = 0U;
    uint8_t telemetryDivider  = 0U;

    /*
     * 全部模块完成初始化前禁止响应中断。
     */
    __disable_irq();

    SYSCFG_DL_init();

    Motor_Init();
    USART_Init();

    SpeedControl_Init();
    LineFollow_Init(&lineConfig);

    IMU_init();

    Encoder_Init();
    LineFollow_TimerInit();
    TimeA1_Init();

    /*
     * 上电安全状态：
     * 循迹禁用，左右轮目标速度为0。
     */
    LineFollow_Stop();

    __enable_irq();

    while (1)
    {
        /*
         * 20 ms IMU任务：
         * 持续执行才能完成开机零偏校准。
         */
        if (g_imu_update_flag != 0U)
        {
            g_imu_update_flag = 0U;

            if (IMU_getYawPitchRoll(g_ypr) == 0)
            {
                g_imu_update_count++;
                g_imu_success_count++;
            }
            else
            {
                g_imu_error_count++;
            }

            telemetryDivider++;

            if (telemetryDivider >= IMU_TELEMETRY_DIVIDER)
            {
                telemetryDivider = 0U;

                printf(
                    "YPR=%.2f/%.2f/%.2f "
                    "STATE=%u READY=%u LINE=%u "
                    "OFFSET=%.4f/%.4f/%.4f "
                    "OK=%lu ERR=%lu MISS=%lu I2C=%d\r\n",
                    g_ypr[0],
                    g_ypr[1],
                    g_ypr[2],
                    (unsigned)IMU_GetState(),
                    (unsigned)IMU_IsReady(),
                    (unsigned)lineFollowStarted,
                    IMU_GetGyroOffsetX(),
                    IMU_GetGyroOffsetY(),
                    IMU_GetGyroOffsetZ(),
                    (unsigned long)g_imu_success_count,
                    (unsigned long)g_imu_error_count,
                    (unsigned long)g_imu_missed_count,
                    (int)I2C_GetLastResult());
            }
        }

        /*
         * 启动门控：
         * IMU首次完成校准后才允许循迹。
         */
        if ((lineFollowStarted == 0U) &&
            (IMU_IsReady() != 0U))
        {
            g_line_update_flag = 0U;
            LineFollow_Reset();
            LineFollow_Start();

            lineFollowStarted = 1U;
        }

        /*
         * 10 ms灰度循迹任务。
         */
        if (g_line_update_flag != 0U)
        {
            g_line_update_flag = 0U;

            if (lineFollowStarted != 0U)
            {
                LineFollow_Update();
            }
        }
    }
}
```

---

# 20. 启动逻辑的安全性

上电后：

```text
Motor_Init()
    → PWM为停止状态

SpeedControl_Init()
    → 左右目标pps为0

LineFollow_Init()
    → 循迹使能为0

LineFollow_Stop()
    → 再次明确下发0/0目标速度

IMU_STATE_CALIBRATING
    → 只采集IMU，不启动车轮
```

校准完成时：

```text
IMU_STATE_READY
    → LineFollow_Reset()
    → LineFollow_Start()
    → 下一次灰度标志到达后计算目标速度
```

这样不会在 `LineFollow_Start()` 执行瞬间直接输出旧目标；真正的非零目标由下一次 `LineFollow_Update()` 根据当前灰度数据生成。

---

# 21. IMU初始化失败的处理

如果：

```c
IMU_init();
```

失败，状态应进入：

```text
IMU_STATE_ERROR
```

此时：

```c
IMU_IsReady()
```

始终返回0，因此循迹不会启动，电机继续保持停止。

本次不要新增自动跳过 IMU、强行启动循迹的逻辑。

不要写：

```c
if (等待超时)
{
    LineFollow_Start();
}
```

因为用户明确要求等待陀螺仪初始化结束后才能循迹。

---

# 22. 运行中偶发IMU错误

本次启动门控只要求：

```text
首次校准完成后启动循迹
```

循迹启动后，如果出现单次 I²C错误：

- 记录 `g_imu_error_count`；
- 保留上一帧姿态；
- 不立即关闭循迹。

原因是当前灰度循迹尚未使用 IMU反馈，偶发 IMU错误不应直接让小车急停。

后续建立航向环后，再单独设计：

```text
连续IMU错误次数
航向数据失效
降级到纯灰度循迹或停车
```

本次不要加入该功能。

---

# 23. 不要进行的“大手术”

Claude本次不要：

1. 新建 `app.c/app.h`；
2. 新建复杂车辆状态机；
3. 把全部全局变量搬到结构体；
4. 重写 UART；
5. 重写定时器 ISR；
6. 修改速度 PI；
7. 修改循迹 PD；
8. 修改 IMU校准算法；
9. 修改 I²C；
10. 修改 SysConfig；
11. 改定时器周期；
12. 改中断优先级；
13. 重命名全部函数；
14. 加入航向环；
15. 加入阻塞等待；
16. 使用 `delay_ms()` 等待校准；
17. 在 ISR 中打印或进行浮点解算。

---

# 24. 清理后的注释风格

建议只保留四类注释：

### 模块说明

```c
/* IMU任务调度状态 */
```

### 为什么这样做

```c
/* 先清标志，避免任务执行期间到达的新请求被覆盖。 */
```

### 时间关系

```c
/* 10 × 20 ms = 200 ms */
```

### 安全约束

```c
/* IMU校准完成前保持循迹关闭和目标速度为0。 */
```

删除：

- 重复解释同一件事的多段注释；
- 已经失效的推荐范围；
- “以前代码如何、现在如何”的历史记录；
- 中英文重复但内容不一致的注释；
- 与当前代码不对应的遥测周期说明。

---

# 25. 编译检查

修改后确保：

```text
没有未使用的 telemetryTick
没有未使用的 txBuf
没有未使用的 TELEMETRY_INTERVAL
lineFollowStarted只在main中定义一次
IMU_TELEMETRY_DIVIDER已定义
g_ypr或ypr名称前后一致
没有同时保留旧LineFollow_Start()和新门控Start()
没有遗漏LineFollow_Stop()
没有在__enable_irq()前进入while循环
TIMER_0 ISR名称没有变化
LINE_TIMER ISR仍由line_follow.c提供
TIMER_TICK ISR仍由encoder.c提供
```

尤其检查不要在 `empty.c` 中重复定义：

```c
LINE_TIMER_INST_IRQHandler()
TIMER_TICK_INST_IRQHandler()
GROUP1_IRQHandler()
I2C_1_INST_IRQHandler()
```

这些 ISR 已分别由其他模块提供。

---

# 26. 测试步骤

## 26.1 上电静止测试

上电后小车保持静止。

预期：

```text
READY=0
LINE=0
左右轮不转
```

约2秒后：

```text
READY=1
LINE=1
循迹开始
```

实际校准时间取决于方差条件，不要求严格等于2秒。

---

## 26.2 上电晃动测试

上电后轻微晃动车体。

预期：

```text
READY保持0
LINE保持0
电机不转
```

停止晃动并保持静止后：

```text
READY变1
LINE变1
循迹开始
```

---

## 26.3 IMU断开测试

断开 ICM42688后上电。

预期：

```text
IMU_STATE=ERROR或ERR持续增加
READY=0
LINE=0
电机不启动
程序不死机
```

---

## 26.4 正常循迹测试

IMU完成校准后：

- 直线循迹效果与整理前一致；
- 圆弧循迹效果与整理前一致；
- 左右轮速度 PI正常；
- IMU `MISS` 长时间保持0；
- `LineFollow_Start()`只执行一次。

---

## 26.5 重复上电测试

连续上电测试5次。

每次应满足：

```text
先静止校准
后启动循迹
不会上电立即窜车
```

---

# 27. 本次验收标准

- [ ] 只对 `empty.c` 进行必要修改；
- [ ] 文件头正确描述当前完整架构；
- [ ] include顺序清晰；
- [ ] 删除未使用变量和旧遥测宏；
- [ ] 删除“第三阶段/第五阶段”等历史注释；
- [ ] 初始化期间全局中断保持关闭；
- [ ] `SYSCFG_DL_init()` 最先执行底层初始化；
- [ ] UART在 `IMU_init()` 前初始化；
- [ ] 各NVIC初始化在阻塞式 IMU初始化之后完成；
- [ ] 开启全局中断前调用 `LineFollow_Stop()`；
- [ ] 不再在初始化结束后直接调用 `LineFollow_Start()`；
- [ ] 主循环持续执行 IMU任务；
- [ ] 使用 `IMU_IsReady()` 非阻塞判断；
- [ ] IMU就绪后只调用一次 `LineFollow_Start()`；
- [ ] IMU未就绪时灰度标志只清除，不执行循迹；
- [ ] IMU初始化失败时循迹不会启动；
- [ ] 不使用 `delay_ms()` 或空循环等待校准；
- [ ] 不修改速度 PI、循迹 PD、IMU和I²C核心代码；
- [ ] 整理后循迹效果与原工程一致。

---

# 28. 给 Claude 的最终指令

请根据当前融合后的 `empty.c`，以尽可能小的修改完成主文件规范化。

具体要求：

1. 只修改 `empty.c`；
2. 不修改其他模块和 `.syscfg`；
3. 用当前完整架构替换旧文件头；
4. 整理 include、全局变量、宏、ISR和 main 的分区；
5. 删除未使用的 `telemetryTick`、`txBuf` 和旧遥测宏；
6. 删除“第三阶段”“第五阶段”等历史修改注释；
7. 保留所有已经测试通过的ISR和接口名称；
8. 初始化顺序按“关闭中断→SysConfig→安全输出/UART→软件控制器→IMU→中断入口→LineFollow_Stop→开启中断”整理；
9. 删除初始化后的直接 `LineFollow_Start()`；
10. 在 main 中增加 `uint8_t lineFollowStarted = 0U`；
11. 主循环必须持续处理 `g_imu_update_flag`；
12. 当 `IMU_IsReady()!=0U` 且尚未启动时，依次执行：
    ```c
    g_line_update_flag = 0U;
    LineFollow_Reset();
    LineFollow_Start();
    lineFollowStarted = 1U;
    ```
13. 灰度标志到达后先清除，只在 `lineFollowStarted!=0U` 时调用 `LineFollow_Update()`；
14. 不使用阻塞等待和 `delay_ms()` 等待校准；
15. IMU失败或一直未完成校准时，循迹必须保持关闭；
16. 遥测建议统一为每200 ms一次，并打印 READY和LINE启动状态；
17. 不建立航向环；
18. 不大面积拆分函数或新建状态机；
19. 输出整理后的完整、可编译 `empty.c`；
20. 说明主要修改点，但不要改动用户已经调好的循迹和速度参数。
