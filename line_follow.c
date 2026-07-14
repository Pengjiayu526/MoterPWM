/**
 * @file  line_follow.c
 * @brief 10路灰度传感器循迹模块实现
 *
 * 算法:
 *   - 10路对称加权位置计算
 *   - 位置 EMA 低通滤波 (抑制传感器噪声)
 *   - 离散 PD 控制器 (Ki=0, dt吸收到Kd)
 *   - 微分项 EMA 滤波 (抑制微分噪声放大)
 *   - 输出限幅 + 单轮不反转保护
 *   - 丢线/全黑/宽线状态识别
 *
 * 安全特性:
 *   - 丢线立即停车 (target=0)
 *   - 全黑单独标记, 低速直行
 *   - 第一版禁止单轮反转 (target >= 0)
 *   - 不直接控制 PWM
 *   - 不在 ISR 中打印串口
 *   - 不修改编码器中断和速度 PI 核心
 */

#include "line_follow.h"
#include "grayscale.h"
#include "speed_control.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>

/*===========================================================================
 * 10 路对称加权数组
 *
 * 物理顺序: CH0(最左) ~ CH9(最右)
 * 负值 = 左侧, 正值 = 右侧, 0 = CH4/CH5 中间
 *
 * 示例:
 *   CH4+CH5 为黑 → position ≈ 0     (居中)
 *   CH1+CH2 为黑 → position ≈ -3.0  (偏左)
 *   CH7+CH8 为黑 → position ≈ +3.0  (偏右)
 *===========================================================================*/

static const float gLineWeights[GRAYSCALE_CHANNEL_COUNT] = {
    -4.5f, -3.5f, -2.5f, -1.5f, -0.5f,
     0.5f,  1.5f,  2.5f,  3.5f,  4.5f
};

/*===========================================================================
 * 内部状态变量
 *
 * gConfig 保存配置副本。
 * 以下变量仅在主循环中访问 (非 ISR), 不需要 volatile。
 * g_line_update_flag 在头文件中声明为 extern volatile。
 *===========================================================================*/

static LineFollow_Config gConfig;

static uint8_t  gEnabled;              /**< 循迹使能标志: 0=禁用, 1=启用  */
static uint8_t  gFilterInitialized;    /**< EMA 滤波器是否已用有效值初始化 */

static uint16_t gSensorBitmap;         /**< 最近一次灰度位图快照            */
static uint8_t  gActiveCount;          /**< 最近一次检测到黑线的通道数      */
static LineFollow_State gState;        /**< 当前循迹状态                    */

static float gRawPosition;             /**< 未滤波的加权位置                */
static float gFilteredPosition;        /**< EMA 滤波后的位置                */
static float gLastValidPosition;       /**< 上一次有效位置 (丢线恢复用)     */

static float gError;                   /**< 当前误差 (0 - filteredPosition) */
static float gPreviousError;           /**< 上一次误差 (用于微分计算)       */
static float gIntegral;                /**< 积分项 (第一版保持为0)          */
static float gDerivativeFiltered;       /**< EMA 滤波后的微分项             */
static float gOutput;                  /**< PD 控制器原始输出               */

static int32_t gTurnPPS;               /**< 转向分量 (pps)                  */
static int32_t gLeftTargetPPS;         /**< 左轮目标速度 (pps)              */
static int32_t gRightTargetPPS;        /**< 右轮目标速度 (pps)              */

/*===========================================================================
 * 10ms 定时器标志
 *
 * ISR 只置位, 主循环清除。必须 volatile。
 *===========================================================================*/

volatile uint8_t g_line_update_flag = 0U;

/*===========================================================================
 * 内部工具函数
 *===========================================================================*/

/**
 * @brief 浮点限幅。
 */
static inline float ClampFloat(float val, float min, float max)
{
    if (val < min) { return min; }
    if (val > max) { return max; }
    return val;
}

/**
 * @brief int32_t 限幅。
 */
static inline int32_t ClampInt32(int32_t val, int32_t min, int32_t max)
{
    if (val < min) { return min; }
    if (val > max) { return max; }
    return val;
}

/**
 * @brief 将 float 转为 int32_t (四舍五入)。
 */
static inline int32_t FloatToInt32(float val)
{
    if (val >= 0.0f) {
        return (int32_t)(val + 0.5f);
    } else {
        return (int32_t)(val - 0.5f);
    }
}

/*===========================================================================
 * 内部辅助函数 — 传感器处理
 *===========================================================================*/

/**
 * @brief 统计位图中置1的位数 (即检测到黑线的通道数)。
 * @param bitmap 灰度传感器位图
 * @return 有效通道数 (0 ~ GRAYSCALE_CHANNEL_COUNT)
 */
static uint8_t LineFollow_CountActive(uint16_t bitmap)
{
    uint8_t count = 0U;
    uint8_t i;

    for (i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++)
    {
        if (((bitmap >> i) & 0x01U) != 0U)
        {
            count++;
        }
    }

    return count;
}

/**
 * @brief 计算加权位置。
 *
 * @param bitmap      灰度传感器位图
 * @param activeCount 已统计的有效通道数 (必须 > 0)
 * @return 加权位置 (负=偏左, 正=偏右, 0=居中)
 *
 * 公式: position = Σ(weight[i] × is_black[i]) / activeCount
 */
static float LineFollow_CalculatePosition(uint16_t bitmap, uint8_t activeCount)
{
    float weightedSum = 0.0f;
    uint8_t i;

    for (i = 0U; i < GRAYSCALE_CHANNEL_COUNT; i++)
    {
        if (((bitmap >> i) & 0x01U) != 0U)
        {
            weightedSum += gLineWeights[i];
        }
    }

    return weightedSum / (float)activeCount;
}

/*===========================================================================
 * 内部辅助函数 — 滤波与控制
 *===========================================================================*/

/**
 * @brief 位置 EMA 低通滤波。
 *
 * 公式: filtered = alpha × raw + (1 - alpha) × prevFiltered
 *
 * @param raw      原始加权位置
 * @param alpha    滤波系数 (0.3 ~ 0.7, 越小越平滑但响应越慢)
 * @param prevFiltered 上一次滤波值
 * @return 滤波后的位置
 */
static float LineFollow_FilterPosition(float raw, float alpha,
                                        float prevFiltered)
{
    return alpha * raw + (1.0f - alpha) * prevFiltered;
}

/**
 * @brief 微分项 EMA 低通滤波。
 *
 * 公式: filtered = alpha × raw + (1 - alpha) × prevFiltered
 *
 * @param rawD       原始微分 (error - prevError)
 * @param alpha      滤波系数 (0.2 ~ 0.6)
 * @param prevFiltered 上一次滤波值
 * @return 滤波后的微分
 */
static float LineFollow_FilterDerivative(float rawD, float alpha,
                                          float prevFiltered)
{
    return alpha * rawD + (1.0f - alpha) * prevFiltered;
}

/**
 * @brief 离散 PD 计算。
 *
 * 注意: dt = 10ms 已吸收到 Kd 中。若更新周期改变, 必须重新调 Kd。
 *
 * @param error        当前位置误差 (0 - filteredPosition)
 * @param prevError    上一次误差
 * @param kp           比例系数
 * @param kd           微分系数 (含dt)
 * @param ki           积分系数 (第一版为 0)
 * @param integral     积分项指针 (会被更新)
 * @param derivFiltered 微分滤波值指针 (会被更新)
 * @param derivAlpha   微分滤波系数
 * @param prevDeriv    上一次微分滤波值
 * @return PD 输出值
 */
static float LineFollow_ComputePD(float error, float prevError,
                                   float kp, float kd, float ki,
                                   float *integral,
                                   float *derivFiltered,
                                   float derivAlpha)
{
    float pTerm;
    float dTerm;
    float derivativeRaw;

    /* 比例项 */
    pTerm = kp * error;

    /* 积分项 (第一版 Ki=0, 但保留计算路径) */
    if (ki > 0.0f)
    {
        *integral += ki * error;
        /* 积分限幅: ±max_turn_pps / kp 粗略范围 */
        *integral = ClampFloat(*integral, -400.0f, 400.0f);
    }

    /* 微分项 — 原始微分 + EMA 滤波 */
    derivativeRaw = error - prevError;

    *derivFiltered = LineFollow_FilterDerivative(
        derivativeRaw,
        derivAlpha,
        *derivFiltered);

    dTerm = kd * (*derivFiltered);

    /* 合成输出 */
    return pTerm + (*integral) + dTerm;
}

/**
 * @brief 复位控制器动态状态 (积分/微分/误差历史/滤波器)。
 * @note  保持配置参数和 gEnabled 不变。
 */
static void LineFollow_ResetControllerState(void)
{
    gFilterInitialized   = 0U;
    gError               = 0.0f;
    gPreviousError       = 0.0f;
    gIntegral            = 0.0f;
    gDerivativeFiltered   = 0.0f;
    gOutput              = 0.0f;
    gTurnPPS             = 0;
    gLeftTargetPPS       = 0;
    gRightTargetPPS      = 0;
    gFilteredPosition    = 0.0f;
}

/*===========================================================================
 * 公开接口 — 生命周期
 *===========================================================================*/

void LineFollow_Init(const LineFollow_Config *config)
{
    if (config != ((void *)0))
    {
        gConfig = *config;  /* 复制配置 */
    }

    gEnabled            = 0U;
    gFilterInitialized  = 0U;
    gState              = LINE_STATE_LOST;

    gSensorBitmap       = 0U;
    gActiveCount        = 0U;
    gRawPosition        = 0.0f;
    gFilteredPosition   = 0.0f;
    gLastValidPosition  = 0.0f;

    gError              = 0.0f;
    gPreviousError      = 0.0f;
    gIntegral           = 0.0f;
    gDerivativeFiltered  = 0.0f;
    gOutput             = 0.0f;

    gTurnPPS            = 0;
    gLeftTargetPPS      = 0;
    gRightTargetPPS     = 0;
}

void LineFollow_TimerInit(void)
{
    /*
     * 清除 LINE_TIMER 可能残留的中断标志,
     * 然后清除 NVIC pending, 最后使能 NVIC。
     *
     * SysConfig 已将定时器配置为 Periodic 模式并启动计数,
     * 这里只负责中断侧初始化。
     */
    DL_Timer_clearInterruptStatus(LINE_TIMER_INST,
        DL_TIMER_IIDX_ZERO);

    NVIC_ClearPendingIRQ(LINE_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(LINE_TIMER_INST_INT_IRQN);
}

void LineFollow_Start(void)
{
    gEnabled = 1U;
}

void LineFollow_Stop(void)
{
    gEnabled = 0U;
    LineFollow_ResetControllerState();
    SpeedControl_SetTargetPPS(0, 0);
}

void LineFollow_Reset(void)
{
    LineFollow_ResetControllerState();
}

/*===========================================================================
 * 公开接口 — 主更新函数 (每 10ms 由主循环调用)
 *===========================================================================*/

void LineFollow_Update(void)
{
    uint16_t sensors;
    uint8_t  activeCount;
    float    rawPosition;
    float    filteredPosition;
    float    error;
    float    output;
    int32_t  turnPPS;
    int32_t  leftTarget;
    int32_t  rightTarget;

    /*-----------------------------------------------------------------
     * 1. 检查使能状态
     *-----------------------------------------------------------------*/
    if (gEnabled == 0U)
    {
        return;
    }

    /*-----------------------------------------------------------------
     * 2. 读取灰度传感器位图 (每个周期只读一次)
     *-----------------------------------------------------------------*/
    sensors     = Grayscale_ReadAll();
    activeCount = LineFollow_CountActive(sensors);

    /* 保存快照供查询接口使用 */
    gSensorBitmap = sensors;
    gActiveCount  = activeCount;

    /*-----------------------------------------------------------------
     * 3. 丢线检测: 全白 → 停车
     *-----------------------------------------------------------------*/
    if (activeCount == 0U)
    {
        gState = LINE_STATE_LOST;
        LineFollow_ResetControllerState();
        SpeedControl_SetTargetPPS(0, 0);
        return;
    }

    /*-----------------------------------------------------------------
     * 4. 全黑检测: 10通道全黑 → 标记, 低速直行
     *-----------------------------------------------------------------*/
    if (activeCount == GRAYSCALE_CHANNEL_COUNT)
    {
        gState = LINE_STATE_ALL_BLACK;

        /* 全黑时不宜使用加权位置, 直接直行 */
        gTurnPPS        = 0;
        gLeftTargetPPS  = gConfig.base_pps;
        gRightTargetPPS = gConfig.base_pps;

        /* 复位控制器状态, 避免恢复后沿用旧误差 */
        LineFollow_ResetControllerState();

        SpeedControl_SetTargetPPS(gLeftTargetPPS, gRightTargetPPS);
        return;
    }

    /*-----------------------------------------------------------------
     * 5. 宽线判断 (6~9 通道, 但未全黑)
     *-----------------------------------------------------------------*/
    if (activeCount >= 6U)
    {
        gState = LINE_STATE_WIDE;
    }
    else
    {
        gState = LINE_STATE_NORMAL;
    }

    /*-----------------------------------------------------------------
     * 6. 计算加权位置
     *-----------------------------------------------------------------*/
    rawPosition = LineFollow_CalculatePosition(sensors, activeCount);
    gRawPosition = rawPosition;

    /*-----------------------------------------------------------------
     * 7. 位置 EMA 低通滤波
     *-----------------------------------------------------------------*/
    if (gFilterInitialized == 0U)
    {
        /* 首次获得有效位置或丢线后重新找到线, 直接初始化滤波器 */
        filteredPosition = rawPosition;
        gFilterInitialized = 1U;
    }
    else
    {
        filteredPosition = LineFollow_FilterPosition(
            rawPosition,
            gConfig.position_alpha,
            gFilteredPosition);
    }

    gFilteredPosition = filteredPosition;
    gLastValidPosition = filteredPosition;

    /*-----------------------------------------------------------------
     * 8. 计算误差: error = 目标位置(0) - 当前位置
     *    黑线在左 → position<0 → error>0 → turn>0 → 左转修正
     *    黑线在右 → position>0 → error<0 → turn<0 → 右转修正
     *-----------------------------------------------------------------*/
    error = 0.0f - filteredPosition;
    gError = error;

    /*-----------------------------------------------------------------
     * 9. PD 控制器
     *    注意: dt=10ms 已吸收到 Kd, 更新周期变化后必须重新调 Kd
     *-----------------------------------------------------------------*/
    output = LineFollow_ComputePD(
        error,
        gPreviousError,
        gConfig.kp,
        gConfig.kd,
        gConfig.ki,
        &gIntegral,
        &gDerivativeFiltered,
        gConfig.derivative_alpha);

    gOutput       = output;
    gPreviousError = error;

    /*-----------------------------------------------------------------
     * 10. 输出转为 turnPPS, 限幅
     *-----------------------------------------------------------------*/
    turnPPS = FloatToInt32(output);
    turnPPS = ClampInt32(turnPPS,
                         -gConfig.max_turn_pps,
                          gConfig.max_turn_pps);

    /*-----------------------------------------------------------------
     * 11. 合成左右目标速度
     *     leftTarget  = basePPS - turnPPS
     *     rightTarget = basePPS + turnPPS
     *
     *     正 turnPPS → 左轮慢、右轮快 → 左转 (向黑线靠拢)
     *     如果实车转向相反, 只交换下面两行的加减号,
     *     不要改传感器权重定义。
     *-----------------------------------------------------------------*/
    leftTarget  = gConfig.base_pps - turnPPS;
    rightTarget = gConfig.base_pps + turnPPS;

    /*-----------------------------------------------------------------
     * 12. 单轮限幅: 第一版禁止单轮反转 (0 ~ max_target_pps)
     *-----------------------------------------------------------------*/
    leftTarget  = ClampInt32(leftTarget,  0, gConfig.max_target_pps);
    rightTarget = ClampInt32(rightTarget, 0, gConfig.max_target_pps);

    /*-----------------------------------------------------------------
     * 13. 保存状态并下发目标速度到速度PI层
     *-----------------------------------------------------------------*/
    gTurnPPS        = turnPPS;
    gLeftTargetPPS  = leftTarget;
    gRightTargetPPS = rightTarget;

    SpeedControl_SetTargetPPS(leftTarget, rightTarget);
}

/*===========================================================================
 * 公开接口 — 参数设置
 *===========================================================================*/

void LineFollow_SetBasePPS(int32_t basePPS)
{
    gConfig.base_pps = basePPS;
}

void LineFollow_SetPD(float kp, float kd)
{
    gConfig.kp = kp;
    gConfig.kd = kd;
}

void LineFollow_SetPID(float kp, float ki, float kd)
{
    gConfig.kp = kp;
    gConfig.ki = ki;
    gConfig.kd = kd;
}

void LineFollow_SetMaxTurnPPS(int32_t maxTurnPPS)
{
    gConfig.max_turn_pps = maxTurnPPS;
}

void LineFollow_SetPositionAlpha(float alpha)
{
    gConfig.position_alpha = alpha;
}

void LineFollow_SetDerivativeAlpha(float alpha)
{
    gConfig.derivative_alpha = alpha;
}

/*===========================================================================
 * 公开接口 — 状态查询
 *===========================================================================*/

uint16_t LineFollow_GetSensorBitmap(void)
{
    return gSensorBitmap;
}

uint8_t LineFollow_GetActiveCount(void)
{
    return gActiveCount;
}

float LineFollow_GetRawPosition(void)
{
    return gRawPosition;
}

float LineFollow_GetFilteredPosition(void)
{
    return gFilteredPosition;
}

float LineFollow_GetError(void)
{
    return gError;
}

float LineFollow_GetOutput(void)
{
    return gOutput;
}

float LineFollow_GetIntegral(void)
{
    return gIntegral;
}

int32_t LineFollow_GetTurnPPS(void)
{
    return gTurnPPS;
}

int32_t LineFollow_GetLeftTargetPPS(void)
{
    return gLeftTargetPPS;
}

int32_t LineFollow_GetRightTargetPPS(void)
{
    return gRightTargetPPS;
}

uint8_t LineFollow_IsEnabled(void)
{
    return gEnabled;
}

LineFollow_State LineFollow_GetState(void)
{
    return gState;
}

/*===========================================================================
 * LINE_TIMER 中断服务函数 (每 10ms)
 *
 * ISR 中只置标志位，不做任何:
 *   - 串口发送
 *   - snprintf()
 *   - 延时
 *   - 阻塞 I/O
 *   - 大量浮点运算
 *
 * 实际循迹计算由主循环中的 LineFollow_Update() 完成。
 *===========================================================================*/

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
