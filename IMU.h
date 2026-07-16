#ifndef __IMU_H
#define __IMU_H

#include "ti_msp_dl_config.h"
#include <math.h>
#undef M_PI
#define M_PI  (float)3.1415926535
typedef struct
{
    float x;
    float y;
    float z;
} xyz_f_t;

/* IMU 校准状态 */
typedef enum
{
    IMU_STATE_UNINITIALIZED = 0,
    IMU_STATE_CALIBRATING,
    IMU_STATE_READY,
    IMU_STATE_ERROR

} IMU_State;

extern xyz_f_t north,west;
extern volatile float yaw[5];   //处理航向的增值
extern float motion6[7];
//Mini IMU AHRS 解算的API
void IMU_init(void); //初始化
int8_t IMU_getYawPitchRoll(float * ypr); //更新姿态
void IMU_TT_getgyro(float * zsjganda);

/* 校准状态查询接口 */
uint8_t IMU_IsReady(void);
IMU_State IMU_GetState(void);

/* 零偏查询接口 */
float IMU_GetGyroOffsetX(void);
float IMU_GetGyroOffsetY(void);
float IMU_GetGyroOffsetZ(void);

/* 校准完成次数查询（用于验证冻结） */
uint32_t IMU_GetCalibrationDoneCount(void);

//uint32_t micros(void);	//读取系统上电后的时间  单位 us
void MPU6050_InitAng_Offset(void);
#endif

//------------------End of File----------------------------
