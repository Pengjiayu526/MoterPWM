
#include "IMU.h"
#include "icm42688.h"
#include <stdio.h>
/* XYZ�ṹ�� */

/* ���ٶȣ������򱱷���ļ��ٶ��ڼ��ٶȼƵķ��� *//* ���ٶȣ��ɶ���������ļ��ٶ��ڼ��ٶȼƵķ��� */
xyz_f_t north,west;
volatile float exInt, eyInt, ezInt;  // ������
volatile float q0, q1, q2, q3; // ȫ����Ԫ��
volatile float integralFBhand,handdiff;
volatile uint32_t lastUpdate, now; // �������ڼ��� ��λ us
volatile float yaw[5]= {0,0,0,0,0};  //�����������ֵ
int16_t Ax_offset=0,Ay_offset=0;
float TTangles_gyro[7]; //ͮͮ�˲��Ƕ�

float Angle_Final[3];	//X������б�Ƕ�
float Kp = 10.0f;

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);


// Fast inverse square-root
/**************************ʵ�ֺ���********************************************
*����ԭ��:	   float invSqrt(float x)
*��������:	   ���ټ��� 1/Sqrt(x)
��������� Ҫ�����ֵ
��������� ���
*******************************************************************************/
float invSqrt1(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/*
 * 陀螺仪零偏校准相关变量（必须在 IMU_init() 之前声明）
 * ─────────────────────────────────────────────────
 */
static double Gyro_fill[3][300];
static double Gyro_total[3];
static double sqrGyro_total[3];
static int GyroinitFlag = 0;
static int GyroCount = 0;

float gyro_offset[3] = {0};
int CalCount = 0;

/*
 * 第五阶段：校准状态与控制变量
 * ────────────────────────────
 * g_imu_state            : IMU 顶层状态机
 * g_gyro_calibrated      : 0=未校准，1=校准完成（冻结 gyro_offset）
 * g_gyro_calibration_done_count : 校准完成次数（正常最多 1）
 */
static IMU_State g_imu_state                 = IMU_STATE_UNINITIALIZED;
static uint8_t  g_gyro_calibrated            = 0U;
static uint32_t g_gyro_calibration_done_count = 0U;


/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_init(void)
*��������:	  ��ʼ��IMU���
			  ��ʼ������������
			  ��ʼ����Ԫ��
			  ����������
			  ����ϵͳʱ��
�����������
���������û��
*******************************************************************************/
void IMU_init(void)
{
	int axis;
	int sample;

	//while(!ICM_Init());	   //��ʼ��ICM42688����
	if (0x00 == bsp_Icm42688Init())
	{
		//Initial_Timer3();
		// initialize quaternion
		q0 = 1.0f;  //��ʼ����Ԫ��
		q1 = 0.0f;
		q2 = 0.0f;
		q3 = 0.0f;
		exInt = 0.0;
		eyInt = 0.0;
		ezInt = 0.0;

		/*
		 * 第五阶段：开机校准状态初始化
		 * 清零零偏、清空滑动窗口累计和、进入 CALIBRATING 状态。
		 */
		gyro_offset[0] = 0.0f;
		gyro_offset[1] = 0.0f;
		gyro_offset[2] = 0.0f;

		GyroinitFlag = 0;
		GyroCount = 0;
		CalCount = 0;

		for (axis = 0; axis < 3; axis++)
		{
			Gyro_total[axis] = 0.0;
			sqrGyro_total[axis] = 0.0;

			for (sample = 0; sample < 300; sample++)
			{
				Gyro_fill[axis][sample] = 0.0;
			}
		}

		g_gyro_calibrated = 0U;
		g_gyro_calibration_done_count = 0U;
		g_imu_state = IMU_STATE_CALIBRATING;

		return;
	}

	g_imu_state = IMU_STATE_ERROR;
	printf("IMU ERROR!!\r\n");
}

// ������ι�ʽ S^2 = (X1^2 + X2^2 + X3^2 + ... +Xn^2)/n - Xƽ��^2
// ��������: calVariance
// ��������: ���㷽��
// �������: data[] --- ������㷽������ݻ�����
//           length --- ���ݳ���
// �������:                                                                                      */
//           sqrResult[] --- ������
//           avgResult[] --- ƽ����

void calGyroVariance(float data[], int length, float sqrResult[], float avgResult[])
{
	int i;
	double tmplen;
	if (GyroinitFlag == 0)
	{
		for (i = 0; i< 3; i++)
		{
			Gyro_fill[i][GyroCount] = data[i];
			Gyro_total[i] += data[i];
			sqrGyro_total[i] += data[i] * data[i];
			sqrResult[i] = 100;
			avgResult[i] = 0;
		}
	}
	else
	{
		for (i = 0; i< 3; i++)
		{
			Gyro_total[i] -= Gyro_fill[i][GyroCount];
			sqrGyro_total[i] -= Gyro_fill[i][GyroCount] * Gyro_fill[i][GyroCount];
			Gyro_fill[i][GyroCount] = data[i];
			Gyro_total[i] += Gyro_fill[i][GyroCount];
			sqrGyro_total[i] += Gyro_fill[i][GyroCount] * Gyro_fill[i][GyroCount];
		}
	}
	GyroCount++;
	if (GyroCount >= length)
	{
		GyroCount = 0;
		GyroinitFlag = 1;
        /* Kp 在校准完成时统一设置，不在滑动窗口函数中修改。 */
	}
	if (GyroinitFlag == 0)
	{
		return;
	}
	tmplen = length;
	for (i = 0; i< 3; i++)
	{
		avgResult[i] = (float)(Gyro_total[i] / tmplen);
		sqrResult[i] = (float)((sqrGyro_total[i] - Gyro_total[i] * Gyro_total[i] / tmplen) / tmplen);
	}
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_getValues(float * values)
*��������:	 ��ȡ���ٶ� ������ ������ �ĵ�ǰֵ
��������� �������ŵ������׵�ַ
���������û��
*******************************************************************************/
int8_t IMU_getValues(float * values) {
	//int16_t accgyroval[7];
	icm42688RealData_t accval;
	icm42688RealData_t gyroval;

	float sqrResult_gyro[3];
	float avgResult_gyro[3];

	if (values == (float *)0)
	{
		return -1;
	}

	//��ȡ���ٶȺ������ǵĵ�ǰADC
	if (bsp_IcmGetRawData(&accval, &gyroval) != 0)
	{
		/*
		 * 读取失败：
		 * 不更新TTangles_gyro；
		 * 不执行方差计算；
		 * 不更新gyro_offset；
		 * 不修改values。
		 */
		return -1;
	}

    TTangles_gyro[0] =  accval.x;
    TTangles_gyro[1] =  accval.y;
    TTangles_gyro[2] =  accval.z;
	TTangles_gyro[3] =  gyroval.x;
	TTangles_gyro[4] =  gyroval.y;
	TTangles_gyro[5] =  gyroval.z;
	TTangles_gyro[6] =  0;

	/*
	 * 第五阶段：只有尚未完成校准时才执行方差统计和零偏计算。
	 * 校准完成后（g_gyro_calibrated == 1U）本块不再执行，
	 * gyro_offset[] 永久冻结。
	 */
	if (g_gyro_calibrated == 0U)
	{
		calGyroVariance(&TTangles_gyro[3], 100, sqrResult_gyro, avgResult_gyro);

		/*
		 * 保留原有 CalCount 逻辑，
		 * 使校准至少经历约100次有效采样。
		 */
		if (CalCount < 100)
		{
			CalCount++;
		}

		if ((GyroinitFlag != 0) &&
		    (CalCount >= 100) &&
		    (sqrResult_gyro[0] < 0.02f) &&
		    (sqrResult_gyro[1] < 0.02f) &&
		    (sqrResult_gyro[2] < 0.02f))
		{
			gyro_offset[0] = avgResult_gyro[0];
			gyro_offset[1] = avgResult_gyro[1];
			gyro_offset[2] = avgResult_gyro[2];

			/*
			 * 校准完成，只允许发生一次。
			 */
			g_gyro_calibrated = 1U;
			g_gyro_calibration_done_count++;
			g_imu_state = IMU_STATE_READY;

			/*
			 * 清除校准阶段积累的姿态误差。
			 */
			q0 = 1.0f;
			q1 = 0.0f;
			q2 = 0.0f;
			q3 = 0.0f;

			exInt = 0.0f;
			eyInt = 0.0f;
			ezInt = 0.0f;

			/*
			 * 将 AHRS 比例增益切换到原工程运行值。
			 */
			Kp = 0.5f;
		}
	}

    values[0] =  accval.x;
    values[1] =  accval.y;
    values[2] =  accval.z;
	values[3] =  gyroval.x - gyro_offset[0];
	values[4] =  gyroval.y - gyro_offset[1];
	values[5] =  gyroval.z - gyro_offset[2];

	/*
	 * 原程序后续会访问mygetqval[6~8]，
	 * 建议明确设为0，避免未初始化值。
	 */
	values[6] = 0.0f;
	values[7] = 0.0f;
	values[8] = 0.0f;

	return 0;

		//�����Ѿ������̸ĳ��� 1000��ÿ��  32.8 ��Ӧ 1��ÿ��
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_AHRSupdate
*��������:	 ����AHRS ������Ԫ��
��������� ��ǰ�Ĳ���ֵ��
���������û��
*******************************************************************************/
   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.001f   // integral gain governs rate of convergence of gyroscope biases

void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  float norm;
  //float hx, hy, hz, bx, bz;
  float vx, vy, vz;//, wx, wy, wz;
  float ex, ey, ez,halfT;
  float tempq0,tempq1,tempq2,tempq3;

  // �Ȱ���Щ�õõ���ֵ���
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;
////====================================================================================================================================
//	//20160323v0.4.6
//	//�˴�������һЩ�������ʹ�õı���
//	//�����ӳ��������������о�
//	static int s_InitTickCount=0;
//	float an[3]={0,0,0};
//	float Cb2n[3*3]={0};
//

////====================================================================================================================================
//

  halfT = 0.01f;


  norm = invSqrt1(ax*ax + ay*ay + az*az);
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;
  //�ѼӼƵ���ά����ת�ɵ�λ������

  norm = invSqrt1(mx*mx + my*my + mz*mz);
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;

  /*
  ���ǰ���Ԫ������ɡ��������Ҿ����еĵ����е�����Ԫ�ء�
�������Ҿ����ŷ���ǵĶ��壬��������ϵ������������ת����������ϵ��������������Ԫ�ء�
���������vx\y\z����ʵ���ǵ�ǰ��ŷ���ǣ�����Ԫ�����Ļ����������ϵ�ϣ����������������λ������
  */
  // compute reference direction of flux
//  hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
//  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
//  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);
//  bx = sqrt((hx*hx) + (hy*hy));
//  bz = hz;

  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;

  /* ���ٶȣ������򱱷���ļ��ٶ��ڼ��ٶȼ�X���� */
	north.x = 1 - 2*(q3*q3 + q2*q2);
	/* ���ٶȣ������򱱷���ļ��ٶ��ڼ��ٶȼ�Y���� */
	north.y = 2* (-q0*q3 + q1*q2);
	/* ���ٶȣ������򱱷���ļ��ٶ��ڼ��ٶȼ�Z���� */
	north.z = 2* (+q0*q2  - q1*q3);
	/* ���ٶȣ��ɶ���������ļ��ٶ��ڼ��ٶȼ�X���� */
	west.x = 2* (+q0*q3 + q1*q2);
	/* ���ٶȣ��ɶ���������ļ��ٶ��ڼ��ٶȼ�Y���� */
	west.y = 1 - 2*(q3*q3 + q1*q1);
	/* ���ٶȣ��ɶ���������ļ��ٶ��ڼ��ٶȼ�Z���� */
	west.z = 2* (-q0*q1 + q2*q3);
//  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
//  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
//  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy);// + (my*wz - mz*wy);
  ey = (az*vx - ax*vz);// + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx);// + (mx*wy - my*wx);
  /*
  axyz�ǻ����������ϵ�ϣ����ٶȼƲ����������������Ҳ����ʵ�ʲ����������������
axyz�ǲ����õ�������������vxyz�����ݻ��ֺ����̬����������������������Ƕ��ǻ����������ϵ�ϵ�����������
������֮�������������������ݻ��ֺ����̬�ͼӼƲ��������̬֮�����
������������������������Ҳ�������������ˣ�����ʾ��exyz�����������������Ĳ����
�����������Ծ���λ�ڻ�������ϵ�ϵģ������ݻ������Ҳ���ڻ�������ϵ�����Ҳ���Ĵ�С�����ݻ����������ȣ����������������ݡ���������Լ��ö�������һ�£����������ǶԻ���ֱ�ӻ��֣����Զ����ݵľ�������ֱ�������ڶԻ�������ϵ�ľ�����
  */
if(ex != 0.0f && ey != 0.0f && ez != 0.0f){
  exInt = exInt + ex * Ki * halfT;
  eyInt = eyInt + ey * Ki * halfT;
  ezInt = ezInt + ez * Ki * halfT;

  // �ò���������PI����������ƫ
  gx = gx + Kp*ex + exInt;
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;

  }

  // ��Ԫ��΢�ַ���
  tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  // ��Ԫ���淶��
  norm = invSqrt1(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
  q0 = tempq0 * norm;
  q1 = tempq1 * norm;
  q2 = tempq2 * norm;
  q3 = tempq3 * norm;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_getQ(float * q)
*��������:	 ������Ԫ�� ���ص�ǰ����Ԫ����ֵ
��������� ��Ҫ�����Ԫ���������׵�ַ
���������û��
*******************************************************************************/
float mygetqval[9];	//���ڴ�Ŵ�����ת�����������
int8_t IMU_getQ(float * q) {

  if (q == (float *)0)
  {
      return -1;
  }

  if (IMU_getValues(mygetqval) != 0)
  {
      /*
       * 数据无效时不调用IMU_AHRSupdate()，
       * 保留上一帧四元数。
       */
      return -1;
  }

  //�������ǵĲ���ֵת�ɻ���ÿ��
  //���ٶȺʹ����Ʊ��� ADCֵ������Ҫת��
 IMU_AHRSupdate(mygetqval[3] * M_PI/180, mygetqval[4] * M_PI/180, mygetqval[5] * M_PI/180,
   mygetqval[0], mygetqval[1], mygetqval[2], mygetqval[6], mygetqval[7], mygetqval[8]);

  q[0] = q0; //���ص�ǰֵ
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;

  return 0;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_getYawPitchRoll(float * angles)
*��������:	 ������Ԫ�� ���ص�ǰ��������̬����
��������� ��Ҫ�����̬�ǵ������׵�ַ
���������û��
*******************************************************************************/
int8_t IMU_getYawPitchRoll(float * angles) {
  float q[4]; //����Ԫ��
  volatile float gx=0.0, gy=0.0, gz=0.0; //������������

  if (angles == (float *)0)
  {
      return -1;
  }

  if (IMU_getQ(q) != 0)
  {
      /*
       * 不覆盖angles，
       * 调用者仍保留上一帧有效ypr。
       */
      return -1;
  }

  angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw
  angles[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch
  angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll
 // if(angles[0]<0)angles[0]+=360.0f;  //�� -+180��  ת��0-360��

  return 0;
}

 void IMU_TT_getgyro(float * zsjganda)
{
	zsjganda[0] = TTangles_gyro[0];
    zsjganda[1] = TTangles_gyro[1];
    zsjganda[2] = TTangles_gyro[2];
	zsjganda[3] = TTangles_gyro[3];
	zsjganda[4] = TTangles_gyro[4];
	zsjganda[5] = TTangles_gyro[5];
	zsjganda[6] = TTangles_gyro[6];
}

void MPU6050_InitAng_Offset(void)
{

}

/*
 * 第五阶段：校准状态与零偏查询接口
 * ────────────────────────────────
 */

uint8_t IMU_IsReady(void)
{
    return (g_imu_state == IMU_STATE_READY) ? 1U : 0U;
}

IMU_State IMU_GetState(void)
{
    return g_imu_state;
}

float IMU_GetGyroOffsetX(void)
{
    return gyro_offset[0];
}

float IMU_GetGyroOffsetY(void)
{
    return gyro_offset[1];
}

float IMU_GetGyroOffsetZ(void)
{
    return gyro_offset[2];
}

uint32_t IMU_GetCalibrationDoneCount(void)
{
    return g_gyro_calibration_done_count;
}

//------------------End of File----------------------------
