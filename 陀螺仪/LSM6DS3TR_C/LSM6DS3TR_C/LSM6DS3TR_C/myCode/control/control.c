#include "control.h"
#include "IIC_Port.h"

#include "stdio.h"
#include "delay.h"

#include "math.h"
#include "lsm6ds3trc.h"

#define Kp 500.0f // 比例增益控制加速度计/磁力计的收敛速率
#define ki 0.005f // 积分增益控制加速度计/磁力计的收敛速率

#define halfT 0.0001f

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;   // quaternion of sensor frame relative to auxiliary frame
float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;    // scaled integral error

float Pitch=0, Roll=0, Yam=0;

void IMU_update(float gx, float gy, float gz, float ax, float ay, float az);
static void EliminateErrors(void);

void setup(void)
{
    delay_init(72);
	IIC_Port_Init();

    IIC_DEVICE *dev = get_IIC_Port_DevHandle(1);
    dev->device_address = LSM6DS3TRC_I2CADDR;

    if(LSM6DS3TRC_Init(LSM6DS3TRC_MODE_I2C) == false)
        printf("LSM6DS3TRC_Init error..\r\n");
    else
        printf("LSM6DS3TRC_Init success..\r\n");
    // delay_ms(1000);

    // LSM6DS3TRC_Set_BDU(1);
}



void loop(void)
{
    float acc[3] = {0};
    float gyr[3] = {0};

    uint8_t status;
    status = LSM6DS3TRC_Get_Status();

    if (status & LSM6DS3TRC_STATUS_ACCELEROMETER)
    {
      LSM6DS3TRC_Get_Acceleration(LSM6DS3TRC_ACC_FSXL_2G, acc);
    //   printf("\r\nacc:X:%2f,\tY:%2f,\tZ:%2f\r\n", acc[0], acc[1], acc[2]);
    //   printf("angle_x %.1f, angle_y %.1f\r\n", 90 - atan2(acc[2],acc[0]) * 57.29577f,  90 - atan2(acc[2],acc[1]) * 57.29577f );
    }
    if (status & LSM6DS3TRC_STATUS_GYROSCOPE)
    {
      LSM6DS3TRC_Get_Gyroscope(LSM6DS3TRC_GYR_FSG_2000, gyr);
        gyr[0] += -879.76;
        gyr[1] += 1627.85;
        gyr[2] += 515.83;

        gyr[0]/=1000;
        gyr[1]/=1000;
        gyr[2]/=1000;

        acc[0]/=1000;
        acc[1]/=1000;
        acc[2]/=1000;

        IMU_update(gyr[0], gyr[1], gyr[2], acc[0], acc[1], acc[2]);  
        // EliminateErrors();
    }

    

    // if (status & LSM6DS3TRC_STATUS_TEMPERATURE)
    // {
    //   printf("\r\ntemp:%2f\r\n", LSM6DS3TRC_Get_Temperature());
    // }

    delay_ms(5);
}

void IMU_update(float gx, float gy, float gz, float ax, float ay, float az)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    // Auxiliary variables to avoid repeated arithmetic
    norm = sqrt(ax * ax + ay * ay + az * az);
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;

    vx = 2 * (q1 * q3 - q0 * q2);
    vy = 2 * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);
    
    exInt = exInt + ex * ki;
    eyInt = eyInt + ey * ki;
    ezInt = ezInt + ez * ki;
    
    gx = gx + Kp * ex + exInt;
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;

    // Apply feedback terms
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

    // Normalise quaternion
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;
    

    // Pitch = asin(2 * q2 * q3 + 2 * q0 * q1)* 57.3;
    // Roll = atan2(-2 * q1 * q3 + 2 * q0 * q2, q0*q0 - q1*q1 - q2*q2 + q3*q3)* 57.3;
    // Pitch = 90 - atan2(az, ax) * 57.29577f;
    // Roll = 90 - atan2(az, ay) * 57.29577f;

    Yam = atan2((q1*q2 - q0*q3), q0*q0-q1*q1+q2*q2-q3*q3)*57.3;

    printf("Pitch:%8.2f,Roll:%8.2f,Yam:%8.2f\r\n", Pitch, Roll, Yam);
}


static void EliminateErrors(void)
{
	static float last_q0 = 1.0f, last_q1 = 0.0f, last_q2 = 0.0f, last_q3 = 0.0f;   // quaternion of sensor frame relative to auxiliary frame
	static float last_exInt = 0.0f, last_eyInt = 0.0f, last_ezInt = 0.0f;    // scaled integral error

	static float  last_Yam=0;

    if(fabs(Yam - last_Yam) < 0.01)
    {
        q0 = last_q0;
        q1 = last_q1;
        q2 = last_q2;
        q3 = last_q3;

        exInt = last_exInt;
        eyInt = last_eyInt;
        ezInt = last_ezInt;
        
        Yam = last_Yam;
    }
    else
    {
        last_q0 = q0;
        last_q1 = q1;
        last_q2 = q2;
        last_q3 = q3;

        last_exInt = exInt;
        last_eyInt = eyInt;
        last_ezInt = ezInt;

        last_Yam = Yam;
    }
}

