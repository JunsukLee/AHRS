#ifndef MPU9150_PI_H
#define MPU9150_PI_H

#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

using namespace std;

class MPU9150
{

public:
    void SendDataToProcessing();
    
public:
    float filtered_angle_x, filtered_angle_y, filtered_angle_z;
    
private:
    void calibAccelGyro();
    void calcAccelYPR();
    void calcGyroYPR();
    void calcFilteredYPR();
    void readAccelGyro();
    void initDT();
    void calcDT();

private:
    int MPU_9150;
    long int ACCEL_X;
    long int ACCEL_Y;
    long int ACCEL_Z;
    long int TEMP_SENSOR;
    int GYRO_X;
    int GYRO_Y;
    int GYRO_Z;

    float gyro_x, gyro_y, gyro_z; //각속도 저장 전역변수 //각속도 : 단위시간당 회전한 각도
    long AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; 
    float accel_angle_x, accel_angle_y, accel_angle_z;
    float gyro_angle_x, gyro_angle_y, gyro_angle_z;    
    float baseAcX, baseAcY, baseAcZ;  //가속도 평균값 저장 변수
    float baseGyX, baseGyY, baseGyZ;  //자이로 평균값 저장 변수

    float dt;
    unsigned long t_now;  //현재 측정 주기 시간
    unsigned long t_prev; //이전 측정 주기 시간
    struct timeval prev_point, now_point;

};

#endif // MPU9150_PI_H
