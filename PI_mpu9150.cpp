#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

using namespace std;


void calibAccelGyro();
void calcAccelYPR();
void calcGyroYPR();
void calcFilteredYPR();
void readAccelGyro();
void initDT();
void calcDT();
void SendDataToProcessing();

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
float filtered_angle_x, filtered_angle_y, filtered_angle_z;
float baseAcX, baseAcY, baseAcZ;  //가속도 평균값 저장 변수
float baseGyX, baseGyY, baseGyZ;  //자이로 평균값 저장 변수

float dt;
unsigned long t_now;  //현재 측정 주기 시간
unsigned long t_prev; //이전 측정 주기 시간
struct timeval prev_point, now_point;

int main(void)
{
	MPU_9150 = wiringPiI2CSetup(0x68); // sensor Address
	wiringPiI2CWriteReg8(MPU_9150, 0x6B, 0); // PWR_MGMT_1

	calibAccelGyro();
	initDT();
	
	while(1)
	{
		readAccelGyro(); //가속도, 자이로 센서 값 읽어드림
		calcDT(); //측정 주기 시간 계산
		calcAccelYPR(); // 가속도 센서 처리 루틴
		calcGyroYPR(); //자이로 센서 처리 루틴
		calcFilteredYPR();

		static int cnt;
		cnt++;
		if(cnt%2 == 0)
			SendDataToProcessing(); //프로세싱으로 값 전달
			//측정 주기 시간이 짝수(2ms 단위로 하기 위해서)이면 프로세싱으로 보낸다.
	}

	return 0;
}

void calibAccelGyro(){
	float sumAcX = 0, sumAcY = 0, sumAcZ = 0;
	float sumGyX = 0, sumGyY = 0, sumGyZ = 0;

	readAccelGyro(); //가속도 자이로 센서 읽어들임

	//평균값 구하기
	for(int i=0; i<10; i++){
		readAccelGyro();
		sumAcX += AcX; sumAcY += AcY; sumAcZ += AcZ;
		sumGyX += GyX; sumGyY += GyY; sumGyZ += GyZ;
		usleep(10000);
	}
	baseAcX = sumAcX / 10; baseAcY = sumAcY / 10; baseAcZ = sumAcZ / 10;
	baseGyX = sumGyX / 10; baseGyY = sumGyY / 10; baseGyZ = sumGyZ / 10;
}


void calcAccelYPR(){
	float accel_x, accel_y, accel_z; //가속도 센서의 최종적인 보정값!!!
	float accel_xz, accel_yz;
	const float RADIANS_TO_DEGREES = 180/3.14159;

	accel_x = AcX - baseAcX; // 가속도(직선) X축에 대한 현재 값 - 가속도 센서의 평균값
	accel_y = AcY - baseAcY;
	accel_z = AcZ + (16384 - baseAcZ);

	//직석 +X축이 기울어진 각도 구함
	accel_yz = sqrt(pow(accel_y, 2) + pow(accel_z, 2));
	accel_angle_y = atan(-accel_x / accel_yz)*RADIANS_TO_DEGREES;

	accel_xz = sqrt(pow(accel_x, 2) + pow(accel_z, 2));
	accel_angle_x = atan(accel_y / accel_xz)*RADIANS_TO_DEGREES;

	accel_angle_z = 0;
}




void calcGyroYPR(){
	const float GYROXYZ_TO_DEGREES_PER_SED = 131; //131 값은 1초동안 1도 돌때 자이로 값이 131이다.

	gyro_x = (GyX - baseGyX) / GYROXYZ_TO_DEGREES_PER_SED;
	gyro_y = (GyY - baseGyY) / GYROXYZ_TO_DEGREES_PER_SED;
	gyro_z = (GyZ - baseGyZ) / GYROXYZ_TO_DEGREES_PER_SED;

	gyro_angle_x += gyro_x * dt; //변화된 각 : 각속도 x 측정 주기 시간
	gyro_angle_y += gyro_y * dt;
	gyro_angle_z += gyro_z * dt;
}




void calcFilteredYPR(){
	const float ALPHA = 0.96;
	float tmp_angle_x, tmp_angle_y, tmp_angle_z;  //이전 필터 각도(prev)

	tmp_angle_x = filtered_angle_x + gyro_x * dt; //자이로 각도 = 각속도 x 센서 입력 주기
                                                //각속도 = 단위시간당 회전한 각도 -> 회전한 각도 / 단위시간
	tmp_angle_y = filtered_angle_y + gyro_y * dt;
	tmp_angle_z = filtered_angle_z + gyro_z * dt;

	filtered_angle_x = ALPHA * tmp_angle_x + (1.0 - ALPHA) * accel_angle_x;
	filtered_angle_y = ALPHA * tmp_angle_y + (1.0 - ALPHA) * accel_angle_y;
	filtered_angle_z = tmp_angle_z; //곡선 +Z 축은 자이로 센서만 이용해서 나타내고 있음(가속도 센서는 안함)
	//그래서 위에(calcAccelYPR) 보게 되면 accel_angle_z 는 0이다.
}

void readAccelGyro(){
	ACCEL_X = wiringPiI2CReadReg8(MPU_9150, (0x3B));
	ACCEL_X = ACCEL_X << 8 | wiringPiI2CReadReg8(MPU_9150, 0x3C);

	ACCEL_Y = wiringPiI2CReadReg8(MPU_9150, (0x3D));
	ACCEL_Y = ACCEL_Y << 8 | wiringPiI2CReadReg8(MPU_9150, 0x3E);

	ACCEL_Z = wiringPiI2CReadReg8(MPU_9150, (0x3F));
	ACCEL_Z = ACCEL_Z << 8 | wiringPiI2CReadReg8(MPU_9150, 0x40);

	GYRO_X = wiringPiI2CReadReg8(MPU_9150, 0x43);
	GYRO_X = GYRO_X << 8 | wiringPiI2CReadReg8(MPU_9150, 0x44);

	GYRO_Y = wiringPiI2CReadReg8(MPU_9150, 0x45);
	GYRO_Y = GYRO_Y << 8 | wiringPiI2CReadReg8(MPU_9150, 0x46);

	GYRO_Z = wiringPiI2CReadReg8(MPU_9150, 0x47);
	GYRO_Z = GYRO_Z << 8 | wiringPiI2CReadReg8(MPU_9150, 0x48);

	AcX = ACCEL_X;
    AcY = ACCEL_Y;
    AcZ = ACCEL_Z;
    //Tmp = Wire.read() << 8 | Wire.read();
    GyX = GYRO_X;
    GyY = GYRO_Y;
    GyZ = GYRO_Z;
}

void initDT(){
	gettimeofday(&prev_point, NULL);
}

void calcDT(){
	gettimeofday(&now_point, NULL);
	dt = (double)(now_point.tv_sec)+(double)(now_point.tv_usec)/1000000.0-(double)(prev_point.tv_sec)-(double)(prev_point.tv_usec)/1000000.0;

	prev_point.tv_sec = now_point.tv_sec;
	prev_point.tv_usec = now_point.tv_usec;

}

void SendDataToProcessing(){
	std::cout << "dt : " << dt << std::endl;
	//printf("accel_angle_x : %f\n", accel_angle_x);
	//printf("accel_angle_y : %f\n", accel_angle_y);
	//printf("accel_angle_z : %f\n", accel_angle_z);

	//printf("gyro_angle_x  : %f\n", gyro_angle_x);
	//printf("gyro_angle_y  : %f\n", gyro_angle_y);
	//printf("gyro_angle_z  : %f\n", gyro_angle_z);

	printf("filtered_angle_x : %f\n", filtered_angle_x);
	printf("filtered_angle_y : %f\n", filtered_angle_y);
	printf("filtered_angle_z : %f\n", filtered_angle_z);

	printf("\n\n");
}
