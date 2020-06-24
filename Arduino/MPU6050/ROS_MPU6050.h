#ifndef _IMU_MPU6050_H
#define _IMU_MPU6050_H
// #include <I2Cdev.h>
// #include <MPU6050_6Axis_MotionApps_V6_12.h>
#include <MPU6050.h>
#include <sensor_msgs/Imu.h>
#include <ros.h>
#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.0174533
#endif

class ROS_MPU6050 : public MPU6050
{
private:
    // MPU6050 *mpu = new MPU6050;
    bool dmpReady = false;
    uint8_t devStatus;
    uint8_t fifoBuffer[64];
    // ros::NodeHandle *nh;

public:

    Quaternion q;
    VectorInt16 aa;
    VectorInt16 gy;
    VectorInt16 aaReal;
    VectorFloat gravity;
    bool newData = false;

    int init();
    uint8_t dmpUpdateCurrentFIFOPacket();
    uint8_t updateAccelGyro();
    uint8_t getSensorDataRaw(sensor_msgs::Imu &imu_msg, ros::NodeHandle *nh);
    
};
#endif
