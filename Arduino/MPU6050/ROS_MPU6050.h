#ifndef _IMU_MPU6050_H
#define _IMU_MPU6050_H
// #include <I2Cdev.h>
// #include <MPU6050_6Axis_MotionApps_V6_12.h>
#include <MPU6050.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros.h>

#ifndef __IMXRT1062__
    #include <elapsedMillis.h>
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.0174533
#endif

#ifndef GRAVITY
#define GRAVITY 9.81
#endif


class ROS_MPU6050 : public MPU6050
{
private:
    // MPU6050 *mpu = new MPU6050;
    bool dmpReady = false;
    uint8_t devStatus;
    uint8_t fifoBuffer[64];
    elapsedMicros mpu6050_dtimer_us;
    uint8_t startup = 0;
    // ros::NodeHandle *nh;

public:

    Quaternion q;
    VectorInt16 aa;
    VectorInt16 gy;
    VectorInt16 aaReal;
    VectorFloat gravity;

    VectorFloat velBody;

    bool newData = false;

    int init(int16_t xAccOff, int16_t yAccOff, int16_t zAccOff, int16_t xGyOff, int16_t yGyOff, int16_t zGyOff);
    int init();
    uint8_t dmpUpdateCurrentFIFOPacket();
    uint8_t updateAccelGyro();
    uint8_t getSensorDataImuVel(sensor_msgs::Imu &imu_msg,geometry_msgs::TwistStamped &vel_msg, ros::NodeHandle *nh);
    uint8_t getSensorDataImu(sensor_msgs::Imu &imu_msg, ros::NodeHandle *nh);
    uint8_t updateLinearVelocity();
    void correctLinearVelocity(float x_vel_corr, float y_vel_corr);
    
};
#endif
