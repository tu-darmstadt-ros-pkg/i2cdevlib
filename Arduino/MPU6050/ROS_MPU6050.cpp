#include <ROS_MPU6050.h>
// #include <MPU6050_6Axis_MotionApps_V6_12.h>


int ROS_MPU6050::init(){
    initialize();
    setFullScaleAccelRange(MPU6050_ACCEL_FS_4);//Accel Range +-g
    setFullScaleGyroRange(MPU6050_GYRO_FS_500);//GYro Range +-deg/s
    devStatus = dmpInitialize();
    setXAccelOffset(-1176);
    setYAccelOffset(-2291);
    setZAccelOffset(1301);
    setXGyroOffset(50);
    setYGyroOffset(-59);
    setZGyroOffset(-1);
    if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our    
    CalibrateAccel(6);
    CalibrateGyro(6);

    // PrintActiveOffsets();

    setDMPEnabled(true);
    dmpReady = true;
  } else {

  }
  return 0;
}

uint8_t ROS_MPU6050::dmpUpdateCurrentFIFOPacket(){
    return dmpGetCurrentFIFOPacket(fifoBuffer);
}

uint8_t ROS_MPU6050::updateAccelGyro(){
    dmpGetQuaternion(&q, fifoBuffer);
    dmpGetGyro(&gy, fifoBuffer);
    dmpGetAccel(&aa, fifoBuffer);
    dmpGetGravity(&gravity, &q);
    dmpGetLinearAccel(&aaReal, &aa, &gravity);
    return 0;
}

uint8_t ROS_MPU6050::getSensorDataRaw(sensor_msgs::Imu &imu_msg, ros::NodeHandle *nh){
    if(dmpUpdateCurrentFIFOPacket()){
        updateAccelGyro();
        imu_msg.header.stamp = nh->now();
        imu_msg.orientation.w = q.w;
        imu_msg.orientation.x = q.x;
        imu_msg.orientation.y = q.y;
        imu_msg.orientation.z = q.z;
        imu_msg.linear_acceleration.x = aaReal.x;
        imu_msg.linear_acceleration.y = aaReal.y;
        imu_msg.linear_acceleration.z = aaReal.z;
        imu_msg.angular_velocity.x = gy.x*DEG_TO_RAD*500/0x3FFF;
        imu_msg.angular_velocity.y = gy.y*DEG_TO_RAD*500/0x3FFF;
        imu_msg.angular_velocity.z = gy.z*DEG_TO_RAD*500/0x3FFF;
	newData = true;
        return 1;
    }
    else
    {
	newData=false;
        return 0;
    }
    
}
