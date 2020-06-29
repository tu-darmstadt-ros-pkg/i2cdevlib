#include <ROS_MPU6050.h>
// #include <MPU6050_6Axis_MotionApps_V6_12.h>


int ROS_MPU6050::init(int16_t xAccOff, int16_t yAccOff, int16_t zAccOff, int16_t xGyOff, int16_t yGyOff, int16_t zGyOff){
    initialize();
    setFullScaleAccelRange(MPU6050_ACCEL_FS_4);//Accel Range +-g
    setFullScaleGyroRange(MPU6050_GYRO_FS_500);//GYro Range +-deg/s
    devStatus = dmpInitialize();
    setXAccelOffset(xAccOff);
    setYAccelOffset(yAccOff);
    setZAccelOffset(zAccOff);
    setXGyroOffset(xGyOff);
    setYGyroOffset(yGyOff);
    setZGyroOffset(zGyOff);
    if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our    
    CalibrateAccel(6);
    CalibrateGyro(6);

    // PrintActiveOffsets();

    setDMPEnabled(true);
    dmpReady = true;
    mpu6050_dtimer_us=0;
  } else {

  }
  return 0;
}

int ROS_MPU6050::init(){
    initialize();
    setFullScaleAccelRange(MPU6050_ACCEL_FS_2);//Accel Range +-2g
    setFullScaleGyroRange(MPU6050_GYRO_FS_500);//GYro Range +-500deg/s
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
    mpu6050_dtimer_us=0;
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

uint8_t ROS_MPU6050::getSensorDataImuVel(sensor_msgs::Imu &imu_msg,geometry_msgs::TwistStamped &vel_msg, ros::NodeHandle *nh){
    if(dmpUpdateCurrentFIFOPacket()){
        updateAccelGyro();
        updateLinearVelocity();
        imu_msg.header.stamp = nh->now();
        imu_msg.orientation.w = q.w;
        imu_msg.orientation.x = q.x;
        imu_msg.orientation.y = q.y;
        imu_msg.orientation.z = q.z;
        imu_msg.linear_acceleration.x = aaReal.x*GRAVITY*2/0x7FFF;
        imu_msg.linear_acceleration.y = aaReal.y*GRAVITY*2/0x7FFF;
        imu_msg.linear_acceleration.z = aaReal.z*GRAVITY*2/0x7FFF;//aaReal.z;
        imu_msg.angular_velocity.x = gy.x*DEG_TO_RAD*500/0x7FFF;
        imu_msg.angular_velocity.y = gy.y*DEG_TO_RAD*500/0x7FFF;
        imu_msg.angular_velocity.z = gy.z*DEG_TO_RAD*500/0x7FFF;

        vel_msg.header = imu_msg.header;
        vel_msg.twist.linear.x = velBody.x;
        vel_msg.twist.linear.y = velBody.y;
        vel_msg.twist.angular = imu_msg.angular_velocity;

	newData = true;
        return 1;
    }
    else
    {
	newData=false;
        return 0;
    }
    
}

uint8_t ROS_MPU6050::getSensorDataImu(sensor_msgs::Imu &imu_msg, ros::NodeHandle *nh){
    if(dmpUpdateCurrentFIFOPacket()){
        updateAccelGyro();
        imu_msg.header.stamp = nh->now();
        imu_msg.orientation.w = q.w;
        imu_msg.orientation.x = q.x;
        imu_msg.orientation.y = q.y;
        imu_msg.orientation.z = q.z;
        imu_msg.linear_acceleration.x = aaReal.x*GRAVITY*2/0x7FFF;
        imu_msg.linear_acceleration.y = aaReal.y*GRAVITY*2/0x7FFF;
        imu_msg.linear_acceleration.z = aaReal.z*GRAVITY*2/0x7FFF;//aaReal.z;
        imu_msg.angular_velocity.x = gy.x*DEG_TO_RAD*500/0x7FFF;
        imu_msg.angular_velocity.y = gy.y*DEG_TO_RAD*500/0x7FFF;
        imu_msg.angular_velocity.z = gy.z*DEG_TO_RAD*500/0x7FFF;

	    newData = true;
        return 1;
    }
    else
    {
	newData=false;
        return 0;
    }
    
}

uint8_t ROS_MPU6050::updateLinearVelocity(){

    uint16_t dt;

    if(millis()<=60000){
        velBody.x=0;
        velBody.y=0;
        mpu6050_dtimer_us = 0;
    }
    else{
        dt = mpu6050_dtimer_us;

        velBody.x += aaReal.x*GRAVITY*2/0x7FFF*dt*1e-6;
        velBody.y += aaReal.y*GRAVITY*2/0x7FFF*dt*1e-6;

        mpu6050_dtimer_us = 0;
    }
    return 0;
}

void ROS_MPU6050::correctLinearVelocity(float x_vel_corr, float y_vel_corr){
    velBody.x = x_vel_corr;
    velBody.y = y_vel_corr;
}
