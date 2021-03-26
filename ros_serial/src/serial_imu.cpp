#include "ros/ros.h"
#include "serial/serial.h"
#include "sensor_msgs/Imu.h"
#include "rosgraph_msgs/Clock.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/poll.h>
#include <sys/types.h>
#include <termios.h>
#include <fcntl.h>

#define IMU_BUF_SIZE       26
#define PI           3.141592

serial::Serial            ser;
sensor_msgs::Imu          imu;
uint8_t     buf[IMU_BUF_SIZE];
double   ibuf[IMU_BUF_SIZE/2];



bool array_checksum(){
  uint32_t sum = 0;
  for(int i = 0 ; i < IMU_BUF_SIZE-2 ; i++){
    sum += buf[i];
  }

  for(int i = 0 ; i < IMU_BUF_SIZE-2 ; i+=2){
    if(buf[i] > 127){
      buf[i] = ~buf[i];
      if(buf[i+1] == 0)     buf[i] += 1;
      else                  buf[i+1] = ~buf[i+1] + 1;
      ibuf[i/2] = -1 * (buf[i]*256 + buf[i+1]);
    }
    else{
      ibuf[i/2] = buf[i]*256 + buf[i+1];
    }
  }

  if((buf[IMU_BUF_SIZE-2]*256 + buf[IMU_BUF_SIZE-1]) == sum)
    return true;
  else {
    return false;
  }
}

void passing(){
  if(array_checksum()){
    imu.header.frame_id = "base_link";
    imu.orientation.z = ibuf[2] / 10000.;
    imu.orientation.y = ibuf[3] / 10000.;
    imu.orientation.x = ibuf[4] / 10000.;
    imu.orientation.w = ibuf[5] / 10000.;
    imu.angular_velocity.x = (ibuf[6] / 10.) * PI /180.;
    imu.angular_velocity.y = (ibuf[7] / 10.) * PI /180.;
    imu.angular_velocity.z = (ibuf[8] / 10.) * PI /180.;
    imu.linear_acceleration.x = ibuf[9] / 1000.;
    imu.linear_acceleration.y = ibuf[10] / 1000.;
    imu.linear_acceleration.z = ibuf[11] / 1000.;

    printf("Quaternion:\n%.4f   %4.4f   %4.4f   %4.4f", imu.orientation.z, imu.orientation.y, imu.orientation.x, imu.orientation.w);
    printf("\n----------------\n");
    printf("angular_velocity:\n%4.4f   %4.4f   %4.4f", imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
    printf("\n----------------\n");
    printf("linear_acceleration:\n%4.4f   %4.4f   %4.4f", imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
    printf("\n--------------------------------------------\n");
  }
}

void ClockCallback(const rosgraph_msgs::Clock::ConstPtr &clk){
  imu.header.stamp.sec = clk->clock.sec;
  imu.header.stamp.nsec = clk->clock.nsec;
}


bool Initialization(){
//  uint8_t cmdBuff[8] = {0,};

//  cmdBuff[0] = '<';
//  cmdBuff[1] = 's';
//  cmdBuff[2] = 'o';
//  cmdBuff[3] = 'r';
//  cmdBuff[4] = '1';
//  cmdBuff[5] = '0';
//  cmdBuff[6] = '0';
//  cmdBuff[7] = '>';
//  for(int i = 0; i < 8; i++)
//      printf("%x ", cmdBuff[i]);
//  ser.write(cmdBuff,sizeof(cmdBuff));

//  cmdBuff[0] = '<';
//  cmdBuff[1] = 's';
//  cmdBuff[2] = 't';
//  cmdBuff[3] = 'a';
//  cmdBuff[4] = 'r';
//  cmdBuff[5] = 't';
//  cmdBuff[6] = '>';
//  for(int i = 0; i < 7; i++)
//      printf("%x ", cmdBuff[i]);
//  ser.write(cmdBuff,sizeof(cmdBuff));

  return true;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "serial_imu");
  ros::NodeHandle nh;
  ros::Publisher  ipub = nh.advertise<sensor_msgs::Imu>("/imu_data",100);
  ros::Subscriber cpub = nh.subscribe("/clock", 1, ClockCallback);

  std::string port;
  nh.param<std::string>("port", port, "/dev/ttyUSB0");
  try
  {
      ser.setPort(port);
      ser.setBaudrate(921600);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open();
  }
  catch (serial::IOException& e)
  {
      ROS_ERROR_STREAM("Unable to open port ");
      return -1;
  }

  if(ser.isOpen()){
    ROS_INFO_STREAM("Serial Port initialized");
  }
  else{
      return -1;
  }

  Initialization();

  ros::Rate loop_rate(100);     // 100ms (1Hz -> 1s)
  while(ros::ok()){
    ros::spinOnce();
    if(ser.available())
    {
      ser.read(buf, IMU_BUF_SIZE);
      passing();
      ipub.publish(imu);
    }
    loop_rate.sleep();
  }
}
