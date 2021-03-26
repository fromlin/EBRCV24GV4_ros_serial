#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
//#include <std_msgs/Empty.h>
#include <string>

#define ENCODER_BUF_SIZE       12

serial::Serial               ser;
std_msgs::String          result;
std_msgs::Float64        encoder;
uint8_t    buf[ENCODER_BUF_SIZE];
double    ebuf[ENCODER_BUF_SIZE];

bool array_checksum(){
  uint16_t sum = 0;
  for(int i = 0 ; i < ENCODER_BUF_SIZE-2 ; i++){
    sum += buf[i];
  }

  if((buf[ENCODER_BUF_SIZE-1]) == sum)
    return true;
  else {
    return false;
  }
}


int main (int argc, char** argv){
    ros::init(argc, argv, "serial_encoder");
    ros::NodeHandle nh;
    ros::Publisher epub = nh.advertise<std_msgs::Float64>("/encoder_theta", 100);

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
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
    }else{
        return -1;
    }

    ros::Rate loop_rate(100);   //100ms
    while(ros::ok()){
        ros::spinOnce();
        if(ser.available()){
            result.data = ser.read(ser.available());
            encoder.data = stod(result.data);
            epub.publish(encoder);
        }
        loop_rate.sleep();
    }
}

