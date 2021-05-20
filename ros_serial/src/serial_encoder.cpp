/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <string>

#define BUF_SIZE       10

serial::Serial ser;
std_msgs::Int16 encoder;
std_msgs::Int16 InitEnco;
uint8_t     buf[BUF_SIZE];


bool array_checksum(){
  uint32_t sum = 0;
  if(buf[0] != 's') return false;

  for(int i = 1 ; i < BUF_SIZE-3 ; i++){
    sum += buf[i];
  }
  if(((sum%256) == buf[BUF_SIZE-3]) && ((sum%17) == buf[BUF_SIZE-2])){
    printf("Checksum Good!\n");
    return true;
  }
  else {
    printf("Checksum Err!>> buf[6] : %d, sum%256 : %d\n",buf[BUF_SIZE-1], sum%256);
    return false;
  }
}

void passing(){
    if(array_checksum()){
        double theta, setzero;
        theta = (double)buf[2] + buf[3] / 100.;
        if(buf[1] == 1) theta *= (-1);
        theta = theta*3.141592/180.0;
        setzero = (double)buf[5] + buf[6] / 100.;
        if(buf[4] == 1) setzero *= (-1);

        printf("encoder theta >> %.2lf\n", theta);
        printf("setzero >> %.2lf\n", setzero);
        encoder.data =  theta*1000. - InitEnco.data;
    }
}

void InitializeCallBack(const std_msgs::String::ConstPtr &msg){
    if(msg->data == "initialize"){
      InitEnco.data = encoder.data;
    }
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_encoder");
    ros::NodeHandle nh;

    ros::Publisher read_pub = nh.advertise<std_msgs::Int16>("/encoder_theta", 1000);
    ros::Subscriber init_sub = nh.subscribe("/initialize", 1, InitializeCallBack);

    try
    {
        ser.setPort("/dev/ttyACM0");
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

        std_msgs::String test_string;
        test_string.data = "Send test string!";
        ser.write(test_string.data);
    }else{
        return -1;
    }

    ros::Rate loop_rate(1000);
    while(ros::ok()){

        ros::spinOnce();

        if(ser.available()){
            std_msgs::String result;

            ser.read(buf, BUF_SIZE);
            passing();

            read_pub.publish(encoder);
        }
        loop_rate.sleep();
    }
}

