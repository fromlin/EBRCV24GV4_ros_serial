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

serial::Serial ser;

// void write_callback(const std_msgs::String::ConstPtr& msg){
//     //ROS_INFO_STREAM("Writing to serial port: " << msg->data);
//     ser.write(msg->data);
//     double encoder = stod(msg->data);
//     printf("Sub >> encoder theta : %.2lf\n", encoder);
// }

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_encoder");
    ros::NodeHandle nh;

    // ros::Subscriber write_sub = nh.subscribe("/encoder_theta", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::Int16>("/encoder_theta", 1000);

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

        std_msgs::String test_string;
        test_string.data = "Send test string!";
        ser.write(test_string.data);
    }else{
        return -1;
    }

    ros::Rate loop_rate(10);
    while(ros::ok()){

        ros::spinOnce();

        if(ser.available()){
            //ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            std_msgs::Int16 encoder;

            result.data = ser.read(ser.available());
            encoder.data = stod(result.data);
            
            //ROS_INFO_STREAM("Read: " << result.data);
            read_pub.publish(encoder);

        }
        loop_rate.sleep();
    }
}

