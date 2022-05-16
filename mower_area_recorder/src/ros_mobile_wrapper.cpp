#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"
#include <string>


ros::Publisher mobwrapp_pub;

/*
 * We implement one callback per button topic published by ROS Mobile
 */
void RecordingBtnCallback(const std_msgs::Bool Msg)
{
  ROS_INFO("I heard RecordingBtn: [%s]", (Msg.data?"true":"false"));
  int val = Msg.data;
  sensor_msgs::Joy msg;
  msg.buttons.push_back(0);
  msg.buttons.push_back(val);
  msg.buttons.push_back(0);
  msg.buttons.push_back(0);
  msg.buttons.push_back(0);
  mobwrapp_pub.publish(msg);
}

void SetBaseBtnCallback(const std_msgs::Bool Msg)
{
  ROS_INFO("I heard SetBaseBtn: [%s]", (Msg.data?"true":"false"));
  int val = Msg.data;
  sensor_msgs::Joy msg;
  msg.buttons.push_back(0);
  msg.buttons.push_back(0);
  msg.buttons.push_back(val);
  msg.buttons.push_back(0);
  msg.buttons.push_back(0);
  mobwrapp_pub.publish(msg);

}

void CompletedRecordingBtnCallback(const std_msgs::Bool Msg)
{
  ROS_INFO("I heard CompletedRecordingBtn: [%s]", (Msg.data?"true":"false"));
  int val = Msg.data;
  sensor_msgs::Joy msg;
  msg.buttons.push_back(0);
  msg.buttons.push_back(0);
  msg.buttons.push_back(0);
  msg.buttons.push_back(val);
  msg.buttons.push_back(0);
  mobwrapp_pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_mobile_wrapper");
  ros::NodeHandle n;
  
  mobwrapp_pub = n.advertise<sensor_msgs::Joy>("mobwrapp_pub", 100); 

 //We subscribe to topics names as defined in parameters
  std::string TopicName;
  n.getParam("/ros_mobile_wrapper/RecordingBtnTopicName", TopicName);
  ros::Subscriber sub1 = n.subscribe(TopicName, 10, RecordingBtnCallback);
  n.getParam("/ros_mobile_wrapper/SetBaseBtnTopicName", TopicName);
  ros::Subscriber sub2 = n.subscribe(TopicName, 10, SetBaseBtnCallback);
  n.getParam("/ros_mobile_wrapper/CompletedRecordingBtnTopicName", TopicName);
  ros::Subscriber sub3 = n.subscribe(TopicName, 10, CompletedRecordingBtnCallback);

  ros::spin();

  return 0;
}