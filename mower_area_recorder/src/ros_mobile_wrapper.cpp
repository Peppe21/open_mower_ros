#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <string>

#define BOOL_STR(const b) (b?"true":"false")

/*
 * We implement one callback per button topic published by ROS Mobile
 */
void RecordingBtnCallback(const std_msgs::Bool Msg)
{
  ROS_INFO("I heard RecordingBtn: [%s]", BOOL_STR(Msg));
}

void SetBaseBtnCallback(const std_msgs::Bool Msg)
{
  ROS_INFO("I heard SetBaseBtn: [%s]", BOOL_STR(Msg));
}

void CompletedRecordingBtnCallback(const std_msgs::Bool Msg)
{
  ROS_INFO("I heard CompletedRecordingBtn: [%s]", BOOL_STR(Msg));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_mobile_wrapper");
  ros::NodeHandle n;

 //We subscribe to topics names as defined in parameters
  string TopicName;
  n.getParam("recording_btn", TopicName);
  ros::Subscriber sub = n.subscribe(TopicName, 10, RecordingBtnCallback);
  n.getParam("set_base_btn", TopicName);
  ros::Subscriber sub = n.subscribe(TopicName, 10, SetBaseBtnCallback);
  n.getParam("completed_recording_btn", TopicName);
  ros::Subscriber sub = n.subscribe(TopicName, 10, CompletedRecordingBtnCallback);

  ros::spin();

  return 0;
}