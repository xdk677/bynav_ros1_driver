#include <bynav_ros_driver/oem7_message_handler_if.hpp>


#include <ros/ros.h>

#include <bynav_ros_driver/oem7_ros_messages.hpp>
#include <novatel_oem7_msgs/RANGECMP.h>

#include <oem7_ros_publisher.hpp>


namespace bynav_ros_driver
{

  class RANGECMPHandler: public Oem7MessageHandlerIf
  {
    Oem7RosPublisher rangecmp_pub_;

    void publishRANGECMP(
        Oem7RawMessageIf::ConstPtr msg)
    {
      boost::shared_ptr<novatel_oem7_msgs::RANGECMP> rangecmp;
      MakeROSMessage(msg, rangecmp);
      rangecmp_pub_.publish(rangecmp);
    }

  public:
    RANGECMPHandler()
    {
    }

    ~RANGECMPHandler()
    {
    }

    void initialize(ros::NodeHandle& nh)
    {
      rangecmp_pub_.setup<novatel_oem7_msgs::RANGECMP>("RANGECMP", nh);
    }

    const std::vector<int>& getMessageIds()
    {
      static const std::vector<int> MSG_IDS({RANGECMP_OEM7_MSGID});
      return MSG_IDS;
    }

    void handleMsg(Oem7RawMessageIf::ConstPtr msg)
    {
      publishRANGECMP(msg);
    }
  };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(bynav_ros_driver::RANGECMPHandler, bynav_ros_driver::Oem7MessageHandlerIf)
