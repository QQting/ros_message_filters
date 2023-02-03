#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// using namespace message_filters;

class Node
{
 public:
  Node(ros::NodeHandle n)
  {
    nh_ = n;
    nh_.param<std::string>("camera", camera_in_topic, "/port_0/camera/image_raw");
    nh_.param<std::string>("lidar", lidar_in_topic, "/ouster/points");
    sub_1_.subscribe(nh_, camera_in_topic, 1);
    sub_2_.subscribe(nh_, lidar_in_topic, 1);
    sync_.reset(new Sync(MySyncPolicy(10), sub_1_, sub_2_));
    sync_->registerCallback(boost::bind(&Node::callback, this, _1, _2));
  }

  void callback(const sensor_msgs::ImageConstPtr &in1, const sensor_msgs::PointCloud2ConstPtr &in2)
  {
	  ROS_INFO("Synchronization successful");
  }

 private:
  ros::NodeHandle nh_;
  message_filters::Subscriber<sensor_msgs::Image> sub_1_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_2_;
  std::string camera_in_topic;
  std::string lidar_in_topic;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "synchronizer");
  ros::NodeHandle nh("~");
  Node synchronizer(nh);

  ros::spin();
}
