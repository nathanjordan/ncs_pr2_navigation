#include <ros/ros.h>
#include <arm_navigation_msgs/CollisionMap.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "empty_collision_map_publisher");
  //figuring out whether robot_description has been remapped

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<arm_navigation_msgs::CollisionMap>("/collision_map_occ", 1, true);

  while(ros::ok()) {
    arm_navigation_msgs::CollisionMap coll;
    coll.header.frame_id = "odom_combined";
    coll.header.stamp = ros::Time::now();
    pub.publish(coll);
  }

  return 0;
}
