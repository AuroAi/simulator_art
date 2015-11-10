#include <string>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

namespace
{
  ros::Subscriber subLaserScan_;
  ros::Publisher pubPointCloud_;
  pcl::PointCloud<pcl::PointXYZI> pc_;
}

void processLaserScan(const sensor_msgs::LaserScan &laserScan) 
{
  // Pass the header information along (including frame ID)
  pcl_conversions::toPCL(laserScan.header,pc_.header);

  int maxPoints = ceil((laserScan.angle_max - laserScan.angle_min)
                       / laserScan.angle_increment) + 1;
  pc_.points.resize(maxPoints);

  float currentAngle = laserScan.angle_min;
  int angleIndex = 0;
  int numPoints = 0;
  while(currentAngle <= laserScan.angle_max) {

    float distance = laserScan.ranges[angleIndex];
    
    // See if an obstacle was detected
    if (distance > laserScan.range_min && distance < laserScan.range_max) { 
      float xPt = cosf(currentAngle) * distance;
      float yPt = sinf(currentAngle) * distance;

      pc_.points[numPoints].x = xPt;
      pc_.points[numPoints].y = yPt;
      pc_.points[numPoints].z = 0.25;
      pc_.points[numPoints].intensity = laserScan.intensities[angleIndex];
      numPoints++;
    }

    angleIndex++;
    currentAngle += laserScan.angle_increment;
  }

  pc_.points.resize(numPoints);
  
  pcl::PCLPointCloud2 pcl_pc2; 
  sensor_msgs::PointCloud2 ros_output;
  pcl::toPCLPointCloud2(pc_, pcl_pc2);
  pcl_conversions::fromPCL(pcl_pc2, ros_output);
  pubPointCloud_.publish(ros_output);
}

int main(int argc, char *argv[]) 
{

  ros::init(argc, argv, "art_simulated_obstacles");
  ros::NodeHandle node;
  
  // Subscribers
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
  subLaserScan_ =
      node.subscribe("front_sick", 10, &processLaserScan, noDelay);
  
  // Publishers
  pubPointCloud_ =
    node.advertise<sensor_msgs::PointCloud2>("velodyne_obstacles", 10);

  ros::spin();                          // handle incoming data
 
  return 0;
}
