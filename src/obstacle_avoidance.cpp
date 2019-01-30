/************************************************************
 * Name: obstacle_avoidance.cpp
 * Author: Alyssa Kubota, Sanmi Adeleye
 * Date: 02/18/2018
 *
 * Description: This program will subscribe to the /blobs topic,
 *        and use the blob information to find and go towards
 *        the blobs in a user-set order.
 ***********************************************************/

#include <kobuki_msgs/BumperEvent.h> 
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <cmvision/Blobs.h>
#include <stdio.h>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <time.h>
#include <math.h>
ros::Publisher pub;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

uint16_t state;
int16_t goal_sum_x = 0;
int16_t goal_sum_y = 0;
double goal_area = 0;
double Center = 320;
std::vector<uint16_t> goal_xs;

/************************************************************
 * Function Name: blobsCallBack
 * Parameters: const cmvision::Blobs
 * Returns: void
 *
 * Description: This is the callback function of the /blobs topic
 ***********************************************************/

void blobsCallBack (const cmvision::Blobs& blobsIn) //this gets the centroid of the color blob corresponding to the goal.
{
    if (blobsIn.blob_count > 0){
		uint16_t blob_sum, blob_centroid_sum, num_blobs;
		int x_sum = 0;
		double z_sum = 0;
		uint16_t num_goal_blobs = 0;

		/************************************************************
		* These blobsIn.blobs[i].red, blobsIn.blobs[i].green, and blobsIn.blobs[i].blue values depend on the
		* values those are provided in the colors.txt file.
		* For example, the color file is like:
		* 
		* [Colors]
		* (255, 0, 0) 0.000000 10 RED 
		* (255, 255, 0) 0.000000 10 YELLOW 
		* [Thresholds]
		* ( 127:187, 142:161, 175:197 )
		* ( 47:99, 96:118, 162:175 )
		* 
		* Now, if a red blob is found, then the blobsIn.blobs[i].red will be 255, and the others will be 0.
		* Similarly, for yellow blob, blobsIn.blobs[i].red and blobsIn.blobs[i].green will be 255, and blobsIn.blobs[i].blue will be 0.
		************************************************************/
		
		std::vector<int> blobs_centroid_vector;

		blobs_centroid_vector.clear();

        std::cout << "analysis" << std::endl;
		for (int i = 0; i < blobsIn.blob_count; i++){
			ROS_INFO("Blob found");
                if (blobsIn.blobs[i].red == 0 && blobsIn.blobs[i].green == 255 && blobsIn.blobs[i].blue == 0){
                    goal_sum_x += blobsIn.blobs[i].x;
                    goal_sum_y += blobsIn.blobs[i].y;
                    num_goal_blobs++;
                }
		}
        std::cout << "goal is " << goal_sum_x / num_goal_blobs << ":" << goal_sum_y / num_goal_blobs<< std::endl;
    }  
}

/************************************************************
 * Function Name: PointCloud_Callback
 * Parameters: const PointCloud::ConstPtr
 * Returns: void
 *
 * Description: This is the callback function of the PointCloud
 * 				topic, flags when an object is below the threshhold 
 ***********************************************************/
void PointCloud_Callback (const PointCloud::ConstPtr& cloud){

  unsigned int n = 0;
  int s,t;
  double min_z = 100, x, y;
  int y_point = 0;
  double ZTHRESH = 0.5;

  std::vector<double> PCL_closest_points;
  std::vector<double> PCL_closest_points_x;
  std::vector<double> PCL_closest_points_y;
  PCL_closest_points.clear();
  PCL_closest_points_x.clear();
  PCL_closest_points_y.clear();
  std::cout << "analysis 2" << std::endl;

  double z_min = 100;
  //Iterate through all the points in the image
  //Convert from pcl to cm
  for(int k = 0; k < 240; k++){
    for(int i = 0; i < 640; i++){
      const pcl::PointXYZ & pt=cloud->points[640*(180+k)+(i)];
      if((pt.z < ZTHRESH)){
        PCL_closest_points_x.push_back(i);
        PCL_closest_points.push_back(pt.z);     
        //Find min z
        if(pt.z < z_min){   
          z_min = pt.z;
        }
        //cout << "z: " << pt.z << endl;
      }
    }
  }
  /*
  if(got_goal_blobs){
    const pcl::PointXYZ& ptg = cloud->points[640 * (goal_loc_y - 1) + (goal_loc_x - 1)];
    goal_depth = ptg.z;
  }
  */

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "blob");
  ros::NodeHandle nh;
  //States variable 
  state = 0;
  
  ros::Subscriber PCSubscriber = nh.subscribe<PointCloud>("/camera/depth/points", 1, PointCloud_Callback);

  //subscribe to /blobs topic 
  ros::Subscriber blobsSubscriber = nh.subscribe("/blobs", 50, blobsCallBack);

  // publishing the geometry message twist message
  ros::Publisher velocityPublisher = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1000);

  ros::Rate loop_rate(10);
  geometry_msgs::Twist T;

  while(ros::ok()){

    T.linear.x = 0.0; T.linear.y = 0.0; T.linear.z = 0.0;
    T.angular.x = 0.0; T.angular.y = 0.0; T.angular.z = 0.0;//-0.5;
    std::cout << "Spin" << std::endl;
    //Looking for goal
    if (state == 0){

    } 
    else if (state == 1) {

    }

    // Spin
    ros::spinOnce();
    loop_rate.sleep();
    velocityPublisher.publish(T);
  }
}

