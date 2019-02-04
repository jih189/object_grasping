//Jiaxiang Liang, Dingcheng Hu, Jiaming Hu
/**********************************************************
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
bool pre_obs = false;
int dis = 0;

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
    if (state == 0 && blobsIn.blob_count > 0){
        
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
		
        float max_area = 0;
        int goalx = 0;
        int goaly = 0;
		for (int i = 0; i < blobsIn.blob_count; i++){
            if(blobsIn.blobs[i].area > max_area){
                max_area = blobsIn.blobs[i].area;
                goalx = blobsIn.blobs[i].x;
                goaly = blobsIn.blobs[i].y;
            }
		}

        if(goalx > 450){
            std::cout << "on right side " << std::endl;
            state = 3;
        }
        else if(goalx < 240){
            std::cout << "on left side " << std::endl;
            state = 4;
        }
        else{
            std::cout << "in the forward" << std::endl;
            state = 5;
        }
    }

    //Cannot find goal
    else if (state == 0){
        std::cout << "can't find the goal, turning" << std::endl;
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
  double totalx = 0.0, totaly = 0.0;
  int totalnum = 0;
  int y_point = 0;
  double ZTHRESH = 0.7;

  std::vector<double> PCL_closest_points;
  std::vector<double> PCL_closest_points_x;
  std::vector<double> PCL_closest_points_y;
  PCL_closest_points.clear();
  PCL_closest_points_x.clear();
  PCL_closest_points_y.clear();

  double z_min = 100;
  if (dis > 0){
       return;
  }
  //Iterate through all the points in the image
  //Convert from pcl to cm
  for(int k = 0; k < 240; k++){
    for(int i = 0; i < 640; i++){
      const pcl::PointXYZ & pt=cloud->points[640*(180+k)+(i)];
      if((pt.z < ZTHRESH)){
        totalx += i;
        totaly += k;
        totalnum += 1;
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

  if(totalnum > 4000){
      /*
      std::cout << "number of points: " << totalnum << std::endl;
      std::cout << "x = " << totalx / totalnum << std::endl;
      */


      std::cout << "found obstacle" << std::endl;
      pre_obs = true;
      if(totalx / totalnum > 320){
         state = 1;
         // std::cout << "Many points? State 1" << std::endl;
      }
      else{
          state = 2;
         // std::cout << "Less points? State 2?" << std::endl;
      }
  }
  else{
	  
	 //if we previous see the obs, we should go forward to avoid it (Enter special state).
         if(pre_obs){
            std::cout << "avoid" <<std::endl;
            dis = 35;
            pre_obs = false;
         }
         state = 0;
  }
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

  ros::Rate st(100000);
  geometry_msgs::Twist T;

  while(ros::ok()){

    T.linear.x = 0.0; T.linear.y = 0.0; T.linear.z = 0.0;
    T.angular.x = 0.0; T.angular.y = 0.0; T.angular.z = 0.0;//-0.5;
	  
//the special state where we go forward a constant distance.
    if(dis > 0){
        std::cout<<"go away!" << std::endl;
        T.linear.x = 0.2;
        dis --;
        if(dis == 0){
            state = 0;
        }
        ros::spinOnce();
        loop_rate.sleep();
        velocityPublisher.publish(T);
        continue;
    }
    std::cout << state << std::endl;
    //Looking for goal
    if (state == 0){
        T.angular.z = -0.5; 
    }
    //encounter obs
    else if (state == 1){
        T.angular.z = -0.5;
    } 
	  // encounter obs
    else if (state == 2) {
        T.angular.z = -0.5;
    }
	  //goal on left
    else if (state == 3){
        T.angular.z = -0.5;
    }
	  //goal on right
    else if (state == 4){
        T.angular.z = 0.5; 
    }
	  //goal straight
    else if (state == 5){ 
        T.linear.x = 0.2;
    }
    // Spin
    ros::spinOnce();
    loop_rate.sleep();
    velocityPublisher.publish(T);
  }
}
