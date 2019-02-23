#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <time.h>
#include <math.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/vision/vpKeyPoint.h>
#include <visp3/core/vpImageTools.h>
#include <visp_bridge/image.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <iostream>
#include <fstream>

#include <visp3/mbt/vpMbEdgeKltTracker.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

cv::Mat inputimage;
// detector info

const std::string detectorName = "ORB";
const std::string extractorName = "ORB";
const std::string matcherName = "BruteForce-Hamming";

vpImage<unsigned char> rI; // rI image
vpImage<unsigned char> I; // topic image
vpImage<unsigned char> II; // topic image
vpImage<unsigned char> iDisp; // what gets displayed on screen


vpKeyPoint keypoint;

vpDisplayOpenCV * display;
std::vector<vpImagePoint> refCorners;
std::vector<vpImagePoint> pairCornerPoints;


// tracker variables
vpCameraParameters cam;
vpHomogeneousMatrix cMo;
vpMbTracker *tracker;
vpMe me;

/**
 * state info
 * 0: search pairs of corners points in refimage and curimage
 * 1: pass those points to VISP for location
 */
int state;

void image_Callback (const sensor_msgs::ImageConstPtr& msg)
{

  II = visp_bridge::toVispImage(*msg);
  vpImageTools::crop(II, 300, 0, II.getHeight()-300, II.getWidth(), I);

  if(state == 0){
    /*open cv */
    cv_bridge::CvImagePtr CVI;
    CVI = cv_bridge::toCvCopy(msg);
    cv::Mat src, dst;
    src = CVI->image;
    // crop the image
    cv::Rect roi(0,300,src.cols,src.rows-300);
    src = src(roi);

    std::vector<vpImagePoint> curCornerPoints;

    /* good features to track*/
    std::vector<cv::Point> corners;
    std::vector<cv::Point> edgecorners;
    cv::Mat src_gray;
    cv::cvtColor(src, src_gray, cv::COLOR_BGR2GRAY);
    // find straight lines
    Canny(src, dst, 50, 200, 3);

    std::vector<cv::Vec4i> lines; 
    HoughLinesP(dst, lines, 1, CV_PI/180, 35, 30, 10);

    std::vector<cv::Point> edgePoints;

    // draw lines
    for(size_t i = 0; i < lines.size(); i++){
        cv::Vec4i l = lines[i];
        edgePoints.push_back(cv::Point(l[0], l[1]));
        edgePoints.push_back(cv::Point(l[2], l[3]));
    }
    
    cv::goodFeaturesToTrack(src_gray, corners, 20, 0.08, 3, cv::Mat(), 3, false, 0.04);
    for(int i = 0; i < corners.size(); i++){
        for(int j = 0; j < edgePoints.size(); j++){
            if(cv::norm(corners[i] - edgePoints[j]) <= 4){
                edgecorners.push_back(corners[i]);
                break;
            }
        }
    }
    for(int i = 0; i < edgecorners.size(); i++){
      cv::circle(src,edgecorners[i],2,cv::Scalar(0,0,255),1,3,0); 
    }

    cv::imshow("view", src);
    cv::waitKey(3);

    for(size_t i = 0; i < corners.size(); i++){
      curCornerPoints.push_back(vpImagePoint((int)corners[i].y,(int)corners[i].x));
    }

    //match and update iDisp
    iDisp.insert(I, vpImagePoint(0, rI.getWidth()));
    vpDisplay::display(iDisp);

    unsigned int nbMatch = keypoint.matchPoint(I);
    vpImagePoint iPref, iPcur;

    // get key points in reference image
    for(unsigned int i = 0; i < nbMatch; i++){
      keypoint.getMatchedPoints(i, iPref, iPcur);
      vpDisplay::displayPoint(iDisp, iPref, vpColor::red, 2);
    }

    // show all corner point currently
    for(unsigned int i = 0; i < curCornerPoints.size(); i++){
      vpDisplay::displayPoint(iDisp, curCornerPoints[i] + vpImagePoint(0, rI.getWidth()), vpColor::green, 2);
    }
    int numOfMatchCorner = 0;
    pairCornerPoints.clear();
    // for each select corner points
    for(unsigned int k = 0; k < refCorners.size(); k++){
      vpDisplay::displayPoint(iDisp, refCorners[k], vpColor::green, 2);
      // search all keypoints around the corner point
      int foundmatch = 0;
      for(unsigned int i = 0; i < nbMatch; i++){
        keypoint.getMatchedPoints(i, iPref, iPcur);
        if(vpImagePoint::distance(iPref, refCorners[k]) < 5){
          vpDisplay::displayLine(iDisp, iPref, iPcur + vpImagePoint(0,rI.getWidth()), vpColor::blue, 1);
          for(unsigned int j = 0; j < curCornerPoints.size(); j++){
            if(vpImagePoint::distance(curCornerPoints[j], iPcur) < 5){
              vpDisplay::displayLine(iDisp, refCorners[k], curCornerPoints[j] + vpImagePoint(0,rI.getWidth()), vpColor::green, 1);
              pairCornerPoints.push_back(curCornerPoints[j]);
              numOfMatchCorner++;
              curCornerPoints.erase(curCornerPoints.begin() + j); 
              foundmatch = 1;
              break;
            } 
          }
        }
        if(foundmatch == 1){
          break;
        }
      }
    }
    vpDisplay::flush(iDisp);
    if(numOfMatchCorner < -1){

      state = 1;
      std::cout << "found match points\n";
      for(unsigned int i = 0; i < pairCornerPoints.size(); i++){
        std::cout << pairCornerPoints[i] << std::endl;
      }
      delete display;
      // get cam value
      double u0 = I.getWidth() / 2;
      double v0 = I.getHeight() / 2;
      double px = 600;
      double py = 600;
      cam.initPersProjWithoutDistortion(px, py, u0, v0);
      cam.computeFov(I.getWidth(), I.getHeight());
      std::cout << cam << std::endl;
      std::cout << "Field of view (horizontal: " << vpMath::deg(cam.getHorizontalFovAngle())
                << " and vertical: " << vpMath::deg(cam.getVerticalFovAngle())
                << " degrees)" << std::endl;
      iDisp.init(I.getHeight(), I.getWidth());
      iDisp.insert(I, vpImagePoint(0,0));
      display = new vpDisplayOpenCV(iDisp, 0, 0, "find the location");

      // initlize tracker
      tracker = new vpMbEdgeKltTracker;
      
      me.setMaskSize(5);
      me.setMaskNumber(180);
      me.setRange(8);
      me.setThreshold(10000);
      me.setMu1(0.5);
      me.setMu2(0.5);
      me.setSampleStep(4);
      dynamic_cast<vpMbEdgeTracker *>(tracker)->setMovingEdge(me);

      vpKltOpencv klt_settings;
      klt_settings.setMaxFeatures(300);
      klt_settings.setWindowSize(10);
      klt_settings.setQuality(0.015);
      klt_settings.setMinDistance(8);
      klt_settings.setHarrisFreeParameter(0.01);
      klt_settings.setBlockSize(3);
      klt_settings.setPyramidLevels(3);
      dynamic_cast<vpMbKltTracker *>(tracker)->setKltOpencv(klt_settings);
      dynamic_cast<vpMbKltTracker *>(tracker)->setKltMaskBorder(5);
      //todo update camera values
      cam.initPersProjWithoutDistortion(600, 600, 400, 300);
      tracker->setCameraParameters(cam);
      tracker->loadModel("megablock1x3.cao");
      tracker->setDisplayFeatures(false);
     
      std::ifstream readfile;
      std::ofstream writefile;
      readfile.open("megablock1x3.init");
      writefile.open("matchpoints.init");
      if(readfile.is_open() && writefile.is_open())
      {
          std::string line;
          while(getline(readfile, line)){
              writefile << line << "\n"; 
          }
          writefile << "4\n";
          for(unsigned int j = 0; j < pairCornerPoints.size(); j++){
            writefile << pairCornerPoints[j].get_i() << " " << pairCornerPoints[j].get_j() << "\n";
          }
          readfile.close();
          writefile.close();
      }
      tracker->initFromPoints(iDisp, "matchpoints.init");

      std::cout << "finish\n";
    }

    // show all proper corner points
    /*
    for(int i = 0; i < refCorners.size(); i++){
      vpDisplay::displayPoint(iDisp, refCorners[i], vpColor::blue, 2);
    }
    */
  }
  else if(state == 1){
    iDisp.insert(I, vpImagePoint(0,0));
    vpDisplay::display(iDisp);
    tracker->track(iDisp);
    tracker->getPose(cMo);
    tracker->getCameraParameters(cam);
    tracker->display(iDisp, cMo, cam, vpColor::red, 2, true);
    vpDisplay::displayFrame(iDisp, cMo, cam, 0.025, vpColor::none, 3);
    vpDisplay::flush(iDisp);
  }
}

void PointCloud_Callback (const PointCloud::ConstPtr& cloud){

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "object_detect");
  ros::NodeHandle nh;
  state = 0;

  //ros::Subscriber sub = nh.subscribe("/cameras/head_camera/image", 100, image_Callback);
  ros::Subscriber sub = nh.subscribe("/cameras/right_hand_camera/image", 100, image_Callback);
  ros::spinOnce();

  vpKeyPoint::vpFilterMatchingType filterType = vpKeyPoint::ratioDistanceThreshold;
  keypoint = vpKeyPoint(detectorName, extractorName, matcherName, filterType);

  // initialize reference image
  //vpImageIo::read(rI, "white-0,0,0.1.png");
  //refCorners.push_back(vpImagePoint(380,373));
  //refCorners.push_back(vpImagePoint(380,412));
  //refCorners.push_back(vpImagePoint(405,373));
  //refCorners.push_back(vpImagePoint(405,412));
  //vpImageIo::read(rI, "right.png");
  //refCorners.push_back(vpImagePoint(303,245));
  //refCorners.push_back(vpImagePoint(303,281));
  //refCorners.push_back(vpImagePoint(223,292));
  //refCorners.push_back(vpImagePoint(324,285));
  vpImageIo::read(rI, "right_hand_img.png");
  refCorners.push_back(vpImagePoint(339,185));
  refCorners.push_back(vpImagePoint(339,292));
  refCorners.push_back(vpImagePoint(167,324));
  refCorners.push_back(vpImagePoint(410,302));
  I.init(600,800);
  std::cout << "reference keypoints = " << keypoint.buildReference(rI) << std::endl;

  //show display image
  iDisp.init(std::max(rI.getHeight(), I.getHeight()-300), rI.getWidth() + I.getWidth());
  iDisp.insert(rI, vpImagePoint(0,0));
  display = new vpDisplayOpenCV(iDisp, 0, 0, "Matching keypoints with ORB keypoints");
  vpDisplay::display(iDisp);
  vpDisplay::flush(iDisp);
  
  ros::spin();
  delete display;
  delete tracker;
  return 0;
}
