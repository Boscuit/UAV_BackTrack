#include<cstdio>
#include<iostream>
#include<string>
#include<cmath>

#include<ros/ros.h>
#include<tf2_ros/transform_broadcaster.h>
#include<cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

#include "System.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/Char.h"

#include"predefine.h"

using namespace std;

class Visualizer
{
public:

  Visualizer(UAV_BackTrack::System* pSystem, const vector<string> &vstrImageFilenames, const vector<double> &vTimestamps)
  :mpSystem(pSystem),mvstrImage(vstrImageFilenames),mvTimestamps(vTimestamps)
  {
    int nImgs = (int)vTimestamps.size();

#ifdef START_INDEX
    nImgsIndexStart = max(0,START_INDEX);
#else
    nImgsIndexStart = 0;
#endif

#ifdef STOP_INDEX
    nImgsIndexStop = min(STOP_INDEX,nImgs);
#else
    nImgsIndexStop = vTimestamps.size();
#endif

#ifdef TARGET_INDEX
    ndesiredIndex = max(0,min(TARGET_INDEX,nImgs));
#else
    ndesiredIndex = vTimestamps.size();
#endif
    
    nImgsIndex = nImgsIndexStart;
    mTcb = mpSystem->InverseT(mTbc);
    mTgb = mpSystem->InverseT(mTbg);

    //pub&sub
    cmd_sub = n.subscribe("/Request", 1, &Visualizer::ProcessImages, this);
  }

  void ProcessImages(const std_msgs::Char& cmd);

  void LoadGroundTruth(const string &strGroundTruthPath);

  UAV_BackTrack::System* mpSystem;

  ros::NodeHandle n;

  vector<float> ex;
  vector<float> ey;
  vector<float> ez;
  vector<float> et;

private:
  size_t nImgsIndexStart;
  size_t nImgsIndexStop;
  size_t nImgsIndex;
  size_t ndesiredIndex;
  vector<string> mvstrImage;
  vector<double> mvTimestamps;
  //cv::Mat mTwv = (cv::Mat_<float>(4,4) << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1);//from c1 to c2
  cv::Mat mTbc = (cv::Mat_<float>(4,4) << 
        0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
         0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
        -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
         0.0, 0.0, 0.0, 1.0);
  cv::Mat mTbg = (cv::Mat_<float>(4,4) << 
        0.33638, -0.01749,  0.94156,  0.06901,
        -0.02078, -0.99972, -0.01114, -0.02781,
        0.94150, -0.01582, -0.33665, -0.12395,
            0.0,      0.0,      0.0,      1.0);
  cv::Mat mTcb;
  cv::Mat mTgb;

  map<double,vector<float> > mmGT;

  tf2_ros::TransformBroadcaster tf2_pub;
  ros::Subscriber cmd_sub;
};

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
  double t;
  char name[25];
  char tmp[10000];
  vTimeStamps.reserve(5000);
  vstrImages.reserve(5000);
  FILE * pTimeFile = fopen(strPathTimes.c_str(),"r");
  if(pTimeFile==NULL)
  {
    cerr << "Open TimeFile failed. Wrong path." << endl;
  }
  else if(fgets(tmp, 10000, pTimeFile) == NULL)
  {
    cerr << "Can't load TimeFile. File is empty."<< endl;
  }
  else
  {
    while (!feof(pTimeFile))
    {
      if(fscanf(pTimeFile, "%lf,%s", &t, name) != EOF)
      {        
        string s(name);
        vstrImages.push_back(strImagePath + "/" +s);
        vTimeStamps.push_back(t/1e9);
      }
    }
    fclose(pTimeFile);
  }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Test");
    ros::start();

    if(argc !=6)
    {
        cerr << endl << "Usage: rosrun UAV_BackTrack Test path_to_vocabulary path_to_settings path_to_image_folder path_to_times_file path_to_groundtruth" << endl;
        ros::shutdown();
        return 1;
    }

    //setup job

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), string(argv[4]), vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    if(nImages<=0)
    {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }

    for (int i=0;i<nImages;i++)
    {
      cout << vstrImageFilenames[i] << endl;
    }

    UAV_BackTrack::System Sys(argv[1],argv[2]);
    
    Visualizer Vis(&Sys,vstrImageFilenames,vTimestamps);
    
    Vis.LoadGroundTruth(argv[5]);

#ifdef AUTO_ITERATION
    std_msgs::Char sudocmd;
    sudocmd.data = 'p';
    for (int i =0; i<AUTO_ITERATION;i++)
    {
      Vis.ProcessImages(sudocmd);
    }
#endif

    ros::spin();


    ros::shutdown();

    //ending job
#ifdef STORE_RESULT
    cout << "Wirting files. size: " << Vis.ex.size() << endl;
    ofstream fx,fy,fz,ft;
    fx.open("fx.txt");
    fy.open("fy.txt");
    fz.open("fz.txt");
    ft.open("ft.txt");
    fx << fixed;
    fy << fixed;
    fz << fixed;
    ft << fixed;
    if (Vis.ex.size() != Vis.ey.size() || Vis.ex.size() != Vis.ez.size() || Vis.ex.size() != Vis.et.size())
      cout << "Invalid size." <<endl;
    else
    {
      for (size_t i=0; i<Vis.ex.size(); i++)
      {
        fx << Vis.ex[i] << ",";
        fy << Vis.ey[i] << ",";
        fz << Vis.ez[i] << ",";
        ft << Vis.et[i] << ",";
      }
    }
    fx.close();
    fy.close();
    fz.close();
    ft.close();
    cout<< "saved"<<endl;
#endif

    return 0;

}

void Visualizer::ProcessImages(const std_msgs::Char& cmd)
{
  if (cmd.data == 'p' || cmd.data == 's')
  {
    if(cmd.data == 'p')
    {    
      if (nImgsIndex<nImgsIndexStop)
        nImgsIndex++;
      else
        nImgsIndex = nImgsIndexStart;
      cout << "processing " << nImgsIndex << " against " << ndesiredIndex<< endl;
    }
    //if 's' don't change

    // Read image from file
    cv::Mat Im1, Im2;
    
    Im1 = cv::imread(mvstrImage[nImgsIndex],CV_LOAD_IMAGE_UNCHANGED);
    Im2 = cv::imread(mvstrImage[ndesiredIndex],CV_LOAD_IMAGE_UNCHANGED);

    double tframe1 = mvTimestamps[nImgsIndex];
    double tframe2 = mvTimestamps[ndesiredIndex];

    if(Im1.empty())
    {
      cerr << endl << "Failed to load image " <<  mvstrImage[nImgsIndex] << endl;
      return;
    }
    else if(Im2.empty())
    {
      cerr << endl << "Failed to load image " <<  mvstrImage[ndesiredIndex] << endl;
      return;
    }
    if(Im1.channels()==3)
      cvtColor(Im1,Im1,CV_RGB2GRAY);
    if(Im2.channels()==3)
      cvtColor(Im2,Im2,CV_RGB2GRAY);

    cv::Mat Tcr = mpSystem->BackTrack(Im1,Im2,tframe1,tframe2,nImgsIndex,ndesiredIndex);
    //Tcr = mTwv.t()*Tcr*mTwv;
    //cout << Tcr << endl;
    cv::Mat Tcbrb = mTbc*Tcr*mTcb;
    cv::Mat Tcgrg = mTgb*Tcbrb*mTbg;

    // Show GroundTruth
    map<double,vector<float> >::iterator GTit1 = mmGT.lower_bound(tframe1);
    map<double,vector<float> >::iterator GTit2 = mmGT.lower_bound(tframe2);
    vector<float> GT1 = GTit1->second;
    vector<float> GT2 = GTit2->second;

    cv::Mat GT_Twc = mpSystem->sevenD2Twc(GT1);//from world to current GT
    cv::Mat GT_Twr = mpSystem->sevenD2Twc(GT2);
    cv::Mat GT_Tcr = (mpSystem->InverseT(GT_Twc)) * GT_Twr;
    vector<float> GT12 = mpSystem->Twc2sevenD(GT_Tcr);

    // Evaluate rotation error by Euler Angle
    vector<float> ea = mpSystem->Twc2eulerangle(Tcgrg,true);
    vector<float> GT_ea = mpSystem->Twc2eulerangle(GT_Tcr,true);
    cout << "Tcgrg:"<<endl;
    cout << Tcgrg << endl;
    cout << "GT_Tcr:" << endl;
    cout << GT_Tcr << endl;
    cout << "Euler angle from current to estimate pose, z: "<<ea[0]<<" y: "<<ea[1]<<" x: "<<ea[2]<<endl;
    cout << "Euler angle from current GT to desired GT, z: "<<GT_ea[0]<<" y: "<<GT_ea[1]<<" x: "<<GT_ea[2]<<endl;

    //Evaluate translation error by vectorial angle
    cv::Mat GT_tcr = GT_Tcr.rowRange(0,3).col(3);
    cv::Mat tcgrg = Tcgrg.rowRange(0,3).col(3);
    double cErr = GT_tcr.dot(tcgrg)/(cv::norm(GT_tcr)*cv::norm(tcgrg));
    double transErr = acos(cErr)*180.0/M_PI;//in degrees
    
    cout << "Vectorial angle between estimated and GT translation: " << transErr << endl;

#ifdef STORE_RESULT
    if (Tcgrg.at<float>(0,2)>0.0001)//T valid
    {
      ex.push_back((GT_ea[0]-ea[0]));
      ey.push_back(GT_ea[1]-ea[1]);
      ez.push_back(GT_ea[2]-ea[2]);
      et.push_back(transErr);
    }
#endif
    

    //Publish Current Groundtruth
    geometry_msgs::TransformStamped current_gt;
    current_gt.header.stamp = ros::Time::now();
    current_gt.header.frame_id = "world";
    current_gt.child_frame_id = "Current_Groundtruth";
    current_gt.transform.translation.x = GT1[0];
    current_gt.transform.translation.y = GT1[1];
    current_gt.transform.translation.z = GT1[2];
    current_gt.transform.rotation.x = GT1[3];
    current_gt.transform.rotation.y = GT1[4];
    current_gt.transform.rotation.z = GT1[5];
    current_gt.transform.rotation.w = GT1[6];
    tf2_pub.sendTransform(current_gt);

    //Publish Desired Groundtruth
    geometry_msgs::TransformStamped desired_gt;
    desired_gt.header.stamp = ros::Time::now();
    desired_gt.header.frame_id = "Current_Groundtruth";
    desired_gt.child_frame_id = "Desired_Groundtruth";
    desired_gt.transform.translation.x = GT12[0];
    desired_gt.transform.translation.y = GT12[1];
    desired_gt.transform.translation.z = GT12[2];
    desired_gt.transform.rotation.x = GT12[3];
    desired_gt.transform.rotation.y = GT12[4];
    desired_gt.transform.rotation.z = GT12[5];
    desired_gt.transform.rotation.w = GT12[6];
    tf2_pub.sendTransform(desired_gt);

    //Publish estimated transform
    vector<float> vPubPose = mpSystem->Twc2sevenD(Tcgrg);
    geometry_msgs::TransformStamped estimation;
    estimation.header.stamp = ros::Time::now();
    estimation.header.frame_id = "Current_Groundtruth";
    estimation.child_frame_id = "est";
    estimation.transform.translation.x = vPubPose[0];
    estimation.transform.translation.y = vPubPose[1];
    estimation.transform.translation.z = vPubPose[2];
    estimation.transform.rotation.x = vPubPose[3];
    estimation.transform.rotation.y = vPubPose[4];
    estimation.transform.rotation.z = vPubPose[5];
    estimation.transform.rotation.w = vPubPose[6];
    tf2_pub.sendTransform(estimation);
  }
}

void Visualizer::LoadGroundTruth(const string &strGroundTruthPath)
{
  FILE * pLGTFile = fopen(strGroundTruthPath.c_str(),"r");
  char tmp[10000];
  if(pLGTFile==NULL)
  {
    cerr << "Open GroundTruth failed. Wrong path." << endl;
  }
  else if(fgets(tmp, 10000, pLGTFile) == NULL)
  {
    cerr << "Can't load GroundTruth. File is empty."<< endl;
  }
  else
  {
    double t;
    vector<float> vGT(7,0.f);//float px, py, pz, qw, qx, qy, qz;
    while (!feof(pLGTFile))
    {
      if(fscanf(pLGTFile, "%lf,%f,%f,%f,%f,%f,%f,%f", &t,
              &vGT[0], &vGT[1], &vGT[2], &vGT[6], &vGT[3], &vGT[4], &vGT[5]) != EOF)
      {
        mmGT.insert(pair<double, vector<float> >(t/1e9,vGT));
      }
    }
    fclose(pLGTFile);
  }
}
