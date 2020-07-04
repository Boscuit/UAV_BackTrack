#include "System.h"
#include "Converter.h"

namespace UAV_BackTrack
{

System::System()
{}

System::System(const string &strVocFile, const string &strSettingsFile)
{
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    mpBackTracker = new BackTracking(mpVocabulary,strSettingsFile);
    
}

cv::Mat System::BackTrack(const cv::Mat &Im1, const cv::Mat &Im2, const double &tframe1, const double &tframe2, int index1, int index2)
{
    cout << "BackTrack..." << endl;
    cv::Mat Tcr = mpBackTracker->BackTrack(Im1,Im2,tframe1,tframe2,index1,index2);

    return Tcr;
}

vector<float> System::Twc2sevenD(cv::Mat Twc)
{
  vector<float> vPubPose;
  cv::Mat twc(3,1,CV_32F);
  cv::Mat Rwc(3,3,CV_32F);
  Rwc = Twc.rowRange(0,3).colRange(0,3);
  twc = Twc.rowRange(0,3).col(3);
  vector<float> q = Converter::toQuaternion(Rwc);
  vPubPose.push_back (twc.at<float>(0));
  vPubPose.push_back (twc.at<float>(1));
  vPubPose.push_back (twc.at<float>(2));
  vPubPose.push_back (q[0]);
  vPubPose.push_back (q[1]);
  vPubPose.push_back (q[2]);
  vPubPose.push_back (q[3]);

  return vPubPose;
}

cv::Mat System::sevenD2Twc(vector<float> sevenD)
{
  cv::Mat Twc(4,4,CV_32F);
  vector<float> q(sevenD.begin()+3,sevenD.end());
  cv::Mat Rwc = Converter::toCvMat(q);
  Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
  Twc.at<float>(0,3) = sevenD[0];
  Twc.at<float>(1,3) = sevenD[1];
  Twc.at<float>(2,3) = sevenD[2];
  Twc.at<float>(3,0) = 0;
  Twc.at<float>(3,1) = 0;
  Twc.at<float>(3,2) = 0;
  Twc.at<float>(3,3) = 1;

  return Twc;
}


cv::Mat System::InverseT(cv::Mat Tcw)
{
  cv::Mat twc(3,1,CV_32F);
  cv::Mat Rwc(3,3,CV_32F);
  cv::Mat Twc(4,4,CV_32F);
  Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
  twc = -Rwc*Tcw.rowRange(0,3).col(3);
  Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
  twc.copyTo(Twc.rowRange(0,3).col(3));
  Twc.at<float>(3,0) = 0;
  Twc.at<float>(3,1) = 0;
  Twc.at<float>(3,2) = 0;
  Twc.at<float>(3,3) = 1;
  return Twc;
}

vector<float> System::Twc2eulerangle(cv::Mat Twc, bool isDegreeds = false)
{
  cv::Mat Rwc(3,3,CV_32F);
  Rwc = Twc.rowRange(0,3).colRange(0,3);
  // Rwc = (cv::Mat_<float>(3,3) << 0.707107, -0.707107, 0, 0.707107, 0.707107, 0, 0, 0, 1);
  // Rwc = (cv::Mat_<float>(3,3) << 0.8627299, -0.3001816,  0.4069252,
  //  0.4980974,  0.6431866, -0.5815582,
  // -0.0871557,  0.7044160,  0.7044160);
  vector<float> YPR = Converter::toEulerAngle(Rwc);
  if (isDegreeds)
  {
    YPR[0] = YPR[0] * (180/M_PI);
    YPR[1] = YPR[1] * (180/M_PI);
    YPR[2] = YPR[2] * (180/M_PI);
  }
  return YPR;
}

}//namespace UAV_BackTrack