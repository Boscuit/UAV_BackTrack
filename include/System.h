#ifndef SYSTEM_H
#define SYSTEM_H

#include<string>
#include<opencv2/core/core.hpp>

#include "ORBVocabulary.h"
#include "BackTracking.h"


namespace UAV_BackTrack
{

class System
{
public:

    System();

    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    System(const string &strVocFile, const string &strSettingsFile);

    cv::Mat BackTrack(const cv::Mat &Im1, const cv::Mat &Im2, const double &tframe1, const double &tframe2, int index1, int index2);

    std::vector<float> Twc2sevenD(cv::Mat Twc);
    cv::Mat sevenD2Twc(vector<float>);
    cv::Mat InverseT(cv::Mat Tcw);
    std::vector<float> Twc2eulerangle(cv::Mat Twc, bool isDegreeds);

private:

    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;

    BackTracking* mpBackTracker;



};


}//namespace UAV_BackTrack


#endif //SYSTEM_H