#ifndef BACKTRACKING_H
#define BACKTRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<unistd.h>
#include<iostream>

#include "Initializer.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"

namespace UAV_BackTrack
{
class BackTracking
{
public:

    BackTracking(ORBVocabulary* pVoc,const string &strSettingPath);

    //Return estimated pose of Im2 base on Im1 in camera frame
    cv::Mat BackTrack(const cv::Mat &Im1, const cv::Mat &Im2, const double &tframe1, const double &tframe2, int index1, int index2);

    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    
private:
    void ShowMatches(const cv::Mat &Im1, const cv::Mat &Im2, const Frame &Frame1, const Frame &Frame2,
                    vector<int> vMatches21, int nBoWmatches);

    //ORB
    ORBextractor* mpORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;
    float mThDepth;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;
};

}


#endif // BACKTRACKING_H
