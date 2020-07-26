#include"BackTracking.h"

#include<iostream>
#include<mutex>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"Initializer.h"



using namespace std;

namespace UAV_BackTrack
{
BackTracking::BackTracking(ORBVocabulary* pVoc,const string &strSettingPath):mpORBVocabulary(pVoc), mpInitializer(static_cast<Initializer*>(NULL))
{
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;


    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractor = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;
}

cv::Mat BackTracking::BackTrack(const cv::Mat &Im1, const cv::Mat &Im2, const double &tframe1, const double &tframe2, int index1, int index2)
{
    Frame Frame1 = Frame(Im1,index1,tframe1,mpORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    Frame Frame2 = Frame(Im2,index2,tframe2,mpORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    Frame1.ComputeBoW();
    Frame2.ComputeBoW();
    ORBmatcher matcher(0.75,true);
    vector<int> vMatches21, vIniMatches12;
    int nBoWmatches = matcher.SearchByBoW(Frame1,Frame2,vMatches21);

    cout << "nBoWmatches: "<<nBoWmatches<<endl;

    ShowMatches(Im1,Im2,Frame1,Frame2,vMatches21,nBoWmatches,"BoW");
    
    mvbPrevMatched.resize(Frame1.mvKeysUn.size());
    for(size_t i=0; i<Frame1.mvKeysUn.size(); i++)
        mvbPrevMatched[i]=Frame1.mvKeysUn[i].pt;
    int nInimatches = matcher.SearchForInitialization(Frame1,Frame2,mvbPrevMatched,vIniMatches12, 100);
    cout << "nInimatches: "<<nInimatches<<endl;

    vector<int> vIniMatches21 = vector<int>(Frame2.mvKeysUn.size(),-1);//store index of desired Frame's keypoints
    for (size_t i=0;i<vIniMatches12.size();i++)
    {
        if(vIniMatches12[i]>=0)
        {
            vIniMatches21[vIniMatches12[i]] = i;
        }
    }
    ShowMatches(Im1,Im2,Frame1,Frame2,vIniMatches21,nInimatches,"Initial");
    
    // vector<int> vMatches12 = vector<int>(Frame1.mvKeysUn.size(),-1);//store index of desired Frame's keypoints
    // for (size_t i=0;i<vMatches21.size();i++)
    // {
    //     if(vMatches21[i]>=0)
    //     {
    //         vMatches12[vMatches21[i]] = i;
    //     }
    // }

    if(mpInitializer != NULL)
    {
        cout <<"delete initializer"<<endl;
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    cout << "New initializer"<<endl;
    mpInitializer =  new Initializer(Frame2.mvKeysUn,mK,1.0,200);//mK is Calibration matrix
    cv::Mat Tcr = cv::Mat::eye(4,4,CV_32F);
    cv::Mat Rcr(3,3,CV_32F); // from current to reference
    cv::Mat tcr(3,1,CV_32F); // from current to reference
    vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
    cout<<"initialize"<<endl;
    if(mpInitializer->Initialize(Frame1.mvKeysUn, vMatches21, Rcr, tcr, mvIniP3D, vbTriangulated))
    {
        for(size_t i=0, iend=vMatches21.size(); i<iend;i++)
        {
            if(vMatches21[i]>=0 && !vbTriangulated[i])
            {
                vMatches21[i]=-1;
                nBoWmatches--;
            }
        }
        Rcr.copyTo(Tcr.rowRange(0,3).colRange(0,3));
        tcr.copyTo(Tcr.rowRange(0,3).col(3));
    }

    ShowMatches(Im1,Im2,Frame1,Frame2,vMatches21,nBoWmatches,"BoW");
    
    return Tcr;
        
}

void BackTracking::ShowMatches(const cv::Mat &Im1, const cv::Mat &Im2, const Frame &Frame1, const Frame &Frame2, 
                vector<int> vMatches21, int nMatches, const string &methodName)
{
    cv::Mat im = cv::Mat(max(Im1.rows,Im2.rows),Im1.cols+Im2.cols,Im1.type());
    Im1.copyTo(im.rowRange(0,Im1.rows).colRange(0,Im1.cols));
    Im2.copyTo(im.rowRange(0,Im2.rows).colRange(Im1.cols,Im1.cols+Im2.cols));
    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);

    const float r = 5;
    vector<cv::KeyPoint> vF1Keys = Frame1.mvKeys;
    vector<cv::KeyPoint> vF2Keys = Frame2.mvKeys;
    for(size_t i=0;i<vF1Keys.size();i++)
    {
        cv::Point2f pt1,pt2;
        pt1.x=vF1Keys[i].pt.x-r;
        pt1.y=vF1Keys[i].pt.y-r;
        pt2.x=vF1Keys[i].pt.x+r;
        pt2.y=vF1Keys[i].pt.y+r;

        cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
        cv::circle(im,vF1Keys[i].pt,2,cv::Scalar(0,255,0),-1);
        
    }
    for(size_t j=0;j<vF2Keys.size();j++)
    {
        cv::Point2f ptR,ptR1,ptR2;
        ptR.x=vF2Keys[j].pt.x+Im1.cols;
        ptR.y=vF2Keys[j].pt.y;
        ptR1.x=ptR.x-r-1;
        ptR1.y=ptR.y-r-1;
        ptR2.x=ptR.x+r+1;
        ptR2.y=ptR.y+r+1;

        cv::rectangle(im,ptR1,ptR2,cv::Scalar(0,253,0));
        cv::circle(im,ptR,2,cv::Scalar(0,253,0),-1);

        if(vMatches21[j]>=0)
        {
            cv::Point2f ptL;
            ptL.x=vF1Keys[vMatches21[j]].pt.x;
            ptL.y=vF1Keys[vMatches21[j]].pt.y;
            cv::line(im,ptR,ptL,cv::Scalar(0,0,255));
        }
    }

    stringstream sText, sTitle;
    sText << "Search By " << methodName << " matches: "<< nMatches;
    sTitle << "Result/" << Frame1.mnId << " and " << Frame2.mnId <<" "<< methodName << "Match: "<<nMatches << ".png";
    int baseline=0;
    cv::Size textSize = cv::getTextSize(sText.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);
    cv::Mat imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,sText.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);
    cv::imwrite(sTitle.str(),imText);
    // cv::imshow(methodName,imText);
    // cv::waitKey(0);
}


} //namespace UAV_BackTrack
