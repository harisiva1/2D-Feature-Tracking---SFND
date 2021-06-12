/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;


/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    
    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 3;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */
    ofstream csv("../check.csv",ios_base::app);
    csv <<"detector type"<<","
        <<"Keypoint detection time(ms)"<<","
        <<"detected points"<<","
        <<"ROI points"<<","
        <<"descriptor type"<<","
        <<"descriptor time taken(ms)"<<","
        <<"matcher type"<<","
        <<"selector type"<<","
        <<"Matcher time taken(ms)"<<","
        <<"No of matched points"<<","
        <<endl;
    vector<mc> diff_combo;
    const vector<std::string> detectorlist{ "HARRIS",  "SIFT","FAST", "BRISK", "SHITOMASI", "AKAZE" ,"ORB"};
    const vector<std::string> descriptorlist{ "FREAK", "AKAZE", "BRIEF", "ORB",  "SIFT" ,"BRISK"};
    const vector<std::string> matcherlist{ "MAT_BF" };
    const vector<std::string> selectorlist{"SEL_KNN"};

    for(auto& det : detectorlist)
        for(auto& desc: descriptorlist)
            for(auto& mat: matcherlist)
            {
                if ((desc.compare("AKAZE") == 0 && det.compare("AKAZE") != 0)||(desc.compare("ORB") == 0 && det.compare("SIFT") == 0))
                {
                    continue;
                }
                mc combo;
                combo.det = det;
                combo.desc = desc;
                combo.mat = mat;
                diff_combo.push_back(combo);
            }
    cout<<"there are"<<diff_combo.size()<<"combinations"<<endl;;
    for(auto& cc:diff_combo)
    {
        string detectorType = cc.det;
        string descriptorType = cc.desc;
        string matcherType = cc.mat; 

        cout<<detectorType << descriptorType << matcherType<<endl;
        dataBuffer.clear();
        for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
        {
            /* LOAD IMAGE INTO BUFFER */

            // assemble filenames for current index
            ostringstream imgNumber;
            imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
            string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

            // load image from file and convert to grayscale
            cv::Mat img, imgGray;
            img = cv::imread(imgFullFilename);
            cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

            //// STUDENT ASSIGNMENT
            //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

            // push image into data frame buffer
            DataFrame frame;
            frame.cameraImg = imgGray;
            if(dataBuffer.size()<dataBufferSize){
                dataBuffer.push_back(frame);
            }
            else{
                dataBuffer.erase(dataBuffer.begin());
                dataBuffer.push_back(frame);
            }
            
            

            //// EOF STUDENT ASSIGNMENT
            cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

            /* DETECT IMAGE KEYPOINTS */

            // extract 2D keypoints from current image
            vector<cv::KeyPoint> keypoints; // create empty feature list for current image
            //string detectorType = cc.det;
            cout<<detectorType<<endl;
            csv<<endl;
            csv<<detectorType<<",";

            //// STUDENT ASSIGNMENT
            //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
            //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
            if (detectorType.compare("SHITOMASI") == 0)
            {
                double t5 = detKeypointsShiTomasi(keypoints, imgGray, false);
                csv<< t5<<",";
            }
            else if(detectorType.compare("HARRIS") == 0)
            {
                double t5 = detKeypointsHarris(keypoints, imgGray, false);
                csv<< t5<<",";
            }
            else 
            {
                double t5 = detKeypointsModern(keypoints,imgGray,detectorType,false);
                csv<< t5<<",";
            }
            csv<<keypoints.size()<<",";
            
            
            //// EOF STUDENT ASSIGNMENT

            //// STUDENT ASSIGNMENT
            //// TASK MP.3 -> only keep keypoints on the preceding vehicle

            // only keep keypoints on the preceding vehicle
            bool bFocusOnVehicle = true;
            cv::Rect vehicleRect(535, 180, 180, 150);
            if (bFocusOnVehicle)
            {
                vector<cv::KeyPoint> fp;
                for(auto it=keypoints.begin(); it!= keypoints.end();it++)
                {
                    if(it->pt.x > vehicleRect.x && it->pt.x< vehicleRect.x+vehicleRect.width && it->pt.y>vehicleRect.y && it->pt.y< vehicleRect.y+vehicleRect.width)
                    {
                        fp.push_back(*it);               
                    }
                }
                keypoints = fp;
                
            }
            cout<<"after restricting it to rectangle "<< keypoints.size()<<endl;
            csv<<keypoints.size()<<",";

            //// EOF STUDENT ASSIGNMENT

            // optional : limit number of keypoints (helpful for debugging and learning)
            bool bLimitKpts = false;
            if (bLimitKpts)
            {
                int maxKeypoints = 50;

                if (detectorType.compare("SHITOMASI") == 0)
                { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                    keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                }
                cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                cout << " NOTE: Keypoints have been limited!" << endl;
            }

            // push keypoints and descriptor for current frame to end of data buffer
            (dataBuffer.end() - 1)->keypoints = keypoints;
            cout << "#2 : DETECT KEYPOINTS done" << endl;

            /* EXTRACT KEYPOINT DESCRIPTORS */

            //// STUDENT ASSIGNMENT
            //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
            //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

            cv::Mat descriptors;
            //string descriptorType = cc.desc; // BRISK,BRIEF, ORB, FREAK, AKAZE, SIFT
            csv<<descriptorType<<",";
            double t1 =descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
            csv<<t1<<",";
            //// EOF STUDENT ASSIGNMENT

            // push descriptors for current frame to end of data buffer
            (dataBuffer.end() - 1)->descriptors = descriptors;

            cout << "#3 : EXTRACT DESCRIPTORS done" << endl;


            if (dataBuffer.size() > 1) // wait until at least two images have been processed
            {

                /* MATCH KEYPOINT DESCRIPTORS */

                vector<cv::DMatch> matches;
                //string matcherType = cc.mat;        // MAT_BF, MAT_FLANN
                string descriptorT; // DES_BINARY, DES_HOG
                if(descriptorType== "SIFT")
                    string descriptorT = "DES_HOG";
                else
                    string descriptorT = "DES_BINARY";

                
                string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN
                csv<<matcherType<<",";
                csv<<selectorType<<",";

                //// STUDENT ASSIGNMENT
                //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
                //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp
                
                double t2 = matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                                (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                                matches, descriptorT, matcherType, selectorType);

                //// EOF STUDENT ASSIGNMENT
                
                csv<<t2<<",";
                // store matches in current data frame
                (dataBuffer.end() - 1)->kptMatches = matches;
                csv<<matches.size();

                cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

                // visualize matches between current and previous image
                bVis = false;
                if (bVis)
                {
                    cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                    cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                    (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                    matches, matchImg,
                                    cv::Scalar::all(-1), cv::Scalar::all(-1),
                                    vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                    string windowName = "Matching keypoints between two camera images";
                    cv::namedWindow(windowName, 7);
                    cv::imshow(windowName, matchImg);
                    cout << "Press key to continue to next image" << endl;
                    cv::waitKey(0); // wait for key to be pressed
                }
                bVis = false;
            }

        } // eof loop over all images
    }  //end of different combinations 
    cout<<"completed"<<endl;
    return 0;
}
