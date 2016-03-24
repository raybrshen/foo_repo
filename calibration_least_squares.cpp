//
//  calibration_least_squares.cpp
//  Algorithm
//
//  Created by Ray Shen on 2015-08-06.
//
//
//  when woking with multiple Kinects, each of them has a camera coordinate
//  the purpose of auto-calibration is to calculate a transformation matrix for each camera
//  so that each single point in the camera coordinate will be mapped to the world coordinate
//  transformation matrix can be calculated with manually calibrated position and rotation of the camera
//  but it's more accurate and easier to do it automatically by tracking moving targets
//  use one Kinect as standard camera so that we get world coordinate points
//  also record the camera coordinate points with the Kinect to be calibrated
//
//  input: camera points, world points
//  output: transformation matrix from camera coordinate to world coordinate
//  output: position and rotation of Kinect in world coordinate
//  output: mean squared error for given testing file
//


#include <iostream>
#include <fstream>
#include <math.h>
#include "opencv_headers.h"

using namespace std;

ofstream logFile;

// calculate a transformation matrix with point positions from camera and world
// the transformation matrix includes rotation and translation
void calTransformationMatrix(const vector<cv::Point3_<float> >& cameraPoints, const vector<cv::Point3_<float> >& worldPoints, cv::Mat& ret)
{
    // input validity is assured by the call function
    int numOfPoints = cameraPoints.size();

    // convert vector of points into matrix of points
    cv::Mat cameraMatrix = cv::Mat::ones(4, numOfPoints, CV_32F);
    for(int i=0; i<numOfPoints; i++)
    {
        cameraMatrix.at<float>(0,i) = cameraPoints[i].x;
        cameraMatrix.at<float>(1,i) = cameraPoints[i].y;
        cameraMatrix.at<float>(2,i) = cameraPoints[i].z;
    }

    // convert vector of points into matrix of points
    cv::Mat worldMatrix = cv::Mat::ones(4, numOfPoints, CV_32F);
    for(int i=0; i<numOfPoints; i++)
    {
        worldMatrix.at<float>(0,i) = worldPoints[i].x;
        worldMatrix.at<float>(1,i) = worldPoints[i].y;
        worldMatrix.at<float>(2,i) = worldPoints[i].z;
    }

    // calculation of transformation matrix
    ret = worldMatrix * cameraMatrix.t() * (cameraMatrix * cameraMatrix.t()).inv();

    cout << "transformation matrix:" << ret << endl << endl;
    logFile << "transformation matrix:" << ret << endl << endl;

    return;
}

// decomposing the rotation data from transformation matrix
// which is the first three row and three column of the transformation matrix
void decomposeRotation(const cv::Mat& matrix)
{
    double radX, radY, radZ, degX, degY, degZ;

    // decomposing the rotation data
    // i made a mistake here of forgetting that matrix index start from 0 and spend long time debugging...
    radX = atan2(matrix.at<float>(2,1), matrix.at<float>(2,2));
    radY = atan2(-matrix.at<float>(2,0), sqrt(matrix.at<float>(2,1)*(matrix.at<float>(2,1))+matrix.at<float>(2,2)*matrix.at<float>(2,2)));
    radZ = atan2(matrix.at<float>(1,0), matrix.at<float>(0,0));

    // logging information
    cout << "radians x: " << radX << endl;
    cout << "radians y:" << radY << endl;
    cout << "radians z:" << radZ << endl;
    logFile << "radians x: " << radX << endl;
    logFile << "radians y:" << radY << endl;
    logFile << "radians z:" << radZ << endl;

    // convert from radian to degree
    degX = radX * (180.0/M_PI);
    degY= radY * (180.0/M_PI);
    degZ = radZ * (180.0/M_PI);

    // logging information
    cout << "degree x: " << degX << endl;
    cout << "degree y:" << degY << endl;
    cout << "degree z:" << degZ << endl;
    logFile << "degree x: " << degX << endl;
    logFile << "degree y:" << degY << endl;
    logFile << "degree z:" << degZ << endl;

    cout << endl;
    logFile << endl;
}

// read calibration data from file
// calibration data are fetched from multiple Kinects
// and automatically generated with code
void readCalibrationData(vector<cv::Point3_<float> >& cameraPoints, vector<cv::Point3_<float> >& worldPoints, string& filename)
{
    string str;
    ifstream ifs;
    float a, b, c, d, e, f;
    string filepath = "data/"+filename+".txt";
    ifs.open(filepath.c_str());

    // loop until the end of file
    while(getline(ifs, str))
    {
        // process only it's not an empty line
        if(!str.empty())
        {
            // for format "x1 y1 z1 x2 y2 z2\n"
            //istringstream iss(str);
            //iss >> a >> b >> c >> d >> e >> f;
            // for format "x1,y1,z1;x2,y2,z2\n", which is easier to process in Matlab
            sscanf(str.c_str(), "%f,%f,%f;%f,%f,%f", &a, &b, &c, &d, &e, &f);

            cameraPoints.push_back(cv::Point3_<float>(a,b,c));
            worldPoints.push_back(cv::Point3_<float>(d,e,f));
        }
    }

    ifs.close();
}

// apply transformation matrix to camera points, which will get the calculated world point
// logging original camera points, calculated world points, and actual world points for comparison
// calculate the mean square error
void applyTransformation(vector<cv::Point3_<float> >& cameraPoints, vector<cv::Point3_<float> >& worldPoints, cv::Mat& tranMat)
{
    int numPoints = cameraPoints.size();
    vector<cv::Point3_<float> > calPoints;
    cv::Mat pointMat = cv::Mat::ones(4, 1, CV_32F);
    cv::Mat calMat;

    // apply transformation and get calculated world points
    for(int i=0; i<numPoints; ++i)
    {
        pointMat.at<float>(0,0) = cameraPoints[i].x;
        pointMat.at<float>(1,0) = cameraPoints[i].y;
        pointMat.at<float>(2,0) = cameraPoints[i].z;

        calMat = tranMat * pointMat;

        calPoints.push_back(cv::Point3_<float>(calMat.at<float>(0,0),calMat.at<float>(1,0),calMat.at<float>(2,0)));

        cout << "point " << i << ": " << endl;
        cout << "original: " << cameraPoints[i].x << "  " << cameraPoints[i].y << "  " << cameraPoints[i].z << endl;
        cout << "calculated: " << calMat.at<float>(0,0) << "  " << calMat.at<float>(1,0) << "  " << calMat.at<float>(2,0) << endl;
        cout << "world: " << worldPoints[i].x << "  " << worldPoints[i].y << "  " << worldPoints[i].z << endl;
        logFile << "point " << i << ": " << endl;
        logFile << "original: " << cameraPoints[i].x << "  " << cameraPoints[i].y << "  " << cameraPoints[i].z << endl;
        logFile << "calculated: " << calMat.at<float>(0,0) << "  " << calMat.at<float>(1,0) << "  " << calMat.at<float>(2,0) << endl;
        logFile << "world: " << worldPoints[i].x << "  " << worldPoints[i].y << "  " << worldPoints[i].z << endl;

        cout << endl;
        logFile << endl;
    }

    // calculate mean squared error
    double mse = 0.0;
    for(int i=0; i<numPoints; ++i)
    {
        mse += sqrt(pow((worldPoints[i].x-calPoints[i].x),2) + pow((worldPoints[i].y-calPoints[i].y),2) + pow((worldPoints[i].z-calPoints[i].z),2));
    }
    mse = mse/numPoints;
    cout << "mean squared error: " << mse << endl;
    logFile << "mean squared error: " << mse << endl;

    cout << endl;
    logFile << endl;

    return;
}

// input: points set data file name
// output: rotation and position of camera
// output: transformation matrix from camera coordinate to world coordinate
int main()
{
    while(true)
    {
        // get input file name
        string fileName;
        cout << "file name (press enter to exit): ";
        getline(cin, fileName);
        if(fileName.empty())
            break;

        // create log file
        string filepath = "data/result_"+fileName+".txt";
        logFile.open(filepath.c_str(), ios::trunc);

        // define input data structures
        vector<cv::Point3_<float> > cameraPoints;
        vector<cv::Point3_<float> > worldPoints;

        // define output data structures
        cv::Mat tranMat;
        cv::Vec3f cameraPosition;
        cv::Vec3f cameraRotation;

        // load input data from file to data structures
        readCalibrationData(cameraPoints, worldPoints, fileName);

        // calibrate transformation matrix with least squares method
        calTransformationMatrix(cameraPoints, worldPoints, tranMat);

        // decompose rotation and position info from transformation matrix
        decomposeRotation(tranMat);

        // get testing file
        string testFileName;
        cout << "testing file name (press enter to exit): ";
        getline(cin, testFileName);
        if(!testFileName.empty())
        {
            // define testing data set
            vector<cv::Point3_<float> > testCameraPoints;
            vector<cv::Point3_<float> > testWorldPoints;
            // load testing data from file to data structures
            readCalibrationData(testCameraPoints, testWorldPoints, testFileName);
            // apply transformation
            applyTransformation(testCameraPoints, testWorldPoints, tranMat);
        }

        // close log file
        logFile.close();
    }

    return 0;
}

/*
    cv::Mat matrix(3,3,CV_32F);

    cv::Mat mean = cv::Mat::ones(1, 1, CV_32F);
    cv::Mat sigma = cv::Mat::ones(1, 1, CV_32F);

    cv::randn(matrix, mean, sigma);
    cout << "origial:" << endl << matrix << endl << endl;

    cv::Mat transpose = matrix.t();
    cout << "transpose:" << endl << transpose << endl << endl;

    cv::Mat inverse = matrix.inv();
    cout << "inverse:" << endl << inverse << endl << endl;

    cv::Mat multiply = matrix * inverse;
    cout << "multiply:" << endl << multiply << endl << endl;

    cv::Mat init = (cv::Mat_<float>(3,3)<<1.1, 2.2, 3.3 ,4.4 ,5.5 ,6.6 ,7.7 ,8.8, 9.9);
    cout << "init:" << endl << init << endl << endl;
*/
