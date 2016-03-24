//
//  calibration_rigid_motion.cpp
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
//  differences between rigid motion method and least squares method:
//  - rigid motion method captures transformation only about Euclidean transformation (rigid motion)
//  - least squares method captures all kinds of transformation (Euclidean, Similarity, Affine, Projective)
//  
//  result from multiple testing data sets:
//  - in most cases, least squares method has slightly lower error than rigid motion method
//  - in some cases, least squares method has much higher error than rigid motion method
//  - conclusion: rigid motion method is more stable for camera calibration
//


#include <iostream>
#include <fstream>
#include <math.h>
#include "opencv_headers.h"

using namespace std;

ofstream logFile;


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

// calculate a transformation matrix with point positions from camera and world
// the transformation matrix includes rotation and translation
void lsCalTransformationMatrix(const vector<cv::Point3_<float> >& cameraPoints, const vector<cv::Point3_<float> >& worldPoints, cv::Mat& ret)
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

    cout << "transformation matrix:" << ret << endl;
    logFile << "transformation matrix:" << ret << endl;

    return;
}

void rmCalTransformationMatrix(cv::Mat& rotMat, cv::Mat& posMat, cv::Mat& tranMat)
{
    // calculate by mapping rotatin and position matrix
    tranMat = cv::Mat::zeros(4,4,CV_32F);
    for(int i=0; i<3; ++i)
        for(int j=0; j<3; ++j)
            tranMat.at<float>(i,j) = rotMat.at<float>(i,j);
    for(int i=0; i<3; ++i)
        tranMat.at<float>(i,3) = posMat.at<float>(i,0);
    tranMat.at<float>(3,3) = 1;

    // log down data
    cout << "transformation matrix:" << tranMat << endl;
    logFile << "transformation matrix:" << tranMat << endl;

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

void rmCalRotationPosition(const vector<cv::Point3_<float> >& cameraPoints, const vector<cv::Point3_<float> >& worldPoints, cv::Mat& rotRet, cv::Mat& posRet)
{
    int numPoints = cameraPoints.size();
    double tempX, tempY, tempZ;

    // compute the 3x1 centroids of both point sets
    cv::Mat centroidCamera = cv::Mat(3, 1, CV_32F);
    cv::Mat centroidWorld = cv::Mat(3, 1, CV_32F);
    tempX = tempY = tempZ = 0.0;
    for(int i=0; i<numPoints; ++i)
    {
        tempX += cameraPoints[i].x;
        tempY += cameraPoints[i].y;
        tempZ += cameraPoints[i].z;
    }
    centroidCamera.at<float>(0,0) = tempX/numPoints;
    centroidCamera.at<float>(1,0) = tempY/numPoints;
    centroidCamera.at<float>(2,0) = tempZ/numPoints;
    tempX = tempY = tempZ = 0.0;
    for(int i=0; i<numPoints; ++i)
    {
        tempX += worldPoints[i].x;
        tempY += worldPoints[i].y;
        tempZ += worldPoints[i].z;
    }
    centroidWorld.at<float>(0,0) = tempX/numPoints;
    centroidWorld.at<float>(1,0) = tempY/numPoints;
    centroidWorld.at<float>(2,0) = tempZ/numPoints;

    // compute the 3xn centered vectors of both point sets
    cv::Mat centeredCamera = cv::Mat(3, numPoints, CV_32F);
    cv::Mat centeredWorld = cv::Mat(3, numPoints, CV_32F);
    for(int i=0; i<numPoints; ++i)
    {
        centeredCamera.at<float>(0,i) = cameraPoints[i].x - centroidCamera.at<float>(0,0);
        centeredCamera.at<float>(1,i) = cameraPoints[i].y - centroidCamera.at<float>(1,0);
        centeredCamera.at<float>(2,i) = cameraPoints[i].z - centroidCamera.at<float>(2,0);
        centeredWorld.at<float>(0,i) = worldPoints[i].x - centroidWorld.at<float>(0,0);
        centeredWorld.at<float>(1,i) = worldPoints[i].y - centroidWorld.at<float>(1,0);
        centeredWorld.at<float>(2,i) = worldPoints[i].z - centroidWorld.at<float>(2,0);
    }

    // compute the 3x3 covariance matrix
    cv::Mat covMat = centeredCamera * centeredWorld.t();

    // compute the singular value decomposition of covariance matrix
    cv::Mat w;      // calculated singular values
    cv::Mat u;      // calculated left singular vectors
    cv::Mat vt;     // 	transposed matrix of right singular values
    cv::SVD::compute(covMat, w, u, vt);
    cv::Mat ut = u.t();
    cv::Mat v = vt.t();

    // compute the rotation matrix
    cv::Mat tempMat = cv::Mat::zeros(3,3,CV_32F);
    tempMat.at<float>(0,0) = 1;
    tempMat.at<float>(1,1) = 1;
    tempMat.at<float>(2,2) = cv::determinant(v*ut);
    rotRet = v*tempMat*ut;

    // compute the translation
    posRet = centroidWorld-(rotRet*centroidCamera);
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
/*
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
*/
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
        cout << "training data file name (press enter to exit): ";
        getline(cin, fileName);
        if(fileName.empty())
            break;

        // create log file
        string filepath = "data/compare_"+fileName+".txt";
        logFile.open(filepath.c_str(), ios::trunc);

        // define input data structures
        vector<cv::Point3_<float> > cameraPoints;
        vector<cv::Point3_<float> > worldPoints;

        // define output data structures
        cv::Mat lsTranMat;
        cv::Mat rmTranMat;
        cv::Mat cameraRotation;
        cv::Mat cameraPosition;

        // load input data from file to data structures
        readCalibrationData(cameraPoints, worldPoints, fileName);

        // for least squares
        cout << "---- least squares ----" << endl;
        logFile << "---- least squares ----" << endl;
        lsCalTransformationMatrix(cameraPoints, worldPoints, lsTranMat);
        decomposeRotation(lsTranMat);

        // for rigid motion
        cout << "---- rigid motion ----" << endl;
        logFile << "---- rigid motion ----" << endl;
        rmCalRotationPosition(cameraPoints, worldPoints, cameraRotation, cameraPosition);
        rmCalTransformationMatrix(cameraRotation, cameraPosition, rmTranMat);
        decomposeRotation(cameraRotation);

        while(true)
        {
            // get testing file
            string testFileName;
            cout << "testing file name (press enter to exit): ";
            getline(cin, testFileName);
            if(testFileName.empty())
                break;
            // define testing data set
            vector<cv::Point3_<float> > testCameraPoints;
            vector<cv::Point3_<float> > testWorldPoints;
            // load testing data from file to data structures
            readCalibrationData(testCameraPoints, testWorldPoints, testFileName);
            // apply transformation
            cout << "---- least squares ----" << endl;
            logFile << "---- least squares ----" << endl;
            applyTransformation(testCameraPoints, testWorldPoints, lsTranMat);
            cout << "---- rigid motion ----" << endl;
            logFile << "---- rigid motion ----" << endl;
            applyTransformation(testCameraPoints, testWorldPoints, rmTranMat);
        }

        // close log file
        logFile.close();
    }

    return 0;
}
