/*------------------------------------------------------------------------------------------*\
   This file contains material supporting chapter 9 of the cookbook:  
   Computer Vision Programming using the OpenCV Library. 
   by Robert Laganiere, Packt Publishing, 2011.

   This program is free software; permission is hereby granted to use, copy, modify, 
   and distribute this source code, or portions thereof, for any purpose, without fee, 
   subject to the restriction that the copyright notice may not be removed 
   or altered from any source or altered source distribution. 
   The software is released on an as-is basis and without any warranties of any kind. 
   In particular, the software is not guaranteed to be fault-tolerant or free from failure. 
   The author disclaims all warranties with regard to this software, any use, 
   and any consequent failure, is purely the responsibility of the user.
 
   Copyright (C) 2010-2011 Robert Laganiere, www.laganiere.name
\*------------------------------------------------------------------------------------------*/

#include <iostream>
#include <iomanip>       //std::setw(2) << std::setfill('0') << i << ".jpg";//输出0i.jpg
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/opencv.hpp"

#include "./class/CameraCalibrator.h"
#include "./class/CameraCalibrator.h"

using namespace cv;
int main()
{

	cv::namedWindow("Image");
	cv::Mat image;
	std::vector<std::string> filelist;
    FileStorage camcalibrate("./data/camcalibrate.xml", FileStorage::WRITE);
    camcalibrate << "read" << "camera calibration by kint at 15.02.03";
    camcalibrate << "imageCount" << 12;
	// generate list of chessboard image filename
    for (int i=1; i<=12; i++)
    {
		std::stringstream str;
        str << "./chessboards/" << std::setw(2) << std::setfill('0') << i << ".jpg"; //输出0i.jpg
		std::cout << str.str() << std::endl;
        filelist.push_back(str.str());
		image= cv::imread(str.str(),0);
        imshow("Image",image);
		 cv::waitKey(100);
	}
	// Create calibrator object
    CameraCalibrator cameraCalibrator;
	// add the corners from the chessboard
    cv::Size boardSize(5,4);
	cameraCalibrator.addChessboardPoints(
		filelist,	// filenames of chessboard image
        boardSize,
        26.5,26.5);	// size of chessboard
		// calibrate the camera
    //	cameraCalibrator.setCalibrationFlag(true,true);
//    std::cout<<image.size()<<std::endl;
    cameraCalibrator.calibrate(image.size());   //相机校正

    // Image Undistortion
    image = cv::imread(filelist[0]);
	cv::Mat uImage= cameraCalibrator.remap(image);


    cv::Mat cameraMatrix = cameraCalibrator.getCameraMatrix();
    cv::Mat DistCoeffsMatrix = cameraCalibrator.getDistCoeffs();
    std::vector<cv::Mat> rot_vectors = cameraCalibrator.getrot();
    std::vector<cv::Mat> trans_vectors = cameraCalibrator.gettrans();

// display camera matrix
    camcalibrate<< "Camera_Matrix" << cameraMatrix << "Distortion_Coefficients" << DistCoeffsMatrix;
    camcalibrate<< "rot_vectors" << rot_vectors << "trans_vectors" << trans_vectors;

/* display()
//	std::cout << " Camera intrinsic: " << cameraMatrix.rows << "x" << cameraMatrix.cols << std::endl;
//	std::cout << cameraMatrix.at<double>(0,0) << " " << cameraMatrix.at<double>(0,1) << " " << cameraMatrix.at<double>(0,2) << std::endl;
//	std::cout << cameraMatrix.at<double>(1,0) << " " << cameraMatrix.at<double>(1,1) << " " << cameraMatrix.at<double>(1,2) << std::endl;
//	std::cout << cameraMatrix.at<double>(2,0) << " " << cameraMatrix.at<double>(2,1) << " " << cameraMatrix.at<double>(2,2) << std::endl;

//    std::cout << " " << std::endl;
//    std::cout << " DistCoeffsMatrix: " << DistCoeffsMatrix.rows << "x" << DistCoeffsMatrix.cols << std::endl;
//    std::cout << DistCoeffsMatrix.at<double>(0,0) << " " <<  DistCoeffsMatrix.at<double>(0,1) << " " <<  DistCoeffsMatrix.at<double>(0,2) << std::endl;
//    std::cout << DistCoeffsMatrix.at<double>(0,3) << " " <<  DistCoeffsMatrix.at<double>(0,4) <<  DistCoeffsMatrix.at<double>(7) << std::endl;
*/

    imshow("Original Image", image);
    imshow("Undistorted Image", uImage);

    camcalibrate.release();//  xml文件完成

    FileStorage out("./data/camcalibrate.xml", FileStorage::READ);//  导出xml文件
    cv::Mat outMat =cv::Mat::eye(3, 3, CV_64F);
    cv::Mat outMat2 = cv::Mat::zeros(5, 1, CV_64F);

    out["Camera_Matrix"] >>  outMat ;
    out["Distortion_Coefficients"] >> outMat2 ;

    std::cout << " Camera intrinsic: " << outMat.rows << "x" << outMat.cols << std::endl;
    std::cout << outMat<< std::endl ;
    std::cout << " " << std::endl;
    std::cout << outMat.at<double>(0,0) << " " << outMat.at<double>(1,1) << " " << outMat.at<double>(2,2)<< " " << outMat.at<double>(0,2)<< " " << outMat.at<double>(1,2) << std::endl ;
    std::cout << " " << std::endl;
    std::cout << " DistCoeffsMatrix: " << outMat2.rows << "x" << outMat2.cols << std::endl;
    std::cout << outMat2<< std::endl ;

    cv::waitKey();
	return 0;

}

