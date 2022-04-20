/*
 * marker.h
 *
 *  Created on: Nov 28, 2017
 *      Author: krishna
 */

#ifndef MARKER_H_
#define MARKER_H_

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>

#include <hemd/markerInfo.h>
#include <hemd/line.h>

#include<opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "zbar.h"

#include <fstream>
#include <iostream>

using namespace std;

namespace HMDETECTION{
class MARKER{
public:
	MARKER();
	void subscriber();
	void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
	void lineCallback(const hemd::line::ConstPtr& msg);
	void detectMarker(cv::Mat frame);
	void publishMarkerImg(cv::Mat frame);
	void videoCap(cv::Mat tmp);
	void advertiseFollow(bool follow);
	cv::Mat cropImg(cv::Mat tmp);

	enum quadState{HOVER, DETECTING, DETECTED, DETECTION_START, DETECTION_END};

private:
	hemd::markerInfo qr; //internal msg for markers

	zbar::ImageScanner scanner_; //zbar scan

    ros::Publisher markerPub; //publish marker info
    ros::Publisher followPub;
    ros::Publisher markerImgPub;

    vector <string> barcodes; //store all the barcodes
    vector <string> barcodeHover; //store two barcodes at hover state
    vector <int> ylocation;
    cv::VideoCapture cap;
    double timeBegin; //time counter when hover is hit

    quadState state; //current state of the quadrotor

    int shelf; //assign marker shelf
    int col; //assign marker col
    int row; //assign marker row

    bool flagAlreadyDetected;

    ofstream markerFile;
};
}

#endif /* MARKER_H_ */
