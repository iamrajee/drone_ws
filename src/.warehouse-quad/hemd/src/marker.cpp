#include "marker.h"

using namespace cv;
using namespace std;
using namespace zbar;
namespace HMDETECTION{
MARKER::MARKER():shelf(1), row(1), col(1), state(DETECTED), flagAlreadyDetected(false){
    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
}

void MARKER::lineCallback(const hemd::line::ConstPtr& msg){
	double currentTime = ros::Time::now().toSec();//check the current time
	if(msg->mode==2 && state==DETECTED && ((currentTime-timeBegin)>5.0)){
		state = HOVER;
	}

	Mat tmp;
	cap >> tmp;
	MARKER::videoCap(tmp); //capture video and do marker operations

	
}

void MARKER::subscriber(){
	ros::NodeHandle nh_;

	//ros::Subscriber imageSub = nh_.subscribe("/usb_cam/image_raw", 10, &MARKER::imageCallback, this);

	markerPub = nh_.advertise<hemd::markerInfo>("/warehouse_quad/marker",10); //publisher for markers
    followPub = nh_.advertise<std_msgs::Bool>("warehouse_quad/follow_line",10);//publisher to command line following after detection
    markerImgPub = nh_.advertise<sensor_msgs::Image>("warehouse_quad/marker/image",10);
	ros::Subscriber sub = nh_.subscribe("/warehouse_quad/line",10, &MARKER::lineCallback, this);

	markerFile.open("markerData.txt");
	bool check=cap.open("http://192.168.42.129:8080/video?x.mjpeg");
	//bool check=cap.open("rtsp://192.168.42.1/live");
	if(!check){
		ROS_ERROR("UABLE TO OPEN CAMERA");
	}
	else{
		ROS_INFO("CAMERA DETECTED");
	}
	//ros::Rate r(84);
	timeBegin = ros::Time::now().toSec();
	while(ros::ok() & check==1){

		ros::spin();		//r.sleep();
	}
	
}

void MARKER::videoCap(cv::Mat tmp){
	advertiseFollow(0);
	if(tmp.empty()){
		ROS_WARN("Image is Empty");
/*		cap.release();
		cap.open("rtsp://192.168.42.1/live");
		ROS_WARN("Camera Detected again");*/
		return;		
	}
	if(state==HOVER){
		ROS_WARN("HOVER");
		state=DETECTION_START;
		advertiseFollow(0);
		return;
	}
	else if(state==DETECTED){
		//ROS_WARN("DETECTED");
		//advertiseFollow(1);
		return;
	}
	else if(state==DETECTION_START){
		ROS_WARN("DETECTION_START");
		timeBegin = ros::Time::now().toSec();
		advertiseFollow(0);
		state=DETECTING;
		return;
	}


	cv::Mat crop = cropImg(tmp); //crop the image
	cv::Mat frame;
	cvtColor( crop, frame, CV_BGR2GRAY ); //convert to gray scale
	publishMarkerImg(frame);

        //cout << barcodeHover.size() <<"\t"<<barcodes.size()<<endl;

	//detect the markers at hover state and store them in hoverBarcode
	detectMarker(frame);

	double currentTime = ros::Time::now().toSec();//check the current time
	//cout << currentTime-timeBegin<<endl;
	if( barcodeHover.size()==2){  //if both barcodes are detected
		ROS_WARN("TWO_MARKERS");
		for(int i=0;i<barcodeHover.size();i++){
			bool check=false; //check if the marker exist in barcodes string array
			for(int j=0;j<barcodes.size();j++){
				if(barcodeHover[i]==barcodes[j]){
					check=true;
				}
			}
			if(check==false){
				barcodes.push_back(barcodeHover[i]);
				qr.marker = barcodeHover[i];

				qr.col = col;
				qr.shelf = shelf;
				for(int j=0; j<ylocation.size(); j++){
					if(j==i){
						continue;
					}
					if(ylocation[i]>ylocation[j]){
						qr.row = 2;
					}
					else{
						qr.row = 1;
					}
				}
				markerFile <<"shelf:"<<"\t"<<qr.shelf<<"row:"<<"\t"<<qr.row<<"col:"<<"\t"<<qr.col<<"QR:"<<"\t"<<qr.marker<<endl;
				markerPub.publish(qr);
			}

		}
//		ros::Duration(20-ros::Time::now().toSec()+currentTime).sleep();
		ros::Duration(10).sleep();
		state=DETECTED;
		timeBegin = ros::Time::now().toSec();
		ROS_WARN("DETECTED");
		advertiseFollow(1);
		barcodeHover.clear();
		ylocation.clear();
		if(col%5==0){
			shelf++;
		}
		if(shelf==2 && col%5==0){
			col=1;
		}
		else{
			col++;
		}
		return;
	}
	else if((currentTime-timeBegin) > 10 && barcodeHover.size()==1){
		ROS_WARN("ONE_MARKER");
		bool check=false; //check if the marker exist in barcodes string array
		for(int j=0;j<barcodes.size();j++){
			if(barcodeHover[0]==barcodes[j]){
				check=true;
			}
		}
		if(check==false){
			barcodes.push_back(barcodeHover[0]);
			qr.marker=barcodeHover[0];
			qr.shelf = shelf;
			qr.col = col;
			if(ylocation[0]>500){
				qr.row =2;
			}
			else{
				qr.row =2;
			}
				markerFile <<"shelf:"<<"\t"<<qr.shelf<<"row:"<<"\t"<<qr.row<<"col:"<<"\t"<<qr.col<<"QR:"<<"\t"<<qr.marker<<endl;
	
			markerPub.publish(qr);
		}
		if(col%5==0){
			shelf++;
		}
		if(shelf==2 && col%5==0){
			col=1;
		}
		else{
			col++;
		}
		ROS_WARN("DETECTED");
		advertiseFollow(1);
		state=DETECTED;
		timeBegin = ros::Time::now().toSec();
		barcodeHover.clear();
		//cout << barcodeHover.size()<<endl;
		ylocation.clear();
		return;
	}
	else if((currentTime-timeBegin) > 10 && barcodeHover.size()==0 && flagAlreadyDetected){
		state=DETECTED;
		ROS_ERROR("Same marker warning");
		ROS_WARN("END DETECTION");
		advertiseFollow(1);
		timeBegin=ros::Time::now().toSec();
		flagAlreadyDetected = false;
	}
	else if((currentTime-timeBegin) > 10 && barcodeHover.size()==0){
		state=DETECTED;
		ROS_WARN("NO MARKER DETECTED");
		ROS_WARN("END_DETECTION");
		advertiseFollow(1);
		timeBegin = ros::Time::now().toSec();
		if(col%5==0){
			shelf++;
		}
		if(shelf==2 && col%5==0){
			col=1;
		}
		else{
			col++;
		}
		return;
	}
	else if(state==DETECTING){
		advertiseFollow(0);
	}
	else{
		return;
	}
}

cv::Mat MARKER::cropImg(cv::Mat tmp){
    cv::Rect roi;
    roi.x = 724;
    roi.y = 0;
    roi.width = 600;
    roi.height = 1535;

    return tmp(roi);
}

void MARKER::detectMarker(cv::Mat frame){

	zbar::Image zbar_image(frame.cols, frame.rows, "Y800", frame.data,frame.cols * frame.rows);
	scanner_.scan(zbar_image);

	for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol){

		std::string barcode = symbol->get_data();
		cout << barcode << endl;
		bool markerCheck = 0;
		// check if the current barcode exits in current string

		for(int i=0;i<barcodeHover.size();i++){
			if(barcode == barcodeHover[i]){
				markerCheck = 1;
			}
		}
		for(int j=0; j<barcodes.size();j++){
			if(barcode == barcodes[j]){
				markerCheck =1;
				flagAlreadyDetected = true;
			}
		}

		if(!markerCheck){
			barcodeHover.push_back(barcode);
			ylocation.push_back(symbol->get_location_y(0));
		}
	 }
}

void MARKER::publishMarkerImg(cv::Mat frame){

	cv_bridge::CvImage outImg;
	outImg.header.stamp   = ros::Time::now();
	outImg.encoding = sensor_msgs::image_encodings::MONO8; // Image encoding
	outImg.image    = frame; // Your cv::Mat

	markerImgPub.publish(outImg.toImageMsg());

}
void MARKER::advertiseFollow(bool follow){

	std_msgs::Bool msg;

	msg.data = follow;
	followPub.publish(msg);
}

void MARKER::imageCallback(const sensor_msgs::Image::ConstPtr& msg){

}

}


