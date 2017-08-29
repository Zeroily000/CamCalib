#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>


const int CAM_NUM = 2;

int main(){
	int nframes = 29;
	int chessRows = 6;
	int chessCols = 9;
	float len = 0.02423;
	cv::Size patternSize(chessCols, chessRows);
	
	std::vector<std::string> prefix = {"/left", "/right"};
	
	cv::Mat K[CAM_NUM];
	cv::Mat D[CAM_NUM];
	
	std::vector<cv::Point3f> objs;
	// Set objectPoints
	for(int i = 0; i < chessRows; i++)
		for(int j = 0; j < chessCols; j++)
			objs.push_back(cv::Point3f(float(j)*len, float(i)*len, 0));

	std::string filename;
	cv::Mat img, gray;
	for(int cam = 0; cam < CAM_NUM; cam++){
		std::vector<std::vector<cv::Point2f>> imagePoints;
		std::vector<std::vector<cv::Point3f>> objectPoints;
		std::vector<cv::Point2f> corners;
		for(int i = 1; i <= nframes; i++){
			// Read image
			filename = "chessboard" + prefix[cam] + prefix[cam] + std::to_string(i) + ".jpg";
			img = cv::imread(filename.c_str());
			cv::cvtColor(img, gray, CV_BGR2GRAY);
			// Find imagePoints
			bool found = cv::findChessboardCorners(img, patternSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FILTER_QUADS);

			if(found){
				cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
				imagePoints.push_back(corners);
				objectPoints.push_back(objs);

				// cv::drawChessboardCorners(img, patternSize, corners, found);
				// cv::imshow("test", img);
				// cv::waitKey(0);
			}
		}
		// Calibrate
		std::vector<cv::Mat> rvecs, tvecs;
		cv::calibrateCamera(objectPoints, imagePoints, img.size(), K[cam], D[cam], rvecs, tvecs);

		// Output
		std::cout << "K" << cam << ": " << std::endl;
		std::cout << K[cam] << std::endl;
		std::cout << std::endl;
		std::cout << "D" << cam << ": " << std::endl;
		std::cout << D[cam] << std::endl;
		std::cout << std::endl;
	}
	return 0;
}