#ifndef matching2d_hpp
#define matching2d_hpp
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <limits>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/plot.hpp>
#include <opencv2/opencv.hpp>

#include "dataStructures.h"

using namespace std;
class performEvaluation
{
	string detectorType = "";
	string descriptorType = "";
	double detTimeTotal = 0.0;
	double desTimeTotal = 0.0;
	double matTimeTotal = 0.0;
	int nDetected = 0;
	int nMatched = 0;
	
	public:
	void setDetector(string inputDetector)
	{
		detectorType = inputDetector;
	}

	void setDescriptor(string inputDescriptor)
	{
		descriptorType = inputDescriptor;
	}
	
	void totalDetTime(double t)
	{
		detTimeTotal += t;
	}
	
	void totalDesTime(double t)
	{
		desTimeTotal += t;
	}
	
	void totalMatTime(double t)
	{
		matTimeTotal += t;
	}

	void setNDetected(vector<cv::KeyPoint> &keypoints)
	{
		nDetected = keypoints.size();
	
	}

	void setNMatched(vector<cv::DMatch> &matches)
	{
		nMatched = matches.size();
	}
	
	void showResult() {
		cout << "Detector Type : " << detectorType << endl;
		cout << "Descriptor Type : " << descriptorType << endl;
		cout << "Time taken for Detector : " << log(detTimeTotal) << endl;
		cout << "Time taken for Descriptor : " << log(desTimeTotal) << endl;
		cout << "Time taken for Matcher : " << log(matTimeTotal) << endl;
		cout << "Number of Detected Keypoints : " << nDetected << endl;
		cout << "Number of Matched Keypoints : " << nMatched << endl;
	}

};


double detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis=false);
double detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis=false);
double detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis=false);
double descKeypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, std::string descriptorType);
double matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef, std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType);
#endif
