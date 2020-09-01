#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/xfeatures2d/cuda.hpp>
#include <opencv2/xfeatures2d.hpp>

//Name spaces used 
using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;

int main() {

	/*
	//TEST Camera, funziona!
	VideoCapture cap1(0); // open the default camera
	if (!cap1.isOpened())  // check if we succeeded
	return -1;

	Mat edges;
	namedWindow("edges", 1);
	for (;;)
	{
	Mat frame;
	cap1 >> frame; // get a new frame from camera
	cvtColor(frame, edges, CV_BGR2GRAY);
	imshow("edges", edges);
	if (waitKey(30) >= 0) break;
	}
	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
	*/

	//Load training image
	Mat object = imread("C:/Users/Domenico/Desktop/floralys.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	if (!object.data) {
		cout << "Can't open image";
		return -1;
	}

	//SURF Detector, and descriptor parameters
	//CAPIRE BENE QUESTI PARAMETRI
	int minHess = 500;
	float thresholdMatchingNN = 0.5;
	unsigned int thresholdGoodMatches = 10;

	vector<KeyPoint> kpObject;
	Mat desObject;
	FlannBasedMatcher matcher;

	Ptr<SURF> surf1 = SURF::create(minHess);
	surf1->detectAndCompute(object, Mat(), kpObject, desObject);

	//Initialize video and display window
	VideoCapture cap(0);
	if (!cap.isOpened()) return -1;
	namedWindow("Good Matches", CV_WINDOW_AUTOSIZE);


	//Object corner points for plotting box
	vector<Point2f> obj_corners(4);
	obj_corners[0] = cvPoint(0, 0);
	obj_corners[1] = cvPoint(object.cols, 0);
	obj_corners[2] = cvPoint(object.cols, object.rows);
	obj_corners[3] = cvPoint(0, object.rows);

	//Video loop
	for (;;)
	{

		Mat frame;
		Mat image;
		cap >> frame;
		cvtColor(frame, image, CV_RGB2GRAY);


		Mat des_image, img_matches, H;
		vector<KeyPoint> kp_image;
		vector<vector<DMatch > > matches;
		vector<DMatch > good_matches;
		vector<Point2f> obj;
		vector<Point2f> scene;
		vector<Point2f> scene_corners(4);

		surf1->detectAndCompute(image, Mat(), kp_image, des_image);


		if (des_image.rows >= 2 && desObject.rows >= 2) {

			matcher.knnMatch(desObject, des_image, matches, 2);

			for (int i = 0; i < min(des_image.rows - 1, (int)matches.size()); i++) //THIS LOOP IS SENSITIVE TO SEGFAULTS
			{
				if ((matches[i][0].distance < thresholdMatchingNN*(matches[i][1].distance)) && ((int)matches[i].size() <= 2 && (int)matches[i].size() > 0))
				{
					good_matches.push_back(matches[i][0]);
				}
			}

			//Draw only "good" matches
			drawMatches(object, kpObject, image, kp_image, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
			//drawMatches(object, kpObject, image, kp_image, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
			//drawMatches(object, kpObject, image, kp_image, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::DEFAULT);

			if (good_matches.size() >= thresholdGoodMatches)
			{

				//Display that the object is found
				putText(img_matches, "Object Found", cvPoint(10, 50), FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0, 0, 250), 1, CV_AA);
				for (unsigned int i = 0; i < good_matches.size(); i++)
				{
					//Get the keypoints from the good matches
					obj.push_back(kpObject[good_matches[i].queryIdx].pt);

					scene.push_back(kp_image[good_matches[i].trainIdx].pt);
				}

				H = findHomography(obj, scene, CV_RANSAC);

				if (!H.empty())
				{
					perspectiveTransform(obj_corners, scene_corners, H);

					//Draw lines between the corners (the mapped object in the scene image )
					line(img_matches, scene_corners[0] + Point2f(object.cols, 0), scene_corners[1] + Point2f(object.cols, 0), Scalar(0, 255, 0), 4);
					line(img_matches, scene_corners[1] + Point2f(object.cols, 0), scene_corners[2] + Point2f(object.cols, 0), Scalar(0, 255, 0), 4);
					line(img_matches, scene_corners[2] + Point2f(object.cols, 0), scene_corners[3] + Point2f(object.cols, 0), Scalar(0, 255, 0), 4);
					line(img_matches, scene_corners[3] + Point2f(object.cols, 0), scene_corners[0] + Point2f(object.cols, 0), Scalar(0, 255, 0), 4);
				}
			}
			else
			{
				putText(img_matches, "", cvPoint(10, 50), FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0, 0, 250), 1, CV_AA);
			}
		}

		//Show detected matches
		imshow("Good Matches", img_matches);

		//Key for exit,
		if (waitKey(30) >= 0) break;

	}

	return 0;

}