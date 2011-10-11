/*
 * main.cpp
 *
 *  Created on: Aug 17, 2011
 *      Author: root
 */

// The short example shows how to use new-style image classes declared in cxcore.hpp.
// There is also a very similar matrix class (CvMatrix) - a wrapper for CvMat

using namespace std;
// OpenCV libraries
#include <cv.h>
#include <highgui.h>
#include <cvaux.h>
#include <cxcore.h>

using namespace cv;

#include <iostream>
#include <stdio.h>
#include <string.h>
// OpenKinect libraries
#include <libfreenect.h>
#include <libfreenect_cv.h>
#include <libfreenect_sync.h>

#include <vector>
#include <cmath>
#include <pthread.h>

#include<sys/stat.h>
#include<sys/types.h>

// Some definitions for OpenKinect
#define         FREENECT_FRAME_W   640
#define         FREENECT_FRAME_H   480
#define         FREENECT_FRAME_PIX   (FREENECT_FRAME_H*FREENECT_FRAME_W) //width*height pixels in the image
#define         FREENECT_VIDEO_RGB_SIZE   (FREENECT_FRAME_PIX*3) //3 bytes per pixel
#define 		FREENECT_DEPTH_11BIT_SIZE   (FREENECT_FRAME_PIX*sizeof(uint16_t))

// functions to bridge OpenCV and OpenKinect
IplImage *freenect_sync_get_depth_cv(int index){
	static IplImage *image = 0;
	static char *data = 0;
	if (!image) image = cvCreateImageHeader(cvSize(640,480), 16, 1);
	unsigned int timestamp;
	if (freenect_sync_get_depth((void**)&data, &timestamp, index, FREENECT_DEPTH_11BIT))
	    return NULL;
	cvSetData(image, data, 640*2);
	return image;
}

IplImage *freenect_sync_get_rgb_cv(int index){
	static IplImage *image = 0;
	static char *data = 0;
	if (!image) image = cvCreateImageHeader(cvSize(640,480), 8, 3);
	unsigned int timestamp;
	if (freenect_sync_get_video((void**)&data, &timestamp, index, FREENECT_VIDEO_RGB))
	    return NULL;
	cvSetData(image, data, 640*3);
	return image;
}

//Builds a colored 8-bit 3 channel image based on depth image
IplImage *GlViewColor(IplImage *depth){
	static IplImage *image = 0;
	if (!image) image = cvCreateImage(cvSize(640,480), 8, 3);
	unsigned char *depth_mid = (unsigned char*)(image->imageData);
	int i;

	for (i = 0; i < 640*480; i++) {
		int lb = ((short *)depth->imageData)[i] % 256;
		int ub = ((short *)depth->imageData)[i] / 256;
		switch (ub) {

			case 0:
			depth_mid[3*i+2] = 255;
			depth_mid[3*i+1] = 255-lb;
			depth_mid[3*i+0] = 255-lb;
			break;

			case 1:
			depth_mid[3*i+2] = 255;
			depth_mid[3*i+1] = lb;
			depth_mid[3*i+0] = 0;
			break;

			case 2:
			depth_mid[3*i+2] = 255-lb;
			depth_mid[3*i+1] = 255;
			depth_mid[3*i+0] = 0;
			break;

			case 3:
			depth_mid[3*i+2] = 0;
			depth_mid[3*i+1] = 255;
			depth_mid[3*i+0] = lb;
			break;

			case 4:
			depth_mid[3*i+2] = 0;
			depth_mid[3*i+1] = 255-lb;
			depth_mid[3*i+0] = 255;
			break;

			case 5:
			depth_mid[3*i+2] = 0;
			depth_mid[3*i+1] = 0;
			depth_mid[3*i+0] = 255-lb;
			break;

			default:
			depth_mid[3*i+2] = 0;
			depth_mid[3*i+1] = 0;
			depth_mid[3*i+0] = 0;
			break;
		}
	}
	return image;
}

// Save the contours on disk. Current implementation is via xml files on file system. future goal is to implement database abstraction
void SaveContour(CvSeq* contour[], string objname){
	const char* attrs[] = {"recursive", "1", 0};
	cout << "the file name will be " << objname << endl;
	string objname1="./objects/"+objname+"_1.xml";
	string objname2="./objects/"+objname+"_2.xml";
	string objname3="./objects/"+objname+"_3.xml";
	cvSave(objname1.c_str(), contour[0], 0, 0, cvAttrList(attrs, 0));
	cvSave(objname2.c_str(), contour[1], 0, 0, cvAttrList(attrs, 0));
	cvSave(objname3.c_str(), contour[2], 0, 0, cvAttrList(attrs, 0));
}

/*
void sum_bgr(IplImage* src, IplImage* dst, int thresh){
	//Alllocate individual image planes
	IplImage* r = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
	IplImage* g = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
	IplImage* b = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);

	//Split image onto the color planes
	cvSplit(src, b, g, r, NULL);

	//Temporary storage
	IplImage* s = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);

	//Add equally weighted rgb values
	cvAddWeighted(r, 1./3., g, 1./3., 0.0, s);
	cvAddWeighted(s, 2./3., b, 1./3., 0.0, s);

	//Trunate values above val
	cvThreshold (s, dst, thresh, 256, CV_THRESH_BINARY);

	cvReleaseImage(&r);
	cvReleaseImage(&g);
	cvReleaseImage(&b);
	cvReleaseImage(&s);
}
*/
// take an image and store identity as contours at various depths by click of space bar
int main(int argc, char **argv)
{
	// declare variables
	IplImage *image = freenect_sync_get_rgb_cv(0);
	IplImage *depth = freenect_sync_get_depth_cv(0);
	IplImage *gray_depth = NULL;
	IplImage *depth1 = NULL;
	IplImage *depth2 = NULL;
	IplImage *depth3 = NULL;
	cvNamedWindow("RGB");
	cvNamedWindow("Depth");
	cvNamedWindow("Depth1");
	cvNamedWindow("Depth2");
	cvNamedWindow("Depth3");
	CvSeq *contour[3];
	contour[0] = 0;
	contour[1] = 0;
	contour[2] = 0;
	CvMemStorage *storage1 = NULL;
	CvMemStorage *storage2 = NULL;
	CvMemStorage *storage3 = NULL;
	string objname = "";
	string delreply = "";
	
	// if the folder for objects is not created, go ahead and create it
	struct stat st;
	if(stat("/tmp",&st) != 0){
		if(mkdir("./objects",0777)==-1)//creating a directory
		{
			cout << "Error while creating objects directory\n";
			exit(1);
		}
	}

	//loop to display video
	while (1) {
		//error checking and set up work
		if (!image) {
			printf("Error: Kinect not connected?\n");
			return -1;
		}
		image = freenect_sync_get_rgb_cv(0);
		cvCvtColor(image, image, CV_RGB2BGR);
		depth = freenect_sync_get_depth_cv(0);
		if (!depth) {
			printf("Error: Kinect not connected?\n");
			return -1;
		}

		// start working on images and derivative images
		depth = GlViewColor(depth);
		cvShowImage("RGB", image);
		cvShowImage("Depth", depth);

		char key = cvWaitKey(10);

		// esc is pressed - exit the program
		if (key == 27) {
			cvReleaseImage(&depth1);
			cvReleaseImage(&depth2);
			cvReleaseImage(&depth3);
			cvReleaseImage(&image);
			cvReleaseImage(&depth);
			cvReleaseImage(&gray_depth);
			cvDestroyWindow("RGB");
			cvDestroyWindow("Depth");
			cvDestroyWindow("Depth1");
			cvDestroyWindow("Depth2");
			cvDestroyWindow("Depth3");
			cvClearMemStorage(storage1);
			cvClearMemStorage(storage2);
			cvClearMemStorage(storage3);
			break;
		// space bar is pressed - try to recognize the object
		// If it is an unrecognized object (new object), save its identity
		} else if (key == 32) {
			depth1 = cvCreateImage(cvGetSize(depth), depth->depth, 1);
			depth2 = cvCreateImage(cvGetSize(depth), depth->depth, 1);
			depth3 = cvCreateImage(cvGetSize(depth), depth->depth, 1);
			gray_depth = cvCreateImage(cvGetSize(depth), depth->depth, 1);
			
			cvCvtColor(depth,gray_depth,CV_BGR2GRAY);

			// 20 inches
			cvThreshold (gray_depth, depth1, 180, 256, CV_THRESH_BINARY_INV);
			// ~20.5 inches
			cvThreshold (gray_depth, depth2, 190, 256, CV_THRESH_BINARY_INV);
			// ~21.5 inches
			cvThreshold (gray_depth, depth3, 200, 256, CV_THRESH_BINARY_INV);

			storage1 = cvCreateMemStorage(0);
			storage2 = cvCreateMemStorage(0);
			storage3 = cvCreateMemStorage(0);

			cvFindContours(depth1, storage1, &contour[0]);
			cvFindContours(depth2, storage2, &contour[1]);
			cvFindContours(depth3, storage3, &contour[2]);

			cvZero(depth1);
			cvZero(depth2);
			cvZero(depth3);

			cvDrawContours(depth1, contour[0], cvScalarAll(255), cvScalarAll(255), 100);
			cvDrawContours(depth2, contour[1], cvScalarAll(255), cvScalarAll(255), 100);
			cvDrawContours(depth3, contour[2], cvScalarAll(255), cvScalarAll(255), 100);

			cout << "I have no idea what this is. What is this object?" << endl;
			getline(cin, objname);
			SaveContour(contour, objname);

			cvShowImage("Depth1", depth1);
			cvShowImage("Depth2", depth2);
			cvShowImage("Depth3", depth3);
		// if the delete key is pressed, we'll delete all the saved contour objects
		} else if (key == 100) {
			while (delreply != "y" && delreply != "n"){
				cout << "Are you sure you want to delete the Object database? (y/n) \n";
				getline(cin, delreply);
				// if confirmed, delete everything in the directory and create anew
				if (delreply == "y") {
					cout << "gonna delete some shit\n";
					system("rm -r ./objects");
					if(mkdir("./objects",0777)==-1)//creating a directory
					{
						cout << "Error while creating objects directory\n";
						exit(1);
					}
				}
				else if (delreply == "n") continue;
			}
		}
	}

	return 0;
}

