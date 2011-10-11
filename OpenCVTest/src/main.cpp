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

#include <sstream>
#include <iostream>
#include <fstream>
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
#include <dirent.h>

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

string convertInt(int number)
{
    if (number == 0)
        return "0";
    string temp="";
    string returnvalue="";
    while (number>0)
    {
        temp+=number%10+48;
        number/=10;
    }
    for (int i=0;i<temp.length();i++)
        returnvalue+=temp[temp.length()-i-1];
    return returnvalue;
}
// Save the contours on disk. Current implementation is via xml files on file system. future goal is to implement database abstraction
void SaveContour(CvSeq* contour[], string objname){

	//code to check that the object name is valid i.e. only alpha letters


	// okay, let's save it
	const char* attrs[] = {"recursive", "1", 0};

	ifstream infile;
	ofstream outfile;
	string line;
	string fobjname = "";
	string num = "";
	infile.open ("./objects/index", ios::in);
	outfile.open ("./objects/index", ios::app);
	int highest = 0;

    while ( getline(infile,line)){
        istringstream liness( line );
        getline( liness, fobjname, ',' );
        getline( liness, num,  ',' );
        if (fobjname == objname) {
        	int i = atoi(num.c_str());
        	if (i > highest) highest = i;
        }
    }

	infile.close();
    outfile << objname << "," << convertInt(highest+1) <<"\n";
    outfile.close();

	cout << "Creating database entry for Object " << objname << endl;
	string objname1="./objects/"+objname+"-"+convertInt(highest+1)+"_1.xml";
	string objname2="./objects/"+objname+"-"+convertInt(highest+1)+"_2.xml";
	string objname3="./objects/"+objname+"-"+convertInt(highest+1)+"_3.xml";
	cvSave(objname1.c_str(), contour[0], 0, 0, cvAttrList(attrs, 0));
	cvSave(objname2.c_str(), contour[1], 0, 0, cvAttrList(attrs, 0));
	cvSave(objname3.c_str(), contour[2], 0, 0, cvAttrList(attrs, 0));
}

// returns the saved contour with the best fit. If this is below the threshold, then null is returned.
CvSeq* FindBestFitContour(CvSeq* seq[]){
	CvSeq* topcontour = NULL;
	string line;
	string fobjname = "";
	string num = "";
	ifstream infile;
	CvSeq *dbcontour[3];
	dbcontour[0] = 0;
	dbcontour[1] = 0;
	dbcontour[2] = 0;
	int highestscore = 0;

	infile.open ("./objects/index", ios::in);
    while (getline(infile,line)){
        istringstream liness( line );
        getline( liness, fobjname, ',' );
        getline( liness, num,  ',' );
        string c1 = fobjname+"-"+num+"_1.xml";
        string c2 = fobjname+"-"+num+"_2.xml";;
        string c3 = fobjname+"-"+num+"_3.xml";;
        char c1_c[c1.size()];
        char c2_c[c2.size()];
        char c3_c[c3.size()];

        // Let's start loading these files for comparison. First we have to do a little conversion to char[].
        cout << "Loading file for " << fobjname << "-" << num << "\n";
        for (int a=0;a<=c1.size();a++){
        	c1_c[a]=c1[a];
		}
        for (int b=0;b<=c1.size();b++){
        	c2_c[b]=c2[b];
		}
        for (int c=0;c<=c1.size();c++){
        	c3_c[c]=c3[c];
		}

        // Next we use the cvLoad to load up the contour array.
        dbcontour[0] = (CvSeq*) cvLoad(c1_c);
        dbcontour[1] = (CvSeq*) cvLoad(c2_c);
        dbcontour[2] = (CvSeq*) cvLoad(c3_c);
        cout << "Computing score for object " << fobjname << "-" << num << "\n";
    }
	return topcontour;
}

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
	CvSeq *currcontour[3];
	currcontour[0] = 0;
	currcontour[1] = 0;
	currcontour[2] = 0;
	CvSeq *dbcontour = 0;
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

			cvFindContours(depth1, storage1, &currcontour[0]);
			cvFindContours(depth2, storage2, &currcontour[1]);
			cvFindContours(depth3, storage3, &currcontour[2]);

			cvZero(depth1);
			cvZero(depth2);
			cvZero(depth3);

			cvDrawContours(depth1, currcontour[0], cvScalarAll(255), cvScalarAll(255), 100);
			cvDrawContours(depth2, currcontour[1], cvScalarAll(255), cvScalarAll(255), 100);
			cvDrawContours(depth3, currcontour[2], cvScalarAll(255), cvScalarAll(255), 100);

			// try to find that object in the database
			dbcontour = FindBestFitContour(currcontour);

			// uh oh, we have no idea what that thing is. let's ask and save it
			if (dbcontour == NULL){
				cout << "I have no idea what this is. What is this object?" << endl;
				getline(cin, objname);
				SaveContour(currcontour, objname);
			}

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
					system("rm -r ./objects");
					cout << "Objects database has been reset!\n";
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

