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

#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>

// Some definitions for OpenKinect
#define         FREENECT_FRAME_W   640
#define         FREENECT_FRAME_H   480
#define         FREENECT_FRAME_PIX   (FREENECT_FRAME_H*FREENECT_FRAME_W) //width*height pixels in the image
#define         FREENECT_VIDEO_RGB_SIZE   (FREENECT_FRAME_PIX*3) //3 bytes per pixel
#define 		FREENECT_DEPTH_11BIT_SIZE   (FREENECT_FRAME_PIX*sizeof(uint16_t))
#define 		CVX_RED                CV_RGB(0xff,0x00,0x00)
#define 		CVX_GREEN        CV_RGB(0x00,0xff,0x00)
#define 		CVX_BLUE        CV_RGB(0x00,0x00,0xff)
#define 		CONTOUR_MATCH_THRESHOLD	2.5

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

	//code to check that the object name is valid i.e. only alpha numeric characters


	// okay, let's save it, but only the exterior contour, not all the holes
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

	cvSave(objname1.c_str(), contour[0]);
	cvSave(objname2.c_str(), contour[0]);
	cvSave(objname3.c_str(), contour[2]);
}

// returns the saved contour with the best fit. If this is below the threshold, then null is returned.
string FindBestFitContour(CvSeq* seq[]){
	CvSeq* topcontour = NULL;
	string line;
	string fobjname = "";
	string bestobjname = "";
	string num = "";
	ifstream infile;
	CvSeq *dbcontour[3];
	double diff = 0;
	double min_diff = CONTOUR_MATCH_THRESHOLD;
	dbcontour[0] = 0;
	dbcontour[1] = 0;
	dbcontour[2] = 0;
    CvContourTree *treedb1 = NULL;
    CvContourTree *treecurr1 = NULL;
    CvContourTree *treedb2 = NULL;
    CvContourTree *treecurr2 = NULL;
    CvContourTree *treedb3 = NULL;
    CvContourTree *treecurr3 = NULL;
	CvMemStorage *storagedb1 = NULL;
	CvMemStorage *storagecurr1 = NULL;
	CvMemStorage *storagedb2 = NULL;
	CvMemStorage *storagecurr2 = NULL;
	CvMemStorage *storagedb3 = NULL;
	CvMemStorage *storagecurr3 = NULL;

	storagedb1 = cvCreateMemStorage(0);
	storagedb2 = cvCreateMemStorage(0);
	storagedb3 = cvCreateMemStorage(0);
	storagecurr1 = cvCreateMemStorage(0);
	storagecurr2 = cvCreateMemStorage(0);
	storagecurr3 = cvCreateMemStorage(0);

	infile.open ("./objects/index", ios::in);
    while (getline(infile,line)){
    	diff = 0;
        istringstream liness( line );
        getline( liness, fobjname, ',' );
        getline( liness, num,  ',' );
        string c1 = "./objects/"+fobjname+"-"+num+"_1.xml";
        string c2 = "./objects/"+fobjname+"-"+num+"_2.xml";;
        string c3 = "./objects/"+fobjname+"-"+num+"_3.xml";;
        char c1_c[c1.size()];
        char c2_c[c2.size()];
        char c3_c[c3.size()];

        // Let's start loading these files for comparison. First we have to do a little conversion to char[].
        for (int a=0;a<=c1.size();a++){
        	c1_c[a]=c1[a];
		}
        for (int b=0;b<=c1.size();b++){
        	c2_c[b]=c2[b];
		}
        for (int c=0;c<=c1.size();c++){
        	c3_c[c]=c3[c];
		}

        // Next we use the cvLoad to load up the contour array and begin computing scores and cre
        // let's create the contour trees for each of these, and then compare them
        // find the maximum over the threshold and try to ID based on a likely object
		// Okay, let's add up a similarity score on each image. if there is only a few coordinates,
		// don't bother calculating anything for the level.
        dbcontour[0] = (CvSeq*) cvLoad(c1_c, storagedb1, 0, 0);
		if (seq[0]->total < 5 || dbcontour[0]->total < 5 ||  dbcontour[0] == NULL) {
			cout << "useless data in currcontour[0]\n";
			diff += 1;
		}
		else {
			treedb1 = cvCreateContourTree(dbcontour[0], storagedb1, 0);
        	treecurr1 = cvCreateContourTree(seq[0], storagecurr1, 0);
            diff += cvMatchContourTrees(treedb1, treecurr1, CV_CONTOUR_TREES_MATCH_I1, CONTOUR_MATCH_THRESHOLD);
		}
        dbcontour[1] = (CvSeq*) cvLoad(c2_c, storagedb2, 0, 0);
		if (seq[1]->total < 5 || dbcontour[1]->total < 5 || dbcontour[1] == NULL) {
			cout << "useless data in currcontour[1]\n";
			diff += 1;
		}
		else {
			treedb2 = cvCreateContourTree(dbcontour[1], storagedb2, 0);
        	treecurr2 = cvCreateContourTree(seq[1], storagecurr2, 0);
            diff += cvMatchContourTrees(treedb2, treecurr2, CV_CONTOUR_TREES_MATCH_I1, CONTOUR_MATCH_THRESHOLD);
		}
        dbcontour[2] = (CvSeq*) cvLoad(c3_c, storagedb3, 0, 0);
		if (seq[2]->total < 5 || dbcontour[2]->total < 5 || dbcontour[2] == NULL) {
			cout << "useless data in currcontour[2]\n";
			diff += 1;
		}
		else {
			treedb3 = cvCreateContourTree(dbcontour[2], storagedb3, 0);
        	treecurr3 = cvCreateContourTree(seq[2], storagecurr3, 0);
            diff += cvMatchContourTrees(treedb3, treecurr3, CV_CONTOUR_TREES_MATCH_I1, CONTOUR_MATCH_THRESHOLD);
		}

        cout << "Computing score for object " << fobjname << "-" << num << ": " << diff << endl;

        // okay, this is the most likely match.
        if (diff < min_diff) {
        	min_diff = diff;
        	bestobjname = fobjname;
        }
    }

	cvClearMemStorage(storagedb1);
	cvClearMemStorage(storagedb2);
	cvClearMemStorage(storagedb3);
	cvClearMemStorage(storagecurr1);
	cvClearMemStorage(storagecurr2);
	cvClearMemStorage(storagecurr3);

	// is the match better than the threshold?
	if (min_diff > CONTOUR_MATCH_THRESHOLD) cout << "The closest matching object is " << bestobjname
												 << " with a score of " << min_diff << endl;
	return bestobjname;
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
	CvSeq *lastcontour[3];
	lastcontour[0] = 0;
	lastcontour[1] = 0;
	lastcontour[2] = 0;
	CvMemStorage *storage1 = NULL;
	CvMemStorage *storage2 = NULL;
	CvMemStorage *storage3 = NULL;
	string objname = "";
	string delreply = "";
	string isitreply = "";

	storage1 = cvCreateMemStorage(0);
	storage2 = cvCreateMemStorage(0);
	storage3 = cvCreateMemStorage(0);

	// initialize gray_depth once so we don't have to keep doing it later in the loop
	depth = GlViewColor(depth);
	gray_depth = cvCreateImage(cvGetSize(depth), depth->depth, 1);
	depth1 = cvCreateImage(cvGetSize(depth), depth->depth, 1);
	depth2 = cvCreateImage(cvGetSize(depth), depth->depth, 1);
	depth3 = cvCreateImage(cvGetSize(depth), depth->depth, 1);

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

		cvZero(depth1);
		cvZero(depth2);
		cvZero(depth3);

		//alright, let's gray it out for diagnostics purposes
		cvCvtColor(depth,gray_depth,CV_BGR2GRAY);
		cvThreshold (gray_depth, gray_depth, 180, 256, CV_THRESH_TOZERO);

		// 20 inches
		cvThreshold (gray_depth, depth1, 180, 256, CV_THRESH_TRUNC);
		// ~20.5 inches
		cvThreshold (gray_depth, depth2, 190, 256, CV_THRESH_TRUNC);
		// ~21.5 inches
		cvThreshold (gray_depth, depth3, 200, 256, CV_THRESH_TRUNC);

		//Let's find the contours
		cvFindContours(depth1, storage1, &currcontour[0], sizeof(CvContour), CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
		cvFindContours(depth2, storage2, &currcontour[1], sizeof(CvContour), CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
		cvFindContours(depth3, storage3, &currcontour[2], sizeof(CvContour), CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

		cvZero(depth1);
		cvZero(depth2);
		cvZero(depth3);

		// go ahead and get the largest contour for each, which represents the outline
		for (currcontour[0]; currcontour[0] != NULL; currcontour[0] = currcontour[0]->h_next){
			lastcontour[0] = currcontour[0];
		}
		currcontour[0] = lastcontour[0];
		for (currcontour[1]; currcontour[1] != NULL; currcontour[1] = currcontour[1]->h_next){
			lastcontour[1] = currcontour[1];
		}
		currcontour[1] = lastcontour[1];
		for (currcontour[2]; currcontour[2] != NULL; currcontour[2] = currcontour[2]->h_next){
			lastcontour[2] = currcontour[2];
		}
		currcontour[2] = lastcontour[2];

		// approximate polygons!
		currcontour[0] = cvApproxPoly(currcontour[0], sizeof(CvContour), storage1, CV_POLY_APPROX_DP, 5, 0);
		currcontour[1] = cvApproxPoly(currcontour[1], sizeof(CvContour), storage2, CV_POLY_APPROX_DP, 5, 0);
		currcontour[2] = cvApproxPoly(currcontour[2], sizeof(CvContour), storage3, CV_POLY_APPROX_DP, 5, 0);

		//let's draw these
		cvDrawContours(depth1, currcontour[0], cvScalarAll(255), cvScalarAll(255), 0);
		cvDrawContours(depth2, currcontour[1], cvScalarAll(255), cvScalarAll(255), 0);
		cvDrawContours(depth3, currcontour[2], cvScalarAll(255), cvScalarAll(255), 0);

		cvShowImage("RGB", image);
		cvShowImage("Depth", gray_depth);
		cvShowImage("Depth1", depth1);
		cvShowImage("Depth2", depth2);
		cvShowImage("Depth3", depth3);

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
			// try to find that object in the database
			objname = FindBestFitContour(currcontour);

			// was there a return for objname?
			if (objname == ""){
				cout << "I have no idea what this is. What is this object?" << endl;
				getline(cin, objname);
				SaveContour(currcontour, objname);
			} else {
				isitreply= "";
				while (isitreply != "y" && isitreply != "n"){

					cout << "Is it a " << objname <<"? (y/n) \n";
					getline(cin, isitreply);
					// if confirmed, then we're set! If not, let's ask what it actually is and save it
					if (isitreply == "y") {
						cout << "woohoo!\n";
					}
					else if (isitreply == "n"){
						cout << "Okay, what is it?" << endl;
						getline(cin, objname);
						SaveContour(currcontour, objname);
					}
				}
			}

		// if the delete key is pressed, we'll delete all the saved contour objects
		} else if (key == 100) {
			delreply= "";
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

