//
//  ObjectTracker.cpp
//  opencv_tracking_algorithm_testbed
//
//  Copyright (c) 2015, Victor Meza
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice, this
//  list of conditions and the following disclaimer.
//
//  * Redistributions in binary form must reproduce the above copyright notice,
//  this list of conditions and the following disclaimer in the documentation
//  and/or other materials provided with the distribution.
//
//  * Neither the name of OpenCV Tracker TestBed nor the names of its
//  contributors may be used to endorse or promote products derived from
//  this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//           SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#include "ObjectTracker.hpp"

ObjectTracker::ObjectTracker(void)
{
    debugEnabled = false;
    cout << "ObjectTracker object created ..." << endl;
}

ObjectTracker::~ObjectTracker()
{
    cout << "ObjectTracker object destroyed ..." << endl;
}

void ObjectTracker::processObjectTracker(Mat& oldFrame, Mat& newFrame, bool debug)
{
    setDebugEnabled(debug);
    
    const static int SENSITIVITY = 20; //Sensitivity for the absdiff() function
    const static int BLUR_SIZE = 10; //Size of blur used to smooth the intensity image
    
    Mat grayImageOld, grayImageNew, diffImage, thresholdImage;
    
    //convert frames to gray scale in preparation for differencing
    cvtColor(oldFrame,grayImageOld,COLOR_BGR2GRAY);
    cvtColor(newFrame,grayImageNew,COLOR_BGR2GRAY);

    //perform frame differencing with the sequential images. This will output an "intensity image"
    //do not confuse this with a threshold image, we will need to perform thresholding afterwards.
    absdiff(grayImageOld,grayImageNew,diffImage);

    //threshold intensity image at a given sensitivity value
    threshold(diffImage,thresholdImage,SENSITIVITY,255,THRESH_BINARY);
    
    displayDebugFrame("Difference Image", diffImage);
    displayDebugFrame("Threshold Image", thresholdImage);

    //blur image to reduce noise, will output an intensity image
    blur(thresholdImage,thresholdImage,Size(BLUR_SIZE,BLUR_SIZE));

    //threshold again to obtain binary image from blur output
    threshold(thresholdImage,thresholdImage,SENSITIVITY,255,THRESH_BINARY);
    
    displayDebugFrame("Final Threshold Image", thresholdImage);

    //if tracking enabled, search for contours in our thresholded image
    searchForMovement(thresholdImage,newFrame);

}

void ObjectTracker::searchForMovement(Mat thresholdImage, Mat& newFrame)
{
    //init locals
    bool objectDetected = false;
    Mat temp;
    thresholdImage.copyTo(temp);
    
    //Vectors for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    
    //Find contours of filtered image
    findContours(temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );// retrieves external contours
    
    //If contours vector is not empty, objects found
    (contours.size() > 0) ? objectDetected = true : objectDetected = false;

    if(objectDetected)
    {
        //Largest contour at the end of vector, assume biggest contour is the object
        vector< vector<Point> > largestContour;
        largestContour.push_back(contours.at(contours.size()-1));

        //Determine bounding box around largest contour and find center - final est. position
        boundingBox = boundingRect(largestContour.at(0));
        tgtCtr.x = boundingBox.x + boundingBox.width/2;
        tgtCtr.y = boundingBox.y + boundingBox.height/2;

        //Draw bounding box around object
        line(newFrame, Point(boundingBox.x,boundingBox.y), Point(boundingBox.x,
            boundingBox.y+boundingBox.height), Scalar(0,255,0),1); //vertical1
        line(newFrame, Point(boundingBox.x+boundingBox.width,boundingBox.y), Point(boundingBox.x+boundingBox.width,
            boundingBox.y+boundingBox.height),Scalar(0,255,0),1); //vertical2
        line(newFrame, Point(boundingBox.x,boundingBox.y), Point(boundingBox.x+boundingBox.width,
            boundingBox.y), Scalar(0,255,0),1); //horizontal1
        line(newFrame, Point(boundingBox.x,boundingBox.y+boundingBox.height), Point(boundingBox.x+boundingBox.width,
            boundingBox.y+boundingBox.height), Scalar(0,255,0),1); //horizontal2
        
        //Draw crosshairs
        line(newFrame,Point(tgtCtr.x,tgtCtr.y),Point(tgtCtr.x,tgtCtr.y-3),Scalar(0,255,0),2);
        line(newFrame,Point(tgtCtr.x,tgtCtr.y),Point(tgtCtr.x,tgtCtr.y+3),Scalar(0,255,0),2);
        line(newFrame,Point(tgtCtr.x,tgtCtr.y),Point(tgtCtr.x-3,tgtCtr.y),Scalar(0,255,0),2);
        line(newFrame,Point(tgtCtr.x,tgtCtr.y),Point(tgtCtr.x+3,tgtCtr.y),Scalar(0,255,0),2);
    
        //Write the position of the object on the screen
        putText(newFrame, "Tracking object at (" + to_string(tgtCtr.x) + "," + to_string(tgtCtr.y) + ")",
                Point(20,20), CV_FONT_HERSHEY_PLAIN, 1, Scalar(255,0,0), 2);
    }
    
    //Notify the track is lost
    if(!objectDetected) putText(newFrame,"Track Lost",Point(20,20),CV_FONT_HERSHEY_PLAIN,1,Scalar(0,0,255),2);
}

void ObjectTracker::setDebugEnabled(bool b)
{ debugEnabled = b; }

void ObjectTracker::displayDebugFrame(string window_title, Mat& frame)
{
    if(debugEnabled) //show the debug frame
        imshow(window_title,frame);
    else //if no debug mode, destroy the window
        destroyWindow(window_title);
}


