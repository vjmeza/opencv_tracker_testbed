//
//  tracker_testbed.cpp
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

#include "tracker_testbed.hpp"

Tracker_testbed::Tracker_testbed(const string& inFile, unsigned int startingSource):camId(0)
{
    //Init class and local variables
    processingEnabled = false;
    videoSourceInput.push_back("webcam");
    selVidSrc = startingSource;
    pauseEnabled = false;
    debugEnabled = false;
    trackerEnabled = false;
    ifstream inFileStream(inFile.c_str());
    string line = "";
    
    //Check for opening errors
    if (!inFileStream)
    { cerr << "*** Unable to open input file: " << inFile << " ***" << endl; exit(1); }
    
    //Read in file names
    while (!inFileStream.eof())
    {
        getline(inFileStream,line);
        if(inFileStream.eof()) continue;
        videoSourceInput.push_back(line);
    }
    inFileStream.close();
    cout << "Tracking test bed constructed ..." << endl;
}

Tracker_testbed::~Tracker_testbed()
{
    videoSource.release();
    destroyWindow(videoSourceInput[selVidSrc]);
    cout << "Windows closed and Tracker_testbed destroyed  ..." << endl;
}

void Tracker_testbed::processTestbed()
{
    processingEnabled = true;
    openSelectedVideoSrc();
    Mat temp_frame;
    
startprocessing: //paying homage to the old FORTRAN goto here

    while (processingEnabled)
    {

        //buffer the first frame at the start
        videoSource >> oldFrame;
        
        if (selVidSrc == 0) //webcam processing
        {
            while (1)
            {
                videoSource >> newFrame; //read frame
                newFrame.copyTo(temp_frame); //hold a clean new frame
            
                // ********************************************************************************
                // *************** The function to process your own tracker goes here *************
                // ********************************************************************************

                if (trackerEnabled) bwTracker.processObjectTracker(oldFrame,newFrame,debugEnabled);
                
                // ********************************************************************************
                // ********************************************************************************
            
                imshow(videoSourceInput[selVidSrc],newFrame); //show frame
            
                selectTestbedCmd(); //user selects cmd in wait key loop
                if (!processingEnabled || selVidSrc != 0) goto startprocessing; //'esc depressed' or input switched
            
                //copy new to old to prepare for next loop pass
                temp_frame.copyTo(oldFrame);
            }
        }
        else //video file processing
        {
            while (videoSource.get(CV_CAP_PROP_POS_AVI_RATIO) < 1)
            {
            
                videoSource >> newFrame; //read frame
                newFrame.copyTo(temp_frame); //hold a clean new frame

                // *** Process object tracker ***
                if (trackerEnabled) bwTracker.processObjectTracker(oldFrame,newFrame,debugEnabled);

                imshow(videoSourceInput[selVidSrc],newFrame); //show frame
            
                selectTestbedCmd(); //user selects cmd in wait key loop
                if (!processingEnabled || selVidSrc == 0) goto startprocessing; //'esc depressed' or input switched
            
                //copy new to old to prepare for next loop pass
                temp_frame.copyTo(oldFrame);
            
            } //while (videoSource.get(CV_CAP_PROP_POS_AVI_RATIO) < 1)
            
            //if the end of file is reached, release and reopen file
            if(videoSource.get(CV_CAP_PROP_POS_AVI_RATIO) == 1)
            {
                videoSource.release();
                openSelectedVideoSrc();
                goto startprocessing;
            }

        }//else

    } //while (processingEnabled)

}

void Tracker_testbed::openSelectedVideoSrc(void)
{
    //Open video source
    if (selVidSrc == 0 || videoSourceInput.empty())
        videoSource.open(selVidSrc); //open webcam
    else
        videoSource.open(videoSourceInput[selVidSrc]); //open file identified with 'selVidSrc'
    
    if(!videoSource.isOpened())
    { cerr << "*** Unable to open video source using: " << videoSourceInput[selVidSrc] << " ***" << endl; exit(2); }
}

void Tracker_testbed::selectTestbedCmd()
{
    switch (waitKey(5)) //display frame for 5ms using waitKey
    {
        case 27: //'esc' - exit processing
            processingEnabled = false;
            cout << ">>> Processing disabled, exiting program." << endl;
            break;
            
        case 100: //'d' - enable/disable debugger
            debugEnabled = !debugEnabled;
            (debugEnabled) ? cout << ">>> Debug enabled, press 'd' again to disable." << endl
                : cout << ">>> Debug disabled, press 'd' again to enable." << endl;
            if (debugEnabled) trackerEnabled = true; //enable tracker
            break;
            
        case 115: //'s' switch video source by advancing through source input vector
            videoSource.release();
            destroyWindow(videoSourceInput[selVidSrc]);
            if (selVidSrc == videoSourceInput.size()-1) //if the end go to front
                selVidSrc = 0;
            else
                ++selVidSrc;
            trackerEnabled = false;
            openSelectedVideoSrc();
            cout << ">>> Switched to next source (tracker disabled): " << videoSourceInput[selVidSrc] << endl;
            break;
            
        case 116: //'t' - enable/disable tracker
            trackerEnabled = !trackerEnabled;
            (trackerEnabled) ? cout << ">>> Tracker enabled, press 't' again to disable." << endl
                : cout << ">>> Tracker disabled, press 't' again to enable." << endl;
            if (!trackerEnabled) debugEnabled = false; //tracker disabled, disable debug mode
            break;
            
        case 112: // 'p' - pause and unpause
            pauseEnabled = !pauseEnabled;
            if (pauseEnabled)
            {
                cout << ">>> Video source has been paused, press 'p' to resume." << endl;
                while (pauseEnabled)
                {
                    switch (waitKey())
                    {
                        case 112:
                            pauseEnabled = false;
                            break;
                    }
                }
            }
            break;
            
        case 109: // 'm' - display menu
            cout << endl << "Command Menu - Press key in video window for command action: " << endl;
            cout << "'esc' - Quit program" << endl;
            cout << "'d'   - Enable/disable debugger(tracker enabled)" << endl;
            cout << "'s'   - Switch video source by advancing through source input vector" << endl;
            cout << "'t'   - Enable/disable tracker" << endl;
            cout << "'p'   - Pause and unpause" << endl;
            cout << "'m'   - Menu listing" << endl << endl;

        default:
            break;
    } //waitKey
}



