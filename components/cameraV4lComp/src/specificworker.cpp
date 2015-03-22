/*
 *    Copyright (C) 2015 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"



/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	qDebug() << "GoodBye";
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	
	cameraList.push_back("default");
	
	grabber.open(0);
	
	
    if(grabber.isOpened() == false)  // check if we succeeded
        qFatal("Aborting. Could not open default camera");
	else
		qDebug() << __FUNCTION__ << "Camera " << QString::fromStdString(cameraList.front()) << " opened!";
	
	//Setting grabber
	RoboCompRGBDBus::CameraParams camParams;
	camParams.colorFPS = 20;
	grabber.set(CV_CAP_PROP_FPS, camParams.colorFPS);
	camParams.colorFocal = 400;
	grabber.set(CV_CAP_PROP_FRAME_HEIGHT, 640);  //Get from PARAMS
	grabber.set(CV_CAP_PROP_FRAME_WIDTH, 480);

	//One frame to get real sizes
	Mat frame;
	grabber >> frame; 		// get a new frame from camera
	Size s = frame.size();
	double rate = grabber.get(CV_CAP_PROP_FPS);
	qDebug() << __FUNCTION__ << "Current frame size:" << s.height << "x" << s.width << ". RGB 8 bits format at" << rate << "fps";
	camParams.colorWidth = s.width;
	camParams.colorHeight = s.height;
	camParams.name = "default"; 
	writeBuffer.resize( camParams.colorWidth * camParams.colorHeight * 3);
	readBuffer.resize( camParams.colorWidth * camParams.colorHeight * 3);
	cameraParamsMap[cameraList.front()] = camParams;

	//namedWindow("img",1);
	sleep(1);
	timer.start(10);

	return true;
}

void SpecificWorker::compute()
{
	
	Mat frame, frameRGB;
	grabber >> frame; 	
	cvtColor( frame, frameRGB, CV_BGR2RGB );
	memcpy( &writeBuffer[0], frameRGB.data, frameRGB.size().area() * 3);
	qDebug() << "Reading..."; // at" << grabber.get(CV_CAP_PROP_FPS) << "fps";
	//imshow("img", frame);
	QMutexLocker ml(mutex);
	readBuffer.swap( writeBuffer);
	
}

////////////////////////////////////////////////////////////
///// SERVANTS
///////////////////////////////////////////////////////////


CameraParamsMap SpecificWorker::getAllCameraParams()
{
	
}

void SpecificWorker::getPointClouds(const CameraList &cameras, PointCloudMap &clouds)
{
}

void SpecificWorker::getImages(const CameraList &cameras, ImageMap &images)
{
	if( cameras.size() <= MAX_CAMERAS and cameras.front() == cameraList.front() )
	{
		uint size = cameraParamsMap[cameras.front()].colorHeight * cameraParamsMap[cameras.front()].colorWidth * 3;
		RoboCompRGBDBus::Image img;
		img.camera = cameraParamsMap[cameras.front()];
		QMutexLocker ml(mutex);
		img.colorImage.resize( readBuffer.size());
		img.colorImage.swap( readBuffer );
		images.insert( std::pair<std::string, RoboCompRGBDBus::Image>("default", img));
		std::cout << "camera name " <<cameras.front() << " " << img.colorImage.size() << " " << images.empty() << std::endl;
		
	}
}

void SpecificWorker::getProtoClouds(const CameraList &cameras, PointCloudMap &protoClouds)
{
}

void SpecificWorker::getDecimatedImages(const CameraList &cameras, const int decimation, ImageMap &images)
{
}



