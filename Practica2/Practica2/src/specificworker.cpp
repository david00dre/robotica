/*
 *    Copyright (C) 2021 by YOUR NAME HERE
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
#include <math.h>
#include <stdlib.h>
#include <time.h>

static float threshold = 400;
static float espiralInicial = true;

//Variables para la espiral
static float rot =3,rotanterior = 3;
static float decremento = 0.015;

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }

	return true;
}



void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
	}

}
void SpecificWorker::compute()
{
    try{
        auto ldata = this->laser_proxy->getLaserData();
        std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;

        for(int i=1;i<15;i++) ldata.erase(ldata.begin());
        std::cout<<"distancia: "<<ldata.front().dist<<std::endl;


        if(!espiralInicial) {


            //Frena cuando esta cerca de un obstaculo
            if (ldata.front().dist < threshold + 200 && ldata.front().dist!=0)
                frena(ldata.front().dist);

            //Si encuentra un obstaculo gira
            if (ldata.front().dist < threshold && ldata.front().dist!=0) {
                this->giroAleatorio();
            }

            //en caso contrario, sigue
            if(ldata.front().dist > threshold + 200 )
                this->differentialrobot_proxy->setSpeedBase(1000,0);

        }else {
            espiral();
            if(ldata.front().dist <threshold+200 && ldata.front().dist!=0)
                espiralInicial = false;
        }
    }catch(const Ice::Exception &e){
        std::cout << "Error reading from Camera" << e << std::endl;
    }
	//computeCODE
	//QMutexLocker locker(mutex);
	//try
	//{
	//  camera_proxy->getYImage(0,img, cState, bState);
	//  memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
	//  searchTags(image_gray);
	//}
	//catch(const Ice::Exception &e)
	//{
	//  std::cout << "Error reading from Camera" << e << std::endl;
	//}
	
	
}

void SpecificWorker::espiral() {
    this->differentialrobot_proxy->setSpeedBase(550,rot);
    rot = rot - decremento;
    if (rot <rotanterior*0.5){
        rotanterior = rot;
        decremento = rot/500;
    }

}

void SpecificWorker::giroAleatorio() {
    srand(time(NULL));

    differentialrobot_proxy->setSpeedBase(0, 3.14);
    usleep(850000 + rand() % (1150000 +1 - 850000 ));
    differentialrobot_proxy->setSpeedBase(100, 0);
    usleep(1500000);
    differentialrobot_proxy->setSpeedBase(400, 0);

}

void SpecificWorker::frena(float front) {
    if(front<threshold+200)
        this->differentialrobot_proxy->setSpeedBase(500,0);
    else if(front<threshold+150)
        this->differentialrobot_proxy->setSpeedBase(300,0);
    else if(front<threshold+100)
        this->differentialrobot_proxy->setSpeedBase(150,0);
    else if(front<threshold+50)
        this->differentialrobot_proxy->setSpeedBase(50,0);

}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}






/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

