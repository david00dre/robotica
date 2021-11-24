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
#include "math.h"
#include "stdlib.h"
#include "time.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx) {
    this->startup_check_flag = startup_check;
}
/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker() {
    std::cout << "Destroying SpecificWorker" << std::endl;
}
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params) {
    return true;
}
void SpecificWorker::initialize(int period) {
    QRect dimensions(-5000, -2500, 10000, 5000);
    viewer = new AbstractGraphicViewer(this, dimensions);
    this->resize(900, 450);
    robot_polygon = viewer->add_robot(ROBOT_LENGTH);
    laser_in_robot_polygon = new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
    laser_in_robot_polygon->setPos(0, 190);     // move this to abstract
    try {
        RoboCompGenericBase::TBaseState r_state ;
        differentialrobot_proxy->getBaseState(r_state );
        last_point = QPointF(r_state .x, r_state .z);
    }
    catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
    std::cout << "Initialize worker" << std::endl;
    this->Period = period;
    if (this->startup_check_flag) {
        this->startup_check();
    } else {
        timer.start(Period);
    }
    grid.initialize(dimensions,100, &viewer->scene, false);

}

void SpecificWorker::compute() {
    try {
        auto r_state = fullposeestimation_proxy->getFullPoseEuler();
        robot_polygon->setRotation(r_state.rz*180/M_PI);
        robot_polygon->setPos(r_state.x, r_state.y);

//        differentialrobot_proxy->getBaseState(r_state );
//        robot_polygon->setRotation(r_state .alpha * 180 / M_PI);
//        robot_polygon->setPos(r_state .x, r_state .z);
        auto ldata = laser_proxy->getLaserData();
        QGraphicsItem* poly = draw_laser(ldata);

//        Grid grid
        QPointF punto;
        switch (state) {
            case State::IDLE:
                qInfo()<<"IDLE";
                if(target.activo) state = State::FORWARD;
                break;
            case State::FORWARD:
                qInfo()<<"FORWARD";
                punto = forward(r_state ,ldata);
                break;
            case State::TURN:
                qInfo()<<"TURN";
                turn(ldata);
                break;
            case State::BORDER:
                qInfo()<<"BORDER";
                border(ldata,poly,punto);
                break;
        }
    } catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }

}

void SpecificWorker::turn(const RoboCompLaser::TLaserData &ldata) {
//    qInfo()<<ldata.size();
    this->differentialrobot_proxy->setSpeedBase(0, 0.5);
    if(ldata[165].dist > 1500 && ldata[45].dist < 450) {
        state = State::BORDER;
        this->differentialrobot_proxy->setSpeedBase(0, 0);
    }
}

Eigen::Vector2f SpecificWorker::goToRobot(RoboCompFullPoseEstimation::FullPoseEuler r_state ) {
    Eigen::Vector2f targ(target.pos.x(), target.pos.y());
    Eigen::Vector2f robot(r_state .x, r_state .z);
    Eigen::Matrix2f m;
    m << cos(r_state.rz), -sin(r_state.rz), sin(r_state.rz), cos(r_state.rz);
    return m.transpose() * (targ - robot);
}
Eigen::Vector2f SpecificWorker::goToWorld(RoboCompFullPoseEstimation::FullPoseEuler r_state , Eigen::Vector2f targ)
{
    Eigen::Vector2f robot(r_state .x, r_state .z);
    Eigen::Matrix2f m;
    m << cos(r_state.rz), -sin(r_state.rz), sin(r_state.rz), cos(r_state.rz);
    return m.transpose().inverse() * targ + robot;
}
QGraphicsItem* SpecificWorker::draw_laser(const RoboCompLaser::TLaserData &ldata) // robot coordinates
{
    static QGraphicsItem *laser_polygon = nullptr;
    // code to delete any existing laser graphic element
    if (laser_polygon != nullptr) {
        viewer->scene.removeItem(laser_polygon);
    }
    QPolygonF poly;
    // code to fill poly with the laser polar coordinates (angle, dist) transformed to cartesian coordinates (x,y), all in the robot's
    // reference system
    poly << QPointF(0, 0);
    for (auto &p: ldata) {
        float x = p.dist * sin(p.angle);
        float y = p.dist * cos(p.angle);
        poly << QPoint(x, y);
    }
    QColor color("LightGreen");
    color.setAlpha(40);
    laser_polygon = viewer->scene.addPolygon(laser_in_robot_polygon->mapToScene(poly), QPen(QColor("DarkGreen"), 30),
                                             QBrush(color));
    laser_polygon->setZValue(3);
    return laser_polygon;
}

int SpecificWorker::startup_check() {
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

void SpecificWorker::new_target_slot(QPointF p) {
//    qInfo()<< p;
    last_point = QPointF(p.x(), p.y());
    target.pos = p;
    target.activo = true;
}

QPointF SpecificWorker::forward(RoboCompFullPoseEstimation::FullPoseEuler r_state , RoboCompLaser::TLaserData &ldata) {
    //pasar target a coordenadas del robot
    Eigen::Vector2f punto = goToRobot(r_state );
    //calcular el angulo que forma el robot con el tagert
    float beta = atan2(punto.x(), punto.y());
    float dist = punto.norm();
    float close_to_target;
    if (dist > 1000)
        close_to_target = 1;
    else
        close_to_target = dist / 1000;
    float expresion = exp(-(pow((beta), 2) / 0.15));
    float adv_speed = 1000 * expresion * close_to_target;
    this->differentialrobot_proxy->setSpeedBase(adv_speed, beta);
    //comprobar si hay un obstáculo delante
    if(ldata[ldata.size()/2].dist < 550) {
        state = State::TURN;
        this->differentialrobot_proxy->setSpeedBase(0, 0);
    }
    if(ldata[45].dist < 650)
        state = State::BORDER;
    for(int i=1;i<15;i++) ldata.erase(ldata.begin());
    //std::cout<<"distancia: "<<ldata.front().dist<<std::endl;
    if (dist < 300) {
        target.activo = false;
        this->differentialrobot_proxy->setSpeedBase(0, 0);
        state = State::IDLE;
    }
    return QPointF(punto.x(),punto.y());
}
void SpecificWorker::border(const RoboCompLaser::TLaserData &ldata, QGraphicsItem* poly, QPointF punto) {
    if(!poly->contains(target.pos)||ldata[45].dist < 400) {
        if (ldata[45].dist < 400)
            this->differentialrobot_proxy->setSpeedBase(300, 0.7);
        else if (ldata[45].dist > 500)
            this->differentialrobot_proxy->setSpeedBase(300, -0.7);
        else
            this->differentialrobot_proxy->setSpeedBase(300, 0);
    }else{
        state = State::FORWARD;
    }
}

void SpecificWorker::update_map(RoboCompFullPoseEstimation::FullPoseEuler r_state, const RoboCompLaser::TLaserData &ldata){
    Eigen::Vector2f lw;
    for(const auto &l : ldata)
    {
        if(l.dist > ROBOT_LENGTH/2)
        {
            Eigen::Vector2f tip(l.dist*sin(l.angle), l.dist*cos(l.angle));
            Eigen::Vector2f p = goToWorld(r_state, tip);
            int target_kx = (p.x() - grid.dim.left()) / grid.TILE_SIZE;
            int target_kz = (p.y() - grid.dim.bottom()) / grid.TILE_SIZE;
            int last_kx = -1000000;
            int last_kz = -1000000;


//            grid.getCell()



            int num_steps = ceil(l.dist/(grid.TILE_SIZE/2.0));
            for(const auto &&step : iter::range(0.0, 1.0-(1.0/num_steps), 1.0/num_steps))
            {
                Eigen::Vector2f p = goToWorld(r_state, tip * step);
                int kx = (p.x() - grid.dim.left()) / grid.TILE_SIZE;
                int kz = (p.y() - grid.dim.bottom()) / grid.TILE_SIZE;
                if(kx != last_kx and kx != target_kx and kz != last_kz and kz != target_kz)
                    grid.add_miss(goToWorld(r_state, tip * step));
                last_kx = kx;
                last_kz = kz;
            }
            if(l.dist <= 4000)
                grid.add_hit(goToWorld(r_state, tip));
            // else
            //     grid.add_miss(from_robot_to_world(tip));
        }
    }

}

