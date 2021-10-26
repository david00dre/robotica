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

static const float velmax = 1000;

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
        RoboCompGenericBase::TBaseState bState;
        differentialrobot_proxy->getBaseState(bState);
        last_point = QPointF(bState.x, bState.z);
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
}

void SpecificWorker::compute() {
    try {
        RoboCompGenericBase::TBaseState bState;
        differentialrobot_proxy->getBaseState(bState);
        robot_polygon->setRotation(bState.alpha * 180 / M_PI);
        robot_polygon->setPos(bState.x, bState.z);
        auto ldata = laser_proxy->getLaserData();
        draw_laser(ldata);
        if (target.activo) {
            //pasar target a coordenadas del robot
            Eigen::Vector2f punto = goToRobot(bState);
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
            if (dist < 300) {
                target.activo = false;
                this->differentialrobot_proxy->setSpeedBase(0, 0);
            }
        }
    } catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }

}

Eigen::Vector2f SpecificWorker::goToRobot(RoboCompGenericBase::TBaseState bState) {
    Eigen::Vector2f targ(target.pos.x(), target.pos.y());
    Eigen::Vector2f robot(bState.x, bState.z);
    Eigen::Matrix2f m;
    m << cos(bState.alpha), -sin(bState.alpha), sin(bState.alpha), cos(bState.alpha);
    return m.transpose() * (targ - robot);
}

void SpecificWorker::draw_laser(const RoboCompLaser::TLaserData &ldata) // robot coordinates
{
    static QGraphicsItem *laser_polygon = nullptr;
    // code to delete any existing laser graphic element
    if (laser_polygon != nullptr) {
        viewer->scene.removeItem(laser_polygon);
    }
    QPolygonF poly;
    // code to fill poly with the laser polar coordinates (angle, dist) transformed to cartesian coordinates (x,y), all in the robot's  // reference system
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
