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
#include <cppitertools/range.hpp>

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
    return true;
}
void SpecificWorker::initialize(int period)
{
    QRect dimensions(-5000, -2500, 10000, 5000);
    viewer = new AbstractGraphicViewer(this, dimensions);
    this->resize(900, 450);
    robot_polygon = viewer->add_robot(ROBOT_LENGTH);
    laser_in_robot_polygon = new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
    laser_in_robot_polygon->setPos(0, 190); // move this to abstract
    try
    {
        RoboCompGenericBase::TBaseState bState;
        differentialrobot_proxy->getBaseState(bState);
        last_point = QPointF(bState.x, bState.z);
    }
    catch (const Ice::Exception &e)
    {
        std::cout << e.what() << std::endl;
    }
    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
    std::cout << "Initialize worker" << std::endl;
    this->Period = period;
    if (this->startup_check_flag)
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
    try
    {
        RoboCompGenericBase::TBaseState bState;
        differentialrobot_proxy->getBaseState(bState);
        robot_polygon->setRotation(bState.alpha * 180 / M_PI);
        robot_polygon->setPos(bState.x, bState.z);
        auto ldata = laser_proxy->getLaserData();
        QGraphicsItem *poly = draw_laser(ldata);

        QPointF punto;
        switch (state)
        {
            case State::IDLE:
                qInfo() << "IDLE";
                if (target.activo)
                {
                    state = State::FORWARD;
                    target.A = bState.z - target.pos.y();
                    target.B = target.pos.x() - bState.x;
                    target.C = (bState.x - target.pos.x()) * bState.z + (target.pos.y() - bState.z) * bState.x;
//                    QLineF line_center(QPointF(bState.x, bState.z), target.pos);
//                    static QGraphicsItem *graphics_line_center = nullptr;
//                    if (graphics_line_center != nullptr)
//                    {
//                        viewer->scene.removeItem(graphics_line_center);
//                    }
//                    graphics_line_center = viewer->scene.addLine(line_center, QPen(QColor("Red"), 30));
                }
                break;
            case State::FORWARD:
                qInfo() << "FORWARD";
                punto = forward(bState, ldata);
                break;
            case State::TURN:
                qInfo() << "TURN";
                turn(ldata);
                break;
            case State::BORDER:
                qInfo() << "BORDER";
                border(ldata, poly, punto, bState);
                break;
        }
    }
    catch (const Ice::Exception &e)
    {
        std::cout << e.what() << std::endl;
    }
}

void SpecificWorker::turn(const RoboCompLaser::TLaserData &ldata)
{
    //    qInfo()<<ldata.size();
    this->differentialrobot_proxy->setSpeedBase(0, 0.5);
    if (ldata[165].dist > 1500 && ldata[45].dist < 600)
    {
        state = State::BORDER;
        this->differentialrobot_proxy->setSpeedBase(0, 0);
    }
}

Eigen::Vector2f SpecificWorker::goToRobot(RoboCompGenericBase::TBaseState bState, Eigen::Vector2f targ)
{
    Eigen::Vector2f robot(bState.x, bState.z);
    Eigen::Matrix2f m;
    m << cos(bState.alpha), -sin(bState.alpha), sin(bState.alpha), cos(bState.alpha);
    return m.transpose() * (targ - robot) ;
}
Eigen::Vector2f SpecificWorker::goToWorld(RoboCompGenericBase::TBaseState bState, Eigen::Vector2f targ)
{
    Eigen::Vector2f robot(bState.x, bState.z);
    Eigen::Matrix2f m;
    m << cos(bState.alpha), -sin(bState.alpha), sin(bState.alpha), cos(bState.alpha);
    return m.transpose().inverse() * targ + robot;
}

QGraphicsItem *SpecificWorker::draw_laser(const RoboCompLaser::TLaserData &ldata) // robot coordinates
{
    static QGraphicsItem *laser_polygon = nullptr;
    // code to delete any existing laser graphic element
    if (laser_polygon != nullptr)
    {
        viewer->scene.removeItem(laser_polygon);
    }
    QPolygonF poly;
    // code to fill poly with the laser polar coordinates (angle, dist) transformed to cartesian coordinates (x,y), all in the robot's
    // reference system
    poly << QPointF(0, 0);
    for (auto &p : ldata)
    {
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

int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

void SpecificWorker::new_target_slot(QPointF p)
{
    //    qInfo()<< p;
    last_point = QPointF(p.x(), p.y());
    target.pos = p;
    target.activo = true;
}

QPointF SpecificWorker::forward(RoboCompGenericBase::TBaseState bState, RoboCompLaser::TLaserData &ldata)
{
    //pasar target a coordenadas del robot
    Eigen::Vector2f punto = goToRobot(bState,  Eigen::Vector2f (target.pos.x(),target.pos.y()));
    //calcular el angulo que forma el robot con el tagert
    float beta = atan2(punto.x(), punto.y());
    float dist = punto.norm();
    float close_to_target;
    check_free_path_to_target(ldata, Eigen::Vector2f (target.pos.x(),target.pos.y()), bState);
    if (dist > 1000)
        close_to_target = 1;
    else
        close_to_target = dist / 1000;
    float expresion = exp(-(pow((beta), 2) / 0.15));
    float adv_speed = 1000 * expresion * close_to_target;
    this->differentialrobot_proxy->setSpeedBase(adv_speed, beta);
    //comprobar si hay un obstáculo delante
    if (ldata[ldata.size() / 2].dist < 550)
    {
        state = State::TURN;
        this->differentialrobot_proxy->setSpeedBase(0, 0);
    }
    if (ldata[45].dist < 600)
        state = State::BORDER;
    for (int i = 1; i < 15; i++)
        ldata.erase(ldata.begin());
    //std::cout<<"distancia: "<<ldata.front().dist<<std::endl;
    if (dist < 300)
    {
        target.activo = false;
        this->differentialrobot_proxy->setSpeedBase(0, 0);
        state = State::IDLE;
    }
    return QPointF(punto.x(), punto.y());
}
void SpecificWorker::border(const RoboCompLaser::TLaserData &ldata, QGraphicsItem *poly, QPointF punto, RoboCompGenericBase::TBaseState bState)
{
    //d = | Ax + By + C |  /  sqrt(A² + B²)
    if (fabs((target.A * bState.x) + (target.B * bState.z) + target.C) / sqrt(pow(target.A,2)+pow(target.B,2)) < 100)
    {
        state = State::FORWARD;
    }
    else if (!poly->contains(target.pos) || ldata[45].dist < 400)
    {
        if (ldata[45].dist < 400)
            this->differentialrobot_proxy->setSpeedBase(300, 0.7);
        else if (ldata[45].dist > 500)
            this->differentialrobot_proxy->setSpeedBase(300, -0.7);
        else
            this->differentialrobot_proxy->setSpeedBase(300, 0);
    }
    else
    {
        state = State::FORWARD;
    }
}

void SpecificWorker::check_free_path_to_target( const RoboCompLaser::TLaserData &ldata, const Eigen::Vector2f &goal,RoboCompGenericBase::TBaseState bState)
{
    // lambda to convert from Eigen to QPointF
    auto toQPointF = [](const Eigen::Vector2f &p){ return QPointF(p.x(),p.y());};

    // create polyggon
    QPolygonF pol;
    pol << QPointF(0,0);
    for(const auto &l: ldata)
        pol << QPointF(l.dist*sin(l.angle), l.dist*cos(l.angle));

    // create tube lines
    auto goal_r = goToRobot(bState,goal);
    Eigen::Vector2f robot(0.0,0.0);
    // number of parts the target vector is divided into
    float parts = (goal_r).norm()/(ROBOT_LENGTH/4);
    Eigen::Vector2f rside(220, 200);
    Eigen::Vector2f lside(-220, 200);
    if(parts < 1) return;

    QPointF p,q, r;

    for(auto l: iter::range(0.0, 1.0, 1.0/parts))
    {
        p = toQPointF(robot*(1-l) + goal_r*l);
        q = toQPointF((robot+rside)*(1-l) + (goal_r+rside)*l);
        r = toQPointF((robot+lside)*(1-l) + (goal_r+lside)*l);
        if( not pol.containsPoint(p, Qt::OddEvenFill) or
            not pol.containsPoint(q, Qt::OddEvenFill) or
            not pol.containsPoint(r, Qt::OddEvenFill))
            break;
    }

    // draw
    QLineF line_center(toQPointF(goToWorld(bState,robot)), toQPointF(goToWorld(bState, Eigen::Vector2f(p.x(),p.y()))));
    QLineF line_right(toQPointF(goToWorld(bState, robot + rside)), toQPointF(goToWorld(bState, Eigen::Vector2f(q.x(),q.y()))));
    QLineF line_left(toQPointF(goToWorld(bState, robot + lside)), toQPointF(goToWorld(bState, Eigen::Vector2f(r.x(),q.y()))));
    static QGraphicsItem *graphics_line_center = nullptr;
    static QGraphicsItem *graphics_line_right = nullptr;
    static QGraphicsItem *graphics_line_left = nullptr;
    static QGraphicsItem *graphics_target = nullptr;
    if (graphics_line_center != nullptr)
        viewer->scene.removeItem(graphics_line_center);
    if (graphics_line_right != nullptr)
        viewer->scene.removeItem(graphics_line_right);
    if (graphics_line_left != nullptr)
        viewer->scene.removeItem(graphics_line_left);
    if (graphics_target != nullptr)
        viewer->scene.removeItem(graphics_target);
    graphics_line_center = viewer->scene.addLine(line_center, QPen(QColor("Blue"), 30));
    graphics_line_right = viewer->scene.addLine(line_right, QPen(QColor("Orange"), 30));
    graphics_line_left = viewer->scene.addLine(line_left, QPen(QColor("Magenta"), 30));
    graphics_target = viewer->scene.addEllipse(-100, -100, 200, 200, QPen(QColor("Blue")), QBrush(QColor("Blue")));
    graphics_target->setPos(goal.x(), goal.y());
}