
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

/**
 \brief
 @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <eigen3/Eigen/Dense>
#include <grid2d/grid.h>



class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
    SpecificWorker(TuplePrx tprx, bool startup_check);
    ~SpecificWorker();
    bool setParams(RoboCompCommonBehavior::ParameterList params);
    QGraphicsItem* draw_laser(const RoboCompLaser::TLaserData &ldata);
    Eigen::Vector2f goToRobot(RoboCompGenericBase::TBaseState bState);



public slots:
    void compute();
    int startup_check();
    void initialize(int period);
    void new_target_slot(QPointF p);

private:
    enum class State {IDLE,FORWARD ,TURN, BORDER };
    State state = State::IDLE;
    struct Target
    {
        QPointF pos;
        bool activo = false;
    };
    std::shared_ptr < InnerModel > innerModel;
    bool startup_check_flag;
    AbstractGraphicViewer *viewer;
    const int ROBOT_LENGTH = 400;
    QGraphicsPolygonItem *robot_polygon;
    QGraphicsRectItem *laser_in_robot_polygon;
    Target target;
    QPointF last_point;
    QPointF forward(RoboCompGenericBase::TBaseState bState, RoboCompLaser::TLaserData &ldata);
    void turn(const RoboCompLaser::TLaserData &ldata);
    void border(const RoboCompLaser::TLaserData &ldata, QGraphicsItem* poly, QPointF punto);
    Grid grid;

};

#endif
