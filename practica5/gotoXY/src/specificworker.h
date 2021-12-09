
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
#include <cppitertools/range.hpp>



class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
    SpecificWorker(TuplePrx tprx, bool startup_check);
    ~SpecificWorker();
    bool setParams(RoboCompCommonBehavior::ParameterList params);
    QGraphicsItem* draw_laser(const RoboCompLaser::TLaserData &ldata);
    Eigen::Vector2f goToRobot(RoboCompFullPoseEstimation::FullPoseEuler r_state );
    Eigen::Vector2f goToWorld(RoboCompFullPoseEstimation::FullPoseEuler r_state, Eigen::Vector2f targ );




public slots:
    void compute();
    int startup_check();
    void initialize(int period);
    void new_target_slot(QPointF p);

private:
    enum class State {IDLE,FORWARD ,TURN, BORDER, TURN_INIT};
    State state = State::TURN_INIT;
    struct Target
    {
        QPointF pos;
        bool activo = false;
    };

    class Door {
    public:
        QPointF a;
        QPointF b;
        int room1;
        int room2;
        mutable bool visited;
        Door(QPointF A, QPointF B){
            a=A;
            b=B;
            room1=0;
            room2=0;
            visited = false;
        }
        void setvisited(){this->visited= true;}
    };
//    bool friend operator == (const Door &a,const  Door &b){
//        if(((sqrt(pow(a.a.x() - b.a.x(), 2) + pow(a.a.y() - b.a.y(), 2)) < 100) && (sqrt(pow(a.b.x() - b.b.x(), 2) + pow(a.b.y() - b.b.y(), 2)) < 100))
//         || ((sqrt(pow(a.a.x() - b.b.x(), 2) + pow(a.a.y() - b.b.y(), 2)) < 100) && (sqrt(pow(a.b.x() - b.a.x(), 2) + pow(a.b.y() - b.a.y(), 2)) < 100)))
//            return true;
//        else
//            return false;
//    }
    std::shared_ptr < InnerModel > innerModel;
    bool startup_check_flag;
    AbstractGraphicViewer *viewer;
    const int ROBOT_LENGTH = 400;
    QGraphicsPolygonItem *robot_polygon;
    QGraphicsRectItem *laser_in_robot_polygon;
    Target target;
    QPointF last_point;
    QPointF forward(RoboCompFullPoseEstimation::FullPoseEuler r_state , RoboCompLaser::TLaserData &ldata);
    void turn(const RoboCompLaser::TLaserData &ldata);
    void turn_init(const RoboCompLaser::TLaserData &ldata, RoboCompFullPoseEstimation::FullPoseEuler r_state);
    void border(const RoboCompLaser::TLaserData &ldata, QGraphicsItem* poly, QPointF punto);
    Grid grid;
    void update_map(RoboCompFullPoseEstimation::FullPoseEuler r_state, const RoboCompLaser::TLaserData &ldata);
    int explore(RoboCompFullPoseEstimation::FullPoseEuler r_state, const RoboCompLaser::TLaserData &ldata);
    int estadoturn = 0;
    void detect_doors(const RoboCompLaser::TLaserData &ldata);
    std::set<Door> doors;
};

#endif
