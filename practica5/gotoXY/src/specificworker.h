
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
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/enumerate.hpp>


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
    SpecificWorker(TuplePrx tprx, bool startup_check);
    ~SpecificWorker();
    bool setParams(RoboCompCommonBehavior::ParameterList params);

public slots:
    void compute();
    int startup_check();
    void initialize(int period);

private:

    //Clases y Structs
    struct Target                                                                                                       //Estructura para guiar el robot hacia un objetivo (target)
    {
        QPointF pos;                                                                                                    //Posición X e Y del objetivo
        bool activo = false;                                                                                            //Indica si hay algún objetivo activo
        float A,B,C;
    };
    class Door {                                                                                                        //Clase para representar las puertas del mapa
    public:
        QPointF a;                                                                                                      //a y b son los puntos que delimitan una puerta
        QPointF b;
        int room1;                                                                                                      //Habitaciones que conecta la puerta
        int room2;
        mutable bool visited;                                                                                           //True si la puerta ha sido visitada por el robot
        Door(QPointF A, QPointF B){
            a=A;
            b=B;
            room1=0;
            room2=0;
            visited = false;
        }

        Door(){
            a=QPointF(0,0);
            b=QPointF(0,0);
            room1=0;
            room2=0;
            visited = false;
        }

        //Pone a visitado la puerta
        void setvisited(){visited= true;}
        Eigen::Vector2f get_midpoint() const {Eigen::Vector2f p1(a.x(), a.y());                                   //Obtiene el punto medio de la puerta
            Eigen::Vector2f p2(b.x(), b.y());return p1 + ((p2-p1)/2.0);};
        Eigen::Vector2f get_external_midpoint() const                                                                   //Obtiene el punto medio de la puerta dentro de la habitacion
        {
            Eigen::Vector2f p1(a.x(), a.y());Eigen::Vector2f p2(b.x(), b.y());
            Eigen::ParametrizedLine<float, 2> r =  Eigen::ParametrizedLine<float, 2>(get_midpoint(),
                                                                                     (p1-p2).unitOrthogonal());
            //qInfo() << __FUNCTION__ << r.pointAt(800.0).x() << r.pointAt(800.0).y();
            return r.pointAt(1000.0);
        };

    };


    //Patametros
    std::shared_ptr < InnerModel > innerModel;                                                                      //TODO Borrar varialbe que de momento no se usa para nada
    bool startup_check_flag;                                                                                        //Parámetro de inicialización del robot
    AbstractGraphicViewer *viewer;                                                                                  //Mapa en el que se sitúa el robot
    const int ROBOT_LENGTH = 400;                                                                                   //Longitud del robot
    QGraphicsPolygonItem *robot_polygon;                                                                            //Polígono que representa al robot
    QGraphicsRectItem *laser_in_robot_polygon;                                                                      //Polígono que representa el rango de visión del robot
    Target target;                                                                                                  //Donde se almacena el objetivo al que dirigir el robot
    QPointF last_point;                                                                                             //TODO Borrar variable que de momento no se usa para nada
    Grid grid;                                                                                                      //Malla en la que se guardan puntos de colision del laser que identificará como paredes
    int estadoturn = 0;                                                                                                 //0 para que turn_init guarde la posicion inicial y 1 para que realice el giro
    std::vector<Door> doors;                                                                                            //Vector en el que se almacenan las puertas encontradas
    Door door;
    enum class State {IDLE,FORWARD ,TURN, BORDER, TURN_INIT};                                                           //Estados de la máquina de estados
    State state = State::TURN_INIT;                                                                                     //Indica el estado actual
    int anguloconelobjetivo ;                                                                                           //1 si el angulo es positivo (objetivo a la derecha), -1 en caso contrario

    //Métodos
    int get_distmin(const RoboCompLaser::TLaserData &ldata);
    QPointF forward(RoboCompFullPoseEstimation::FullPoseEuler r_state , RoboCompLaser::TLaserData &ldata);          //Método para llevar el robot hacia el objetivo
    void turn(const RoboCompLaser::TLaserData &ldata);                                                              //Método auxiliar de fordward, alinea el robot con el obstáculo para bordearlo
    void turn_init(const RoboCompLaser::TLaserData &ldata, RoboCompFullPoseEstimation::FullPoseEuler r_state);      //Giro que realiza el robot en una habitación para reconocer más puertas
    void border(const RoboCompLaser::TLaserData &ldata, QGraphicsItem* poly);                        //Método para que el robot bordee algún obstáculo
    void detect_doors(RoboCompFullPoseEstimation::FullPoseEuler r_state, const RoboCompLaser::TLaserData &ldata);   //Método que detecta puertas y las añade en caso de que no estuvieran antes
    void update_map(RoboCompFullPoseEstimation::FullPoseEuler r_state, const RoboCompLaser::TLaserData &ldata);     //Inserta puntos de colisión (identificados como pared)
    QGraphicsItem* draw_laser(const RoboCompLaser::TLaserData &ldata);
    Eigen::Vector2f goToRobot(RoboCompFullPoseEstimation::FullPoseEuler r_state );
    Eigen::Vector2f goToRobot(RoboCompFullPoseEstimation::FullPoseEuler r_state, const Eigen::Vector2f &goal );         //Pasa el objetivo almacenado en target al mundo del robot
    Eigen::Vector2f goToWorld(RoboCompFullPoseEstimation::FullPoseEuler r_state, Eigen::Vector2f targ );                //Para el parametro target a coordenadas del mundo real
    float distancia_entre_puntos(QPointF a,QPointF b);                                                                  //Obtiene a distancia entre dos posiciones
    float distancia_entre_puntos(Eigen::Vector2f a,Eigen::Vector2f b);                                                  //Devuelve la distancia entre dos puntos
    RoboCompLaser::TLaserData sector1(const RoboCompLaser::TLaserData &ldata){                                          //Obtiene el primer sector del campo de visión 
        return RoboCompLaser::TLaserData (ldata.begin(),ldata.begin()+ldata.size()/5);}
    RoboCompLaser::TLaserData sector2(const RoboCompLaser::TLaserData &ldata){                                           //Obtiene el segundo sector del campo de visión
        return RoboCompLaser::TLaserData (ldata.begin()+ldata.size()/5,ldata.begin()+ldata.size()*2/5);
    }
    RoboCompLaser::TLaserData sector3(const RoboCompLaser::TLaserData &ldata){                                           //Obtiene el tercer sector del campo de visión
        return RoboCompLaser::TLaserData (ldata.begin()+ldata.size()*2/5,ldata.begin()+ldata.size()*3/5);
    }
    RoboCompLaser::TLaserData sector4(const RoboCompLaser::TLaserData &ldata){                                            //Obtiene el cuarto sector del campo de visión
        return RoboCompLaser::TLaserData (ldata.begin()+ldata.size()*3/5,ldata.begin()+ldata.size()*4/5);
    }
    RoboCompLaser::TLaserData sector5(const RoboCompLaser::TLaserData &ldata){                                            //Obtiene el quinto sector del campo de visión
        return RoboCompLaser::TLaserData (ldata.begin()+ldata.size()*4/5,ldata.end());
    }
    void check_free_path_to_target( const RoboCompLaser::TLaserData &ldata, const Eigen::Vector2f &goal,                  //Dibuja el camino recto hacia el objetivo
    RoboCompFullPoseEstimation::FullPoseEuler bState);
};


#endif
