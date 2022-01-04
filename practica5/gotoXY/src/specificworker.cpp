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
    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates,
            this, &SpecificWorker::new_target_slot);
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

//        differentialrobot_proxy->getBaseState(r_state );                                                              //TODO Borrar forma anterior de girar el robot
//        robot_polygon->setRotation(r_state .alpha * 180 / M_PI);
//        robot_polygon->setPos(r_state .x, r_state .z);
        auto ldata = laser_proxy->getLaserData();                                                          //Vector con cada laser de visión del robot
        QGraphicsItem* poly = draw_laser(ldata);                                                                        //Polígono del laser
        update_map(r_state,ldata);                                                                                      //Actualiza el grid para pintar
        QPointF punto;                                                                                                  //Donde se almacena el vector a un objetivo en el mundo del robot
        switch (state) {                                                                                                //MÁQUINA DE ESTADOS
            case State::TURN_INIT:                                                                                      //El robot gira buscando puertas
                qInfo()<<"TURN_INIT";
                turn_init(ldata, r_state);
                break;
            case State::IDLE:                                                                                           //El robot no hace nada
                qInfo()<<"IDLE";
                break;
            case State::FORWARD:                                                                                        //El robot avanza hacia un objetivo, el indicado en target
                qInfo()<<"FORWARD";
                punto = forward(r_state ,ldata);
                break;
            case State::TURN:                                                                                           //Metodo auxiliar del fordward. EL robot se alinea con el obstaculo
                qInfo()<<"TURN";
                turn(ldata);
                break;
            case State::BORDER:                                                                                         //Método auxiliar de fordward. Bordea un obstaculo
                qInfo()<<"BORDER";
                border(ldata,poly,punto);
                break;
        }
    } catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }

}

void SpecificWorker::turn(const RoboCompLaser::TLaserData &ldata)
{
    this->differentialrobot_proxy->setSpeedBase(0, 0.5);                                                       //Hace girar el robot
    if (ldata[165].dist > 1500 && ldata[45].dist < 600)                                                                 //Hasta que esté alineado con el obstaculo
    {
        state = State::BORDER;
        this->differentialrobot_proxy->setSpeedBase(0, 0);
    }
}

void SpecificWorker::turn_init(const RoboCompLaser::TLaserData &ldata,
                               RoboCompFullPoseEstimation::FullPoseEuler r_state) {
    switch (estadoturn) {
        case 0:
            static auto initial_angle = (r_state.rz < 0) ? (2 * M_PI + r_state.rz) : r_state.rz;                 //Almacena la posición antes del giro
            estadoturn=1;
            break;
        case 1:
            detect_doors(ldata);
            float current = (r_state.rz < 0) ? (2 * M_PI + r_state.rz) : r_state.rz;
            if (fabs(current - initial_angle) < (M_PI + 0.1) and
            fabs(current - initial_angle) > (M_PI - 0.1)) {
                int i=0;
                Door d= doors[i];
                while (d.visited && i<doors.size()){                                                                    //Busca la última puerta no visitada
                    i++;
                    d = doors[i];
                }
                if(!d.visited) {                                                                                        //Si hay alguna puerta no visitada, va hacia ella (su punto medio)
                    QPointF pmedio = QPoint((d.a.x() + d.b.x()) / 2, (d.a.y() + d.b.y()) / 2);
                    target.pos = pmedio;
                    target.activo = true;
                    state = State::FORWARD;
                    estadoturn = 0;
                }else{                                                                                                  //En caso contrario, detiene la búsqueda
                    state = State::IDLE;
                }

            }
            break;

    }
}

void SpecificWorker::detect_doors(const RoboCompLaser::TLaserData &ldata){

    std::vector<float> distancias;                                                                                      //Se guardan las distancias entre lo puntos finales de cada laser
                                                                                                                        //Con el siguiente
    for (int i = 0;i<(int)ldata.size()-1;i++){
        distancias.push_back(abs(ldata[i].dist - ldata[i+1].dist));
    }

    std::vector<QPointF> peaks;
    for(int i = 0; i< (int)distancias.size();i++){                                                                      //Se almacenan aquellas distnacias que son mayores que 1000,
        if (distancias[i]>1000)                                                                                         //Indicando que es un punto de una puerta
            peaks.push_back(QPointF(ldata[i].dist * sin(ldata[i].angle),ldata[i].dist * cos(ldata[i].angle)));
    }
    for(int i = 0;i<(int)peaks.size();i++) {                                                                            //Recorre cada par de posibles puntos de puerta, y aquellos que
        for (int k = 0; k <(int) peaks.size(); k++) {                                                                   //Disten entre 900 y 1100 se consideran puntos de una puerta
            float dist = (sqrt(pow(peaks[i].x() - peaks[k].x(), 2) + pow(peaks[i].y() - peaks[k].y(), 2)));
            if (i != k && dist < 1100 && dist > 900) {
                Door a = Door(peaks[i], peaks[k]);
                Door b;
                bool encontrado = false;
                for(int j =0;j<doors.size();j++){                                                                       //Se recorre el vector de puertas para comprobar que no se repitan
                    b = doors[j];
                    if(((sqrt(pow(a.a.x() - b.a.x(), 2) + pow(a.a.y() - b.a.y(), 2)) < 100) && (sqrt(pow(a.b.x() - b.b.x(), 2) + pow(a.b.y() - b.b.y(), 2)) < 100))  //TODO hacer eficiente
                       || ((sqrt(pow(a.a.x() - b.b.x(), 2) + pow(a.a.y() - b.b.y(), 2)) < 100) && (sqrt(pow(a.b.x() - b.a.x(), 2) + pow(a.b.y() - b.a.y(), 2)) < 100)))
                        encontrado = true;
                }
                if(!encontrado)
                    doors.push_back(a);
            }

        }
    }
//        sqrt(pow(x.first - y.first, 2) +
//         pow(x.second - y.second, 2)) //Distancia entre puntos

}

Eigen::Vector2f SpecificWorker::goToRobot(RoboCompFullPoseEstimation::FullPoseEuler r_state ) {
    Eigen::Vector2f targ(target.pos.x(), target.pos.y());                                                         //Vector auxiliar en el que se guarda el objetivo
    Eigen::Vector2f robot(r_state .x, r_state .z);                                                                //Vector del robot
    Eigen::Matrix2f m;
    m << cos(r_state.rz), -sin(r_state.rz), sin(r_state.rz), cos(r_state.rz);                               //Matriz de translación
    return m.transpose() * (targ - robot);                                                                              //Devuelve el vector en el mundo del robot
}
Eigen::Vector2f SpecificWorker::goToWorld(RoboCompFullPoseEstimation::FullPoseEuler r_state , Eigen::Vector2f targ)     //Similar al anterior pero para pasar del mundo del robot al real
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
//    last_point = QPointF(p.x(), p.y());
//    target.pos = p;
//    target.activo = true;
}

QPointF SpecificWorker::forward(RoboCompFullPoseEstimation::FullPoseEuler r_state , RoboCompLaser::TLaserData &ldata) {
    Eigen::Vector2f punto = goToRobot(r_state );                                                                        //Pasa target a coordenadas del robot
    //calcular el angulo que forma el robot con el tagert
    float beta = atan2(punto.x(), punto.y());
    float dist = punto.norm();
    float close_to_target;                                                                                              //Factor de reducción de velocidad por cercanía al objetivo
    if (dist > 1000)                                                                                                    //En caso de que la distancia sea mas de 1000 es 1
        close_to_target = 1;                                                                                            //Si es menor, se divide entre 1000 para obtener 0,...
    else
        close_to_target = dist / 1000;
    float expresion = exp(-(pow((beta), 2) / 0.15));                                                            //Factor de reducción de velocidad a mayor ángulo con el objetivo
    float adv_speed = 1000 * expresion * close_to_target;
    this->differentialrobot_proxy->setSpeedBase(adv_speed, beta);                                               //Se aplica la velocidad de avance y rotacion
    //comprobar si hay un obstáculo delante
    if(ldata[ldata.size()/2].dist < 550) {                                                                              //Si se topa con un obstaculo //TODO Echar un vistazo a turn
         state = State::TURN;                                                                                            //Se alinea con el
        this->differentialrobot_proxy->setSpeedBase(0, 0);                                                      //Y se detiene
    }
    if(ldata[45].dist < 650)                                                                                            //Comprueba que este alineado con el obstaculo
        state = State::BORDER;                                                                                          //Pasa a estado bordear
    for(int i=1;i<15;i++) ldata.erase(ldata.begin());                                                           //Borra los primeros ceros del vector de laseres para correción de error
    if (dist < 300) {                                                                                                   //Comprueba que haya llegado al objetivo
        target.activo = false;                                                                                          //Pone el objetivo a inválido, ya ha llegado
        this->differentialrobot_proxy->setSpeedBase(0, 0);
        state = State::TURN_INIT;                                                                                       //Pasa al estado en el que gira para buscar puertas
        int i =0;
        Door d= doors[i];
        while (d.visited && i<doors.size()){                                                                            //Pone la puerta a la que ha llegado a visitada
            i++;                                                                                                        //(La útlima no visitada, porque las visitamos en orden )
            d = doors[i];
        }
        d.setvisited();
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


            grid.percentage_changed();



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



