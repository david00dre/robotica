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
    QRect dimensions(-5100, -2600, 10200, 5200);
    viewer = new AbstractGraphicViewer(this, dimensions);
    this->resize(900, 450);
    robot_polygon = viewer->add_robot(ROBOT_LENGTH);
    laser_in_robot_polygon = new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
    laser_in_robot_polygon->setPos(0, 190);     // move this to abstract
    anguloconelobjetivo =0;                                                                                             //Se utiliza para saber si el objetivo esta a la derecha o izquierda del robot
    try {
        RoboCompGenericBase::TBaseState r_state ;
        differentialrobot_proxy->getBaseState(r_state );
        last_point = QPointF(r_state .x, r_state .z);
    }
    catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }
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
        auto ldata = laser_proxy->getLaserData();                                                          //Vector con cada laser de visión del robot
        QGraphicsItem* poly = draw_laser(ldata);                                                                        //Polígono del laser
        update_map(r_state,ldata);
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

                forward(r_state ,ldata);
                break;
            case State::TURN:                                                                                           //Metodo auxiliar del fordward. EL robot se alinea con el obstaculo
                qInfo()<<"TURN";
                turn(ldata);
                break;
            case State::BORDER:                                                                                         //Método auxiliar de fordward. Bordea un obstaculo
                qInfo()<<"BORDER";
                border(ldata,poly);
                break;
        }
    } catch (const Ice::Exception &e) { std::cout << e.what() << std::endl; }

}

void SpecificWorker::turn(const RoboCompLaser::TLaserData &ldata)
{
    this->differentialrobot_proxy->setSpeedBase(0, 0.2*anguloconelobjetivo);                                            //Hace girar el robot
    RoboCompLaser::TLaserData pos = (anguloconelobjetivo > 0) ? sector5(ldata) : sector1(ldata);
    if (ldata[ldata.size()/2].dist > 1000 && get_distmin(pos) > 250)                                                                 //Hasta que esté alineado con el obstaculo
    {
        state = State::BORDER;
        this->differentialrobot_proxy->setSpeedBase(0, 0);
    }
}

void SpecificWorker::turn_init(const RoboCompLaser::TLaserData &ldata,
                               RoboCompFullPoseEstimation::FullPoseEuler r_state) {
    switch (estadoturn) {
        case 0:
//            qInfo()<<"PRIMERA FASE DE TURN_INIT";
            static auto initial_angle = (r_state.rz < 0) ? (2 * M_PI + r_state.rz) : r_state.rz;                 //Almacena la posición antes del giro
            this->differentialrobot_proxy->setSpeedBase(0, 0.4);
            estadoturn=1;
            break;
        case 1:
//            qInfo()<<"SEGUNDA FASE DE TURN_INIT";
            detect_doors(r_state, ldata);
            float current = (r_state.rz < 0) ? (2 * M_PI + r_state.rz) : r_state.rz;
            if (fabs(current - initial_angle) < (M_PI + 0.1) and
            fabs(current - initial_angle) > (M_PI - 0.1)) {
                this->differentialrobot_proxy->setSpeedBase(0, 0);                                             //Detiene el giro del robot
                float min=9999;
                if(door.get_midpoint().norm() != 0){
                    for(auto d: doors){//Busca la puerta mas cercana no visitada
                        if(!d.visited) qInfo()<<"esta puerta no esta visitada"<<d.get_midpoint().x()<<d.get_midpoint().y();
                        if(!d.visited && distancia_entre_puntos(d.get_midpoint(), door.get_midpoint()) < min){
                            min = distancia_entre_puntos(d.get_midpoint(), door.get_midpoint());
                            door = d;
                        }
                    }
                }else{
                    door= doors[0];
                }
                if(!door.visited) {
                    estadoturn = 0;
//                    qInfo()<<"PUERTA PARA VISITAR, EN CAMINO..."<<i<<"..........................................";
                    target.pos = QPointF(door.get_external_midpoint().x(),door.get_external_midpoint().y());
                    target.activo = true;
                    state = State::FORWARD;
                }else{                                                                                                  //En caso contrario, detiene la búsqueda
                    state = State::IDLE;
                    qInfo()<<"NO HAY MAS PUERTAS PARA VISITAR..............................";
                }

            }
            break;

    }
}

void SpecificWorker::detect_doors(RoboCompFullPoseEstimation::FullPoseEuler r_state, const RoboCompLaser::TLaserData &ldata){
//    qInfo()<<"BUSCANDO PUERTAS";
    std::vector<float> derivatives(ldata.size());
    derivatives[0] = 0;
    for (const auto &&[k, l]: iter::sliding_window(ldata, 2) | iter::enumerate)
        derivatives[k + 1] = l[1].dist - l[0].dist;

    std::vector<Eigen::Vector2f> peaks;
    for (const auto &&[k, der]: iter::enumerate(derivatives))
    {
        RoboCompLaser::TData l;
        if (der > 500)
        {
            l = ldata.at(k - 1);
            peaks.push_back(goToWorld(r_state,Eigen::Vector2f(l.dist * sin(l.angle), l.dist * cos(l.angle))));
        }
        else if (der < -500)
        {
            l = ldata.at(k);
            peaks.push_back(goToWorld(r_state, Eigen::Vector2f(l.dist * sin(l.angle), l.dist * cos(l.angle))));
        }
    }


    static std::vector<QGraphicsItem*> door_points;                                                                     //Dibuja los posibles puntos pertenencientes a puertas
    for(auto dp : door_points) viewer->scene.removeItem(dp);
    door_points.clear();
    for(const auto p: peaks)
    {
        door_points.push_back(viewer->scene.addRect(QRectF(p.x()-100, p.y()-100, 200, 200),
                                                    QPen(QColor("Magenta")),
                                                    QBrush(QColor("Magenta"))));
        door_points.back()->setZValue(200);
    }


    for(int i = 0;i<(int)peaks.size();i++) {                                                                            //Recorre cada par de posibles puntos de puerta, y aquellos que
        for (int k = 0; k <(int) peaks.size(); k++) {                                                                   //Disten entre 900 y 1100 se consideran puntos de una puerta
            float dist = distancia_entre_puntos(peaks[i],peaks[k]);
            if (i != k && dist < 1100 && dist > 700) {
//                qInfo()<<"PUERTA ENCONTRADA";
                Door a = Door(QPointF(peaks[i].x(),peaks[i].y()),QPointF(peaks[k].x(),peaks[k].y()));
                bool encontrado = false;
                for(int j =0;j<(int)doors.size()&&!encontrado;j++){                                                                       //Se recorre el vector de puertas para comprobar que no se repitan
                    Door b = doors[j];
                    QPointF mida = QPointF (a.get_midpoint().x(), a.get_midpoint().y());
                    QPointF midb = QPointF (b.get_midpoint().x(), b.get_midpoint().y());
                    if(distancia_entre_puntos(mida,midb) < 700)
                        encontrado = true;
                }
                if(!encontrado){
                    doors.push_back(a);
//                    qInfo()<<"PUERTA AÑADIDA";
                    }/*else qInfo()<<"PUERTA AÑADIDA ANTERIORMENTE";*/
            }

        }
    }
//        sqrt(pow(x.first - y.first, 2) +
//         pow(x.second - y.second, 2)) //Distancia entre puntos

}

Eigen::Vector2f SpecificWorker::goToRobot(RoboCompFullPoseEstimation::FullPoseEuler r_state ) {
    Eigen::Vector2f targ(target.pos.x(), target.pos.y());                                                         //Vector auxiliar en el que se guarda el objetivo
    Eigen::Vector2f robot(r_state.x, r_state.y);                                                                //Vector del robot
    Eigen::Matrix2f m;
    m << cos(r_state.rz), -sin(r_state.rz), sin(r_state.rz), cos(r_state.rz);                               //Matriz de translación
    return m.transpose() * (targ - robot);                                                                              //Devuelve el vector en el mundo del robot
}

Eigen::Vector2f SpecificWorker::goToRobot(RoboCompFullPoseEstimation::FullPoseEuler r_state, const Eigen::Vector2f &targ ) {

    Eigen::Vector2f robot(r_state.x, r_state.y);                                                                //Vector del robot
    Eigen::Matrix2f m;
    m << cos(r_state.rz), -sin(r_state.rz), sin(r_state.rz), cos(r_state.rz);                               //Matriz de translación
    return m.transpose() * (targ - robot);                                                                              //Devuelve el vector en el mundo del robot
}

Eigen::Vector2f SpecificWorker::goToWorld(RoboCompFullPoseEstimation::FullPoseEuler r_state , Eigen::Vector2f targ)     //Similar al anterior pero para pasar del mundo del robot al real
{
    Eigen::Vector2f robot(r_state .x, r_state .y);
    Eigen::Matrix2f m;
    m << cos(r_state.rz), -sin(r_state.rz), sin(r_state.rz), cos(r_state.rz);
    return (m * targ) + robot;
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

QPointF SpecificWorker::forward(RoboCompFullPoseEstimation::FullPoseEuler r_state , RoboCompLaser::TLaserData &ldata) {
//    qInfo()<<"AVANZANDO AL PUNTO MEDIO DE LA SIGUIENTE PUERTA--------";
    Eigen::Vector2f punto = goToRobot(r_state );                                                                        //Pasa target a coordenadas del robot
    //calcular el angulo que forma el robot con el tagert
    float beta = atan2(punto.x(), punto.y());
    float dist = punto.norm();
    float close_to_target;                                                                                              //Factor de reducción de velocidad por cercanía al objetivo
    if (dist > 1000)                                                                                                    //En caso de que la distancia sea mas de 1000 es 1
        close_to_target = 1;                                                                                            //Si es menor, se divide entre 1000 para obtener 0,...
    else
        close_to_target = (dist / 1000);
    close_to_target = close_to_target* get_distmin(ldata)/1000;
    check_free_path_to_target(ldata, Eigen::Vector2f (target.pos.x(),target.pos.y()), r_state);
    float expresion = exp(-(pow((beta), 2) / 0.15));                                                            //Factor de reducción de velocidad a mayor ángulo con el objetivo
    float adv_speed = 600 * expresion * close_to_target;
    this->differentialrobot_proxy->setSpeedBase(adv_speed, beta);                                               //Se aplica la velocidad de avance y rotacion
    if(get_distmin(sector2(ldata)) < 250
    || get_distmin(sector3(ldata)) < 250
    || get_distmin(sector4(ldata)) < 250 ) {
        if(get_distmin(sector2(ldata)) < get_distmin(sector4(ldata))) anguloconelobjetivo = -1;        //SI el objetivo esta a la derecha
        else if(get_distmin(sector2(ldata)) > get_distmin(sector4(ldata))) anguloconelobjetivo = 1;  //Si el objetivo esta a la izquierda
        else {                                                                                                      //Si el objetivo esta en frente
            float min =4000;
            int i=0;
            for(int k =0;k< (int)sector3(ldata).size();k++){
                if(sector3(ldata)[k].dist < min){
                    min = sector3(ldata)[k].dist;
                    i = k;
                }
            }
            anguloconelobjetivo = (i > 62) ? -1 : 1;
        }
         state = State::TURN;                                                                                            //Trata de alienarse con el obstaculo
        this->differentialrobot_proxy->setSpeedBase(0, 0);
    }
    if (dist < 150) {                                                                                                   //Comprueba que haya llegado al objetivo
//        qInfo()<<"-----------------PUERTA ALCANZADA-----------------";
        target.activo = false;                                                                                          //Pone el objetivo a inválido, ya ha llegado
        this->differentialrobot_proxy->setSpeedBase(0, 0);
        state = State::TURN_INIT;                                                                                       //Pasa al estado en el que gira para buscar puertas
        int i =0;
        bool encontrado = false;
        while (!encontrado &&i<(int)doors.size()){                                                                            //Pone la puerta a la que ha llegado a visitada
            if(distancia_entre_puntos(doors[i].get_midpoint(), door.get_midpoint()) < 100) {
                doors[i].setvisited();
                encontrado = true;
            }
            i++;                                                                                                        //(La útlima no visitada, porque las visitamos en orden )
        }
//        qInfo()<<"PONIENDO PUERTA A VISITADA";
    }
    return QPointF(punto.x(),punto.y());
}
void SpecificWorker::border(const RoboCompLaser::TLaserData &ldata, QGraphicsItem* poly) {
    RoboCompLaser::TLaserData diagonal = (anguloconelobjetivo < 0) ? sector2(ldata): sector4(ldata);
    RoboCompLaser::TLaserData pos = (anguloconelobjetivo < 0) ? sector1(ldata) : sector5(ldata);
    if(!poly->contains(target.pos)  || get_distmin(diagonal) < 250 )  {                                                         //Comprueba que el objetivo no este a la vista ni tenga obstaculos en el camino
        if (get_distmin(pos) < 250 || get_distmin(diagonal) < 250) {
            this->differentialrobot_proxy->setSpeedBase(100, 0.7 * anguloconelobjetivo);
        }
        else if (get_distmin(pos) > 300 || get_distmin(diagonal) > 300) {
            this->differentialrobot_proxy->setSpeedBase(100, -0.7 * anguloconelobjetivo);
        }
        else
            this->differentialrobot_proxy->setSpeedBase(100, 0);
    }
    else if(get_distmin(pos) < 350)
        this->differentialrobot_proxy->setSpeedBase(100, 0);
    else{
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
             else
                 grid.add_miss(goToWorld(r_state, tip));
        }
    }

}

int SpecificWorker::get_distmin(const RoboCompLaser::TLaserData &ldata) {
    int min =9999;
    for(auto l:ldata){
        if(l.dist<min) min = l.dist;
    }
    return min;
}

float SpecificWorker::distancia_entre_puntos(Eigen::Vector2f a, Eigen::Vector2f b) {
    return sqrt(pow(a.x() - b.x(), 2) + pow(a.y() - b.y(), 2));
}

float SpecificWorker::distancia_entre_puntos(QPointF a, QPointF b) {
    return sqrt(pow(a.x() - b.x(), 2) + pow(a.y() - b.y(), 2));
}

void SpecificWorker::check_free_path_to_target( const RoboCompLaser::TLaserData &ldata, const Eigen::Vector2f &goal,RoboCompFullPoseEstimation::FullPoseEuler bState)
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



