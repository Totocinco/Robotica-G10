#include "controller.h"

controller::controller(float Kp, float Ki, float Kd, float error_max){
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->error_max = error_max;
}
controller::~controller(){
    
}
unsigned long controller::millis(){
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

float controller::PIDajustar_error(float error){
   if(error > error_max)
   return error_max;
   unsigned long instante_actual = millis();
   float diff_tiempo;
   if(diff_tiempo = (instante_actual - instante_anterior) / 1000.0; diff_tiempo <= 0 || instante_anterior == 0)
   diff_tiempo = 0.001;


   float P = Kp * error;

   error_acumulado += error * diff_tiempo;
   float I = Ki * error_acumulado;

   float tasa_cambio = (error - error_previo) / diff_tiempo;
   float D = Kd * tasa_cambio;


   error_previo = error;
   instante_anterior = instante_actual;

    float valor_ajustado = P + I + D;

  
    if(valor_ajustado > error_max)
   return error_max;
   return P + I + D;
}

std::tuple<float, float> controller::DWA(RoboCompVisualElements::TObject persona, RoboCompLidar3D::TPoints points, float depth, Eigen::Vector2f CordPersona)
{
    bool far =  depth > 1100;
    reset_arcs();
    int currentBest = -1;
     std::optional<arc_> arc = std::nullopt;
    for(int v = 0; v<=1000; v +=100)
    {
        for(float w = Wprev - 1; w <= Wprev + 1; w = w + 0.1 ) // exactamente 21 posiciones de saltos de 0,1 del Wprev -1 al Wprev + 1
        {
        if(std::abs(w) > 1.5)
        continue;

        std::tuple<float, float, float, float, bool> res = rollout(v, w, obstacle_points(points, CordPersona), depth);
        const auto & [x, y, h, d_min, hit] = res;
        if(hit)
        continue;

        if(v > sqrt(2 * 500 * std::max(d_min - 400, 0.f)))
        continue;

        float cost = COST(v, x, y, h, d_min, CordPersona.x(), CordPersona.y());
        bool slow = far && v < 150;
        if(currentBest == -1 && !arcs.back().hit){
            arcs.back().chosen = true;
            arcs.back().cost = cost;
            arcs.back().slow = slow; 
            currentBest = arcs.size() - 1; 
        }
        else{
            if((!slow && arcs.at(currentBest).slow) || (cost < arcs.at(currentBest).cost && slow == arcs.at(currentBest).slow)){
              arcs.at(currentBest).chosen = false;
              currentBest = arcs.size() - 1; 
              arcs.back().chosen = true;
              arcs.back().cost = cost;
              arcs.back().slow = slow; 
            }
        }
        }
    }
    if(currentBest == -1){
        Wprev = 0;
        return {0, 0};
    }
    Wprev = arcs.at(currentBest).w;
    return {arcs.at(currentBest).v, arcs.at(currentBest).w};
}

//person-> x e y  de la persona
std::vector<Eigen::Vector2f> controller::obstacle_points(RoboCompLidar3D::TPoints points,const Eigen::Vector2f& person)const{
    std::vector<Eigen::Vector2f> out;
    for(RoboCompLidar3D::TPoint point : points){
        //distancia de la persona y el punto del rallo
        float distance2D  = std::sqrt(std::pow(point.x - person.x() , 2) + std::pow(point.y - person.y() , 2));
        if(point.z> Z_MIN){
            if(point.z < Z_MAX){
                if(point.distance2d < RANGE_MAX){
                    if(point.distance2d > SELF_RANGE){
                        if(distance2D > PERSON_RADIUS){
                            Eigen::Vector2f aux(point.x,point.y);
                            out.push_back(aux);
                        }
                    }
                }
            }
        }
    }
    return out;
}

//Terminar
std::tuple<float, float, float, float, bool> controller:: rollout (float v, float w,std::vector<Eigen::Vector2f> points, float depth)
{
    float x = 0.0f, y = 0.0f, h = 0.0f;
    float dmin = std::numeric_limits<float>::infinity();
    float dt = 0.1;
    arc_ arc = {
                .v = v,
                .w = w,
                .chosen = false,
                .hit = false
            };
    for(int i=0; i<20;i++)
    {
        x -= v * std::sin(h)*dt;
        y += v * std::cos(h) * dt;
        h += w * dt;
        arc.points.push_back(Eigen::Vector2f (x , y));
        for (const auto &q : points)
        {
            float dx = x - q.x();
            float dy = y - q.y();
            float d = std::sqrt(dx * dx + dy * dy);
            if(d < SELF_RANGE && d < depth){
                arc.hit = true;
                arcs.push_back(arc);
                return {0,0,0,0,true};
            }
            if(d<dmin){
                dmin = d;
            }
        }
    }
     arcs.push_back(arc);
    return { x,y,h,dmin,false};
}

float controller::COST(float v, float x, float y, float h, float d_min, float personax, float personay){
    float gx, gy;
    gx = personax - x;
    gy = personay - y;

    float h_ideal = atan2(-gx, gy);
    float Ch = std::abs(std::remainder(h_ideal - h, 2 * M_PI)) / M_PI;

    float Cd = std::abs(std::sqrt(std::pow(gx , 2) + std::pow(gy , 2)) - 1000) / 2000;

    float Cc = std::pow(std::max(0.f, 1 - (d_min - SELF_RANGE) / 500) , 2);

    float Cv= 1 - v/1000; 

    return Ch + Cd + Cc + Cv; 
}

void controller::reset_arcs(){
    arcs.clear();
}