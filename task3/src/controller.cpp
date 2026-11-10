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
std::tuple<float, float, float, float, bool> controller:: rollout (float v, float w,RoboCompLidar3D::TPoints points)
{
    float x, y, h = 0;
    dmin = 1000000;

    float dt = 0.1;

    for(int i=0; i<20;i++)
    {
        x= x - v * std::sin(h)*dt;
        y += v * std::cos(h) * dt;
        h += w * dt;
        for (const auto &q : points)
        {
            float dx = x - q.x;
            float dy = y - q.y;
            float d = std::sqrt(dx * dx + dy * dy);
        }
    }
}


std::tuple<float, float> controller::DWA(RoboCompVisualElements::TObject persona, RoboCompLidar3D::TPoints points)
{
    bool far = ;
    float best;
    for(int v = 0; v<=1000; v +=100)
    {
        for(int w= -1; w<1;w+=0.1) // exactamente 21 posiciones de saltos de 0,1 del -1 al 1
        {
            
        }
    }
    return ;
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