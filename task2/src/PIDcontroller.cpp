#include "PIDcontroller.h"

PIDcontroller::PIDcontroller(float Kp, float Ki, float Kd, float error_max){
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->error_max = error_max;
}
PIDcontroller::~PIDcontroller(){
    
}
unsigned long PIDcontroller::millis(){
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
float PIDcontroller::ajustar_error(float error){
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

   printf("%f \n" , P + I + D);
   return P + I + D;
}