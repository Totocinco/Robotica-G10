#ifndef CONTROLLER_H
#define CONTROLLER_H
  #include <chrono>

class controller{
private:
float Kp , Ki , Kd, error_max;
float error_previo, error_acumulado = 0;
unsigned long instante_anterior = 0;


public:
controller(float Kp, float Ki, float Kd, float error_max);
~controller();
unsigned long millis();
float PIDajustar_error(float error);


};

#endif