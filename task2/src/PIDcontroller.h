#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H
  #include <chrono>

class PIDcontroller{
private:
float Kp , Ki , Kd, error_max;
float error_previo, error_acumulado = 0;
unsigned long instante_anterior = 0;


public:
PIDcontroller(float Kp, float Ki, float Kd, float error_max);
~PIDcontroller();
unsigned long millis();
float ajustar_error(float error);


};

#endif