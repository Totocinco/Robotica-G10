#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <chrono>
#include <math.h>
#include <genericworker.h>

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
std::tuple<float, float> DWA (RoboCompVisualElements::TObject persona, RoboCompLidar3D::TPoints points);
std::tuple<float, float, float, float, bool> rollout (float v, float w,RoboCompLidar3D::TPoints points);
};

#endif