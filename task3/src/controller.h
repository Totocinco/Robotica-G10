#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <chrono>
#include <math.h>
#include <genericworker.h>

class controller{
private:
float Kp , Ki , Kd, error_max;
float error_previo, error_acumulado, Wprev = 0;
unsigned long instante_anterior = 0;
typedef struct best_arc{
float cost;
float v; 
float w;
bool slow;
} best_s;


public:
controller(float Kp, float Ki, float Kd, float error_max);
~controller();
unsigned long millis();
float PIDajustar_error(float error);
std::tuple<float, float> DWA (RoboCompVisualElements::TObject persona, RoboCompLidar3D::TPoints points, float depth);
std::tuple<float, float, float, float, bool> rollout (float v, float w,RoboCompLidar3D::TPoints points);
};
float COST(float v, float x, float y, float h, float d_min, RoboCompVisualElements::TObject persona); 

#endif