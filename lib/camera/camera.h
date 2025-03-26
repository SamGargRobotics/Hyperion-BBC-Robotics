#ifndef CAMERA_H
#define CAMERA_H
 
#include <Arduino.h>
 
class Camera {
public:
    Camera();
    void init_camera();
    void read_camera();
    float calculate_hypot(float x, float y);
    float calculate_theta(float o, float h);
    float goal_index;
    float angle_to_goal;
private:
    int goal_x;
    int goal_y;
};
 
#endif