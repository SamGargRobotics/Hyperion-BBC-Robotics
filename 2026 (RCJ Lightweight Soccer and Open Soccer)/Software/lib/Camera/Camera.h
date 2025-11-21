#ifndef CAMERA_H
#define CAMERA_H

#include <Arduino.h>
#include <math.h>
#include <Configuration.h>

class TargetData {
public:
    float angle()   const { return _angle; }
    float dist()    const { return _dist; }
    bool  visible() const { return _visible; }
private:
    friend class Camera;
    float _angle   = 0;
    float _dist    = 0;
    bool  _visible = false;
};

class BallData {
public:
    float angle() const { return _angle; }
    float dist()  const { return _dist; }
private:
    friend class Camera;
    float _angle = 0;
    float _dist  = 0;
};

class Camera {
public:
    Camera() {}
    void init();
    void update(bool attackBlue);
    const TargetData&   attack()        const { return attackGoal; }
    const TargetData&   defend()        const { return defendGoal; }
    const BallData&     ball()          const { return ballInfo; }
private:
    float calc_distance(float x, float y);
    float calc_angle(float opp, float adj);
    void shift(int& x, int& y);
    TargetData      attackGoal;
    TargetData      defendGoal;
    BallData        ballInfo;
};

#endif
