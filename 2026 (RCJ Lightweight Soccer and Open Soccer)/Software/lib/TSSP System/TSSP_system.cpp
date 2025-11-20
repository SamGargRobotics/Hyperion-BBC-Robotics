#include <TSSP_system.h>


void TsspSystem::init() {
    for(int i = 0; i < TSSP_NUM; i++){
        pinMode(tsspPins[i],INPUT);
    }
}

void TsspSystem::update() {
    ballStr = 0;
    uint8_t tsspValues[TSSP_NUM] = {0};

    for(int i = 0; i < 255; i++){
        for(int j = 0; j < TSSP_NUM; j++){
            tsspValues[j] += 1 - digitalRead(tsspPins[j]);
            delayMicroseconds(1);
        }
    }

    int index = 0;
    for(int i = 1; i < TSSP_NUM; i++){
        if(tsspValues[i] >= tsspValues[index]) index = i;
    }

    ballDir = index * (360 / TSSP_NUM);
    ballStr = tsspValues[index];

    moveDir = fmod(ballDir + (((ballDir > 180 ? ballDir - 360 : ballDir) < 0) ? 
              -(constrain(0.02f * constrain(ballStr / ORBIT_TUNER, 0, 1) * 
              expf(4.5f * constrain(ballStr / ORBIT_TUNER, 0, 1)), 0.0f, 1.0f) *
              min(0.4f * expf(0.25f * abs(ballDir > 180 ? ballDir - 360 : 
              ballDir)) - 0.4f, 90.0f)) : (constrain(0.02f * constrain(ballStr /
              ORBIT_TUNER, 0, 1) * expf(4.5f * constrain(ballStr / ORBIT_TUNER, 
              0, 1)), 0.0f, 1.0f) * min(0.4f * expf(0.25f * abs(ballDir > 180 ? 
              ballDir - 360 : ballDir)) - 0.4f, 90.0f))), 360.0f);
    moveSpd = BASE_SPEED + (SURGE_SPEED - BASE_SPEED) * (1.0f - ((constrain(
              0.02f * constrain(ballStr / ORBIT_TUNER, 0, 1) * expf(4.5f * 
              constrain(ballStr / ORBIT_TUNER, 0, 1)), 0.0f, 1.0f) * min(0.4f * 
              expf(0.25f * abs(ballDir > 180 ? ballDir - 360 : ballDir)) - 0.4f,
              90.0f)) / 90.0f));
}


float TsspSystem::get_ball_dir() {
    return ballDir;
}

int TsspSystem::get_ball_str() {
    return ballStr;
}

float TsspSystem::get_move_dir() {
    return moveDir;
}

float TsspSystem::get_move_spd() {
    return moveSpd;
}