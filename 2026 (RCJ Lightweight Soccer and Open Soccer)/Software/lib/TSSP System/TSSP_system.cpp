#include <TSSP_system.h>


void TsspSystem::init(){
    for(int i = 0; i < TSSP_NUM; i++){
        pinMode(tsspPins[i],INPUT);
    }
}

/*!
* @brief ➡️⚽
*/
void TsspSystem::read_all_tssp(){
    ballStr = 0;
    uint8_t tsspValues[TSSP_NUM] = {0};
    for(int i = 0; i < 255; i++){
        for(int j = 0; j < TSSP_NUM; j++){
            tsspValues[j] += 1-digitalRead(tsspPins[j]);
            delayMicroseconds(1);
        }
    }
    int index = 0;
    for(int i = 1; i < TSSP_NUM; i++){
        index = (tsspValues[index] <= tsspValues[i]) ? i : index;
    }
    ballDir = (index)*(360/TSSP_NUM);
    ballStr = tsspValues[index];
}

float TsspSystem::get_ball_dir(){
    return ballDir;
}
int TsspSystem::get_ball_str(){
    return ballStr;
}