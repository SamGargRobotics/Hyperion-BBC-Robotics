#include <Dribbler.h>

void Dribbler::run(float spd){
    int rspd = round(abs(spd));
    analogWrite(DRPWM, rspd);
    digitalWrite(DRINA, (rspd > 0));
    digitalWrite(DRINB, (rspd < 0));
}