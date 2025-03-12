#include "batread.h"

void BatRead::init() {
    pinMode(BAT_READ_PIN, INPUT);
}

float BatRead::read() { 
    rawValue = analogRead(BAT_READ_PIN);
    float scaledValue = ((rawValue - lowest) / (highest - lowest)) * 100;
    return scaledValue; //returns a value from 0-100.
}