#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>
#include <TSSP_system.h>
#include <Camera.h>

class Debug {
public:
    void init();
    void update(Tssp_system* tssp, Camera* cam);

private:
    String serialMode = "none"; // keeps track of mode
};

#endif
