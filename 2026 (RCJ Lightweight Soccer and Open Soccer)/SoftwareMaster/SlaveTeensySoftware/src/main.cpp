#include <Arduino.h>
#include <TSSP_system.h>

TsspSystem tssp;

void setup() {
  Serial1.begin(115200);   // high speed, stable
  tssp.init();
}

void loop() {
  tssp.update();
  Serial1.printf("DIR:%.2f,STR:%.2f\n", tssp.get_ball_dir(), tssp.get_ball_str());
}