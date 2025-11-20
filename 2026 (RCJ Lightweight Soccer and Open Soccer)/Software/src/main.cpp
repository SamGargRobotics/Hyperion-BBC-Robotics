#include <Arduino.h>
#include <Drive_system.h>
#include <Debug.h>
#include <PID.h>   
#include <Voltage_divider.h>
#include <Timer.h>
#include <TSSP_system.h>
#include <Bluetooth.h>

DriveSystem motors;
TsspSystem tssp;
Bluetooth bt;

void setup() {
    motors.init();
    tssp.init();
    bt.init();
}

void loop() {
    tssp.update();
    bt.update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    float _dir = 0;
    float _spd = 0;
    float _cor = 0;
    if(bt.get_role()) {
        _dir = tssp.get_move_dir();
        _spd = tssp.get_move_spd();
    }
    motors.run(_spd, _dir, _cor);
}