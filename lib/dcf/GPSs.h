#ifndef GPSs_H
#define GPSs_H


#include "Sensor.h"
#include <Adafruit_GPS.h>
#include <BufferPacker.h>


class GPSs final : public Sensor {

public:
    GPSs(int id, HardwareSerial* serialIn);
    void begin();
    bool ready() override;
    SensorData read() override;
    Health healthCheck() override;

private:

    Adafruit_GPS wGPS;

};

#endif