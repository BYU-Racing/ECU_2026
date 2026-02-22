#include "GPSs.h"
#include <Adafruit_GPS.h>
#include <FlexCAN_T4.h>

constexpr int NO_LOCK = 0;

GPSs::GPSs(int id, HardwareSerial* serialIn) {
    this->id = id;
    this->wGPS = Adafruit_GPS(serialIn);
}

void GPSs::begin() {
    this->wGPS.begin(9600);
    this->wGPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ); //Sets 5hz Sensor rate
    this->wGPS.sendCommand(PMTK_ENABLE_WAAS); // Enables WAAS
}

Health GPSs::healthCheck() {
    unsigned long startCheck = millis();

    while(millis() - startCheck <= 500) {
        this->wGPS.read();
        if(this->wGPS.newNMEAreceived()) {
            return Health::HEALTHY;
        }
    }
    return Health::UNRESPONSIVE;
}

bool GPSs::ready() {
    this->wGPS.read();
    return this->wGPS.newNMEAreceived();
}

SensorData GPSs::read() {
    SensorData sensorData = SensorData(id, 1);
    uint8_t buf[8];
    BufferPacker<8> packer;
    packer.reset();

    if(this->wGPS.parse(this->wGPS.lastNMEA())) {
        if(this->wGPS.fix) {
            packer.pack(float(this->wGPS.latitudeDegrees));
            packer.pack(float(this->wGPS.longitudeDegrees));
            packer.deepCopyTo(buf);

        } else {
            packer.pack(NO_LOCK);
            packer.deepCopyTo(buf);
        }
    }
    sensorData.setMsg(buf, 8, 0);
    return sensorData;
}