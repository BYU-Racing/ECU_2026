#ifndef DATACOLLECTOR_TPP
#define DATACOLLECTOR_TPP

#include "DataCollector.h"
#include <SensorData.h>

DataCollector::DataCollector(const ReservedIDs id, const size_t numSensors, Sensor** sensors, const bool debug)
{
    this->id = id;
    this->numSensors = numSensors;
    this->sensors = sensors;
    this->debug = debug;
}

void DataCollector::begin(FlexCAN_T4<CAN1, RX_SIZE_256>* motorCAN, FlexCAN_T4<CAN2, RX_SIZE_256>* dataCAN)
{
    this->motorCAN = motorCAN;
    this->dataCAN = dataCAN;
}

void DataCollector::read(Sensor* sensor) const
{
    const SensorData data = sensor->read();
    const CAN_message_t* msgs = data.getMsgs();
    for (size_t msgIndex = 0; msgIndex < data.getMsgCount(); msgIndex++)
    {
		if (sensor->isCritical())
		{
		    motorCAN->write(msgs[msgIndex]);
		} else
		{
		    dataCAN->write(msgs[msgIndex]);
		}
        if (debug) { sensor->debugPrint(msgs[msgIndex]); }
    }
}

void DataCollector::healthCheck()
{
    Health status[numSensors];
    for (size_t sensorIndex = 0; sensorIndex < numSensors; sensorIndex++)
    {
        status[sensorIndex] = sensors[sensorIndex]->healthCheck();
    }
    healthMsg.id = id;
    healthMsg.len = numSensors;
    for (size_t statusIndex = 0; statusIndex < numSensors && statusIndex < MAX_BUF_SIZE; statusIndex++)
    {
        healthMsg.buf[statusIndex] = status[statusIndex];
    }
    dataCAN->write(healthMsg);
}

void DataCollector::checkSensors() {
    if (dataCAN->read(healthMsg) && healthMsg.id == ReservedIDs::HealthCheckId)
    {
        healthCheck();
    }
    for (size_t sensorIndex = 0; sensorIndex < numSensors; sensorIndex++)
    {
        if (Sensor* sensor = sensors[sensorIndex]; sensor->ready())
        {
            read(sensor);
        }
    }
}

#endif // DATACOLLECTOR_TPP
