#ifndef sensor_H_
#define sensor_H_
#endif

#include <Wire.h>

uint8_t init_sensors(TwoWire *theWire);
int8_t readSensor(uint8_t sensorNumber, char * dataString, TwoWire *theWire);
void read_sensors(char * data, TwoWire *theWire);
void write_others_Data();
void write_line_end();

uint8_t reduce_gain();
uint8_t double_gain();
void tcaselect(uint8_t channel, TwoWire *theWire);
void write_Sensor_Data();
void write_Gain_Data();








