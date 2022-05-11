#ifndef sensor_H_
#define sensor_H_
class sensor{
public:
void init_sensor();

void read_sensors();
void write_others_Data();
void write_line_end();

private:
void reduce_gain();
void double_gain();
void tcaselect();
void write_Sensor_Data();
void write_Gain_Data();
};







#endif