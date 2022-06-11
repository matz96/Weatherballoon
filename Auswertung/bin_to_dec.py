

## This code imports the bit data from the flight and converts it to integer Values


import struct
from typing import Dict
data = {}

row={}

def set_gain(gain):
    switch={
        0:512,
        1:256,
        2:128,
        3:64,
        4:32,
        5:16,
        6:8,
        7:4,
        8:2,
        9:1,
        10:0.5
    } 
        
    


def append_dict(line,name,long):
    value = line[0:long]
    line=line[long:len(line)]
    row[name]=int(value)    
    return line 

def sensor_out(line,sensor_nr):
    gain_pos = (2*12)
    len = gain_pos+1
    Sensor1 = []
    Sensor2 = []
    gains=bin(line[len])
    gain1=set_gain(int(gains[0:4]))
    gain2=set_gain(int(gains[4:8]))
    for i in range(6):
        value = line[0:2]
        line=line[2:len(line)]
        value=(int(value))*gain1
        Sensor1.append(value)
    for i in range(6):
        value = line[0:2]
        line=line[2:len(line)]
        value=(int(value))*gain2
        Sensor2.append(value)
    sens = "Sensor{}"
    row[sens.format(sensor_nr)]=Sensor1  
    row[sens.format(sensor_nr+1)]=Sensor2
            
    return line



#file2 = open('output_dec.txt','w')
file1 = open('output.bin', 'rb')
#nline = Dict[Time,Sensor_0[int,int,int,int,int,int],Sensor_1,Sensor_2,Sensor_3,Sensor_4,Sensor_5,GPS_long,GPS_lat,GPS_height,Bar_pre,Bar_temp,Bar_height,mag_x,mag_y,mag_z,heading,Temp_out]
#while file1.read(103)!= '' :
file=file1.read(-1) #writes whole file into string
Sensors = ["GPS_long","GPS_lat","GPS_height","Bar_pre","Bar_temp","Bar_height","mag_x","mag_y","mag_z","heading","Temp_out"]
mark = 0
line_width = 103
line_nr=0

while :#TODO how long
    
    line=file[mark:(mark+line_width)]
    append_dict(line,"time",1)#timestamp
    
    for i in range(3):#sensors and gains
        line = sensor_out(line,(i*2))    
        
    for name in Sensors:
        append_dict(line,name,2)  
        
    line_name = "line{}"
    data[line_name.format(line_nr)]=row
    row.clear()   
    mark=mark+line_width
    line_nr=+1
    
  
    
    
    
    
file1.close()      

    
    
    

    
    
        
        
 
 
 

