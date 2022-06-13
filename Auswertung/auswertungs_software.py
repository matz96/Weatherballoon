

## This code imports the bit data from the flight and converts it to integer Values


from cProfile import label
import struct
import numpy as np
import matplotlib.pyplot as plt
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
    return switch.get(gain,"Invalid input")
        
    


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

def get_data(item):
    ar=[] 
    for i in len(data):
        line_name = "line{}"
        ar.append=data[line_name.format(line_name)][item]
    return ar


def mafs(sens1,sens2,nr_ch):
    tot = {}
    for i in range(nr_ch):
        x=sens1[-1][i]
        y=sens2[-1][i]
        chan = "channel{}"
        tot.update({chan.format(i):(x/y)})
    return tot


def average(sensor):
    
   return np.average(sensor,axis=0)
        

        

def av_dif(sensor1,sensor2):
    sensor1=average(sensor1)
    sensor2=average(sensor2)
    return sensor1[-1]/sensor2[-1]
    






file1 = open("/home/matz/Documents/Git_software/pro4/pro4/Auswertung/output.bin", 'rb')
file=file1.read(-1) #writes whole file into string
file1.close()    
Sensors = ["GPS_long","GPS_lat","GPS_height","Bar_pre","Bar_temp","Bar_height","mag_x","mag_y","mag_z","heading","Temp_out"]
mark = 0
line_width = 103
line_nr=0

while file[mark+1] != '' :
    
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

timestr = get_data("time")
gps_height = get_data("GPS_height")
bar_height = get_data("Bar_height")
bar_pre = get_data("Bar_pre")

plt.plot(timestr,gps_height,'b-',label="GPS Height")
plt.plot(timestr,bar_height,'r-',label="Barometric Height")
plt.plot(timestr,bar_height,'g-',label="Barometic Pressure")
plt.xlabel('time [s]')    
plt.ylabel('height [m]')
plt.legend()
plt.show()

sensor0 = get_data("Sensor0")
sensor1 = get_data("Sensor1")
sensor3 = get_data("Sensor3")
sensor4 = get_data("Sensor4")
sensor5 = get_data("Sensor5")

trans_gr_1=mafs(sensor0,sensor1,5)
trans_gr_2=mafs(sensor4,sensor5,5)

for i in range(6):
    line_name = "line{}"
    plt.plot(timestr,trans_gr_1[line_name.format(line_name)],'-')
for i in range(6):
    line_name = "line{}"
    plt.plot(timestr,trans_gr_2[line_name.format(line_name)],'.')
plt.xlabel('time [s]')
plt.ylabel('Transmittance [%]')
plt.show()
#TODO: plot Average of all channels for the Sensor pairs
av_1=av_dif(sensor0,sensor1)
av_2=av_dif(sensor4,sensor5)

plt.plot(timestr,av_1,'r-')
plt.plot(timestr,av_2,'b.')
plt.xlabel("time [s]")
plt.ylabel("average Transmittance [%]")







  

    
    
    

    
    
        
        
 
 
 

