import numpy as np


file2 = open('output_dec.txt','w')
file1 = open('output.bin', 'rb')
file_bin = file1.readlines()

def write_output(line,long):
    value = line[0:long]
    line=line[long:len(line)]
    file2.write(int(value))
    


def sensor_out(line):
    
    for count in range(6):
        line = write_output(line,16)
        

def gain_out(line):
    
    for count in range(2):
        line = write_output(line,4)
        

def others_out(line):
    for count in range(9):
        line = write_output(line,16)






for line in file_bin:
    
    for i in 10:
        if i>=9: 
            others_out(line) 
            break
        elif i%2 == 0:
            line = sensor_out(line)
            
        elif i%2 == 1:
            line = gain_out
            
    
    
    
    

    
    
        
        
 
 
 

