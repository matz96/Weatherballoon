import numpy as np




def write_output(line,long):
    value = line[0:long]
    line=line[long:len(line)]
    file2.write(int(value))
    return line


def sensor_out(line):
    
    for count in range(12):
        line = write_output(line,16)
    return line

def gain_out(line):
    
    for count in range(2):
        line = write_output(line,4)
    return line

def others_out(line):
    for count in range(9):
        line = write_output(line,16)
    return line




def file_write():
    file2 = open('output_dec.txt','w')
    file1 = open('output.bin', 'rb')
    file_bin = file1.readlines()
    for line in file_bin:
        
        for i in range(6):

            if i%2 == 0:
                line = sensor_out(line)
                
            elif i%2 == 1:
                line = gain_out
    
        others_out(line)  
        file2.write("\n")       
    
    
    
    

    
    
        
        
 
 
 

