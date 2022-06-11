''' data={}


thisdict =	{
  "brand": "Ford",
  "model": "Mustang",
  "year": 1964
}

for i in range(2):
	row="col{}"
	row = row.format(i)
	data[row] = thisdict
 '''
 
file1 = open("/home/matz/Documents/Git_software/pro4/pro4/Auswertung/output.bin", 'rb')
line= file1.read(-1)



print(line[4:8])

