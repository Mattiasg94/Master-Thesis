import serial
import time

print("Start")
port="COM13" #This will be different for various devices and on windows it will probably be a COM port.
bluetooth=serial.Serial(port, 9600) #Start communications with the bluetooth unit
print("Connected")
bluetooth.flushInput() #This gives the bluetooth a little kick
myFloat2 = 53.5
myFloat = 31.5

for i  in range(10):
    print("In loop")
    bluetooth.write(b"<"+str.encode(str(myFloat))+b","+str.encode(str(myFloat2))+b">")#These need to be bytes not unicode, plus a number +str.encode(str(myFloat2))+
    print(myFloat)
    input_data=bluetooth.readline() #This reads the incoming data. In this particular example it will be the "Hello from Blue" line
    print(input_data.decode())# Plot what the arduino board is getting. Incoming in bytes, thus decode
    time.sleep(1) #A pause between bursts
    #print(myFloat)                         
    myFloat += 20.1 # Increment float.
    
bluetooth.flush()
print("Done")
bluetooth.close() # Shut down connection

