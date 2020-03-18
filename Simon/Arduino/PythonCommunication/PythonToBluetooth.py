import serial
import time

print("Start")
port="COM13" #This will be different for various devices and on windows it will probably be a COM port.
bluetooth=serial.Serial(port, 9600) #Start communications with the bluetooth unit
print("Connected")
bluetooth.flushInput() #This gives the bluetooth a little kick


k = 0.95
myFloat2 =105.0*k    # vänster
myFloat = 105.0      # Höger

for i  in range(5000):
    print("In loop")
    bluetooth.write(b"<"+str.encode(str(myFloat))+b","+str.encode(str(myFloat2))+b">")#These need to be bytes not unicode, plus a number +str.encode(str(myFloat2))+
    print(myFloat)
    input_data=bluetooth.readline() #This reads the incoming data. In this particular example it will be the "Hello from Blue" line
    print(input_data.decode())# Plot what the arduino board is getting. Incoming in bytes, thus decode
    time.sleep(1) #A pause between bursts
    #print(myFloat)                        
    myFloat += 0 # Increment float.
    myFloat2 += 0
    #print(myFloat)
bluetooth.flush()
print("Done")
bluetooth.close() # Shut down connection

