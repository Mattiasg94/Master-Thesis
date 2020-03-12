// Receive with start- and end-markers combined with parsing
#include "SoftwareSerial.h"
SoftwareSerial serial_connection(11, 10);//Create a serial connection with TX and RX on these pins

const byte numChars = 64;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

// variables to hold the parsed data
float uvrfloatFromPC;
float uvlfloatFromPC;
int RightWheelPin = 9;
int Imax = 0.140;
int Rmax = 64.287;
float r = 0.07;
float k = 0.0232;
boolean newData = false;
float PWM_r;
float PWM_l;
int uvrintFromPC;

//============

void setup() {
    // Initialize everything
    serial_connection.begin(9600);
    serial_connection.println("Enter data in this style <24.7>  ");    
    serial_connection.println("Ready!!!");
    pinMode(RightWheelPin,OUTPUT);
}

//============

void loop() { // MAIN
    recvWithStartEndMarkers(); // This function constructs a string of max length size of numChars. 
                               // sets newData = true if there us any data to read
    if (newData == true) {
        strcpy(tempChars, receivedChars); // This is necessary if we use multiple input types, since strtok() replaces , with \0
        parseData();                      // Split the data (Not needed for us, currently this is the lamp function)
                                          // Also converts Chars to floats.
        showParsedData();                 // Print data to python
        applyInput();
        newData = false;                  // Reset that we are done with this piece of data. 
    }
}

//============

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (serial_connection.available() > 0 && newData == false) {
        rc = serial_connection.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//============

void parseData() {      // splits the data into its parts, converts chars to float
  
    char * strtokIndx; // this is used by strtok() as an index
    
    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    uvrfloatFromPC = atof(strtokIndx);     // convert this part to a float
    //serial_connection.println(strtokIndx);
    strtokIndx = strtok(NULL,","); // Vrf "," ??
    uvlfloatFromPC = atof(strtokIndx);     // convert this part to a float
    
}
    

//============

void showParsedData() {
   serial_connection.print("Float1:");
   serial_connection.println(uvrfloatFromPC);
   //serial_connection.print("Float2:");
   //serial_connection.println(uvlfloatFromPC);
}

//===========

void applyInput(){
    uvrintFromPC = round(uvrfloatFromPC);
    if(uvrintFromPC>=255){
      uvrintFromPC = 255;
      }
      
    analogWrite(RightWheelPin, round(uvrfloatFromPC));
}
