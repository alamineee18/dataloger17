//GY-30 Light intensity sensor. Connection pin : D1 = SCL (GY-30) and D2 = SOA (GY-30)
#include <Wire.h>
const int GY30_ADDR = 0x23; // I2C address of GY-30 sensor


//YF-S201 Water flow sensor, connection pin D5 and D6
int flow1_sensorPin = D5;
volatile long pulse1;
unsigned long lastTime1;
float volume1;


//18B20 Temp. Sensor, connection pin D7
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 13 // Data wire is plugged into port D7 on the NodeMCU

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature. 
int numberOfDevices; // Number of temperature devices found
DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address


void setup() {
  Serial.begin(9600);
 
  //GY-30
  Wire.begin();
  delay(200);
  
  //YF-S201
  pinMode(flow1_sensorPin, INPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(flow1_sensorPin), increase1, RISING);
//  pinMode(flow2_sensorPin, INPUT);
//  attachInterrupt(digitalPinToInterrupt(flow2_sensorPin), increase2, RISING);

  //18B20 Temp. sensor
  Serial.begin(9600);  // start serial port
  sensors.begin(); // Start up the library
  numberOfDevices = sensors.getDeviceCount();  // Grab a count of devices on the wire
 
 
  Serial.println(" ");
  Serial.println("Light, Flow,  Temp1, Temp2, Temp3, Temp4, Temp5.");

}

void loop() {

  // Request 2 bytes of data from GY-30 sensor
  Wire.beginTransmission(GY30_ADDR);
  Wire.write(0x10); // 1 [Lux] resolution mode
  Wire.endTransmission();
  delay(200);

  // Read 2 bytes of data from GY-30 sensor
  Wire.requestFrom(GY30_ADDR, 2);
  if (Wire.available() == 2) {
    int highByte = Wire.read();
    int lowByte = Wire.read();
    int lux = (highByte << 8) + lowByte;

    // Print the light intensity in lux
    //Serial.print("Light intensity: ");
    Serial.print(lux);
    Serial.print(", ");  
  }

  delay(1000); // Wait for 1 second before taking the next reading

  //YF-S201 start
  volume1 = 2.663 * pulse1 / 1000 * 30;
  if (millis() - lastTime1 > 2000) {
    pulse1 = 0;
    lastTime1 = millis();
  }
  Serial.print(String(volume1) + ", ");

  //YF-S201 end
  
  
  //18B20 Start
  sensors.requestTemperatures(); // Send the command to get temperatures
  
  // Loop through each device, print out temperature data
  for(int i=0;i<numberOfDevices; i++) {
    
    if(sensors.getAddress(tempDeviceAddress, i))
    {
      // Search the wire for address		
      float tempC = sensors.getTempC(tempDeviceAddress);    // Print the data
      Serial.print(tempC);
      Serial.print(", ");
     
    } 
    delay(100);	
  }

  //18B20 end
  
  Serial.println(" ");

}

//function for YF-S201
ICACHE_RAM_ATTR void increase1() {
  pulse1++;
}


// function to print a device address for 18B20
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
      Serial.print(deviceAddress[i], HEX);
  }
}