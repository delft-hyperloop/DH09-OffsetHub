#include <Arduino.h>
<<<<<<< HEAD
//#include <SoftwareSerial.h>
#include <FlexCAN_T4.h> 
=======
#include <FlexCAN_T4.h>
>>>>>>> bd94fd2e6917339f4ddfc87c68fb413021436fcf

// ↓↓↓↓CONFIGURE THIS↓↓↓↓
const int NUM_SENSORS = 8;                                          // Number of sensors plugged in
const int PLUGGED_PORTS[NUM_SENSORS] = {1, 2, 3, 4, 5, 6, 7, 8};    // Phyiscal ports plugged in (1-8) (e.g. Serial1, Serial2 plugged --> {1,2})

<<<<<<< HEAD
const int NUM_PORTS = 8;
const int PACKET_SIZE = 16;
const unsigned long PRINT_INTERVAL = 50; // milliseconds

unsigned long lastPrintTime = 0;
unsigned long lastCANsendTime = 0;
unsigned long CANsendInterval = 50;
=======
const float sendFrequency = 2000;         // Frequency to send data (in Hz)
const unsigned long printFrequency = 1;   // Frequency to print data (in Hz)

const int CAN_ID_HEADER = 0x33D;          // CAN ID for header message
const int CAN_ID_DATA1 = CAN_ID_HEADER+1; // CAN ID for data1 message
const int CAN_ID_DATA2 = CAN_ID_HEADER+2; // CAN ID for data2 message
>>>>>>> bd94fd2e6917339f4ddfc87c68fb413021436fcf

// ↓↓↓↓DON'T TOUCH↓↓↓↓
const int sensorBaudRate = 460800; // Baud rate for the sensors
const double sensorOutputRate = 1.0 / (44.0 / sensorBaudRate + 1e-5); // Sensor output rate in Hz (460800 baud rate = 9.4 kHz) (from datasheet)
const int noReadings = sensorOutputRate/sendFrequency; // Number of readings to average (unused for now, might come in handy later)

// Struct to hold sensor reading data
struct SensorReading {
  uint8_t rawData[4]; // 4 bytes from sensor (each byte: 4 bit header, 4 bit data)
  uint16_t headers; //4x 4 bits
  uint16_t data; // 4x 4 bits
  uint16_t distance; // in mm (unused for now)
};

uint16_t sensorReadingsArray[NUM_SENSORS]; // Array to store sensor readings
uint8_t flagByte = 0x00; // If sensor rejected, then corresponding bit is set to 1

CAN_message_t msg; // CAN message object
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can; // CAN bus object
const int CAN_BAUD_RATE = 1000000; // CAN bus baud rate (1 Mbps)

IntervalTimer sendingTimer; // Timer for sending CAN messages
IntervalTimer printTimer; // Timer for printing data

// Array of Serial pointers
HardwareSerial* serialPorts[8] = {
  &Serial1, &Serial2, &Serial3, &Serial4,
  &Serial5, &Serial6, &Serial7, &Serial8
};
// Array of RE/DE pins for each port
// Format: {RE, DE}
uint8_t RE_DE_PINS[8][2] = {
  {3, 2},    // RE_1, DE_1
  {10, 9},   // RE_2, DE_2
  {39, 38},  // RE_3, DE_3
  {41, 40},  // RE_4, DE_4
  {19, 18},  // RE_5, DE_5
  {27, 26},  // RE_6, DE_6
  {31, 30},  // RE_7, DE_7
  {33, 32}   // RE_8, DE_8
};

<<<<<<< HEAD
// Create buffers for each port
uint8_t buffers[NUM_PORTS][PACKET_SIZE];
uint8_t bufferIndices[NUM_PORTS] = {0};

const uint32_t CANID1 = 0x123;
const uint32_t CANID2 = 0x321;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;
CAN_message_t msg;



#define DE 93
#define RE 90

#define DE_1 2
#define RE_1 3

#define DE_2 9
#define RE_2 10

#define DE_6 26
#define RE_6 27

#define DE_7 30
#define RE_7 31

#define DE_8 32
#define RE_8 33
#define BUFFER_SIZE 32  // Maximum number of bytes to read at a time

  // byte dataToSend[6] = {0x01, 0x83, 0x84, 0x80, 0x80, 0x83};    //change baud rate to 2400 * 48 = 115200 bit/s
  // byte dataToSend3[3] = {0x01, 0x84, 0xAA};  // Save Parameters to Flash
  // Serial7.write(dataToSend3, sizeof(dataToSend3));  // Send data over RS-422
  // Serial7.flush();

  // if (Serial7.available() >=0 ) 
  // {
  //   Serial.println("Parameters Saved");  
  //   uint8_t response = Serial7.read();
  //   Serial.print(response);
  // }
=======
// put function declarations here:
void setupSerialPorts(); // Function to set up serial ports
void setupREDEPorts(); // Function to set up RE/DE pins
void sendDataToAllPorts(uint8_t* data, size_t length); // Function to send data to all ports
SensorReading extractData(SensorReading reading); // Function to extract data from the sensor
bool areHeadersValid(SensorReading reading); // Function to check if headers are valid
void sendCANMessages(); // Function to send CAN messages
void printingFunction(); // Function to print data
>>>>>>> bd94fd2e6917339f4ddfc87c68fb413021436fcf

void setup() {
  Serial.begin(115200);   // USB serial
  delay(1000); // Wait for serial to initialize
  Serial.println("Initializing...");

<<<<<<< HEAD
  Serial1.begin(115200, SERIAL_8E1); // RS485 Communication to Offset Sensor
  Serial2.begin(115200, SERIAL_8E1); // RS485 Communication to Offset Sensor
  Serial7.begin(115200, SERIAL_8E1);
  Serial6.begin(115200, SERIAL_8E1); // RS485 Communication to Offset Sensor
  Serial8.begin(9600, SERIAL_8E1); // RS485 Communication to Offset Sensor
=======
  Serial.println("Configuring serial ports: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    int port = PLUGGED_PORTS[i] - 1; // Get the serial port number for the current sensor
    Serial.print("Serial");
    Serial.print(port + 1);
    Serial.print("; ");
  }
  Serial.println();

  setupSerialPorts(); // Initialize serial ports
  setupREDEPorts(); // Initialize RE/DE pins
  Serial.println("Serial ports initialized!");

  // Start stream of data
  byte dataToSend[2] = {0x01, 0x87};
  sendDataToAllPorts(dataToSend, 2);
  Serial.println("Data stream started!");

  can.begin(); // Initialize CAN bus
  can.setBaudRate(CAN_BAUD_RATE); // Set CAN bus baud rate
  Serial.println("CAN bus initialized!");

  sendingTimer.begin(sendCANMessages, 1e6/sendFrequency); // Start the timer to send CAN messages at the specified frequency
  printTimer.begin(printingFunction, 1e6/printFrequency); // Start the timer to print data at the specified frequency
  Serial.println("Timers initialized!");

  Serial.println("Setup complete!");
  Serial.println("Starting data acquisition...");
}

void loop() {
  for (int sensor = 0; sensor < NUM_SENSORS; sensor++){
    int port = PLUGGED_PORTS[sensor] - 1; // Get the serial port number for the current sensor 

    if (serialPorts[port]->available() >= 4){
      SensorReading reading;
      
      serialPorts[port]->readBytes(reading.rawData, 4); // Read 4 bytes from the serial port
      
      reading = extractData(reading); // populate reading.headers and reading.data
      
      // check if headers are valid (all equal). if not, skip this reading and flag it
      if (!areHeadersValid(reading)){
        flagByte |= (1 << sensor); // Set the corresponding bit in the flag byte
        continue;
      } 

      sensorReadingsArray[sensor] = (2500 + (reading.data * 2500) / 0x4000); // formula from datasheet
      flagByte &= ~(1 << sensor); // Reset to 0 the corresponding bit in the flag byte
>>>>>>> bd94fd2e6917339f4ddfc87c68fb413021436fcf

  Serial.println("Setting up CAN Bus...");
  can.begin();
  can.setBaudRate(1000000);


    } else{ // If not enough data is available, flag this sensor
      flagByte |= (1 << sensor); // Set the corresponding bit in the flag byte
      continue;
    }
  }

}

void setupSerialPorts() {
  for (int i = 0; i < NUM_SENSORS; i++) {
      if (PLUGGED_PORTS[i] < 1 || PLUGGED_PORTS[i] > 8) {
        Serial.println("Error: Invalid port number inputted in PLUGGED_PORTS array.");
        return;  // Or handle the error as needed.
    }
    int port = PLUGGED_PORTS[i] - 1; // Get the serial port number for the current sensor

    serialPorts[port]->begin(sensorBaudRate, SERIAL_8E1); 
  }
}

void setupREDEPorts() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int port = PLUGGED_PORTS[i] - 1; // Get the serial port number for the current sensor
    pinMode(RE_DE_PINS[port][0], OUTPUT); // Set RE pin as output
    pinMode(RE_DE_PINS[port][1], OUTPUT); // Set DE pin as output
  }
}

void sendDataToAllPorts(uint8_t* data, size_t length) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int port = PLUGGED_PORTS[i] - 1; // Get the serial port number for the current sensor
    
    // Set RE and DE pins to driver mode
    digitalWrite(RE_DE_PINS[port][0], HIGH);
    digitalWrite(RE_DE_PINS[port][1], HIGH); 
    delay(1); // Wait for the pins to stabilize

    serialPorts[port]->write(data, length); // Send data to the current sensor
    serialPorts[port]->flush(); // Wait for the data to be sent

<<<<<<< HEAD
  pinMode(DE_1, OUTPUT);
  pinMode(RE_1, OUTPUT);

  pinMode(DE_2, OUTPUT);
  pinMode(RE_2, OUTPUT);
 
  pinMode(DE_6, OUTPUT);
  pinMode(RE_6, OUTPUT);

  pinMode(DE_7, OUTPUT);
  pinMode(RE_7, OUTPUT);
 
  pinMode(DE_8, OUTPUT);
  pinMode(RE_8, OUTPUT);


  // digitalWrite(DE_7, HIGH);  // Enable driver mode
  // digitalWrite(RE_7, HIGH);  // Disable receiver mode (inverted)


  //obtain_serial_number();

  start_data_stream();
  delay(50);
  start_data_stream();
  delay(50);
  start_data_stream();
  delay(50);
  start_data_stream();


  

  // digitalWrite(DE_7, LOW);  // Disable driver mode
  // digitalWrite(RE_7, LOW);  // Enable receiver mode (inverted)

 
}

void loop() 
{
  unsigned long currentTime = millis();

  for (int i = 0; i < NUM_PORTS; i++)
  {

    while (serialPorts[i]->available()) 
    {
      uint8_t byteRead = serialPorts[i]->read();
      buffers[i][bufferIndices[i]++] = byteRead;

      if (bufferIndices[i] >= PACKET_SIZE) 
      {
        
        uint16_t dist_array[2] = {0};
        distance_array[i] = processBuffer(buffers[i], dist_array, 16);
        
        // Process the full packet
        // Serial.print("Port ");
        // Serial.print(i + 1);
        // Serial.print(": ");

        // for (int j = 0; j < PACKET_SIZE; j++) {
        //   Serial.print(buffers[i][j], HEX);
        //   Serial.print(" ");
        // }
        //Serial.println();

        // Reset buffer index
        bufferIndices[i] = 0;
      }
    }

  }


  if (currentTime - lastPrintTime >= PRINT_INTERVAL) 
  {
    lastPrintTime = currentTime;

    Serial.println("All distances: ");
    for (int j = 0; j<8; j++)
    {
      Serial.println(distance_array[j]);
    }
    Serial.println();
  }

  if (millis() - lastCANsendTime > CANsendInterval) 
  {
    lastCANsendTime = millis();
    uint8_t can_msg1[8];
    uint8_t can_msg2[8];

  // First 4 uint16 values into CAN message 1
  for (int i = 0; i < 4; i++) 
  {
    can_msg1[i * 2]     = distance_array[i] & 0xFF;         // Low byte
    can_msg1[i * 2 + 1] = (distance_array[i] >> 8) & 0xFF;  // High byte
  }

    // Send distance data over CAN
    msg.id = CANID1;
    msg.len = 8;    // Send Eight Bytes (4 Data Points)
    for (int i = 0; i < 8; i++) 
    {
        msg.buf[i] = can_msg1[i];
    }
    can.write(msg);

    // Next 4 uint16 values into CAN message 2
    for (int i = 0; i < 4; i++) 
    {
      can_msg2[i * 2]     = distance_array[i + 4] & 0xFF;
      can_msg2[i * 2 + 1] = (distance_array[i + 4] >> 8) & 0xFF;
    }

  delay(1); //(necessary for proper can transmission, try to fix)
    
    // Send distance data over CAN
    msg.id = CANID2;
    msg.len = 8;    // Send Eight Bytes (4 Data Points)
    for (int i = 0; i < 8; i++) 
    {
        msg.buf[i] = can_msg2[i];
    }
    can.write(msg);

  }
    
  

=======
    // Set RE and DE pins to receiver mode
    digitalWrite(RE_DE_PINS[port][0], LOW);
    digitalWrite(RE_DE_PINS[port][1], LOW);
    delay(10); // Small delay to allow for switching modes

  }
>>>>>>> bd94fd2e6917339f4ddfc87c68fb413021436fcf
}


SensorReading extractData(SensorReading reading)
{
  // Extract the 4-bit header and data from the raw data
  for (int i = 0; i < 4; i++) {
    reading.headers |= (reading.rawData[i] >> 4) & 0x0F;
    reading.data |= (reading.rawData[i] & 0x0F) << (i * 4);
  }

  return reading;
}

bool areHeadersValid(SensorReading reading){
  // Check if the headers are valid (i.e. all the same)
  //TODO: HIGH CHANGE THIS DOESN'T WORK BC ITS NOW TAKING THE FIRSTHEADER AS THE LAST BYTE, CHECK!!!
  uint8_t firstHeader = reading.headers & 0x0F; // Extract the first header 
  for (int i = 1; i < 4; i++) {
    if ((reading.headers >> (i * 4) & 0x0F) != firstHeader) {
      return false; // Headers are not valid
    }
  }
  return true; // Headers are valid

}

void sendCANMessages(){
  // Header message
  msg.id = CAN_ID_HEADER;
  msg.len = 2;
  msg.buf[0] = 0xFF; // Send header
  msg.buf[1] = flagByte; // Send flag byte
  can.write(msg); // Send the message

<<<<<<< HEAD
  byte dataToSend[2] = {0x01, 0x87};  // Device Identification

  digitalWrite(DE_1, HIGH);  // Enable driver mode
  digitalWrite(RE_1, HIGH);  // Disable receiver mode (inverted)

  digitalWrite(DE_2, HIGH);  // Enable driver mode
  digitalWrite(RE_2, HIGH);  // Disable receiver mode (inverted)

  digitalWrite(DE_6, HIGH);  // Enable driver mode
  digitalWrite(RE_6, HIGH);  // Disable receiver mode (inverted)

  digitalWrite(DE_7, HIGH);  // Enable driver mode
  digitalWrite(RE_7, HIGH);  // Disable receiver mode (inverted)

  digitalWrite(DE_8, HIGH);  // Enable driver mode
  digitalWrite(RE_8, HIGH);  // Disable receiver mode (inverted)


  Serial1.write(dataToSend, sizeof(dataToSend));  // Send data over RS-422
  Serial1.flush();
  
  digitalWrite(DE_1, LOW);  
  digitalWrite(RE_1, LOW);  

  Serial2.write(dataToSend, sizeof(dataToSend));  // Send data over RS-422
  Serial2.flush();
  
  digitalWrite(DE_2, LOW);  
  digitalWrite(RE_2, LOW);  



  Serial6.write(dataToSend, sizeof(dataToSend));  // Send data over RS-422
  Serial6.flush();

  digitalWrite(DE_6, LOW);  
  digitalWrite(RE_6, LOW);  


  Serial7.write(dataToSend, sizeof(dataToSend));  // Send data over RS-422
  Serial7.flush();

  digitalWrite(DE_7, LOW); 
  digitalWrite(RE_7, LOW);  


  Serial8.write(dataToSend, sizeof(dataToSend));  // Send data over RS-422
  Serial8.flush();

  digitalWrite(DE_8, LOW); 
  digitalWrite(RE_8, LOW);  
}

uint16_t obtain_serial_number()
{
  digitalWrite(DE_8, HIGH);  // Enable driver mode
  digitalWrite(RE_8, HIGH);  // Disable receiver mode (inverted)

  uint16_t serial_number = 0;
  uint8_t received[16] = {0};
  byte dataToSend[2] = {0x01, 0x81};

  Serial.println("Sending request...");
  Serial8.write(dataToSend, sizeof(dataToSend));
  Serial8.flush();

  digitalWrite(DE_8, LOW);  // Disable driver mode
  digitalWrite(RE_8, LOW);  // Enable receiver mode (inverted)



  unsigned long start = millis();
  while (Serial8.available() < 16 && (millis() - start < 5000)) 
  {
    // wait with timeout
=======
  // Data message 1
  msg.id = CAN_ID_DATA1;
  msg.len = NUM_SENSORS > 4? 8 : NUM_SENSORS * 2; // Set length based on number of sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    msg.buf[i * 2] = (sensorReadingsArray[i] >> 8) & 0xFF; // High byte
    msg.buf[i * 2 + 1] = sensorReadingsArray[i] & 0xFF; // Low byte
>>>>>>> bd94fd2e6917339f4ddfc87c68fb413021436fcf
  }
  can.write(msg); // Send the message 

  // Data message 2
  int remainingSensors = NUM_SENSORS - 4;
  if (remainingSensors > 0) {
    msg.id = CAN_ID_DATA2;
    msg.len = remainingSensors * 2; // Set length based on number of sensors
    for (int i = 0; i < remainingSensors; i++) {
      msg.buf[i * 2] = (sensorReadingsArray[i + 4] >> 8) & 0xFF; // High byte
      msg.buf[i * 2 + 1] = sensorReadingsArray[i + 4] & 0xFF; // Low byte
    }
    can.write(msg); // Send the message 
  }

  Serial.println("CAN messages sent");
}

void printingFunction() {
  // Print the sensor readings
  Serial.println("Time: " + String(millis()/3600000) + ":" + String((millis()/60000)%60) + ":" + String((millis()/1000)%60));

  Serial.print("Flag byte: ");
  Serial.println(flagByte, BIN);

  Serial.print("Sensor Readings: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print("( Serial");
    Serial.print(PLUGGED_PORTS[i]);
    Serial.print(" ) :");
    Serial.print(sensorReadingsArray[i]);
    Serial.print(" | ");
  }
  Serial.println();
}