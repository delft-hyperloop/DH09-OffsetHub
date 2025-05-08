#include <Arduino.h>
#include <FlexCAN_T4.h>

// ↓↓↓↓CONFIGURE THIS↓↓↓↓
const int NUM_SENSORS = 8;                                          // Number of sensors plugged in
const int PLUGGED_PORTS[NUM_SENSORS] = {1, 2, 3, 4, 5, 6, 7, 8};    // Phyiscal ports plugged in (1-8) (e.g. Serial1, Serial2 plugged --> {1,2})

const float sendFrequency = 2000;         // Frequency to send data (in Hz)
const unsigned long printFrequency = 1;   // Frequency to print data (in Hz)

const int CAN_ID_HEADER = 0x33D;          // CAN ID for header message
const int CAN_ID_DATA1 = CAN_ID_HEADER+1; // CAN ID for data1 message
const int CAN_ID_DATA2 = CAN_ID_HEADER+2; // CAN ID for data2 message

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

// put function declarations here:
void setupSerialPorts(); // Function to set up serial ports
void setupREDEPorts(); // Function to set up RE/DE pins
void sendDataToAllPorts(uint8_t* data, size_t length); // Function to send data to all ports
SensorReading extractData(SensorReading reading); // Function to extract data from the sensor
bool areHeadersValid(SensorReading reading); // Function to check if headers are valid
void sendCANMessages(); // Function to send CAN messages
void printingFunction(); // Function to print data

void setup() {
  Serial.begin(115200);   // USB serial
  delay(1000); // Wait for serial to initialize
  Serial.println("Initializing...");

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

    // Set RE and DE pins to receiver mode
    digitalWrite(RE_DE_PINS[port][0], LOW);
    digitalWrite(RE_DE_PINS[port][1], LOW);
    delay(10); // Small delay to allow for switching modes

  }
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

  // Data message 1
  msg.id = CAN_ID_DATA1;
  msg.len = NUM_SENSORS > 4? 8 : NUM_SENSORS * 2; // Set length based on number of sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    msg.buf[i * 2] = (sensorReadingsArray[i] >> 8) & 0xFF; // High byte
    msg.buf[i * 2 + 1] = sensorReadingsArray[i] & 0xFF; // Low byte
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