#include <Arduino.h>
#include <FlexCAN_T4.h> 

// *********************************************************
// *************** CHANGES ALLOWED FROM HERE ***************
// *********************************************************

// -------- Description of the variables: ----------

// HUB_FRONT: if defined, use front hub settings; if commented out, use back hub settings
// NUM_PORTS: amount of physically connected cables (max 8)
// CANID1 & CANID2: CAN IDs for the front and back hubs, don't change
// portLabels: which sensors are connected to the board, no predefined order (make something that makes sense)
// serialPortIndex: to which ports each sensor is connected (follow the order of portLabels)

// Generally, make 'portLabels' an array with sensible order and then 'serialPortIndex' follows its order
// Of course make sure that the correct sensors are connected to the correct ports
// The can MSG will follow the order of 'serialPortIndex' (first 4 in CANID1, last 4 in CANID2) (if NUM_PORTS < 8, the rest will be 0)

// --------------------------------------------------

// #define HUB_FRONT  // <--- Comment this out for back hub

#ifdef HUB_FRONT
#define NUM_PORTS 6
#define CANID1 0x100
#define CANID2 0x101

const String portLabels[NUM_PORTS] = {"VFL", "VFR", "LFL", "LFR", "LML", "LMR"};
const int serialPortIndex[NUM_PORTS] = {2, 3, 4, 6, 5, 7}; // Serial port indices for each label
#else  // Set to Back if HUB_FRONT is not defined
#define NUM_PORTS 6 
#define CANID1 0x200
#define CANID2 0x201

const String portLabels[NUM_PORTS] = {"VML", "VMR", "VBL", "VBR", "LBL", "LBR"};
const int serialPortIndex[NUM_PORTS] = {8, 4, 5, 2, 3, 6}; // Serial port indices for each label
#endif

// *********************************************************
// *************** CHANGES ZONE OVER ***********************
// *********************************************************




#define PACKET_SIZE 16  // Size of the packet to read from each serial port
#define BUFFER_SIZE 32 // Size of the buffer for each port

HardwareSerial* serialPorts[8] = {
  &Serial1, &Serial2, &Serial3, &Serial4,
  &Serial5, &Serial6, &Serial7, &Serial8
};

// put function declarations here:
void printByteArrayBinary(uint8_t *arr, size_t length);
uint16_t processBuffer(byte* buffer, uint16_t* dist_array, int size);
uint16_t obtain_serial_numbers(HardwareSerial* SerialPtr,  int port);
uint16_t processSerial(HardwareSerial& serialPort, const char* label); 
void setupREDEPorts();
void enableDriver();
void enableReceiver();

unsigned long printInterval = 500; // milliseconds
unsigned long lastPrintTime = 0;
unsigned long lastCANsendTime = 0;
unsigned long CANsendInterval = 500;    //microseconds

uint16_t distanceArray[8] = {0};

int RE_DE_PINS[8][2] = {
  {3, 2},    // RE_1, DE_1
  {10, 9},   // RE_2, DE_2
  {39, 38},  // RE_3, DE_3
  {41, 40},  // RE_4, DE_4
  {19, 18},  // RE_5, DE_5
  {27, 26},  // RE_6, DE_6
  {31, 30},  // RE_7, DE_7
  {33, 32}   // RE_8, DE_8
};
long outOfBoundsCounter[8] = {0};
long msgCounter[8] = {0};

// Create buffers for each port
uint8_t buffers[8][PACKET_SIZE];
uint8_t bufferIndices[8] = {0};

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;
CAN_message_t msg;
long canMsgCounter = 0;

void setup() {
  Serial.begin(9600);   // USB serial

  Serial1.begin(921600, SERIAL_8E1); // RS485 Communication to Offset Sensor
  Serial2.begin(921600, SERIAL_8E1); // RS485 Communication to Offset Sensor
  Serial3.begin(921600, SERIAL_8E1); // RS485 Communication to Offset Sensor
  Serial4.begin(921600, SERIAL_8E1); // RS485 Communication to Offset Sensor
  Serial5.begin(921600, SERIAL_8E1); // RS485 Communication to Offset Sensor
  Serial6.begin(921600, SERIAL_8E1); // RS485 Communication to Offset Sensor
  Serial7.begin(921600, SERIAL_8E1);
  Serial8.begin(921600, SERIAL_8E1); // RS485 Communication to Offset Sensor

  Serial.println("Setting up CAN Bus...");
  can.begin();
  can.setBaudRate(1000000);

  setupREDEPorts();   //  Set RE/DE pins as Output
}

void loop() {
  long beginningLoop = micros();

  for (int i = 0; i < 8; i++) {
    while (serialPorts[i]->available()) {
      uint8_t byteRead = serialPorts[i]->read();
      buffers[i][bufferIndices[i]++] = byteRead;
      
      if (bufferIndices[i] >= PACKET_SIZE) 
      {
        msgCounter[i]++;
        
        uint16_t dist_array[2] = {0};
        distanceArray[i] = processBuffer(buffers[i], dist_array, 16);

        if (distanceArray[i] > 5000 || distanceArray[i] < 2500){
          outOfBoundsCounter[i]++;
        }         

        // Reset buffer index
        bufferIndices[i] = 0;
      }
    }
  }

  if (micros() - lastCANsendTime > CANsendInterval) {
    lastCANsendTime = micros();
    uint8_t can_msg1[8];
    uint8_t can_msg2[8];

    // First 4 uint16 values into CAN message 1
    for (int i = 0; i < 4; i++) {
      can_msg1[i * 2]     = distanceArray[serialPortIndex[i] - 1] & 0xFF;         // Low byte
      can_msg1[i * 2 + 1] = (distanceArray[serialPortIndex[i] - 1] >> 8) & 0xFF;  // High byte

    }

    // Send distance data over CAN
    msg.id = CANID1;
    msg.len = 8;    // Send Eight Bytes (4 Data Points)
    for (int i = 0; i < 8; i++) {
      msg.buf[i] = can_msg1[i];
    }
    can.write(msg);
    
    // Only send second CAN message if there are more than 4 ports
    if (NUM_PORTS > 4) {
      // Next 4 uint16 values into CAN message 2
      for (int i = 0; i < 4; i++) {
        // Fill in 0s for unused ports
        if (i + 4 >= NUM_PORTS) {
          can_msg2[i * 2]     = 0; 
          can_msg2[i * 2 + 1] = 0;
          continue;
        }

        can_msg2[i * 2]     = distanceArray[serialPortIndex[i + 4] - 1] & 0xFF;
        can_msg2[i * 2 + 1] = (distanceArray[serialPortIndex[i + 4] - 1] >> 8) & 0xFF;
      }

      
      // Send distance data over CAN
      msg.id = CANID2;
      msg.len = 8;    // Send Eight Bytes (4 Data Points)
      
      for (int i = 0; i < 8; i++){
        msg.buf[i] = can_msg2[i]; 
      }

        can.write(msg);
    } 
    canMsgCounter++;
  }
    
  long endLoop = micros();
  long loopDuration = endLoop - beginningLoop;

  if (millis() - lastPrintTime >= printInterval) {

    lastPrintTime = millis();
    
    Serial.println("\n\n\n\n\n\n\n\n\n\n======================================================================");
    Serial.println("Distance Array: ");
    for (int j = 0; j < NUM_PORTS; j++) 
    {
      Serial.print("Serial ");
      Serial.print(serialPortIndex[j]);
      Serial.print(": ");
      Serial.print(distanceArray[serialPortIndex[j] - 1]);
      Serial.print(" [");
      Serial.print(portLabels[j]);
      Serial.print("] |");
      Serial.print(" (message count: ");
      Serial.print(msgCounter[serialPortIndex[j] - 1]);
      Serial.print("; out of bounds count: ");
      Serial.print(outOfBoundsCounter[serialPortIndex[j] - 1]);
      Serial.print(")");
      Serial.println();

      msgCounter[serialPortIndex[j] - 1] = 0;
      outOfBoundsCounter[serialPortIndex[j] - 1] = 0;
      }
    Serial.println("---------------------------------------------------------------------");
    Serial.println("CAN messages: " + String(canMsgCounter) + " in " + String(printInterval) + " ms");
    canMsgCounter = 0;
    Serial.println("Loop Duration: " + String(loopDuration) + " microseconds\n");
    Serial.println("======================================================================");
    Serial.print("Raw readings: [ ");
    for (int i = 0; i < 8; i++) {
      Serial.print(distanceArray[i]);
      Serial.print(" ");
    }
    Serial.println("]");
  }
}

void printByteArrayBinary(uint8_t *arr, size_t length) {
  Serial.print("{ ");
  for (size_t i = 0; i < length; i++) {
      for (int bit = 7; bit >= 0; bit--) {
          Serial.print((arr[i] >> bit) & 1);  // Extract and print each bit
      }
      if (i < length - 1) Serial.print(", ");  // Add comma between bytes
  }
  Serial.println(" }");
}

uint16_t processBuffer(byte* buffer, uint16_t* dist_array, int size) {
  
  uint16_t distance_mm = 0;
  uint16_t validgroupcount = 0;

  for (int i = 0; i <= size - 4; i++) {
    uint16_t result = 0;
    byte ref_bits = (buffer[i] >> 4) & 0b11; // extract bits 5 and 4

    // Check next 3 bytes
    bool group_valid = true;
    for (int j = 1; j < 4; j++) {
      byte next_bits = (buffer[i + j] >> 4) & 0b11;
      if (next_bits != ref_bits) {
        group_valid = false;
        break;
      }
    }

    if (group_valid) {
      // Process valid group
      for (int j = 0; j < 4; j++) {
        result |= (buffer[i+j] & 0x0F) << 4*j;
      }

      distance_mm = (2500 + (result * 2500) / 0x4000);
      dist_array[validgroupcount] = distance_mm;

      i += 3; // skip ahead by 4 total
      validgroupcount++;
    }
  }
  return distance_mm;
}

void start_data_stream(void) {

  byte dataToSend[2] = {0x01, 0x87};  // Device Identification

  enableDriver();

  Serial1.write(dataToSend, sizeof(dataToSend));  // Send data over RS-422
  Serial1.flush();

  digitalWrite(RE_DE_PINS[0][0], LOW);  // Disable Receiver
  digitalWrite(RE_DE_PINS[0][1], LOW);  // Disable Driver
  
  Serial2.write(dataToSend, sizeof(dataToSend));  // Send data over RS-422
  Serial2.flush();
  
  digitalWrite(RE_DE_PINS[1][0], LOW);  // Disable Receiver
  digitalWrite(RE_DE_PINS[1][1], LOW);  // Disable Driver

  Serial3.write(dataToSend, sizeof(dataToSend));  // Send data over RS-422
  Serial3.flush();
  
  digitalWrite(RE_DE_PINS[2][0], LOW);  // Disable Receiver
  digitalWrite(RE_DE_PINS[2][1], LOW);  // Disable Driver

  Serial4.write(dataToSend, sizeof(dataToSend));  // Send data over RS-422
  Serial4.flush();
  
  digitalWrite(RE_DE_PINS[3][0], LOW);  // Disable Receiver
  digitalWrite(RE_DE_PINS[3][1], LOW);  // Disable Driver 

  Serial5.write(dataToSend, sizeof(dataToSend));  // Send data over RS-422
  Serial5.flush();
  
  digitalWrite(RE_DE_PINS[4][0], LOW);  // Disable Receiver
  digitalWrite(RE_DE_PINS[4][1], LOW);  // Disable Driver

  Serial6.write(dataToSend, sizeof(dataToSend));  // Send data over RS-422
  Serial6.flush();

  digitalWrite(RE_DE_PINS[5][0], LOW);  // Disable Receiver
  digitalWrite(RE_DE_PINS[5][1], LOW);  // Disable Driver

  Serial7.write(dataToSend, sizeof(dataToSend));  // Send data over RS-422
  Serial7.flush();

  digitalWrite(RE_DE_PINS[6][0], LOW);  // Disable Receiver
  digitalWrite(RE_DE_PINS[6][1], LOW);  // Disable Driver

  Serial8.write(dataToSend, sizeof(dataToSend));  // Send data over RS-422
  Serial8.flush();

  digitalWrite(RE_DE_PINS[7][0], LOW);  // Disable Receiver
  digitalWrite(RE_DE_PINS[7][1], LOW);  // Disable Driver
}

uint16_t obtain_serial_numbers(HardwareSerial* SerialPtr, int port)
{
  uint16_t serial_number = 0;
  uint8_t received[16] = {0};

  byte dataToSend[2] = {0x01, 0x81};

  //Serial.println("Sending request...");


  unsigned long start = millis();
  while (SerialPtr -> available() < 16 && (millis() - start < 100)) 
  {
    // wait with timeoutl
  }

  if (SerialPtr -> available() >= 16) 
  {
   // Serial.println("Message Received");
    SerialPtr -> readBytes(received, 16);
    //printByteArrayBinary(received, 16);

    for (int i = 7; i > 3; i--) {
      serial_number = (serial_number << 4) | (received[i] & 0x0F);
      //Serial.println(received[i] & 0x0F, BIN);
    }

    //Serial.print("The Serial Number is: ");
    //Serial.println(serial_number);
  } 
  
  else 
  {
    Serial.println("Timeout: No response received.");
  }

  return serial_number;
}

void setupREDEPorts() 
{
  for (int i = 0; i < 8; i++) 
  {
      pinMode(RE_DE_PINS[i][0], OUTPUT); // Set RE pin as output
      pinMode(RE_DE_PINS[i][1], OUTPUT); // Set DE pin as output
  }
}

void enableDriver() 
{
  for (int i = 0; i < 8; i++) 
  {
      digitalWrite(RE_DE_PINS[i][1], HIGH);   //  Disable Receiver
      digitalWrite(RE_DE_PINS[i][0], HIGH);   //  Enable Driver
  }
}

void enableReceiver() 
{
  for (int i = 0; i < 8; i++) 
  {
      digitalWrite(RE_DE_PINS[i][0], LOW);   //  Enable Receiver
      digitalWrite(RE_DE_PINS[i][1], LOW);   //  Disable Driver
  }
}