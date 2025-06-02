#include <Arduino.h>
#include <FlexCAN_T4.h> 

// put function declarations here:
void printByteArrayBinary(uint8_t *arr, size_t length);
uint16_t calculate_distance(void);
void inquire_result(void);
void start_data_stream(void);
uint16_t processBuffer(byte* buffer, uint16_t* dist_array, int size);
uint16_t obtain_serial_number(HardwareSerial* SerialPtr);
uint16_t processSerial(HardwareSerial& serialPort, const char* label); 
void setupREDEPorts();
void EnableDriver();
void EnableReceiver();

const int NUM_PORTS = 8;
const int PACKET_SIZE = 16;
const unsigned long PRINT_INTERVAL = 1000; // milliseconds

unsigned long lastPrintTime = 0;
unsigned long lastCANsendTime = 0;
unsigned long CANsendInterval = 500;    //microseconds

uint16_t distance_array[8] = {0};

// Create array of Serial pointers
HardwareSerial* serialPorts[NUM_PORTS] = {
  &Serial1, &Serial2, &Serial3, &Serial4,
  &Serial5, &Serial6, &Serial7, &Serial8
};

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

// Create buffers for each port
uint8_t buffers[NUM_PORTS][PACKET_SIZE];
uint8_t bufferIndices[NUM_PORTS] = {0};

const uint32_t CANID1 = 0x123;
const uint32_t CANID2 = 0x321;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;
CAN_message_t msg;
long canMsgCounter = 0;

#define DE_1 2
#define RE_1 3

#define DE_2 9
#define RE_2 10

#define DE_3 38
#define RE_3 39

#define DE_4 40
#define RE_4 41

#define DE_5 18
#define RE_5 19

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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);   // USB serial

  Serial1.begin(115200, SERIAL_8E1); // 
  Serial2.begin(115200, SERIAL_8E1); // 
  Serial3.begin(115200, SERIAL_8E1); // 
  Serial4.begin(115200, SERIAL_8E1); // 
  Serial5.begin(115200, SERIAL_8E1); // 

  Serial7.begin(115200, SERIAL_8E1);
  Serial6.begin(115200, SERIAL_8E1); // 
  Serial8.begin(921600, SERIAL_8E1); // Increased Baud rate for sensor delay test, 30 us Exposure Time, 1 us Transmission Time

  Serial.println("Setting up CAN Bus...");
  can.begin();
  can.setBaudRate(1000000);


  // for (int i = 0; i < NUM_PORTS; i++) {
  //   if (i == 6)
  //   {
  //     serialPorts[i]->begin(115200);  // Or whatever baud rate you're using
  //   }
    
  //   else
  //   {
  //     serialPorts[i]->begin(9600);  // Or whatever baud rate you're using
  //   }
    
  // }

  setupREDEPorts();   //  Set RE/DE pins as Output

  // pinMode(DE_1, OUTPUT);
  // pinMode(RE_1, OUTPUT);

  // pinMode(DE_2, OUTPUT);
  // pinMode(RE_2, OUTPUT);

  // pinMode(DE_3, OUTPUT);
  // pinMode(RE_3, OUTPUT);

  // pinMode(DE_4, OUTPUT);
  // pinMode(RE_4, OUTPUT);

  // pinMode(DE_5, OUTPUT);
  // pinMode(RE_5, OUTPUT);
 
  // pinMode(DE_6, OUTPUT);
  // pinMode(RE_6, OUTPUT);

  // pinMode(DE_7, OUTPUT);
  // pinMode(RE_7, OUTPUT);
 
  // pinMode(DE_8, OUTPUT);
  // pinMode(RE_8, OUTPUT);

  for (int i = 0; i < NUM_PORTS; i++) 
  {
    obtain_serial_number(serialPorts[i]);  // Or whatever baud rate you're using
    //obtain_serial_number();
  }
}
void loop() 
{
  long beginningLoop = micros();

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

    // Serial.println("All distances: ");
    // for (int j = 0; j<8; j++)
    // {
    //   Serial.println(distance_array[j]);
    // }

    Serial.println("CAN messages: " + String(canMsgCounter) + " in " + String(PRINT_INTERVAL) + " ms");
    canMsgCounter = 0;
  }

  if (can.read(msg)) 
  {
    Serial.print("Received CAN message with ID: ");
    Serial.println(msg.id, HEX);
    Serial.print("Data: ");
    // printByteArrayBinary(msg.buf, msg.len);
    for (int i = 0; i < msg.len; i++) 
    {
      Serial.print(msg.buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  if (micros() - lastCANsendTime > CANsendInterval) 
  {
    lastCANsendTime = micros();
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
    
    // Send distance data over CAN
    msg.id = CANID2;
    msg.len = 8;    // Send Eight Bytes (4 Data Points)
    for (int i = 0; i < 8; i++) 
    {
        msg.buf[i] = can_msg2[i];
    }
    can.write(msg);
    
    canMsgCounter++;
  }
    
  
  long endLoop = micros();
  long loopDuration = endLoop - beginningLoop;
  if (millis() - lastPrintTime >= PRINT_INTERVAL) {
    Serial.println("Loop Duration: " + String(loopDuration) + " microseconds\n");
  }
}

// put function definitions here:

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

void inquire_result(void)
{
  digitalWrite(DE_1, HIGH);  // Enable driver mode
  digitalWrite(RE_1, HIGH);  // Disable receiver mode (inverted)

  
  //byte dataToSend[2] = {0x01, 0x87};  // stream of results (In pairs of 4 Bytes)
  byte dataToSend[2] = {0x01, 0x86};  // inquiry of results (4 Bytes)

  Serial7.write(dataToSend, sizeof(dataToSend));  // Send data over RS-422
  Serial7.flush();

  digitalWrite(DE_1, LOW);  // Disable driver mode
  digitalWrite(RE_1, LOW);  // Enable receiver mode (inverted)

}

uint16_t calculate_distance(void) 
{
  uint16_t result = 0;
  uint16_t distance_mm = 0;

  

  if (Serial7.available() >= 4) 
  {  
    Serial.println("Received msg");
    uint8_t received[4];

    for (int i = 0; i <= 3; i++)
    {
      received[i] = Serial7.read();
    }

    result |= (received[3] & 0x0F) << 12;
    result |= (received[2] & 0x0F) << 8;
    result |= (received[1] & 0x0F) << 4;
    result |= (received[0] & 0x0F);
    printByteArrayBinary(received, 4);
    Serial.println(result);

    distance_mm = (2500 + (result * 2500) / 0x4000);
  } 
  return distance_mm;
}

uint16_t processBuffer(byte* buffer, uint16_t* dist_array, int size) {
  
  uint16_t distance_mm = 0;
  uint16_t validgroupcount = 0;
  //uint8_t valid_buffer[32] = {0};

  for (int i = 0; i <= size - 4; i++) 
  {

    
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

    if (group_valid) 
    {
      // Process valid group
      // Serial.print("Valid group at index ");
      // Serial.println(i);

      for (int j = 0; j < 4; j++) 
      {
        //Serial.println(buffer[i + j], BIN);
        //valid_buffer[i + j] = buffer[i + j];
        result |= (buffer[i+j] & 0x0F) << 4*j;
      }

      distance_mm = (2500 + (result * 2500) / 0x4000);
      dist_array[validgroupcount] = distance_mm;

      // Serial.print("za mm :");
      // Serial.println(distance_mm);

      i += 3; // skip ahead by 4 total
      validgroupcount++;
    }
  }
  return distance_mm;
  //printByteArrayBinary(valid_buffer, 32);
}

void start_data_stream(void) 
{

  byte dataToSend[2] = {0x01, 0x87};  // Device Identification

  // digitalWrite(DE_1, HIGH);  // Enable driver mode
  // digitalWrite(RE_1, HIGH);  // Disable receiver mode (inverted)

  // digitalWrite(DE_2, HIGH);  // Enable driver mode
  // digitalWrite(RE_2, HIGH);  // Disable receiver mode (inverted)

  // digitalWrite(DE_3, HIGH);  // Enable driver mode
  // digitalWrite(RE_3, HIGH);  // Disable receiver mode (inverted)

  // digitalWrite(DE_4, HIGH);  // Enable driver mode
  // digitalWrite(RE_4, HIGH);  // Disable receiver mode (inverted)

  // digitalWrite(DE_5, HIGH);  // Enable driver mode
  // digitalWrite(RE_5, HIGH);  // Disable receiver mode (inverted)

  // digitalWrite(DE_6, HIGH);  // Enable driver mode
  // digitalWrite(RE_6, HIGH);  // Disable receiver mode (inverted)

  // digitalWrite(DE_7, HIGH);  // Enable driver mode
  // digitalWrite(RE_7, HIGH);  // Disable receiver mode (inverted)

  // digitalWrite(DE_8, HIGH);  // Enable driver mode
  // digitalWrite(RE_8, HIGH);  // Disable receiver mode (inverted)

  EnableDriver();

  Serial1.write(dataToSend, sizeof(dataToSend));  // Send data over RS-422
  Serial1.flush();
  
  digitalWrite(DE_1, LOW);  
  digitalWrite(RE_1, LOW);  

  Serial2.write(dataToSend, sizeof(dataToSend));  // Send data over RS-422
  Serial2.flush();
  
  digitalWrite(DE_2, LOW);  
  digitalWrite(RE_2, LOW);  

  Serial3.write(dataToSend, sizeof(dataToSend));  // Send data over RS-422
  Serial3.flush();
  
  digitalWrite(DE_3, LOW);  
  digitalWrite(RE_3, LOW);  

  Serial4.write(dataToSend, sizeof(dataToSend));  // Send data over RS-422
  Serial4.flush();
  
  digitalWrite(DE_4, LOW);  
  digitalWrite(RE_4, LOW);  

  Serial5.write(dataToSend, sizeof(dataToSend));  // Send data over RS-422
  Serial5.flush();
  
  digitalWrite(DE_5, LOW);  
  digitalWrite(RE_5, LOW);  


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

uint16_t obtain_serial_number(HardwareSerial* SerialPtr)
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
  }

  if (Serial8.available() >= 16) 
  {
    Serial.println("Message Received");
    Serial8.readBytes(received, 16);
    printByteArrayBinary(received, 16);

    for (int i = 8; i > 3; i--) {
      serial_number = (serial_number << 4) | (received[i] & 0x0F);
    }

    Serial.print("The Serial Number is: ");
    Serial.println(serial_number);
  } 
  
  else 
  {
    Serial.println("Timeout: No response received.");
  }

  return serial_number;
}

uint16_t processSerial(HardwareSerial& serialPort, const char* label) 
{
  uint8_t received[32] = {0};
  uint16_t dist_array[7] = {0};

  if (serialPort.available() >= 32) 
  {
    // Serial.print(label);
    // Serial.println(" Result:");

    serialPort.readBytes(received, 32);
    processBuffer(received, dist_array, 32);

    // for (int i = 0; i < 7; i++) 
    // {
    //   Serial.print(dist_array[i]);
    //   Serial.println(",");
    // }
  }
  return dist_array[6];
}

void setupREDEPorts() 
{
  for (int i = 0; i < NUM_PORTS; i++) 
  {
      pinMode(RE_DE_PINS[i][0], OUTPUT); // Set RE pin as output
      pinMode(RE_DE_PINS[i][1], OUTPUT); // Set DE pin as output
  }
}

void EnableDriver() 
{
  for (int i = 0; i < NUM_PORTS; i++) 
  {
      digitalWrite(RE_DE_PINS[i][0], HIGH);   //  Disable Receiver
      digitalWrite(RE_DE_PINS[i][1], HIGH);   //  Enable Driver
  }
}

void EnableReceiver() 
{
  for (int i = 0; i < NUM_PORTS; i++) 
  {
      digitalWrite(RE_DE_PINS[i][0], LOW);   //  Enable Receiver
      digitalWrite(RE_DE_PINS[i][1], LOW);   //  Disable Driver
  }
}