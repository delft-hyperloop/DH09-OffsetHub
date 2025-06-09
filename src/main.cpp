#include <Arduino.h>
#include <FlexCAN_T4.h> 

// put function declarations here:
void printByteArrayBinary(uint8_t *arr, size_t length);
uint16_t calculate_distance(void);
void start_data_stream(void);
void stop_data_stream(void);
uint16_t processBuffer(byte* buffer, uint16_t* dist_array, int size);
uint16_t obtain_serial_numbers(HardwareSerial* SerialPtr,  int port);
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
uint16_t Serial_array[8] = {0};

// Create array of Serial pointers
HardwareSerial* serialPorts[NUM_PORTS] = {
  &Serial1, &Serial2, &Serial3, &Serial4,
  &Serial5, &Serial6, &Serial7, &Serial8
};

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

const uint16_t pod_serial_numbers[8] = {
34487, 37867, 37866, 37868, //    Serial 1 ... Serial 4
34488, 37871, 34514,20639   //    Serial 5 ... Serial 8
};

const uint16_t test_serial_numbers[8] = {
20640, 31978, 34485, 34486, //    Serial 1 ... Serial 4
34489, 0, 0, 0   //    Serial 5 ... Serial 8
};


// Create buffers for each port
uint8_t buffers[NUM_PORTS][PACKET_SIZE];
uint8_t bufferIndices[NUM_PORTS] = {0};

const uint32_t CANID1 = 0x123;
const uint32_t CANID2 = 0x321;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;
CAN_message_t msg;
long canMsgCounter = 0;


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

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(921600);   // USB serial

  //Serial.println("Setting up CAN Bus...");
  can.begin();
  can.setBaudRate(1000000);

  for (int i = 0; i < NUM_PORTS; i++) 
  {
    if (i == 0)
    {
      serialPorts[i]->begin(115200, SERIAL_8E1);  // Or whatever baud rate you're using
    }
    else
    {
      serialPorts[i]->begin(921600, SERIAL_8E1);  // Or whatever baud rate you're using
    }
}

  setupREDEPorts();   //  Set RE/DE pins as Output
 // delay(50);
  stop_data_stream();
  //delay(5); 

  //EnableReceiver();
  //EnableDriver();
  for (int i = 0; i < 8; i++) 
  {
    Serial.print("Serial port ");
    Serial.print(i);
    Serial.print(":");
    Serial_array[i] = obtain_serial_numbers(serialPorts[i], i);
    Serial.println(Serial_array[i]);
  }



  start_data_stream();


  
  //obtain_serial_number();
}
void loop() 
{

  long beginningLoop = micros();

  unsigned long currentTime = millis();

  for (int i = 0; i < NUM_PORTS; i++)
  {
    uint16_t current_serial_number = Serial_array[i];

    while (serialPorts[i]->available()) 
    {
      uint8_t byteRead = serialPorts[i]->read();
      buffers[i][bufferIndices[i]++] = byteRead;

      if (bufferIndices[i] >= PACKET_SIZE) 
      {
        
        uint16_t dist_array[2] = {0};

        switch(current_serial_number)
        {
          case 20640:
              distance_array[0] = processBuffer(buffers[i], dist_array, 16);
              break;
          case 31978:
              distance_array[1] = processBuffer(buffers[i], dist_array, 16);
              break;
          case 34485:
              distance_array[2] = processBuffer(buffers[i], dist_array, 16);
              break;
          case 34486:
              distance_array[3] = processBuffer(buffers[i], dist_array, 16);
              break;
          case 34489:
              distance_array[4] = processBuffer(buffers[i], dist_array, 16);
              break;
          case 0:
              distance_array[5] = 0;
              distance_array[6] = 0;
              distance_array[7] = 0;
              break;
          default:
            for (int i = 0; i < 8; i++) 
              {
                Serial.print("Serial port ");
                Serial.print(i);
                Serial.print(":");
                Serial_array[i] = obtain_serial_numbers(serialPorts[i], i);
                Serial.println(Serial_array[i]);
              }
              break;
        }
        //distance_array[i] = processBuffer(buffers[i], dist_array, 16);
        // Serial.print("The serial number read out from port :");
        // Serial.print(i);
        // // Serial.print("Equals: ");
        // Serial.println(Serial_array[i]);
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

    //Serial.println("CAN messages: " + String(canMsgCounter) + " in " + String(PRINT_INTERVAL) + " ms");
    canMsgCounter = 0;
  }

  // if (can.read(msg)) 
  // {
  //   Serial.print("Received CAN message with ID: ");
  //   Serial.println(msg.id, HEX);
  //   Serial.print("Data: ");
  //   // printByteArrayBinary(msg.buf, msg.len);
  //   for (int i = 0; i < msg.len; i++) 
  //   {
  //     Serial.print(msg.buf[i], HEX);
  //     Serial.print(" ");
  //   }
  //   Serial.println();
  // }
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
  byte dataToSend[2] = {0x01, 0x87};  

  EnableDriver();
  
  for (int i = 0; i < NUM_PORTS; i++)
  {
    serialPorts[i] -> write(dataToSend, sizeof(dataToSend));
    serialPorts[i] -> flush();

    digitalWrite(RE_DE_PINS[i][1], LOW);   //  Enable Receiver
    digitalWrite(RE_DE_PINS[i][0], LOW);   //  Disable Driver
  }

  Serial.println("Requested Data Stream.");
}

void stop_data_stream(void) 
{
  byte dataToSend[2] = {0x01, 0x88};   //  Stop Data Stream Command

  EnableDriver();
  delay(50);

  for (int i = 0; i < NUM_PORTS; i++)
  {
    serialPorts[i] -> write(dataToSend, sizeof(dataToSend));
    serialPorts[i] -> flush();

    digitalWrite(RE_DE_PINS[i][0], LOW);   //  Enable Receiver
    digitalWrite(RE_DE_PINS[i][1], LOW);   //  Disable Driver
  }


  Serial.println("Data Stream Stopped.");

}

uint16_t obtain_serial_numbers(HardwareSerial* SerialPtr, int port)
{
  digitalWrite(RE_DE_PINS[port][0], HIGH);   //  Enable Driver
  digitalWrite(RE_DE_PINS[port][1], HIGH);   //  Disable Receiver

  uint16_t serial_number = 0;
  uint8_t received[16] = {0};

  byte dataToSend[2] = {0x01, 0x81};

  //Serial.println("Sending request...");

  SerialPtr -> write(dataToSend, sizeof(dataToSend));
  SerialPtr -> flush();

  digitalWrite(RE_DE_PINS[port][0], LOW);   //  Enable Receiver
  digitalWrite(RE_DE_PINS[port][1], LOW);   //  Disable Driver

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
      digitalWrite(RE_DE_PINS[i][1], HIGH);   //  Disable Receiver
      digitalWrite(RE_DE_PINS[i][0], HIGH);   //  Enable Driver
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