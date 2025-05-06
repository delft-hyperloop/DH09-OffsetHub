#include <Arduino.h>
//#include <SoftwareSerial.h>
#include <mcp_can.h>

// put function declarations here:
int myFunction(int, int);
void printByteArrayBinary(uint8_t *arr, size_t length);
uint16_t calculate_distance(void);
void inquire_result(void);
void start_data_stream(void);
uint16_t processBuffer(byte* buffer, uint16_t* dist_array, int size);
uint16_t obtain_serial_number();
uint16_t processSerial(HardwareSerial& serialPort, const char* label); 

const int NUM_PORTS = 8;
const int PACKET_SIZE = 16;
const unsigned long PRINT_INTERVAL = 50; // milliseconds
unsigned long lastPrintTime = 0;
MCP_CAN CAN0(10);     // Set CS to pin 10

uint16_t distance_array[8] = {0};

// Create array of Serial pointers
HardwareSerial* serialPorts[NUM_PORTS] = {
  &Serial1, &Serial2, &Serial3, &Serial4,
  &Serial5, &Serial6, &Serial7, &Serial8
};

// Create buffers for each port
uint8_t buffers[NUM_PORTS][PACKET_SIZE];
uint8_t bufferIndices[NUM_PORTS] = {0};



#define DE 93
#define RE 90

#define DE_1 2
#define RE_1 3

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

  Serial1.begin(9600, SERIAL_8E1); // RS485 Communication to Offset Sensor
  Serial7.begin(9600, SERIAL_8E1);
  Serial6.begin(9600, SERIAL_8E1); // RS485 Communication to Offset Sensor
  Serial8.begin(9600, SERIAL_8E1); // RS485 Communication to Offset Sensor

  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted

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


  pinMode(DE_1, OUTPUT);
  pinMode(RE_1, OUTPUT);
 
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

  write(const CanMsg& msg)

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
  digitalWrite(DE, HIGH);  // Enable driver mode
  digitalWrite(RE, HIGH);  // Disable receiver mode (inverted)

  
  //byte dataToSend[2] = {0x01, 0x87};  // stream of results (In pairs of 4 Bytes)
  byte dataToSend[2] = {0x01, 0x86};  // inquiry of results (4 Bytes)

  Serial7.write(dataToSend, sizeof(dataToSend));  // Send data over RS-422
  Serial7.flush();

  digitalWrite(DE, LOW);  // Disable driver mode
  digitalWrite(RE, LOW);  // Enable receiver mode (inverted)

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

      // Serial.print("distance mm :");
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

  digitalWrite(DE_1, HIGH);  // Enable driver mode
  digitalWrite(RE_1, HIGH);  // Disable receiver mode (inverted)

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