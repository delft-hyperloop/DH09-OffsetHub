#include <Arduino.h>
#include <SoftwareSerial.h>

// put function declarations here:
int myFunction(int, int);
void printByteArrayBinary(uint8_t *arr, size_t length);
uint16_t calculate_distance(void);
void inquire_result(void);
void processBuffer(byte* buffer, uint16_t* dist_array, int size);
#define DE 30
#define RE 31
#define BUFFER_SIZE 32  // Maximum number of bytes to read at a time



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);   // USB serial
  Serial7.begin(115200, SERIAL_8E1); // RS485 Communication to Offset Sensor

  pinMode(DE, OUTPUT);
  pinMode(RE, OUTPUT);

  digitalWrite(DE, HIGH);  // Enable driver mode
  digitalWrite(RE, HIGH);  // Disable receiver mode (inverted)

  
  //byte dataToSend[2] = {0x01, 0x87};  // stream of results (In pairs of 4 Bytes)
  //byte dataToSend[2] = {0x01, 0x86};  // inquiry of results (4 Bytes)

  // byte dataToSend[6] = {0x01, 0x83, 0x84, 0x80, 0x80, 0x83};    //change baud rate to 2400 * 48 = 115200 bit/s
  // Serial7.write(dataToSend, sizeof(dataToSend));  // Send data over RS-422
  // Serial7.flush();
  // Serial7.begin(115200, SERIAL_8E1); // RS485 Communication to Offset Sensor
  // delay(50);

  byte dataToSend3[3] = {0x01, 0x84, 0xAA};  // Save Parameters to Flash
  Serial7.write(dataToSend3, sizeof(dataToSend3));  // Send data over RS-422
  Serial7.flush();
  if (Serial7.available() >0 ) 
  {
    Serial.println("Parameters Saved");
    uint8_t response = Serial7.read();
    Serial.print(response);
  }
  
  // byte dataToSend2[2] = {0x01, 0x87};  // stream of results (In pairs of 4 Bytes)
  // Serial7.write(dataToSend2, sizeof(dataToSend2));  // Send data over RS-422
  // Serial7.flush();

  digitalWrite(DE, LOW);  // Disable driver mode
  digitalWrite(RE, LOW);  // Enable receiver mode (inverted)
}

void loop() 
{

  // uint8_t received[32] = {0};
  // uint16_t distance_mm_array[7] = {0};


  // if (Serial7.available() >= 32) 
  // {  
  //   Serial.println("Received msg");
  //   Serial7.readBytes(received, 32);
  //   processBuffer(received, distance_mm_array, 32);

  //   for (int k = 0; k<7; k++)
  //   {
  //     Serial.print(",");
  //     Serial.println(distance_mm_array[k]);
  //   }
  // } 


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

void processBuffer(byte* buffer, uint16_t* dist_array, int size) {
  
  uint16_t distance_mm = 0;

  byte valid_buffer[32] = {0};
  uint16_t distance_mm_array[7] = {0};
  uint16_t validgroupcount = 0;

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

      for (int j = 0; j < 4; j++) {
        //Serial.println(buffer[i + j], BIN);
        valid_buffer[i + j] = buffer[i + j];
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
  //printByteArrayBinary(valid_buffer, 32);
}