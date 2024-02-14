#include "Arduino.h"

#include "Wire.h"

//TwoWire Wire1 = TwoWire(1);


const char I2C_ADDR = 0x55; //Set to desired i2c-adress
#undef DEBUG    //Define for various debug outputs (#undef to disable) - !!!ENABLING SLOWS DOWN CODE SIGNIFICANTLY!!!

#define Module1 0x12
#define Module2 0x13
#define Module3 0x14

extern uint8_t SCL_2;
extern uint8_t SDA_2;

#include "psu_control.h"

//TwoWire Wire1 = TwoWire(1);

void sendData(float data1, float data2);  //Function to send data back to the master
void sendData(int data1, int data2);  //Overload to accept int as argument
void sendData(char data1, char data2);  //Overload to accept char as argument
void onRequest(); //Code to execute when master requests data from the slave
void onReceive(int len);  //Code to execute when master sends data to the slave

#pragma region sendData

void sendData(float data1 = 0, float data2 = 0){  //Function to send data back to the master
  //Pointer to the float
  uint8_t *bytePointer1 = reinterpret_cast<uint8_t*>(&data1);

  //Iterate throught the adresses of the pointer, to read the bytes of the float, and send them via i2c
  for (uint8_t i = 0; i < sizeof(float); ++i) {
      Wire.write((*bytePointer1));
      bytePointer1++;
  }

  //Pointer to the second float
  uint8_t *bytePointer2 = reinterpret_cast<uint8_t*>(&data2);

  //Iterate throught the adresses of the pointer, to read the bytes of the float, and send them via i2c
  for (uint8_t i = 0; i < sizeof(float); ++i) {
      Wire.write((*bytePointer2));
      bytePointer2++;
  }
}

void sendData(int data1 = 0, int data2 = 0){ //Overload to accept int as argument
  sendData((float)data1, (float)data2);
}

void sendData(char data1 = 0, char data2 = 0){  //Overload to accept char as argument
  sendData((float)data1, (float)data2);
}

#pragma endregion

#ifdef DEBUG
void blink(){
  for (char i = 0; i<10; i++){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
  }
}
#endif

void onRequest(){ //Code to execute when master requests data from the slave
  #ifdef DEBUG
  Serial.println("OnRequest");
  Serial.println(Wire.peek());
  blink();
  #endif
  char module = Wire.read();  //Read from which sensor/module the master wants data
  switch(module){
    case Module1:
      #ifdef DEBUG
        Serial.println("Module 1 called");
      #endif
      //Code to execute when Module1 is being called
      break;
    case Module2:
      #ifdef DEBUG
        Serial.println("Module 2 called");
      #endif
      //Code to execute when Module2 is being called
      break;
    case Module3:
      #ifdef DEBUG
        Serial.println("Module 3 called");
      #endif
      //Code to execute when Module3 is being called
      break;
    default:
      //Code to execute when unkown module is being called
      #ifdef DEBUG
        Serial.println("Unknown module called");
      #endif
      break;
  }
}

void onReceive(int len){
  #ifdef DEBUG
  Serial.println("OnReceive");
  blink();
  #endif
  //Code to execute when master sends data to the slave
  char module = Wire.read();  //Read from which sensor/module the master wants to change
  char data = Wire.read();  //Read the data the master wants to send
  switch(module){
    case Module1:
      #ifdef DEBUG
        Serial.println("Module 1 called");
      #endif
      //Code to execute when Module1 is being called
      break;
    case Module2:
      #ifdef DEBUG
        Serial.println("Module 2 called");
      #endif
      //Code to execute when Module2 is being called
      break;
    case Module3:
      #ifdef DEBUG
        Serial.println("Module 3 called");
      #endif
      //Code to execute when Module3 is being called
      break;
    default:
      //Code to execute when unkown module is being called
      #ifdef DEBUG
        Serial.println("Unknown module called");
      #endif
      break;
  }
}

void setup() {
  // put your setup code here, to run once:
  #ifdef DEBUG
  pinMode(LED_BUILTIN, OUTPUT);
  #endif
  Serial.begin(115200);

  Wire.onReceive(onReceive);  //Function to be called when a master sends data to the slave
  Wire.onRequest(onRequest);  //Function to be called when a master requests data from the slave
  Wire.begin((uint8_t)I2C_ADDR);  //Register this device as a slave on the i2c-bus (on bus 0)
    
    Wire1.begin(SDA_2, SCL_2);
  
  while (psu_mem_addr == 0xFF){
    Serial.println("Scanning for EEPROM.");
    scan_for_device(0x50, 0x57, psu_mem_addr);
  }
  read_eeprom();

  while (psu_mcu_addr == 0xFF){
    Serial.println("Scanning for MCU.");
    scan_for_device(0x58, 0x5F, psu_mcu_addr);
  }
  read_psu_mcu_registers_init();
  setup_ignore_registers();
}

void loop() {
  // put your main code here, to run repeatedly:

}
