#include <Arduino.h>
#include <stdint.h>
#include <cstdint>
#include "psu_control.h"

#include "Wire.h"
//extern TwoWire Wire1;

void read_all_psu_registers();
void write_psu_mcu_u16(uint8_t, uint16_t);

bool ignore_registers[0xFF+1];
uint8_t inspect_register;

uint8_t test_cmd  = 0x03;
uint16_t test_data = 0x0000;

uint8_t psu_mcu_addr = 0xFF;
uint8_t psu_mem_addr = 0xFF;

extern uint8_t SCL_2;
extern uint8_t SDA_2;

#ifdef pico
    uint8_t SCL_2 = 11;
    uint8_t SDA_2 = 10;
    uint8_t LED_BUILTIN = 25;
#endif

#ifdef esp32dev
    uint8_t SCL_2 = 32;
    uint8_t SDA_2 = 33;
    uint8_t LED_BUILTIN = 2;
#endif

void setup_ignore_registers(){



  ignore_registers[0x00] = false; 
  ignore_registers[0x02] = false; // [?D] Flags. Power Good?
  ignore_registers[0x04] = false; // [?T] Flags? 0x00 0x00 -> 0x00 0x16 when pulling power
  ignore_registers[0x06] = false; // [?T] Under voltage alarm flag? b.3 = pot3 overflow?
  ignore_registers[0x08] = true;  // [!T] grid voltage
  ignore_registers[0x0A] = true;  // [!T] grid amperage
  ignore_registers[0x0C] = true;  // [!T] grid wattage
  ignore_registers[0x0E] = true;  // [!T] out voltage

  ignore_registers[0x10] = true;  // [!T] out amperage
  ignore_registers[0x12] = true;  // [!T] out wattage
  ignore_registers[0x14] = false;
  ignore_registers[0x16] = false;
  ignore_registers[0x18] = false;
  ignore_registers[0x1A] = true;  // [!T] intake temp
  ignore_registers[0x1C] = true;  // [!T] internal temp
  ignore_registers[0x1E] = true;  // [!T] fan speed actual


  ignore_registers[0x20] = false;
  ignore_registers[0x22] = false;
  ignore_registers[0x24] = false;
  ignore_registers[0x26] = false;
  ignore_registers[0x28] = false;
  ignore_registers[0x2A] = false;  
  ignore_registers[0x2C] = true;  // [!T] Ws in 32bits
  ignore_registers[0x2E] = false;  // [?T] Very slow counter. Resets when unplugging

  
  ignore_registers[0x30] = true;  // [!T] Uptime s
  ignore_registers[0x32] = true;  // [?D] Peak watts in 
  ignore_registers[0x34] = true;  // [?D] Min amps in
  ignore_registers[0x36] = true;  // [?D] Peak amps out
  ignore_registers[0x38] = false; 
  ignore_registers[0x3A] = false; // [?D] Cool flags1 

  ignore_registers[0x3C] = false; // [?D] Cool flags2
  ignore_registers[0x48] = false; // [?A]

  ignore_registers[0x4A] = false; // [?A]
  

  ignore_registers[0x4C] = false; // [?A]
  ignore_registers[0x4E] = false; // [?A]
  ignore_registers[0x50] = false; // [?D] Maybe under voltage thresh
  ignore_registers[0x52] = false; // [?D] Maybe over voltage thresh
  ignore_registers[0x54] = false; // [?] 
  ignore_registers[0x56] = false; // [?] 

  ignore_registers[0x58] = false; // [?] 
  ignore_registers[0x5A] = false; // [?] 
  ignore_registers[0x5C] = false; // [?] 
  ignore_registers[0x5E] = false; // [?] 

  ignore_registers[0x60] = false; // [?] 
  ignore_registers[0x62] = false; // [?] 
  ignore_registers[0x64] = false; // [?] 
  ignore_registers[0x66] = false; // [?] 
  ignore_registers[0x68] = false; // [?] 
  ignore_registers[0x6A] = false; // [?] 
  ignore_registers[0x6C] = false; // [?] 
  ignore_registers[0x6E] = false; // [?] 

  ignore_registers[0x70] = false; // [?] 
  ignore_registers[0x72] = false; // [?] 
  ignore_registers[0x74] = false; // [?] 
  ignore_registers[0x76] = false; // [?] 
  ignore_registers[0x78] = false; // [?] 
  ignore_registers[0x7A] = false; // [?] 
  ignore_registers[0x7C] = false; // [?] 
  ignore_registers[0x7E] = false; // [?] 

  ignore_registers[0x80] = false; // [?] 
  ignore_registers[0x82] = false; // [?] 
  ignore_registers[0x84] = false; // [?] 
  ignore_registers[0x86] = false; // [?] 
  ignore_registers[0x88] = false; // [?] 
  ignore_registers[0x8A] = false; // [?] 
  ignore_registers[0x8C] = false; // [?] 
  ignore_registers[0x8E] = false; // [?] 

  ignore_registers[0x90] = false; // [?] 
  ignore_registers[0x92] = false; // [?] 
  ignore_registers[0x94] = false; // [?] 
  ignore_registers[0x96] = false; // [?] 
  ignore_registers[0x98] = false; // [?] 
  ignore_registers[0x9A] = false; // [?] 
  ignore_registers[0x9C] = false; // [?] 
  ignore_registers[0x9E] = false; // [?] 

  ignore_registers[0xA0] = false; // [?] 
  ignore_registers[0xA2] = false; // [?] 
  ignore_registers[0xA4] = false; // [?] 
  ignore_registers[0xA6] = false; // [?] 
  ignore_registers[0xA8] = false; // [?] 
  ignore_registers[0xAA] = false; // [?] 
  ignore_registers[0xAC] = false; // [?] 
  ignore_registers[0xAE] = false; // [?] 

  ignore_registers[0xB0] = false; // [?] 
  ignore_registers[0xB2] = false; // [?] 
  ignore_registers[0xB4] = false; // [?] 
  ignore_registers[0xB6] = false; // [?] 
  ignore_registers[0xB8] = false; // [?] 
  ignore_registers[0xBA] = false; // [?] 
  ignore_registers[0xBC] = false; // [?] 
  ignore_registers[0xBE] = false; // [?] 

  ignore_registers[0xC0] = false; // [?] 
  ignore_registers[0xC2] = false; // [?] 
  ignore_registers[0xC4] = false; // [?] 
  ignore_registers[0xC6] = false; // [?] 
  ignore_registers[0xC8] = false; // [?] 
  ignore_registers[0xCA] = false; // [?] 
  ignore_registers[0xCC] = false; // [?] 
  ignore_registers[0xCE] = false; // [?] 

  ignore_registers[0xD0] = false; // [?] 
  ignore_registers[0xD2] = false; // [?] 
  ignore_registers[0xD4] = false; // [?] 
  ignore_registers[0xD6] = false; // [?] 
  ignore_registers[0xD8] = false; // [?] 
  ignore_registers[0xDA] = false; // [?] 
  ignore_registers[0xDC] = false; // [?] 
  ignore_registers[0xDE] = false; // [?] 

  ignore_registers[0xE0] = false; // [?] 
  ignore_registers[0xE2] = false; // [?] 
  ignore_registers[0xE4] = false; // [?] 
  ignore_registers[0xE6] = false; // [?] 
  ignore_registers[0xE8] = false; // [?] 
  ignore_registers[0xEA] = false; // [?] 
  ignore_registers[0xEC] = false; // [?] 
  ignore_registers[0xEE] = false; // [?] 

  ignore_registers[0xF0] = false; // [?] 
  ignore_registers[0xF2] = false; // [?] 
  ignore_registers[0xF4] = false; // [?] 
  ignore_registers[0xF6] = false; // [?] 
  ignore_registers[0xF8] = false; // [?] 
  ignore_registers[0xFA] = false; // [?] 
  ignore_registers[0xFC] = false; // [?] 
  ignore_registers[0xFE] = false; // [?] 


}




uint8_t factory_eeprom[] = {
  0xFE,0x00,0x00,0x01,0x05,0x0E,0x00,0xEB,
  0x01,0x04,0x19,0x08,0x06,0x6C,0xC0,0xC0,
  0xC0,0xCA,0x34,0x34,0x31,0x38,0x33,0x30,
  0x2D,0x30,0x30,0x31,0xC8,0x31,0x30,0x2F,
  0x31,0x30,0x2F,0x30,0x38,0xC1,0x00,0x5B,
  0x01,0x09,0x19,0xC5,0x44,0x45,0x4C,0x54,
  0x41,0xDA,0x48,0x50,0x20,0x50,0x52,0x4F,
  0x4C,0x49,0x41,0x4E,0x54,0x20,0x53,0x45,
  0x52,0x56,0x45,0x52,0x20,0x50,0x53,0x20,
  0x20,0x20,0x20,0x20,0xCA,0x34,0x33,0x37,
  0x35,0x37,0x32,0x2D,0x42,0x32,0x31,0xC2,
  0x30,0x31,0xCE,0x35,0x41,0x4D,0x4A,0x51,
  0x30,0x44,0x34,0x44,0x58,0x4F,0x33,0x4C,
  0x55,0x00,0x00,0xC1,0x00,0x00,0x00,0x0A,
  0x00,0x02,0x18,0x12,0xD4,0xB0,0x04,0xA0,
  0x05,0x1E,0x05,0x28,0x23,0x90,0x33,0x50,
  0x46,0x20,0x67,0x2F,0x3F,0x0A,0x1A,0xA0,
  0x15,0x00,0x00,0x00,0x00,0x01,0x02,0x0D,
  0xB2,0x3E,0x01,0xCE,0x04,0x74,0x04,0xEC,
  0x04,0x78,0x00,0x64,0x00,0x10,0x27,0x01,
  0x02,0x0D,0xEF,0x01,0x82,0xB0,0x04,0x38,
  0x04,0x28,0x05,0x78,0x00,0x00,0x00,0xFA,
  0x00,0xD0,0x82,0x12,0xE3,0xB9,0x0B,0x00,
  0x00,0x03,0x20,0x03,0xC0,0x13,0x01,0x80,
  0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x48,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

char psu_spn[11];
char psu_date[9];
char psu_name[27];
char psu_ct[15];



 double psu_grid_voltage = 0;
 double psu_grid_amperage = 0;
 double psu_grid_wattage = 0;
 double psu_out_voltage = 0;
 double psu_out_amperage = 0;
 double psu_out_wattage = 0;
 double psu_intake_temp = 0;
 double psu_internal_temp = 0;
 double psu_fan_speed_actual = 0;
 double psu_fan_speed_desired = 0;



/****************************************************** U T I L I T Y */

void print_byte(uint8_t b){
  Serial.print("0x");
  if (b < 16)
    Serial.print("0");
  Serial.print(b, HEX);
}

    
void print_bits(uint8_t b){    
  for(uint8_t mask = 0x80; mask; mask >>= 1){
   if (mask & b)
       Serial.print('1');
   else
       Serial.print('0');
  }
}


/****************************************************** i 2 C - f u n c t i o n s */


void scan_for_device(uint8_t from, uint8_t to, uint8_t& ret) {
  byte address;
  
  
  for (address = from ; address <= to; address++ )  {
/*    
#ifdef SSI2CM
    bool ack = i2c_start(i<<1 | I2C_WRITE); 
#else
*/
    Wire1.beginTransmission(address);
    uint8_t ack = Wire1.endTransmission();
//#endif
    if (ack == 0){
      Serial.print("Device found at ");
      print_byte(address);
      Serial.println();
      ret = address;
      return;
    } 
  }

  if (ret == 0xFF) {
    Serial.print("No device found between ");
    print_byte(from);
    Serial.print(" and ");
    print_byte(to);
    Serial.println(".");
  }
  
}


/****************************************************** P S U  E E P R O M */


uint8_t read_eeprom_byte(long addr)
{
  Wire1.beginTransmission(psu_mem_addr);

  Wire1.write((uint8_t)(addr));
  Wire1.endTransmission();

  Wire1.requestFrom((uint8_t)psu_mem_addr, (uint8_t)1);

  uint8_t rdata = 0xFF;
  if (Wire1.available()) rdata = Wire1.read();
  return rdata;
}

void write_eeprom_byte(uint8_t addr, uint8_t b){
  Wire1.beginTransmission(psu_mem_addr);
  Wire1.write((uint8_t)(addr));
  Wire1.write((uint8_t)(b));
  Wire1.endTransmission();
}

void read_eeprom_string(uint8_t addr, uint8_t len, char* s)
{
  Wire1.beginTransmission(psu_mem_addr);

  Wire1.write((uint8_t)(addr));
  Wire1.endTransmission();

  Wire1.requestFrom((uint8_t)psu_mem_addr, (uint8_t)len);


  uint8_t i=0;
  while (Wire1.available() && (i <= len)){
    s[i] = Wire1.read();
    i++;
  }
  s[i] = '\0';
}
//#endif
void read_eeprom(){

  read_eeprom_string(0x12, 10, psu_spn);
  read_eeprom_string(0x1D, 8, psu_date);
  read_eeprom_string(0x32, 26, psu_name);
  read_eeprom_string(0x5B, 14, psu_ct);
  
  Serial.print("Found: ");
  Serial.println(psu_name);

}

void read_entire_eeprom(){
  for (uint16_t a = 0 ; a<0xFF; a+=8){
    print_byte(a);
    Serial.print(": ");
    for (uint8_t b = 0 ; b<8; b++){
      uint8_t bte = read_eeprom_byte((uint8_t)(a+b));
      print_byte(bte);
      Serial.print(" ");
    }
    for (uint8_t b = 0 ; b<8; b++){
      uint8_t bte = read_eeprom_byte((uint8_t)(a+b));
      if ((bte >= 0x20) && (bte <= 0x7E)){ //printable chars
        Serial.print((char)bte);
      } else {
        Serial.print('.');
      }
    }
    Serial.println();
  }

}

void factory_reset_eeprom(){
  Serial.println("Factory reset of eeprom");
  delay(3000);
  for (uint16_t a = 0 ; a<0xFF; a++){
    write_eeprom_byte(a, factory_eeprom[a]);
  }
  Serial.println("done");
  read_entire_eeprom();
}



/****************************************************** P S U  M C U */


bool checksum(uint8_t* msg, uint8_t msg_len){

  uint8_t cs = 0;
  for (uint8_t i=0; i<msg_len; ++i){
    //print_byte(msg[i]); 
    //Telnet.print(" ");
    cs += msg[i];
  }
  cs = (( 0xFF - cs) + 1) & 0xFF;
  //Telnet.print("  cs: ");
  //print_byte(cs);
  //Telnet.println();
  if (cs) { 
    Serial.print("Wrong checksum: ");
    Serial.println(cs);
  }
  // cs == 0 > A-OK
  // (cs == 0) == true > A-OK
  //
  return (cs==0);

}
/*
#ifdef SSI2CM

#else
*/
bool read_psu_mcu(uint8_t reg, uint8_t len, uint8_t* msg, bool verbose = true){
    digitalWrite(LED_BUILTIN, HIGH);
    //reg = reg << 1;
    uint8_t cs = reg + (psu_mcu_addr<<1);
    uint8_t reg_cs = ((0xff-cs)+1) & 0xff;
    Wire1.beginTransmission(psu_mcu_addr);
    if (verbose) {
      Serial.print("[>");
      print_byte(psu_mcu_addr);
      Serial.print("] ");
    }
    Wire1.write((uint8_t)(reg));
    if (verbose) {
      print_byte(reg);
      Serial.print(": ");
    }
    Wire1.write((uint8_t)(reg_cs));
    if (verbose) {
      print_byte(reg_cs);
      Serial.print(" ");
    }
    Wire1.endTransmission();
    if (verbose) {
      Serial.println("[S]");
    }
    delay(1);

    Wire1.requestFrom((uint8_t)psu_mcu_addr, (uint8_t)len, (uint8_t)false);
    if (verbose) {
      Serial.print("[<");
      print_byte(psu_mcu_addr);
      Serial.print("] ");
      print_byte(reg);
      Serial.print(": ");
    }
    uint8_t i = 0;
    //uint8_t msg[len];
    while (Wire1.available() && (i < len)){ 
      msg[i] = Wire1.read(); 
      if (verbose) {
        print_byte((uint8_t)msg[i]);
        Serial.print(" ");
      }
      i++;
    }
    if (verbose) {
      Serial.println("");
    }
    if (i!=len){
      Serial.print("Expected to read ");
      Serial.print(len);
      Serial.print(" bytes, but got ");
      Serial.print(i);
      Serial.println(" bytes.");
    }
    digitalWrite(LED_BUILTIN, LOW);
    return checksum(msg, len);

}
/*
#endif
*/
bool read_psu_mcu_u8(uint8_t reg, uint8_t& ret, bool verbose=true){
  uint8_t msg[2];
  if (!read_psu_mcu(reg, 2, msg, verbose)){
    return false;
  }
  ret = msg[0];
  return true;
}

bool read_psu_mcu_u16(uint8_t reg, uint16_t& ret, bool verbose=true){
  uint8_t msg[3];
  if (!read_psu_mcu(reg, 3, msg, verbose)){
    return false;
  }
  ret = (msg[1] << 8) + msg[0];
  return true;
}


bool read_psu_mcu_f16(uint8_t reg, double scale, double& ret, bool verbose=true){
  uint16_t u16;
  if (!read_psu_mcu_u16(reg, u16, verbose)){
    return false;
  }
  ret = scale * u16;
  return true;
}

bool read_psu_mcu_flags16(uint8_t reg, uint16_t& ret, bool verbose=true){
  if (!read_psu_mcu_u16(reg, ret, verbose)){
    return false;
  }
  return true;
}



bool read_psu_grid_voltage(double& ret, bool verbose = true){
  return (read_psu_mcu_f16(0x08, 0.032, ret, verbose));
}

bool read_psu_grid_amperage(double& ret, bool verbose = true){
  return (read_psu_mcu_f16(0x0A, 1/128., ret, verbose));
}

bool read_psu_grid_wattage(double& ret, bool verbose = true){
  return (read_psu_mcu_f16(0x0C, 2., ret, verbose));
}

bool read_psu_out_voltage(double& ret, bool verbose = true){
  return (read_psu_mcu_f16(0x0E, 1./256, ret, verbose));
}

bool read_psu_out_amperage(double& ret, bool verbose = true){
  return (read_psu_mcu_f16(0x10, 1./128, ret, verbose));
}

bool read_psu_out_wattage(double& ret, bool verbose = true){
  return (read_psu_mcu_f16(0x1A, 2., ret, verbose));
}

bool read_psu_intake_temp(double& ret, bool verbose = true){
  bool r = (read_psu_mcu_f16(0x18, 1./32., ret, verbose));
  //ret = (ret - 32.) * 5./9.;
  return r;
}

bool read_psu_internal_temp(double& ret, bool verbose = true){
  bool r = (read_psu_mcu_f16(0x1C, 1./32., ret, verbose));
  //ret = (ret - 32.) * 5./9.;
  return r;
}

bool read_psu_fan_speed_actual(double& ret, bool verbose = true){
  return (read_psu_mcu_f16(0x1E, 1., ret, verbose));
}

bool read_psu_fan_speed_desired(double& ret, bool verbose = true){
  return (read_psu_mcu_f16(0x40, 1., ret, verbose));
}

bool read_psu_cool_flags_1(uint16_t& ret, bool verbose = true){
  return (read_psu_mcu_u16(0x3A, ret, verbose));
}

bool read_psu_cool_flags_2(uint16_t& ret, bool verbose = true){
  return (read_psu_mcu_u16(0x3A, ret, verbose));
}


bool read_psu_out_max_amperage(double& ret, bool verbose = true){
  return (read_psu_mcu_f16(0x36, 1./128, ret, verbose));
}


bool read_psu_is_enabled(bool& ret, bool verbose = true){
  uint16_t u16;
  bool r = read_psu_mcu_flags16(0x02, u16, verbose);
  ret = ((u16 & 0x05)  == 0x05);
  return r;
}

bool read_psu_data(bool verbose = true){
  return ((read_psu_grid_voltage(psu_grid_voltage, verbose)) &&
  (read_psu_grid_amperage(psu_grid_amperage, verbose)) &&
  (read_psu_grid_wattage(psu_grid_wattage, verbose)) &&
  (read_psu_out_voltage(psu_out_voltage, verbose)) &&
  (read_psu_out_amperage(psu_out_amperage, verbose)) &&
  (read_psu_out_wattage(psu_out_wattage, verbose)) &&
  (read_psu_intake_temp(psu_intake_temp, verbose)) &&
  (read_psu_internal_temp(psu_internal_temp, verbose)) &&
  (read_psu_fan_speed_actual(psu_fan_speed_actual, verbose)) &&
  (read_psu_fan_speed_desired(psu_fan_speed_desired, verbose)));
}

uint16_t registers[255];
uint16_t registers_old[255];
uint16_t registers_last_change[255];
uint8_t registers_last_change_age[255];


void read_psu_mcu_registers_init(){
  uint16_t u16 = 0;
  for (uint8_t i=0; i<255;++i){
    read_psu_mcu_u16(i, u16);
    registers[i] = u16;
    registers_old[i] = u16;
    registers_last_change[i] = u16;
    registers_last_change_age[i]=0xFF;
    ignore_registers[i] = false;
  }
}


void dump_all_mcu_registers(){

  uint16_t u16 = 0;
  for (uint8_t i=0; i<255;++i){
    read_psu_mcu_u16(i, u16, true);
    registers[i] = u16;
  }
  for (uint8_t i=0; i<255;++i){
 
    print_byte(i);
    Serial.print(": ");
    
    print_byte((uint8_t)(registers[i]>>8));
    Serial.print(" ");
    print_byte(registers[i] & 0xFF);
    Serial.print(" ");
    
    print_bits((uint8_t)(registers[i]>>8));
    Serial.print(" ");
    print_bits(registers[i] & 0xFF);

    Serial.print(" ");
    Serial.print(registers[i]);
    
    Serial.println();
  }

  Serial.println();
}

void read_all_psu_registers(){
  uint16_t u16;
  for (uint8_t i=0; i<255;++i){
    if (ignore_registers[i]) continue;
    read_psu_mcu_u16(i, u16);
    if (registers[i] != registers_old[i]) {
      registers_last_change_age[i] = 0;
      registers_last_change[i] = registers_old[i];
    }
    else {
      if (registers_last_change_age[i]!=0xFF){
        registers_last_change_age[i]++;
      }
    }
    registers_old[i] = registers[i];
  }
}


void search_for_voltage_registers(){
  uint16_t u16;
  for (uint8_t i=0; i <= 0x7F; i+=2){


    
    read_psu_mcu_u16(i, u16);
    registers[i] = u16;
    if ((i==0x0E) ||
        //((i > 0x40) && (i<0x60)) ||

        ((i!=0x40) &&
        (i!=0x1E) && 
        (u16 >= 0xb00) && 
        (u16 <= 0xF00))){
      Serial.print("Possible voltage register: ");
      print_byte(i);
      Serial.print(" ");
      print_byte(u16 >> 8);
      Serial.print(" ");
      print_byte(u16 & 0xFF);
      
      Serial.print(" ");
      Serial.print(u16 / 256.);

      Serial.println("v");
    }
  }
}


void search_for_voltage_eeprom(){
  uint16_t u16;
  for (uint8_t i=0; i <= 0x7F; i+=2){


    
    read_psu_mcu_u16(i, u16);
    registers[i] = u16;
    if ((i==0x0E) ||
        //((i > 0x40) && (i<0x60)) ||

        ((i!=0x40) &&
        (i!=0x1E) && 
        (u16 >= 0xb00) && 
        (u16 <= 0xF00))){
      Serial.print("Possible voltage register: ");
      print_byte(i);
      Serial.print(" ");
      print_byte(u16 >> 8);
      Serial.print(" ");
      print_byte(u16 & 0xFF);
      
      Serial.print(" ");
      Serial.print(u16 / 256.);

      Serial.println("v");
    }
  }
}

void write_psu_mcu_nothing(uint8_t reg){
  
  uint8_t cs = (psu_mcu_addr<<1) + reg;
  uint8_t reg_cs = ((0xff-cs)+1) & 0xff;

  Wire1.beginTransmission(psu_mcu_addr);
  Wire1.write((uint8_t)(reg));
  Wire1.write((uint8_t)(reg_cs));
  uint8_t res = Wire1.endTransmission();
  Serial.print("nothing result: ");
  Serial.println(res);

}


void write_psu_mcu_u8(uint8_t reg, uint8_t val){
  
  uint8_t cs = (psu_mcu_addr<<1) + reg + val;
  uint8_t reg_cs = ((0xff-cs)+1) & 0xff;

  Wire1.beginTransmission(psu_mcu_addr);
  Wire1.write((uint8_t)(reg));
  Wire1.write((uint8_t)(val));
  Wire1.write((uint8_t)(reg_cs));
  Wire1.endTransmission();

}

void write_psu_mcu_u16(uint8_t reg, uint16_t val){
  
  uint8_t val_lsb = val & 0xff;
  uint8_t val_msb = val >> 8;

  uint8_t cs = (psu_mcu_addr<<1) + reg + val_lsb + val_msb;
  uint8_t reg_cs =((0xff-cs)+1) & 0xff;

  Wire1.beginTransmission(psu_mcu_addr);
  Serial.print("[>");
  print_byte(psu_mcu_addr); 
  Serial.print("] ");
  Wire1.write((uint8_t)(reg));
  print_byte(reg); 
  Serial.print(": ");
  Wire1.write((uint8_t)(val_lsb));
  print_byte(val_lsb); 
  Serial.print(" ");
  Wire1.write((uint8_t)(val_msb));
  print_byte(val_msb); 
  Serial.print(" ");
  Wire1.write((uint8_t)(reg_cs));
  print_byte(reg_cs); 
  Serial.print(" ");
  Wire1.endTransmission();
  Serial.println("[S]");

}

void force_psu_fan(uint16_t rpm){
  write_psu_mcu_u16(0x40, rpm);
}

void write_psu_mcu_f16(uint8_t reg, double val, double scale){
  uint16_t v = (uint16_t)(val / scale);
  write_psu_mcu_u16(reg, v);
}




// /****************************************************** S E T U P */

// void setup() {

  

//   Serial.begin(115200);
//   Serial.println("Start");

//   //Wire1.setClock(100000);
//   Wire1.begin(SDA_2,SCL_2);  
//   while (psu_mem_addr == 0xFF){
//     Serial.println("Scanning for EEPROM.");
//     scan_for_device(0x50, 0x57, psu_mem_addr);
//   }
//   read_eeprom();

//   while (psu_mcu_addr == 0xFF){
//     Serial.println("Scanning for MCU.");
//     scan_for_device(0x58, 0x5F, psu_mcu_addr);
//   }
//   read_psu_mcu_registers_init();
//   setup_ignore_registers();
//   //force_psu_fan(3200);

// }




// /****************************************************** M A I N  L O O P */

// uint8_t bootdelay = 20;
// void loop() {
//   //factory_reset_eeprom();
//   //read_psu_mcu_changing_registers();
//   Serial.println(read_eeprom_byte(0x12), HEX);
//   Serial.println("EEPROM");
//   read_entire_eeprom();
//   Serial.println("MCU");
//   dump_all_mcu_registers();
// }
  