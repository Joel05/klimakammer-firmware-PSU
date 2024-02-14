#ifndef PSU_CONTROL_H
#define PSU_CONTROL_H

#undef LED_BUILTIN
#include <cstdint>


extern uint8_t LED_BUILTIN;

extern uint8_t I2C_SCL_1;
extern uint8_t I2C_SDA_1;

extern uint8_t psu_mcu_addr;
extern uint8_t psu_mem_addr;

void read_all_psu_registers();
void write_psu_mcu_u16(uint8_t, uint16_t);
void setup_ignore_registers();
void print_byte(uint8_t);
void print_bits(uint8_t);
void scan_for_device(uint8_t, uint8_t, uint8_t&);
uint8_t read_eeprom_byte(long);
void write_eeprom_byte(uint8_t, uint8_t);
void read_eeprom_string(uint8_t, uint8_t, char*);
void read_eeprom();
void read_entire_eeprom();
void factory_reset_eeprom();
bool checksum(uint8_t*, uint8_t);
bool read_psu_mcu(uint8_t, uint8_t, uint8_t*, bool);
bool read_psu_mcu_u8(uint8_t, uint8_t&, bool);
bool read_psu_mcu_u16(uint8_t, uint16_t&, bool);
bool read_psu_mcu_f16(uint8_t, double, double&, bool);
bool read_psu_mcu_flags16(uint8_t, uint16_t&, bool);
bool read_psu_grid_voltage(double&, bool);
bool read_psu_grid_amperage(double&, bool);
bool read_psu_grid_wattage(double&, bool);
bool read_psu_out_voltage(double&, bool);
bool read_psu_out_amperage(double&, bool);
bool read_psu_out_wattage(double&, bool);
bool read_psu_intake_temp(double&, bool);
bool read_psu_internal_temp(double&, bool);
bool read_psu_fan_speed_actual(double&, bool);
bool read_psu_fan_speed_desired(double&, bool);
bool read_psu_cool_flags_1(uint16_t&, bool);
bool read_psu_cool_flags_2(uint16_t&, bool);
bool read_psu_out_max_amperage(double&, bool);
bool read_psu_is_enabled(bool&, bool);
bool read_psu_data(bool);
void read_psu_mcu_registers_init();
void dump_all_mcu_registers();
void read_all_psu_registers();
void search_for_voltage_registers();
void search_for_voltage_eeprom();
void write_psu_mcu_nothing(uint8_t);
void write_psu_mcu_u8(uint8_t, uint8_t);
void write_psu_mcu_u16(uint8_t, uint16_t);
void force_psu_fan(uint16_t);
void write_psu_mcu_f16(uint8_t, double, double);

#endif