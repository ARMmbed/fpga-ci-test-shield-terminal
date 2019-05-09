/*
 * Copyright (c) 2019, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * This is a terminal program for interacting with the FPGA CI Test Shield
 *
 * It is intended for use during hardware bring up and general FPGA debugging.
 * This program provides the following functionality:
 * - read/write values on all logical and system pins
 * - Update FPGA firmware (SD card must be inserted)
 * - Readback FPGA firmware (SD card must be inserted)
 *
 * To interact with this terminal program connect to the Mbed board's
 * serial port at a baudrate of 9600. To see detailed information on each
 * command use the "help" command.
 *
 */

#include "mbed.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FATFileSystem.h"
#include "SDBlockDevice.h"
#include "SPIFBlockDevice.h"

#include "MbedTester.h"
#include "pinmap.h"

static bool help_handler(const char *line);
static bool gpio_read_handler(const char *line);
static bool gpio_write_handler(const char *line);
static bool sys_read_handler(const char *line);
static bool sys_write_handler(const char *line);
static bool sys_tristate_handler(const char *line);
static bool set_pull_handler(const char *line);
static bool io_exp_read_handler(const char *line);
static bool set_mux_en_and_addr_handler(const char *line);
static bool set_pwm_handler(const char *line);
static bool pwm_sweep_handler(const char *line);
static bool set_analog_out_handler(const char *line);
static bool io_exp_test_handler(const char *line);
static bool mux_test_handler(const char *line);
static bool update_handler(const char *line);
static bool dump_handler(const char *line);
static bool reprogram_handler(const char *line);

static bool run_command();
static char get_next_val();
static bool readline(char *buf, size_t size);
static bool starts_with(const char *str, const char *prefix);
static void print_progress(uint8_t percent);

typedef struct {
    const char *name;
    bool (*handler)(const char *);
    const char *help;

} command_t;

extern size_t command_count;
extern command_t commands[];
extern size_t sys_pin_count;
extern const char *sys_pin_names[];

SDBlockDevice sd(PTE3, PTE1, PTE2, PTE4, 40000000);
FATFileSystem fs("fs");

const PinList *form_factor = pinmap_ff_default_pins();
const PinList *restricted = pinmap_restricted_pins();
MbedTester tester(pinmap_ff_default_pins(), pinmap_restricted_pins());

int main()
{
    printf("FPGA CI Test Shield command line test utility\r\n\r\n");

    while (true) {
        run_command();
    }
}

static bool run_command()
{
    char line_buf[64];
    if (!readline(line_buf, sizeof(line_buf))) {
        printf("Line too long\r\n");
        return false;
    }

    command_t *command = NULL;
    for (size_t i = 0; i < command_count; i++) {
        command_t *temp = commands + i;
        if (starts_with(line_buf, temp->name)) {
            command = temp;
            break;
        }
    }

    if (command) {
        size_t cmd_size = strlen(command->name);
        bool success = command->handler(line_buf + cmd_size);
        if (!success) {
            printf("Invalid argument(s) \"%s\" for for command \"%s\"\r\n", line_buf + cmd_size, command->name);
            return false;
        }
    } else {
        printf("Unrecognized command \"%s\". Type \"help\" for more information.\r\n", line_buf);
        return false;
    }
    return true;
}

static bool help_handler(const char *line)
{
    printf("\r\n");
    printf("\r\n");
    printf("Supported commands:\r\n");
    for (size_t i = 0; i < command_count; i++) {
        printf("  %s%s\r\n", commands[i].name, commands[i].help);
    }

    return true;
}

static bool set_control_pins_handler(const char *line)
{
    const PinList *form_factor = pinmap_ff_default_pins();

    int clk;
    int mosi;
    int miso;
    int aux;
    if (sscanf(line, "%i %i %i %i", &clk, &mosi, &miso, &aux) != 4) {
        return false;
    }

    if ((clk < 0) || (clk >= form_factor->count)) {
        return false;
    }
    if ((mosi < 0) || (mosi >= form_factor->count)) {
        return false;
    }
    if ((miso < 0) || (miso >= form_factor->count)) {
        return false;
    }
    if ((clk & 1) != 0) {
        printf("Control CLK must be on an even pin index\r\n");
        return false;
    }
    if (clk + 1 != mosi) {
        printf("Control MOSI must be the index after CLK\r\n");
        return false;
    }
    if (miso == clk) {
        printf("Control MISO must be different than CLK\r\n");
        return false;
    }
    if (miso == mosi) {
        printf("Control MISO must be different than MOSI\r\n");
        return false;
    }
    if (aux == clk) {
        printf("Control AUX must be different than CLK\r\n");
        return false;
    }
    if (aux == mosi) {
        printf("Control AUX must be different than MOSI\r\n");
        return false;
    }
    if (aux == miso) {
        printf("Control AUX must be different than MISO\r\n");
        return false;
    }

    const PinName *pins = form_factor->pins;
    tester.set_control_pins_manual(pins[clk], pins[mosi], pins[miso], pins[aux]);

    printf("Control pins set CLK=%i, MOSI=%i, MISO=%i, AUX=%i\r\n", clk, mosi, miso, aux);
    return true;
}

static bool set_control_auto_handler(const char *line)
{
    tester.set_control_pins_auto();

    printf("Control pins set to automatic\r\n");
    return true;
}

static bool gpio_read_handler(const char *line)
{
    int pin;
    if (sscanf(line, "%i", &pin) != 1) {
        return false;
    }
    if ((pin < 0) || (pin > 127)) {
        return false;
    }

    tester.pin_map_reset();
    tester.pin_map_index(pin, MbedTester::LogicalPinGPIO0);
    tester.peripherals_reset();
    tester.select_peripheral(MbedTester::PeripheralGPIO);

    printf("Reading GPIO pin %i (IO%i): %i\r\n", pin, pin, tester.gpio_read(MbedTester::LogicalPinGPIO0));
    return true;
}

static bool gpio_write_handler(const char *line)
{
    int pin;
    int drive;
    if (sscanf(line, "%i %i", &pin, &drive) != 2) {
        return false;
    }
    if ((pin < 0) || (pin > 127)) {
        return false;
    }
    if ((drive != 0) && (drive != 1)) {
        return false;
    }

    tester.pin_map_reset();
    tester.pin_map_index(pin, MbedTester::LogicalPinGPIO0);
    tester.peripherals_reset();
    tester.select_peripheral(MbedTester::PeripheralGPIO);

    tester.gpio_write(MbedTester::LogicalPinGPIO0, drive, 1);

    printf("Setting GPIO pin %i (IO%i) to %i\r\n", pin, pin, drive);
    return true;
}

static bool sys_read_handler(const char *line)
{
    int pin;
    if (sscanf(line, "%i", &pin) != 1) {
        return false;
    }
    if ((pin < 0) || (pin > 40)) {
        return false;
    }

    printf("Reading System pin %i (IO%s): %i\r\n", pin, sys_pin_names[pin], tester.sys_pin_read((MbedTester::SystemPin)pin));
    return true;
}

static bool sys_write_handler(const char *line)
{
    int pin;
    int value;
    if (sscanf(line, "%i %i", &pin, &value) != 2) {
        return false;
    }
    if ((pin < 0) || (pin > 40)) {
        return false;
    }
    if ((value != 0) && (value != 1)) {
        return false;
    }

    printf("Setting System pin %i (IO%s) to %i\r\n", pin, sys_pin_names[pin], value);
    tester.sys_pin_write((MbedTester::SystemPin)pin, value, 1);
    return true;
}

static bool sys_tristate_handler(const char *line)
{
    int pin;
    if (sscanf(line, "%i", &pin) != 1) {
        return false;
    }
    if ((pin < 0) || (pin > 40)) {
        return false;
    }

    printf("Tristating System pin %i (IO%s)\r\n", pin, sys_pin_names[pin]);
    tester.sys_pin_write((MbedTester::SystemPin)pin, 0, 0);
    return true;
}

static bool set_pull_all_handler(const char *line)
{
    MbedTester::PullMode mode;
    char mode_s[10];
    if (sscanf(line, "%s", mode_s) != 1) {
        return false;
    }

    if      (strcmp(mode_s, "PULLDOWN") == 0) { mode = MbedTester::PullDown; }
    else if (strcmp(mode_s, "PULLUP") == 0)   { mode = MbedTester::PullUp; }
    else if (strcmp(mode_s, "TRISTATE") == 0) { mode = MbedTester::PullNone; }
    else { return false; }

    printf("Setting pull on all pins\r\n");
    for (int i = 0; i < 128; i++) {
        tester.pin_set_pull_index(i, mode);
    }
    printf("Set pull on all pins\r\n");

    return true;
}

static bool set_pull_handler(const char *line)
{
    int index;
    MbedTester::PullMode mode;
    char mode_s[10];
    if (sscanf(line, "%i %s", &index, mode_s) != 2) {
        return false;
    }
    if ((index < 0) || (index > 127)) {
        return false;
    }

    if      (strcmp(mode_s, "PULLDOWN") == 0) { mode = MbedTester::PullDown; }
    else if (strcmp(mode_s, "PULLUP") == 0)   { mode = MbedTester::PullUp; }
    else if (strcmp(mode_s, "TRISTATE") == 0) { mode = MbedTester::PullNone; }
    else { return false; }

    PinName pin = form_factor->pins[index];
    if (index >= form_factor->count) {
        printf("Setting pull for pin NC to %s (index %d)\r\n", mode_s, index);
    } else {
        printf("Setting pull for pin %s to %s (index %d)\r\n", pinmap_ff_default_pin_to_string(pin), mode_s, index);
    }
    tester.pin_set_pull_index(index, mode);
    return true;
}

static bool io_exp_read_handler(const char *line)
{
    int index;
    MbedTester::IOExpanderReg reg;
    char reg_s[10];
    if (sscanf(line, "%i %s", &index, reg_s) != 2) {
        return false;
    }
    if ((index < 0) || (index > 127)) {
        return false;
    }

    if      (strcmp(reg_s, "INPUT") == 0)  { reg = MbedTester::RegInput; }
    else if (strcmp(reg_s, "OUTPUT") == 0) { reg = MbedTester::RegOutput; }
    else if (strcmp(reg_s, "CONFIG") == 0) { reg = MbedTester::RegConfig; }
    else { return false; }

    uint8_t bit = tester.io_expander_read_index(index, reg);
    PinName pin = form_factor->pins[index];
    if (index >= form_factor->count) {
        printf("%s bit for pin NC = %d (index %d)\r\n", reg_s, bit, index);
    } else {
        printf("%s bit for pin %s = %d (index %d)\r\n", reg_s, pinmap_ff_default_pin_to_string(pin), bit, index);
    }
    return true;
}

static bool set_mux_en_and_addr_handler(const char *line)
{
    int enable;
    int index;
    if (sscanf(line, "%i %i", &enable, &index) != 2) {
        return false;
    }
    if ((enable != 0) && (enable != 1)) {
        return false;
    }
    if ((index < 0) || (index > 127)) {
        return false;
    }

    tester.set_mux_enable((bool)enable);
    tester.set_mux_addr_index(index);
    printf("Setting MUX enable = %d, address = %d\r\n", enable, index);

    return true;
}

static bool set_pwm_handler(const char *line)
{
    int enable;
    int period;
    int cycles_high;
    if (sscanf(line, "%i %i %i", &enable, &period, &cycles_high) != 3) {
        return false;
    }
    if ((enable != 0) && (enable != 1)) {
        return false;
    }
    if (period < 0) {
        return false;
    }
    if ((cycles_high < 0) || (cycles_high > period)) {
        return false;
    }

    tester.set_pwm_period_and_cycles_high(period, cycles_high);
    tester.set_pwm_enable((bool)enable);
    printf("Setting PWM enable = %d, period = %d clk cycles, and cycles high = %d clk cycles\r\n", enable, period, cycles_high);

    return true;
}

static bool pwm_sweep_handler(const char *line)
{
    for (int i = 0; i < 11; i += 1) {
        tester.set_pwm_period_and_cycles_high(100, i * 10);
        tester.set_pwm_enable(true);
        printf("Setting PWM enable = %d, period = %d clk cycles, and cycles high = %d clk cycles\r\n", 1, 100, i * 10);
        wait_ms(10);
    }
    tester.set_pwm_enable(false);

    return true;
}

static bool set_analog_out_handler(const char *line)
{
    int enable;
    float voltage;
    if (sscanf(line, "%i %f", &enable, &voltage) != 2) {
        return false;
    }
    if ((enable != 0) && (enable != 1)) {
        return false;
    }
    if ((voltage < 0.0) || (voltage > 1.0)) {
        return false;
    }

    float an_out = 3.3 * voltage;
    tester.set_analog_out(enable, voltage);
    printf("Setting analog out enable = %d, voltage = (%f * 3.3V) = %fV\r\n", enable, voltage, an_out);

    return true;
}

static bool io_exp_test_handler(const char *line)
{
    printf("Testing IO expander for all pins\r\n");

    // Reset tester stats and select GPIO
    tester.peripherals_reset();

    // Select GPIO peripheral
    tester.select_peripheral(MbedTester::PeripheralGPIO);

    // Remap pins for test
    tester.pin_map_reset();

    //reset IO expander
    tester.pin_pull_reset_all();

    // IO Expander Test using the system i2c bus
    printf("\n\nIO Expander Test\r\n\n");
    for (uint32_t i = 0; i < 128; i++) {
        if (i < form_factor->count) {
            const PinName test_pin = form_factor->pins[i];
            if (pinmap_list_has_pin(restricted, test_pin)) {
                printf("Skipping pin %s (%i)\r\n", pinmap_ff_default_pin_to_string(test_pin), test_pin);
                continue;
            }
            printf("\nIO Expander i2c system bus test on pin %s (%i)\r\n", pinmap_ff_default_pin_to_string(test_pin), i);
        }
        else {
            printf("\nIO Expander i2c system bus test on pin NC (%i)\r\n", i);
        }

        tester.pin_map_index(i, MbedTester::LogicalPinGPIO0);
        tester.gpio_write(MbedTester::LogicalPinGPIO0, 0, false);

        //test pulldown
        tester.pin_set_pull_index(i, MbedTester::PullDown);
        //config bit should be 0 for output
        //output bit should be 0
        if ((tester.io_expander_read_index(i, MbedTester::RegConfig) != 0) || (tester.io_expander_read_index(i, MbedTester::RegOutput) != 0)) {
            printf("i2c bus malfunction on pin index %d\r\n", i);
        }
        if (tester.gpio_read(MbedTester::LogicalPinGPIO0) != 0) {
            printf("Pin index %d pulldown malfunction\r\n", i);
        }

        //test pullup
        tester.pin_set_pull_index(i, MbedTester::PullUp);
        //config bit should be 0 for output
        //output bit should be 1
        if ((tester.io_expander_read_index(i, MbedTester::RegConfig) != 0) || (tester.io_expander_read_index(i, MbedTester::RegOutput) != 1)) {
            printf("i2c bus malfunction on pin index %d\r\n", i);
        }
        if (tester.gpio_read(MbedTester::LogicalPinGPIO0) != 1) {
            printf("Pin index %d pullup malfunction\r\n", i);
        }

        //test tristate
        tester.pin_set_pull_index(i, MbedTester::PullNone);
        //config bit should be 1 for input
        if (tester.io_expander_read_index(i, MbedTester::RegConfig) != 1) {
            printf("i2c bus malfunction on pin index %d\r\n", i);
        }

        tester.gpio_write(MbedTester::LogicalPinGPIO0, 0, true);//write a 0 to the pin index
        //input bit should be 0
        if (tester.io_expander_read_index(i, MbedTester::RegInput) != 0) {
            printf("Pin index %d tristate malfunction\r\n", i);
        }
        tester.gpio_write(MbedTester::LogicalPinGPIO0, 1, true);//write a 1 to the pin index
        //input bit should be 1
        if (tester.io_expander_read_index(i, MbedTester::RegInput) != 1) {
            printf("Pin index %d tristate malfunction\r\n", i);
        }
        tester.gpio_write(MbedTester::LogicalPinGPIO0, 0, false);//un-drive the pin index
    }
    return true;
}

static bool mux_test_handler(const char *line)
{
    printf("Testing AnMUX for all pins both directions\r\n");

    // Reset tester stats and select GPIO
    tester.peripherals_reset();

    // Select GPIO peripheral
    tester.select_peripheral(MbedTester::PeripheralGPIO);

    // Remap pins for test
    tester.pin_map_reset();

    // enable MUX
    tester.set_mux_enable(true);

    // enable ADC
    tester.set_sample_adc(true);

    printf("\n\nTest driving DUT_ANALOG from Mbed pins\r\n\n");
    //tristate MUX output
    tester.sys_pin_write(MbedTester::AnalogMuxPwmOut, 0, false);
    tester.sys_pin_write(MbedTester::AnalogMuxIn, 0, false);

    for (uint32_t i = 0; i < 128; i++) {

        if (i < form_factor->count) {
            const PinName test_pin = form_factor->pins[i];
            if (pinmap_list_has_pin(restricted, test_pin)) {
                printf("Skipping pin %s (%i)\r\n", pinmap_ff_default_pin_to_string(test_pin), test_pin);
                continue;
            }
            printf("\nAnMUX Test: Driving DUT_ANALOG from Mbed pin %s (%i)\r\n", pinmap_ff_default_pin_to_string(test_pin), i);
        }
        else {
            printf("\nAnMUX Test: Driving DUT_ANALOG from Mbed pin NC (%i)\r\n", i);
        }

        tester.set_mux_addr_index(i);
        tester.pin_map_index(i, MbedTester::LogicalPinGPIO0);

        tester.gpio_write(MbedTester::LogicalPinGPIO0, 0, true);//write a 0 to the pin index
        //assert AnalogMuxIn is 0
        float reading = tester.get_analog_in();
        if (reading > 0.05) {
            printf("DUT_ANALOG != Mbed pin index %d\r\n",i);
        }
        tester.gpio_write(MbedTester::LogicalPinGPIO0, 1, true);//write a 1 to the pin index
        //assert AnalogMuxIn is 1
        reading = tester.get_analog_in();
        if (reading < 0.95) {
            printf("DUT_ANALOG != Mbed pin index %d\r\n",i);
        }
        tester.gpio_write(MbedTester::LogicalPinGPIO0, 0, false);//tristate pin index
    }

    printf("\n\nTest driving Mbed pins from DUT_ANALOG\r\n\n");

    for (uint32_t i = 0; i < 128; i++) {

        if (i < form_factor->count) {
            const PinName test_pin = form_factor->pins[i];
            if (pinmap_list_has_pin(restricted, test_pin)) {
                printf("Skipping pin %s (%i)\r\n", pinmap_ff_default_pin_to_string(test_pin), test_pin);
                continue;
            }
            printf("\nAnMUX Test: Driving Mbed pin %s (%i) from DUT_ANALOG\r\n", pinmap_ff_default_pin_to_string(test_pin), i);
        }
        else {
            printf("\nAnMUX Test: Driving Mbed pin NC (%i) from DUT_ANALOG\r\n", i);
        }
        tester.set_mux_addr_index(i);
        tester.pin_map_index(i, MbedTester::LogicalPinGPIO0);
        tester.gpio_write(MbedTester::LogicalPinGPIO0, 0, false);//tristate the pin index

        tester.set_analog_out(1, 0.0);
        wait_ms(10);
        //assert pin index is 0
        if (tester.gpio_read(MbedTester::LogicalPinGPIO0) != 0) {
            printf("Mbed pin index %d != DUT_ANALOG\r\n",i);
        }
        tester.set_analog_out(1, 1.0);
        wait_ms(10);
        //assert pin index is 1
        if (tester.gpio_read(MbedTester::LogicalPinGPIO0) != 1) {
            printf("Mbed pin index %d != DUT_ANALOG\r\n",i);
        }
        tester.set_analog_out(0, 0.0);
    }
    //disable MUX
    tester.set_mux_enable(false);
    return true;
}

static bool sys_pins_handler(const char *line)
{
    printf("\r\n");
    printf("\r\n");
    printf("System pins:\r\n");
    for (size_t i = 0; i < sys_pin_count; i++) {
        printf("  %i - %s\r\n", i, sys_pin_names[i]);
    }

    return true;
}

static bool mbed_pins_handler(const char *line)
{
    printf("\r\n");
    printf("\r\n");
    printf("Mbed pins:\r\n");
    for (uint32_t i = 0; i < form_factor->count; i++) {
        const PinName test_pin = form_factor->pins[i];
        if (pinmap_list_has_pin(restricted, test_pin)) {
            printf("Pin name: %s | pin_index: %d | (restricted)\r\n",pinmap_ff_default_pin_to_string(test_pin), i);
        }
        else {
            printf("Pin name: %s | pin_index: %d\r\n",pinmap_ff_default_pin_to_string(test_pin), i);
        }
    }

    return true;
}

static bool update_direct(File *src)
{
    SPIFBlockDevice flash(D11, D12, D13, D10);

    print_progress(0);

    if (flash.init() != BD_ERROR_OK) {
        return false;
    }

    bd_size_t erase_size = flash.get_erase_size();
    uint32_t addr = 0;
    uint8_t buf[256];
    uint32_t total_size = src->size();
    uint32_t prev_percent_done = 0;
    while (addr < total_size) {
        uint32_t program_size = total_size - addr;
        if (program_size > sizeof(buf)) {
            program_size = sizeof(buf);
        }
        ssize_t read_size = src->read(buf, program_size);
        if (read_size < 0) {
            return false;
        } else if (read_size == 0) {
            break;
        }
        program_size = read_size;

        if (addr % erase_size == 0) {
            if (flash.erase(addr, erase_size) != BD_ERROR_OK) {
                return false;
            }
        }

        if (flash.program(buf, addr, read_size) != BD_ERROR_OK) {
            return false;
        }

        addr += program_size;

        const uint8_t percent_done = (addr * 100) / total_size;
        if (percent_done != prev_percent_done) {
            print_progress(percent_done);
            prev_percent_done = percent_done;
        }
    }

    print_progress(100);

    return true;
}

static bool update_direct_handler(const char *line)
{
    char filename[64];
    if (sscanf(line, "%s", filename) != 1) {
        return false;
    }

    printf("Programming firmware from file \"%s\"\r\n", filename);

    int ret = fs.mount(&sd);
    if (ret != 0) {
        printf("Error mounting filesystem\r\n");
        return true;
    }

    File file;

    ret = file.open(&fs, filename);
    if (ret != 0) {
        printf("Open failed\r\n");
        fs.unmount();
        return true;
    }

    printf("Programming image to serial flash\r\n");
    if (update_direct(&file)) {
        printf("\r\nFirmware successfully programmed from \"%s\"\r\n", filename);
    } else {
        printf("\r\nError programming firmware to the device\r\n");
    }

    file.close();
    fs.unmount();

    return true;
}

static bool dump_direct(File *dest)
{
    SPIFBlockDevice flash(D11, D12, D13, D10);

    print_progress(0);

    if (flash.init() != BD_ERROR_OK) {
        return false;
    }

    uint32_t flash_size = flash.size();
    uint32_t pos = 0;
    uint8_t buf[256];
    uint32_t prev_percent_done = 0;
    while (pos < flash_size) {
        uint32_t read_size = flash_size - pos;
        if (read_size > sizeof(buf)) {
            read_size = sizeof(buf);
        }
        if (flash.read(buf, pos, read_size) != BD_ERROR_OK) {
            return false;
        }
        ssize_t write_size = dest->write(buf, read_size);
        if (write_size != read_size) {
            return false;
        }
        pos += read_size;

        const uint8_t percent_done = (pos * 100) / flash_size;
        if (percent_done != prev_percent_done) {
           print_progress(percent_done);
           prev_percent_done = percent_done;
        }
    }

    print_progress(100);

    return true;
}

static bool dump_direct_handler(const char *line)
{
    char filename[64];
    if (sscanf(line, "%s", filename) != 1) {
        return false;
    }

    printf("Dumping firmware to file \"%s\"\r\n", filename);

    int ret = fs.mount(&sd);
    if (ret != 0) {
        printf("Error mounting filesystem\r\n");
        return true;
    }

    File file;

    ret = file.open(&fs, filename, O_WRONLY | O_CREAT | O_TRUNC);
    if (ret != 0) {
        printf("Open failed\r\n");
        fs.unmount();
        return true;
    }

    printf("Reading image from serial flash\r\n");
    if (dump_direct(&file)) {
        printf("\r\nFirmware successfully written to \"%s\"\r\n", filename);
    } else {
        printf("\r\nError reading firmware from the device\r\n");
    }

    file.close();
    fs.unmount();

    return true;
}

static bool update_handler(const char *line)
{
    char filename[64];
    if (sscanf(line, "%s", filename) != 1) {
        return false;
    }

    printf("Programming firmware from file \"%s\"\r\n", filename);

    int ret = fs.mount(&sd);
    if (ret != 0) {
        printf("Error mounting filesystem\r\n");
        return true;
    }

    File file;

    ret = file.open(&fs, filename);
    if (ret != 0) {
        printf("Open failed\r\n");
        fs.unmount();
        return true;
    }

    printf("Programming image to serial flash\r\n");
    if (tester.firmware_update(&file, callback(print_progress))) {
        printf("\r\nFirmware successfully programmed from \"%s\"\r\n", filename);
    } else {
        printf("\r\nError programming firmware to the device\r\n");
    }

    file.close();
    fs.unmount();

    return true;
}

static bool dump_handler(const char *line)
{
    char filename[64];
    if (sscanf(line, "%s", filename) != 1) {
        return false;
    }

    printf("Dumping firmware to file \"%s\"\r\n", filename);

    int ret = fs.mount(&sd);
    if (ret != 0) {
        printf("Error mounting filesystem\r\n");
        return true;
    }

    File file;

    ret = file.open(&fs, filename, O_WRONLY | O_CREAT | O_TRUNC);
    if (ret != 0) {
        printf("Open failed\r\n");
        fs.unmount();
        return true;
    }

    printf("Reading image from serial flash\r\n");
    if (tester.firmware_dump(&file, callback(print_progress))) {
        printf("\r\nFirmware successfully written to \"%s\"\r\n", filename);
    } else {
        printf("\r\nError reading firmware from the device\r\n");
    }

    file.close();
    fs.unmount();

    return true;
}

static bool reprogram_handler(const char *line)
{
    printf("Starting reprograming\r\n");
    tester.reprogram();

    return true;
}

static bool read_analog_handler(const char *line)
{
    tester.set_sample_adc(true);

    float reading = tester.get_analog_in();
    printf("ADC reading = %f v\r\n", reading);

    return true;
}

static bool read_power_handler(const char *line)
{
    int channel;
    int duration;
    if (sscanf(line, "%i %i", &channel, &duration) != 2) {
        return false;
    }
    if ((channel < 0) || (channel > 3)) {
        return false;
    }
    if (duration < 0) {
        return false;
    }

    tester.set_sample_adc(true);

    if (duration == 0) {
        float reading = tester.get_anin_voltage(channel);
        printf("Power channel %i reading = %f v\r\n", channel, reading);
    } else {
        printf("Averaging for %i seconds on power channel %i\r\n", duration, channel);

        uint64_t sum[2];
        uint32_t samples[2];
        uint64_t cycles[2];
        tester.get_anin_sum_samples_cycles(channel, sum + 0, samples + 0, cycles + 0);
        wait(duration);
        tester.get_anin_sum_samples_cycles(channel, sum + 1, samples + 1, cycles + 1);

        uint32_t total_samples = samples[1] - samples[0];
        uint64_t total_sum = sum[1] - sum[0];
        float average = (float)total_sum / total_samples / 4095.0f;
        printf("Average voltage %f v\r\n", average);
    }

    return true;
}
static char get_next_val()
{
    char val = getchar();
    putchar(val);
    if (val == '\r') {
        // Inject newline after carriage return
        putchar('\n');
    }
    return val;
}

static bool readline(char *buf, size_t size)
{
    buf[size - 1] = 0;
    bool prev_cr = false;
    size_t pos = 0;
    while (pos < size) {
        char next = get_next_val();

        // End of line handling
        if ((next == '\n') || (next == '\r')) {
            if (pos == 0) {
                // Ignore empty lines
                continue;
            }
            buf[pos] = 0;
            return true;
        }

        // Backspace handling
        if (next == 127) {
            if (pos > 0) {
                pos--;
            }
            continue;
        }
        //
        buf[pos] = next;
        pos++;

    }
    return false;
}

static bool starts_with(const char *str, const char *prefix)
{
    while ((*str != 0) && (*prefix != 0)) {
        if (*str != *prefix) {
            return false;
        }
        str++;
        prefix++;
    }
    return *prefix == 0 ? true : false;
}

static void print_progress(uint8_t percent)
{
    printf("\r[");
    for (int i = 1; i <= 25; i++) {
        printf((percent >= i * 4) ? "=" : " ");
    }
    printf("] %2i%%", percent);
}

command_t commands[] = {
    {
        "help",
        help_handler,
        " - Print this help info\r\n"
    },
    {
        "set_control_pins",
        set_control_pins_handler,
        " <CLK> <MOSI> <MISO> <AUX> - Set the pins used to control the FPGA\r\n"
        "    By default the FPGA control pins are chosen automatically. This\r\n"
        "    commands allows the control pins to be set explicitly. When pins are\r\n"
        "    manually set care needs to be taken to ensure that they are not used for\r\n"
        "    other purposes, such as gpio_read or gpio_write. The control pins have the\r\n"
        "    following restrictions:\r\n"
        "    - All pins must be in the current form factor\r\n"
        "    - CLK must be on an even pin index\r\n"
        "    - MOSI must be the index of CLK + 1\r\n"
        "    - MISO can be on any remaining pin\r\n"
        "    - AUX can be on any remaining pin\r\n"
        "    Example: \"set_control_pins 0 1 2 3\"\r\n"
        "    Example: \"set_control_pins 4 5 1 0\"\r\n"
    },
    {
        "set_control_auto",
        set_control_auto_handler,
        " - Let the control pins be moved automatically.\r\n"
        "    This is the default mode.\r\n"
    },
    {
        "gpio_read",
        gpio_read_handler,
        " <GPIO pin> - Read the value on a GPIO pin.\r\n"
        "    Allowed values for <logical pin> are 0 to 127\r\n"
        "    Example: \"gpio_read 27\"\r\n"
        "    Note: This may change the pins used to control the FPGA if they conflict\r\n"
    },
    {
        "gpio_write",
        gpio_write_handler,
        " <GPIO pin> <value> - Write the value on a GPIO pin and set it as output.\r\n"
        "    Allowed values for <logical pin> are 0 to 127\r\n"
        "    Allowed values for <value> are 0 and 1\r\n"
        "    Example: \"gpio_write 27, 1\"\r\n"
        "    Example: \"gpio_write 27, 0\"\r\n"
        "    Note: This may change the pins used to control the FPGA if they conflict\r\n"
        "    Note: This pin will return to tristate as soon as gpio_read or gpio_write is performed on a separate pin.\r\n"
    },
    {
        "sys_read",
        sys_read_handler,
        " <system pin> - Read the value on a system pin.\r\n"
        "    Allowed values for <system pin> are 0 to 40\r\n"
        "    Example: \"sys_read 2\"\r\n"
    },
    {
        "sys_write",
        sys_write_handler,
        " <system pin> <value> - Write the value on a GPIO pin and set it as output.\r\n"
        "    Allowed values for <system pin> are 0 to 40\r\n"
        "    Allowed values for <value> are 0 and 1\r\n"
        "    Example: \"sys_write 5 1\"\r\n"
        "    Example: \"sys_write 5 0\"\r\n"
    },
    {
        "sys_tristate",
        sys_tristate_handler,
        " <system pin> - Pin to tristate.\r\n"
        "    Allowed values for <system pin> are 0 to 40\r\n"
        "    Example: \"sys_tristate 2\"\r\n"
    },
    {
        "set_pull_all",
        set_pull_all_handler,
        " <mode> - Set all pins to a pull mode.\r\n"
        "    Allowed values for <mode> are PULLDOWN, PULLUP, or TRISTATE\r\n"
        "    Example: \"set_pull_all PULLUP\"\r\n"
    },
    {
        "set_pull",
        set_pull_handler,
        " <pin_index> <mode> - Set pin to a pull mode.\r\n"
        "    Allowed values for <pin_index> are 0 to 127\r\n"
        "    Allowed values for <mode> are PULLDOWN, PULLUP, or TRISTATE\r\n"
        "    Example: \"set_pull 2 PULLUP\"\r\n"
    },
    {
        "io_exp_read",
        io_exp_read_handler,
        " <pin_index> <reg> - Read a bit for a specific Mbed pin that is set in the input, output, or configuration registers inside of the IO expander.\r\n"
        "    Allowed values for <pin_index> are 0 to 127\r\n"
        "    Allowed values for <reg> are INPUT, OUTPUT, or CONFIG\r\n"
        "    Example: \"io_exp_read 2 OUTPUT\"\r\n"
    },
    {
        "set_mux_en_and_addr",
        set_mux_en_and_addr_handler,
        " <enable> <pin_index> - Set the analog MUX enable and address.\r\n"
        "    Allowed values for <enable> are 0 or 1\r\n"
        "    Allowed values for <pin_index> are 0 to 127\r\n"
        "    Example: \"set_mux_en_and_addr 1 25\"\r\n"
    },
    {
        "set_pwm",
        set_pwm_handler,
        " <enable> <period> <cycles_high> - Set the pwm to be enabled/disabled, period in clk cycles, and cycles_high in clk cycles.\r\n"
        "    Allowed values for <enable> are 0 or 1\r\n"
        "    Allowed values for <period> are any value in clk cycles >= 0\r\n"
        "    Allowed values for <cycles_high> are 0 to period (clk cycles)\r\n"
        "    Example: \"set_pwm 1 100 75\" (enable pwm at 100 clk cycle period and 75%% duty cycle)\r\n"
    },
    {
        "pwm_sweep",
        pwm_sweep_handler,
        " Sweep through pwm duty cycles 0-100 in 10-step increments.\r\n"
        "    Example: \"pwm_sweep\"\r\n"
    },
    {
        "set_analog_out",
        set_analog_out_handler,
        " <enable> <voltage> - Create an analog voltage via the FPGA sys pwm.\r\n"
        "    Allowed values for <enable> are 0 or 1\r\n"
        "    Allowed values for <voltage> 0.0 to 1.0 (float)\r\n"
        "    Example: \"set_analog_out 1 0.75\" (create a voltage of (0.75 * 3.3V) = 2.475V)\r\n"
    },
    {
        "io_exp_test",
        io_exp_test_handler,
        " Test the IO expander, all 3 pull states for all pins.\r\n"
        "    Example: \"io_exp_test\"\r\n"
    },
    {
        "mux_test",
        mux_test_handler,
        " Test the analog MUX, all pins, both directions.\r\n"
        "    Example: \"mux_test\"\r\n"
    },
    {
        "sys_pins",
        sys_pins_handler,
        " - Display the system pins.\r\n"
    },
    {
        "mbed_pins",
        mbed_pins_handler,
        " - Display the Mbed pins.\r\n"
    },
    {
        "update_direct",
        update_direct_handler,
        " <filename> - Directly program a serial flash part from the SD card.\r\n"
        "    The serial flash must have the following connections:\r\n"
        "      D10 - Chip Select\r\n"
        "      D11 - MOSI\r\n"
        "      D12 - MISO\r\n"
        "      D13 - CLOCK\r\n"
    },
    {
        "dump_direct",
        dump_direct_handler,
        " <filename> - Directly dump the serial flash contents to the SD card.\r\n"
        "    The serial flash must have the following connections:\r\n"
        "      D10 - Chip Select\r\n"
        "      D11 - MOSI\r\n"
        "      D12 - MISO\r\n"
        "      D13 - CLOCK\r\n"
    },
    {
        "update",
        update_handler,
        " <filename> - Start a firmware update from the SD card.\r\n"
    },
    {
        "dump",
        dump_handler,
        " <filename> - Dump FPGA firmware to an SD card.\r\n"
    },
    {
        "reprogram",
        reprogram_handler,
        " - Trigger the FPGA to reprogram its image from flash\r\n"
    },
    {
        "read_analog",
        read_analog_handler,
        " - Take an analog reading from the multiplexer\r\n"
    },
    {
        "read_power",
        read_power_handler,
        " <channel> <duration> - Take an average of the voltage on the given channel\r\n"
        "    Allowed values for <channel> are 0 through 3\r\n"
        "    Allowed values for <duration> are all positive whole numbers\r\n"
        "    Example: \"read_power 3 60\"\r\n"
    }
};
size_t command_count = sizeof(commands) / sizeof(commands[0]);

const char *sys_pin_names[] = {
    "Reset",
    "Reprogram",
    "DigitalID0",
    "DigitalID1",
    "DigitalID2",
    "Led0",
    "Led1",
    "Led2",
    "Led3",
    "SPIIO0",
    "SPIIO1",
    "SPIIO2",
    "SPIIO3",
    "SPIClk",
    "SPICS",
    "I2CReset",
    "I2CSda0",
    "I2CSda1",
    "I2CSda2",
    "I2CScl0",
    "I2CScl1",
    "I2CScl2",
    "AnalogMuxEnable",
    "AnalogMuxPwmOut",
    "AnalogMuxIn",
    "AnalogMuxAddr0",
    "AnalogMuxAddr1",
    "AnalogMuxAddr2",
    "AnalogMuxAddr3",
    "AnalogMuxAddr4",
    "AnalogMuxAddr5",
    "AnalogMuxAddr6",
    "AnalogMuxAddr7",
    "AnalogInP0",
    "AnalogInP1",
    "AnalogInP2",
    "AnalogInP3",
    "AnalogInN0",
    "AnalogInN1",
    "AnalogInN2",
    "AnalogInN3",
};
size_t sys_pin_count = sizeof(sys_pin_names) / sizeof(sys_pin_names[0]);

char get_next_val_test()
{
    static int index = 0;
    const char *test_str =
        "test line 1\r\n"       // Invalid
        "test line 2\n"         // Invalid
        "help\n"                // Valid
        "gpio_read 1 2\n"       // Valid
        "gpio_read asdfwe\n"    // Invalid
        "gpio_read 256\n"       // Invalid
        "sys_read 256\n"        // Invalid
        "sys_read 0\n"          // valid
        "sys_read 40\n"         // valid
        "sys_write 0 10\n"      // Invalid
        "sys_write 0\n"         // Invalid
        "sys_write 0 1\n"       // Valid
        "sys_write 40 0\n";     // Valid

    char val = test_str[index];
    if (val == 0) {
        exit(0);
    } else {
        index++;
    }
    return val;
}
