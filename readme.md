# FPGA CI Test Shield Terminal

This program is for initial hardware bring-up and verification of the FPGA CI Test Shield. It provides a terminal at 9600 baud. Within this terminal there are functions for controlling FPGA pins, running FPGA self tests, and applying updates.

## Required hardware

The hardware needed for this repository is as follows:
 - K64F-FRDM board
 - SD card
 - FPGA CI Test Shield

## Commands

The supported commands can be found by running the `help` command on the serial terminal:

```
Supported commands:
  help - Print this help info

  set_control_pins <CLK> <MOSI> <MISO> <AUX> - Set the pins used to control the FPGA
    By default the FPGA control pins are chosen automatically. This
    commands allows the control pins to be set explicitly. When pins are
    manually set care needs to be taken to ensure that they are not used for
    other purposes, such as gpio_read or gpio_write. The control pins have the
    following restrictions:
    - All pins must be in the current form factor
    - CLK must be on an even pin index
    - MOSI must be the index of CLK + 1
    - MISO can be on any remaining pin
    - AUX can be on any remaining pin
    Example: "set_control_pins 0 1 2 3"
    Example: "set_control_pins 4 5 1 0"

  set_control_auto - Let the control pins be moved automatically.
    This is the default mode.

  gpio_read <GPIO pin> - Read the value on a GPIO pin.
    Allowed values for <logical pin> are 0 to 127
    Example: "gpio_read 27"
    Note: This may change the pins used to control the FPGA if they conflict

  gpio_write <GPIO pin> <value> - Write the value on a GPIO pin and set it as output.
    Allowed values for <logical pin> are 0 to 127
    Allowed values for <value> are 0 and 1
    Example: "gpio_write 27, 1"
    Example: "gpio_write 27, 0"
    Note: This may change the pins used to control the FPGA if they conflict
    Note: This pin will return to tristate as soon as gpio_read or gpio_write is performed on a separate pin.

  sys_read <system pin> - Read the value on a system pin.
    Allowed values for <system pin> are 0 to 40
    Example: "sys_read 2"

  sys_write <system pin> <value> - Write the value on a GPIO pin and set it as output.
    Allowed values for <system pin> are 0 to 40
    Allowed values for <value> are 0 and 1
    Example: "sys_write 5 1"
    Example: "sys_write 5 0"

  sys_tristate <system pin> - Pin to tristate.
    Allowed values for <system pin> are 0 to 40
    Example: "sys_tristate 2"

  set_pull <pin_index> <mode> - Set pin to a pull mode.
    Allowed values for <pin_index> are 0 to 127
    Allowed values for <mode> are PULLDOWN, PULLUP, or TRISTATE
    Example: "set_pull 2 PULLUP"

  io_exp_read <pin_index> <reg> - Read a bit for a specific Mbed pin that is set in the input, output, or configuration registers inside of the IO expander.
    Allowed values for <pin_index> are 0 to 127
    Allowed values for <reg> are INPUT, OUTPUT, or CONFIG
    Example: "io_exp_read 2 OUTPUT"

  set_mux_en_and_addr <enable> <pin_index> - Set the analog MUX enable and address.
    Allowed values for <enable> are 0 or 1
    Allowed values for <pin_index> are 0 to 127
    Example: "set_mux_en_and_addr 1 25"

  set_pwm <enable> <period> <cycles_high> - Set the pwm to be enabled/disabled, period in clk cycles, and cycles_high in clk cycles.
    Allowed values for <enable> are 0 or 1
    Allowed values for <period> are any value in clk cycles >= 0
    Allowed values for <cycles_high> are 0 to period (clk cycles)
    Example: "set_pwm 1 100 75" (enable pwm at 100 clk cycle period and 75%% duty cycle)

  set_analog_out <enable> <voltage> - Create an analog voltage via the FPGA sys pwm.
    Allowed values for <enable> are 0 or 1
    Allowed values for <voltage> 0.0 to 1.0 (float)
    Example: "set_analog_out 1 0.75" (create a voltage of (0.75 * 3.3V) = 2.475V)

  io_exp_test Test the IO expander, all 3 pull states for all pins.
    Example: "io_exp_test"

  mux_test Test the analog MUX, all pins, both directions.
    Example: "mux_test"

  sys_pins - Display the system pins.

  mbed_pins - Display the Mbed pins.

  update_direct <filename> - Directly program a serial flash part from the SD card.
    The serial flash must have the following connections:
      D10 - Chip Select
      D11 - MOSI
      D12 - MISO
      D13 - CLOCK

  dump_direct <filename> - Directly dump the serial flash contents to the SD card.
    The serial flash must have the following connections:
      D10 - Chip Select
      D11 - MOSI
      D12 - MISO
      D13 - CLOCK

  update <filename> - Start a firmware update from the SD card.

  dump <filename> - Dump FPGA firmware to an SD card.

  reprogram - Trigger the FPGA to reprogram its image from flash
```

## License and contributions

The software is provided under the [Apache-2.0 license](https://github.com/ARMmbed/mbed-os/blob/master/LICENSE-apache-2.0.txt). Contributions to this project are accepted under the same license. Please see [contributing.md](https://github.com/ARMmbed/mbed-os/blob/master/CONTRIBUTING.md) for more information.

This project contains code from other projects. The original license text is included in those source files. They must comply with our [license guide](https://os.mbed.com/docs/mbed-os/latest/contributing/license.html).

Folders containing files under different permissive license than Apache 2.0 are listed in the [LICENSE](https://github.com/ARMmbed/mbed-os/blob/master/LICENSE.md) file.
