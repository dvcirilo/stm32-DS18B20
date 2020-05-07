# DS18B20 libopencm3 OneWire library for STM32

The verification was carried out on STM32F103C8T6, however, the list of
supported devices should be much wider. No code specific to
STM32F1 is used.

## Description of the library

The library is designed to work with the widespread Maxim (Dallas) DS18B20
temperature sensors according to the OneWire protocol.

The library [libopencm3](https://libopencm3.github.io/) was chosen as the basis
for the implementation.  libopencm3 is a very good basis for STM32 programming,
however, due to poor documentation it is slightly underrated by hobbyists. The
resulting code is easy to read, compact, and efficient compared to SPL and HAL
from ST Microelectronics.

The OneWire library implements SKIP, SEARCH, MATCH, READ, READ SCRATCHPAD,
CONVERT TEMPERATURE, RECALL E2 and other commands and works on the
STM32F103C8T6 and, most likely, other microcontrollers.

The library involves connecting sensors (up to 75 on one line) to any of the TX
USART / UART outputs. Presumed Availability external power supply of sensors
and pull-up of a DATA line to a power line of resistance with a rating
corresponding to the task (default 4.7K). The USART / UART operating mode in
this case is half duplex. The developer has at his disposal free (e) gpio.

It assumes [using hardware USART/UART]. The library should work well as part
of RTOS, because, in fact, all critical operations are made atomic.

The library allows you to organize 5 independent OneWire buses (by the number
of USART/UART hardware support). The number of devices on the bus is
determined by the developer. In default mode, up to 5 devices are expected on
the bus.

The project is built on Clion on any OS (Mac OS X, Windows, Linux) +
arm-none-eabi + cmake. Debugging by blackmagic probe and .gdbinit corresponds
to the initialization of the connection process.

## A bit about the sensor

The DS18B20 sensor, as well as the DS18S20, belongs to the class of devices,
the purpose and operation of which requires explanation.

Firstly, these sensors have their own memory, settings and logic. Their
interaction with µC is based on possible interaction protocols.  In the process
of preparing new data, the sensor consumes quite a lot of electricity (up to
1.5mA) and therefore does not measure continuously.  Therefore, the last
measurement he made does not mean "current temperature". It is reasonable to
assume that while the sensor has been sent to temperature measurement, the
microcontroller should be busy with something useful (or sleep), instead of
just waiting for the sensor to be ready.

Secondly, the sensor can do quite a lot in addition to measuring temperature.
So, for example, the sensor can set the boundaries of "alarm" and in response
to a special request to report that the temperature has exceeded these limits.
This may be useful, for example, for such a scenario: placing a group of
sensors across the territory (for example, along the wall), simultaneously
sending a request for temperature measurement, and then, commands to
determination of those sensors whose measured temperature is beyond the
previously established limits. Accordingly, the survey then not all the
sensors, but only those who formed the "alarm".

In addition, these two sensor bytes (setting alarm limits) can be used
arbitrarily to store some information that can be stored.  in the EEPROM sensor
and be used later.

It is clear that these sensors are designed for very remote placement from uK.
Its distance can be tens of meters.  (really up to 30 meters).


## Program structure

Suppose that the OneWire bus will be implemented on the USART3 blue tablet
(bluepile) STM32F103C8T6, a very cheap and affordable experimental
board.  The library uses the appropriate interrupt(s) to organize the reading
of the RX at the time of transmission to the line via TX. Reminds creation
loopback on UART.

Therefore, declare:

```C
// Must be declared BEFORE include "OneWire.h" so that an appropriate
// interrupt handler is added
// # define ONEWIRE_UART5
// # define ONEWIRE_UART4
#define ONEWIRE_USART3
// # define ONEWIRE_USART2
// # define ONEWIRE_USART1

// maximum number of devices on the bus by default
// MAXDEVICES_ON_THE_BUS 5

#include "OneWire.h"
```

In order to use the library, you should initialize the "clock" in `static void
clock_setup (void)`:

```C
    rcc_periph_clock_enable (RCC_GPIOB);
```

After, configure gpio in `static void gpio_setup (void)`:

```C
    gpio_set_mode (GPIOB, GPIO_MODE_OUTPUT_10_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_USART3_TX | GPIO_USART3_RX);

```

And declare (local or global - as convenient) a variable that will store
information about devices on the bus:

```C
OneWire ow;
```

If you want to increase/decrease the number of devices about which default
information will be saved

[using hardware USART/UART]: (https://www.maximintegrated.com/en/app-notes/index.mvp/id/214)
