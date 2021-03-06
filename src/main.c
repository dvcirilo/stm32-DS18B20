#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/f1/nvic.h>
#include <stdio.h>
#include <errno.h>

// Must be declared BEFORE include "OneWire.h" so that an appropriate interrupt handler is added
//#define ONEWIRE_UART5
//#define ONEWIRE_UART4
#define ONEWIRE_USART3
//#define ONEWIRE_USART2
//#define ONEWIRE_USART1

//#define MAXDEVICES_ON_THE_BUS 3

#include "OneWire.h"

#define USART_CONSOLE USART2

int _write(int file, char *ptr, int len);

/* STM32 в 72 MHz. */
static void clock_setup(void) {
  rcc_clock_setup_in_hse_8mhz_out_72mhz();

  /* Enable GPIOB, GPIOC, and AFIO clocks. */
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);

  rcc_periph_clock_enable(RCC_AFIO);

  /* Enable clocks for USARTs. */
  rcc_periph_clock_enable(RCC_USART2); // enable if debugging is used
  rcc_periph_clock_enable(RCC_USART3);
}

void usart3_isr() {
  owReadHandler(USART3);
}

int _write(int file, char *ptr, int len) {
  int i;

  if (file == 1) {
    for (i = 0; i < len; i++)
      usart_send_blocking(USART_CONSOLE, ptr[i]);
    return i;
  }
  errno = EIO;
  return -1;
}


static void gpio_setup(void) {
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX | GPIO_USART2_RX);

  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_10_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_USART3_TX | GPIO_USART3_RX);

  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

  AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_FULL_SWJ_NO_JNTRST;

  /* Preconf USART2 for output*/
// Set up USART for debugging
  usart_set_baudrate(USART_CONSOLE, 115200);
  usart_set_databits(USART_CONSOLE, 8);
  usart_set_stopbits(USART_CONSOLE, USART_STOPBITS_1);
  usart_set_mode(USART_CONSOLE, USART_MODE_TX_RX);
  usart_set_parity(USART_CONSOLE, USART_PARITY_NONE);
  usart_set_flow_control(USART_CONSOLE, USART_FLOWCONTROL_NONE);
  usart_enable(USART_CONSOLE);

  /* Preconf LED. */
  gpio_clear(GPIOC, GPIO13);
}

OneWire ow;

int main(void) {

  clock_setup();
  gpio_setup();

  ow.usart = USART3;

  uint32_t pDelay = 300, i;

  while (1) {
    if (owResetCmd(&ow) != ONEWIRE_NOBODY) {    // is anybody on the bus?
      int devices = owSearchCmd(&ow); // get the ROMid of all devices on the bus or return an error code
      if (devices <= 0) {
        printf("\n\rError has happened!");
        pDelay = 8000000;
        gpio_toggle(GPIOC, GPIO13);    /* LED on/off */
        for (i = 0; i < pDelay * 4; i++)    /* Wait a bit. */
            __asm__("nop");
        continue;
      }
      printf("\n\rfound %d devices on 1-wire bus", devices);
      if (devices < 1)
        continue; // suddenly something could "come off" from the sensors
      i = 0;
      for (; i < devices; i++) {
        RomCode *r = &ow.ids[i];
        Temperature t;
        uint8_t crc = owCRC8(r);
        char *crcOK = (crc == r->crc)?"CRC OK":"CRC ERROR!";
        printf("\n\rdevice %d (SN: %02X/%02X%02X%02X%02X%02X%02X/%02X) ", i, r->family, r->code[5], r->code[4], r->code[3],
               r->code[2], r->code[1], r->code[0], r->crc);
        printf(crcOK);
        if (crc != r->crc) {
          printf("\n\r can't read cause CRC error");
          continue;
        }
        switch (r->family) {
          case DS18B20:
        // the value of the previous dimension will be returned!
            t = readTemperature(&ow, &ow.ids[i], true);
            printf("\n\rDS18B20 , Temp: %3d.%dC", t.inCelsus, t.frac);
            break;
          case DS18S20:
            t = readTemperature(&ow, &ow.ids[i], true);
            printf("\n\rDS18S20 , Temp: %3d.%dC", t.inCelsus, t.frac);
            break;
          case 0x00:
            break;
          default:
            printf("\n\rUNKNOWN Family:%x (SN: %x%x%x%x%x%x)", r->family, r->code[0], r->code[1], r->code[2],
                   r->code[3], r->code[4], r->code[5]);
            break;
        }
        pDelay = 8000000;
      }
      printf("\n\r...");
    } else {
      printf("there is no device on the bus");
      pDelay = 8000000;
    }
    gpio_toggle(GPIOC, GPIO13);    /* LED on/off */
    for (i = 0; i < pDelay * 4; i++)    /* Wait a bit. */
        __asm__("nop");
  }

  return 0;
}
