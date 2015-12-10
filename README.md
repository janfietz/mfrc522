# mfrc522
Chibios high level driver for mfrc522 RFID

The driver is a rework of:
http://stm32f4-discovery.com/2014/07/library-23-read-rfid-tag-mfrc522-stm32f4xx-devices/

Only reading of card uid is tested and used for now.

Here a short sample for stm32f4 discovery board.
```C
#include "ch.h"
#include "hal.h"

#include "mfrc522.h"

#define LED_ORANGE GPIOD_LED3
#define LED_GREEN GPIOD_LED4


MFRC522Driver RFID1;

static uint8_t txbuf[2];
static uint8_t rxbuf[2];

static MUTEX_DECL(CardIDMutex);
static struct MifareUID CardID;

/*
 * RFID1 configuration structure.
 */
static MFRC522Config RFID1_cfg = {
};

/*
 * SPI1 configuration structure.
 */
static const SPIConfig SPI1cfg = {
  NULL,
  /* HW dependent part.*/
  GPIOC,
  GPIOC_PIN5,
  SPI_CR1_BR_0 | SPI_CR1_BR_1
};

/*
* Implement functions to write and read values using spi driver.
*/
void MFRC522WriteRegister(MFRC522Driver* mfrc522p, uint8_t addr, uint8_t val)
{
    (void)mfrc522p;
    spiSelect(&SPID1);
    txbuf[0] = (addr << 1) & 0x7E;
    txbuf[1] = val;
    spiSend(&SPID1, 2, txbuf);
    spiUnselect(&SPID1);
}

uint8_t MFRC522ReadRegister(MFRC522Driver* mfrc522p, uint8_t addr)
{
    (void)mfrc522p;
    spiSelect(&SPID1);
    txbuf[0] = ((addr << 1) & 0x7E) | 0x80;
    txbuf[1] = 0xff;
    spiExchange(&SPID1, 2, txbuf, rxbuf);
    spiUnselect(&SPID1);
    return rxbuf[1];
}

/*
 * This is a periodic thread that reads uid from rfid periphal
 */
static THD_WORKING_AREA(waThread1, 256);
static THD_FUNCTION(Thread1, arg) {

  systime_t time;

  (void)arg;
  chRegSetThreadName("reader");

  /* Reader thread loop.*/
  time = chVTGetSystemTime();
  bool active = false;
  palClearPad(GPIOD, LED_ORANGE);

  while (true) {
      if (active == true)
      {
          palClearPad(GPIOD, LED_ORANGE);
      }
      else
      {
          palSetPad(GPIOD, LED_ORANGE);
      }
      active = !active;

      chMtxLock(&CardIDMutex);
      if (MifareCheck(&RFID1, &CardID) == MIFARE_OK) {
          palSetPad(GPIOD, LED_GREEN);
      } else {
          palClearPad(GPIOD, LED_GREEN);
          CardID.size = 0;
      }
      chMtxUnlock(&CardIDMutex);


    /* Waiting until the next 250 milliseconds time interval.*/
    chThdSleepUntil(time += MS2ST(100));
  }
}

/*
 * Application entry point.
 */
int main(void)
{
    /*
     * System initializations.
     * - HAL initialization, this also initializes the configured device drivers
     *   and performs the board-specific initializations.
     * - Kernel initialization, the main() function becomes a thread and the
     *   RTOS is active.
     */
    halInit();
    chSysInit();

    /*
    * Init drivers
    */
    palSetPadMode(GPIOC, GPIOC_PIN5, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);
    MFRC522ObjectInit(&RFID1);

    /*
    * Start drivers
    */
    spiStart(&SPID1, &SPI1cfg);
    MFRC522Start(&RFID1, &RFID1_cfg);

    /*
    * Creates rfid reader.
    */
    chThdCreateStatic(waThread1, sizeof(waThread1), LOWPRIO, Thread1, NULL);

    while (TRUE)
    {
        chThdSleepMilliseconds(500);
    }
}
```
A further example provides my repository https://github.com/janfietz/toddlerMusicBoxControl.
