#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <bcm2835.h>

#include <wiringPi.h>

#include <cc3k.h>

// CS, EN, IRQ
// 17, 27, 22  (BCM Pins)
// 0, 2, 3
#define CS_PIN 0
#define EN_PIN 2
#define IRQ_PIN 3

cc3k_t driver;
cc3k_config_t config;

void _delay(uint32_t us)
{
  usleep(us);
}

void _enable(int enable)
{
  fprintf(stderr, "Chip enable %d\n", enable);
  if(enable)
    digitalWrite(EN_PIN, HIGH);
  else
    digitalWrite(EN_PIN, LOW);
}

int _read_int()
{
  return digitalRead(IRQ_PIN);  
}

void _enable_int(int enable)
{
  fprintf(stderr, "Int enable %d\n", enable);  
}

void _assert_cs(int assert)
{
  fprintf(stderr, "CS assert %d\n", assert);
  if(assert)
    digitalWrite(CS_PIN, LOW);
  else
    digitalWrite(CS_PIN, HIGH);
}

void _spi(uint8_t *out, uint8_t *in, uint16_t length, int async)
{
  int i;

  fprintf(stderr, "SPI %d\n", length);

  bcm2835_spi_transfernb(out, in, length);
  
  for(i=0;i<length;i++)
    fprintf(stderr, "%02X ", out[i]);
  fprintf(stderr, "\n");

  for(i=0;i<length;i++)
    fprintf(stderr, "%02X ", in[i]);
  fprintf(stderr, "\n");

  cc3k_spi_done(&driver);
}

void _transition(cc3k_state_t from, cc3k_state_t to)
{
  fprintf(stderr, "Transition %d -> %d\n", from, to);
}

void setup_driver()
{
  config.delayMicroseconds = _delay;
  config.enableChip = _enable;
  config.readInterrupt = _read_int;
  config.enableInterrupt = _enable_int;
  config.assertChipSelect = _assert_cs;
  config.spiTransaction = _spi;
  config.transitionCallback = _transition;
}

void _isr(void)
{
  fprintf(stderr, "Interrupt!\n");
  cc3k_interrupt(&driver);
}

int setup_spi()
{
  int fd;

  pinMode(CS_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(IRQ_PIN, INPUT);

  wiringPiISR(IRQ_PIN, INT_EDGE_FALLING, _isr);

  bcm2835_spi_begin();
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64);
  bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
  bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);

  return fd;
}

int main(int argc, char **argv)
{
  uint32_t ms = 0;
  wiringPiSetup();

  if (!bcm2835_init())
  {
    fprintf(stderr, "Failed to init libbcm2835\n");
    return EXIT_FAILURE;
  }

  if(setup_spi() < 0)
  {
    fprintf(stderr, "Failed to initialize SPI\n");
    return EXIT_FAILURE;
  }

  setup_driver();

  cc3k_init(&driver, &config);

  while(1)
  {
    cc3k_loop(&driver, ms);
    usleep(1000);
    ms++;
  }

  return EXIT_SUCCESS;
}
