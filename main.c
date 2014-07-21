#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>

#include <bcm2835.h>

#include <wiringPi.h>

#include <cc3k.h>

#define LOG(format, ...) { \
    fprintf(stderr, "[%lf] " format, time_now(), ##__VA_ARGS__); \
  }

// CS, EN, IRQ
// 17, 27, 22  (BCM Pins)
// 0, 2, 3
#define CS_PIN 0
//#define EN_PIN 2
//#define CS_PIN 17
#define EN_PIN 27
#define IRQ_PIN 3

cc3k_t driver;
cc3k_config_t config;

cc3k_socket_t client;

int int_en = 0;
volatile int int_pending = 0;

pthread_mutex_t lock;

double time_now()
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);

  return ((double)(ts).tv_sec + ((double)(ts).tv_nsec * 1e-9));
}

void _delay(uint32_t us)
{
  usleep(us);
}

void _enable(int enable)
{
  LOG("Chip enable %d\n", enable);
  if(enable)
    bcm2835_gpio_set(EN_PIN);
  else
    bcm2835_gpio_clr(EN_PIN);
}

int _read_int()
{
  return digitalRead(IRQ_PIN);  
}

void _enable_int(int enable)
{
  //LOG("Int enable %d\n", enable);  
  int_en = enable;

  if(enable && int_pending)
  {
    cc3k_interrupt(&driver);
    int_pending = 0;
  }
}

void _assert_cs(int assert)
{
  //LOG("CS assert %d\n", assert);
  if(assert)
    digitalWrite(CS_PIN, LOW);
  else
    digitalWrite(CS_PIN, HIGH);
/*
  if(assert)
    bcm2835_gpio_clr(CS_PIN);
  else
    bcm2835_gpio_set(CS_PIN);
*/
}

void _spi(uint8_t *out, uint8_t *in, uint16_t length, int async)
{
  int i;

  //LOG("SPI %d\n", length);

  //bcm2835_spi_transfernb(out, in, length);
  spi_transfer(out, in, length);
 
/* 
  for(i=0;i<length;i++)
    fprintf(stderr, "%02X ", out[i]);
  fprintf(stderr, "\n");

  for(i=0;i<length;i++)
    fprintf(stderr, "%02X ", in[i]);
  fprintf(stderr, "\n");
*/

  // The SPI transaction on the pi is synchronous, so notifiy the cc3k driver we are done
  cc3k_spi_done(&driver);
}

void _transition(cc3k_state_t from, cc3k_state_t to)
{
  //LOG("Transition %d -> %d\n", from, to);
}

void _command(uint16_t opcode, uint8_t *data, uint16_t length)
{
  //LOG("Command 0x%04X\n", opcode);
}

void _event(uint16_t opcode, uint8_t *data, uint16_t length)
{
  //LOG("Event 0x%04X\n", opcode);
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
  config.commandCallback = _command;
  config.eventCallback = _event;
}

void _isr(void)
{
 
  pthread_mutex_lock(&lock); 
  if(int_en)
  {
    //LOG("Interrupt!\n");
    cc3k_interrupt(&driver);
  }
  else
  {
    LOG("MASKED Interrupt!\n");
    //int_pending = 1;
  }
  pthread_mutex_unlock(&lock);
    
}

int setup_spi()
{
  pinMode(CS_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(IRQ_PIN, INPUT);

  wiringPiISR(IRQ_PIN, INT_EDGE_FALLING, _isr);

  SpiOpenPort();

/*
  bcm2835_spi_begin();
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_512);
  bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
*/
  //bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);

  return 0;
}

int setup_client()
{
  // Setup the client socket
  bzero(&client, sizeof(cc3k_socket_t));
  client.family = AF_INET;
  client.type = SOCK_STREAM;
  client.protocol = IPPROTO_TCP;

  client.sockaddr.family = AF_INET;
  client.sockaddr.port = 0xAAAA; // Port 43690
  // 10.78.100.173
  client.sockaddr.addr = 0xAD644E0A;

  cc3k_socket_add(&driver, &client);
}

int main(int argc, char **argv)
{
  uint32_t ms = 0;
  int init_done = 0;

  pthread_mutex_init(&lock, NULL);

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

  piHiPri(99);

  setup_driver();

  cc3k_init(&driver, &config);
  setup_client();

  while(1)
  {
    pthread_mutex_lock(&lock);
    cc3k_loop(&driver, ms);
    pthread_mutex_unlock(&lock);

    if(driver.dhcp_complete == 1 && init_done == 0)
    {
      uint8_t *a;
      LOG("Got DHCP\n");

      a = (uint8_t *)&driver.ipconfig.ip;
      LOG("IP Address: %d.%d.%d.%d\n",
        a[3], a[2], a[1], a[0]
        );

      init_done = 1;
    }

    usleep(1000);
    ms++;
  }

  return EXIT_SUCCESS;
}
