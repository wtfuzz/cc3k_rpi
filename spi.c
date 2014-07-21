#include <fcntl.h>        //Needed for SPI port
#include <sys/ioctl.h>      //Needed for SPI port
#include <linux/spi/spidev.h> //Needed for SPI port
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>


int spi_cs0_fd;       //file descriptor for the SPI device
int spi_cs1_fd;       //file descriptor for the SPI device
unsigned char wr_mode;
unsigned char rd_mode;
unsigned char spi_bitsPerWord;
unsigned int spi_speed;




//***********************************
//***********************************
//********** SPI OPEN PORT **********
//***********************************
//***********************************
//spi_device  0=CS0, 1=CS1
int SpiOpenPort (void)
{
  int status_value = -1;
    int *spi_cs_fd;


    //----- SET SPI MODE -----
    //SPI_MODE_0 (0,0)  CPOL=0 (Clock Idle low level), CPHA=0 (SDO transmit/change edge active to idle)
    //SPI_MODE_1 (0,1)  CPOL=0 (Clock Idle low level), CPHA=1 (SDO transmit/change edge idle to active)
    //SPI_MODE_2 (1,0)  CPOL=1 (Clock Idle high level), CPHA=0 (SDO transmit/change edge active to idle)
    //SPI_MODE_3 (1,1)  CPOL=1 (Clock Idle high level), CPHA=1 (SDO transmit/change edge idle to active)
    wr_mode = SPI_MODE_1;
    rd_mode = SPI_MODE_1;
    
    //----- SET BITS PER WORD -----
    spi_bitsPerWord = 8;
    
    //----- SET SPI BUS SPEED -----
    spi_speed = 8000000;    //1000000 = 1MHz (1uS per bit) 


    spi_cs_fd = &spi_cs0_fd;

    *spi_cs_fd = open("/dev/spidev0.0", O_RDWR);

    if (*spi_cs_fd < 0)
    {
        perror("Error - Could not open SPI device");
        exit(1);
    }

    status_value = ioctl(*spi_cs_fd, SPI_IOC_WR_MODE, &wr_mode);
    if(status_value < 0)
    {
        perror("Could not set SPIMode (WR)...ioctl fail");
        exit(1);
    }

    status_value = ioctl(*spi_cs_fd, SPI_IOC_RD_MODE, &rd_mode);
    if(status_value < 0)
    {
      perror("Could not set SPIMode (RD)...ioctl fail");
      exit(1);
    }

    status_value = ioctl(*spi_cs_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bitsPerWord);
    if(status_value < 0)
    {
      perror("Could not set SPI bitsPerWord (WR)...ioctl fail");
      exit(1);
    }

    status_value = ioctl(*spi_cs_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bitsPerWord);
    if(status_value < 0)
    {
      perror("Could not set SPI bitsPerWord(RD)...ioctl fail");
      exit(1);
    }

    status_value = ioctl(*spi_cs_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
    if(status_value < 0)
    {
      perror("Could not set SPI speed (WR)...ioctl fail");
      exit(1);
    }

    status_value = ioctl(*spi_cs_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
    if(status_value < 0)
    {
      perror("Could not set SPI speed (RD)...ioctl fail");
      exit(1);
    }
    return(status_value);
}



//************************************
//************************************
//********** SPI CLOSE PORT **********
//************************************
//************************************
int SpiClosePort (int spi_device)
{
  int status_value = -1;
    int *spi_cs_fd;

    if (spi_device)
      spi_cs_fd = &spi_cs1_fd;
    else
      spi_cs_fd = &spi_cs0_fd;


    status_value = close(*spi_cs_fd);
    if(status_value < 0)
    {
      perror("Error - Could not close SPI device");
      exit(1);
    }
    return(status_value);
}

int spi_transfer(uint8_t *out, uint8_t *in, uint16_t length)
{
  struct spi_ioc_transfer spi;
  int i = 0;
  int retVal = -1;
  int *spi_cs_fd;

  spi.tx_buf        = (void *)out; // transmit from "data"
  spi.rx_buf        = (void *)in; // receive into "data"
  spi.len           = length ;
  spi.delay_usecs   = 0 ;
  spi.speed_hz      = spi_speed ;
  spi.bits_per_word = spi_bitsPerWord ;
  spi.cs_change = 1;

  retVal = ioctl(spi_cs0_fd, SPI_IOC_MESSAGE(1), &spi) ;

  if(retVal < 0)
  {
    perror("Error - Problem transmitting spi data..ioctl");
    exit(1);
  }

  return retVal;
}
