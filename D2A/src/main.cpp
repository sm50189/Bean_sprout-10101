#include "mbed.h"
#include "MCP4922.h"

MCP4922 MCP(SPI_MOSI, SPI_SCK,D10);  // MOSI, SCLK, CS
Timer t;

Serial pc(D1,D0);

uint16_t f(uint16_t ms){
  uint16_t y;
  if (0<ms && ms<17){
    y = (2048/17)*ms;
  }

  else if (17<=ms && ms<34){
    y = 4096-(2048/17)*ms;
  }

  else if (34<=ms && ms<51){
    //(y-2048) = 4096-(2048/17)*(ms-17);
    y = 8192 - (2048/17)*ms;
  }

  else if (51<=ms && ms<68){
      // (y-2048) = (2048/17)(ms-51);
      y = (2048/17)*ms - 4096;
  }
  return y;
}

/**** Main Function  ***/
int main(void)
{
    MCP.frequency(1000000);

    uint16_t i=0;
    uint8_t state='c';

    t.start();
    uint16_t begin = t.read_ms();
    uint16_t now = t.read_ms();
    uint16_t different;

    while(1){
      now = t.read_ms();
      if (now-begin>67){
        t.reset();
        begin = t.read_ms();
        now = t.read_ms();
      }
      // pc.printf("%d\n",now-begin );
      i = f(now-begin);
      MCP.writeA(i);
      // wait_us(1);

    }

}
