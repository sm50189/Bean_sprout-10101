#ifndef SLAVE_H
#define SLAVE_H

#include "mbed.h"

#define SLAVE_MBED_ADDRESS 0x70

I2CSlave slave(D14, D15);


// slave.address(0x70);

void get_recieve(int recieve,char *msg,char buf[],int NbyteRead,int NbyteWrite){
  int status;
  switch (recieve) {
    case I2CSlave::ReadAddressed:
      status = slave.write(msg,1);
      slave.stop();
      printf("write :0x%x\n", msg[0]);
      //if(msg[0]>0x02){wait(0.01);}
      if(status == 0){msg[0] = 0x00;}
      break;
    case I2CSlave::WriteAddressed:

      slave.read(buf,NbyteRead);
      slave.stop();
      // printf("read\n");
      printf("read :0x%x 0%x\n",buf[0],buf[1]);
      break;
    case I2CSlave::WriteGeneral:
      slave.read(buf, NbyteRead);
      slave.stop();
      wait_ms(5);
      //printf("Read G: %s\r\n", buf);
      break;
    case I2CSlave::NoData:
      //printf("Nodata\n");
      slave.stop();
      break;
    default:
    slave.stop();
  }
}
#endif
