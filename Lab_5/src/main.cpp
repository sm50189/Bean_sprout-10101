#include "mbed.h"
#include "eeprom.h"

EEPROM memory(I2C_SDA,I2C_SCL,0);

DigitalIn wp(A0);

BusIn swIn(D2,D3,D4,D5,D6,D7,D8);

BusOut ledOut(D9,D10,D11,D12,D13,A5,A4);

InterruptIn Save(A2);

Serial pc(PA_11,PA_12);

int Input;

int8_t data[1];

void Save_data(){
  if (wp == 0){
    Input = swIn.read();
    memory.write(2,(int8_t)Input);
    wait_ms(1);
    pc.printf("I wrote %d ", Input);
    memory.read(2,data,1);
    wait_ms(1);
    pc.printf("and it is %d\n",data[0]);
    ledOut = 0;
  }
  else if(wp==1){
    memory.read(2,data,1);
    wait_ms(1);
    pc.printf("It is %d\n",data[0]);
    ledOut = data[0];
    // ledOut = 45;
  }
}

int main(int argc, char const *argv[]) {
  Save.rise(&Save_data);
  while (1);
}
// pc.printf("I am fucking running\n");
// if (wp == 0){
//   Input = swIn;
// }
// return 0;
