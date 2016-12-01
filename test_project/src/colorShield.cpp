#include "mbed.h"
#include "colorShield.h"

BusOut LINE(D4,D8,D9,D10,D11,D12,D13,D3);
// BusOut LINE(D3,D4,D8,D9,D10,D11,D12,D13);
// BusOut LINE(D8,D9,D10,D11,D12,D13,D3,D4);
DigitalOut RST_LED(PA_4);
DigitalOut LAT_LED(PA_1);
DigitalOut SLB_LED(PA_0);
DigitalOut SDA_LED(D7);
DigitalOut SCL_LED(D6);

void Color_shield::init(){
  SCL_LED = 1;
  RST_LED = 1;
  RST_LED =0;
  RST_LED =1;
  LAT_LED = 0;
  SLB_LED  =0;
  LINE = 1;
  for (int i =0;i<8;i++){
    for(int j =0;j<3;j++){
      for(int k=0;k<6;k++){
        SDA_LED = 1;
        SCL_LED = 0;
        wait_us(0.1);
        SCL_LED = 1;
        }
      }
      LINE = LINE*2;

    }

  LAT_LED = 1;
  LAT_LED = 0;
  wait(1);
}

void Color_shield::display(uint8_t *color[8][8],int multiplier){
  LAT_LED = 0;
  SLB_LED = 1;
  int counter = 1;
  uint8_t c;
  for (int l = 0; l<8 ; l++){
    LINE = counter;
    counter *= 2;
    for(int i = 0; i<8;i++){
      for(int j = 0; j <3; j++){
        c = color[l][i][j];
        c *= multiplier;
        for(int k = 0; k <8; k++){
          if (c&0x80){
            SDA_LED = 1;
          }
          else{
            SDA_LED = 0;
          }
          c= c<<1;
          SCL_LED = 0;
          wait_us(0.1);
          SCL_LED = 1;

        }
      }
    }
    LAT_LED = 1;
    LAT_LED = 0;

  }
}
void Color_shield::display(uint8_t color[8][8][3],int multiplier){
  LAT_LED = 0;
  SLB_LED = 1;
  int counter = 1;
  uint8_t c;
  for (int l = 0; l<8 ; l++){
    LINE = counter;
    counter *= 2;
    for(int i = 0; i<8;i++){
      for(int j = 0; j <3; j++){
        c = color[l][i][j];
        c *= multiplier;
        for(int k = 0; k <8; k++){
          if (c&0x80){
            SDA_LED = 1;
          }
          else{
            SDA_LED = 0;
          }
          c= c<<1;
          SCL_LED = 0;
          wait_us(0.1);
          SCL_LED = 1;

        }
      }
    }
    LAT_LED = 1;
    LAT_LED = 0;
    // LINE = 0;
  }
}
