#include "mbed.h"
#include "colorShield.h"
// #include "selfmpu.h"
#include "MPU9250.h"
#include "slave.h"

Timer t;
Timer T;

int timeout = 120;
bool is_win = false;

uint8_t wall[3] = {25,25,0};
uint8_t path[3] = {0,0,0};
uint8_t character[3] = {255,255,255};
uint8_t end_item[3] = {0,255,0};
uint8_t bad_item[3] = {0,0,255};
uint8_t good_item[3] = {255,0,255};
Serial pc(D1,D0);
Serial bt(PA_15,PB_7);
Color_shield color_s;
// MPU9555 mpu;
SPI spi(PB_15,PB_14,PB_13);
mpu9250_spi mpu(spi,PC_4);
bool is_revert = false;



float gyroBias[3],accBias[3];

int frameCountx = 0;
int frameCounty = 0;
int centerNow[2] = {3,3};
int special_item[2];
int increase_time[2];
int revert_item_pos[2];
int previousCenter[2] = {3,3};
int mapsize[2] = {48,48};

uint8_t *output[8][8];

uint8_t *Bigmap1[48][48] = {{wall,path,path,wall,wall,wall,wall,wall,path,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,wall,path,path,path,path,path,path,path,path},
                            {wall,path,path,wall,path,path,path,path,path,path,wall,path,path,path,path,path,path,path,path,path,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,path,wall,path,path,path,path,path,path,path,path},
                            {wall,path,path,wall,path,path,path,path,path,path,wall,path,path,wall,wall,wall,wall,wall,wall,wall,path,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,path,wall,path,wall,wall,wall},
                            {wall,path,path,wall,path,path,wall,wall,path,path,wall,path,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,path,wall,path,wall,path,path},
                            {wall,path,path,wall,path,path,wall,wall,path,path,wall,path,path,path,path,path,path,path,path,path,path,wall,path,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,path,wall,path,wall,wall,wall,path,path,path,wall,path,wall,path,path},
                            {wall,path,path,wall,path,path,wall,wall,path,path,wall,path,path,path,path,path,path,path,path,path,path,wall,path,path,wall,path,path,path,path,path,path,path,wall,path,path,wall,path,wall,wall,wall,path,path,path,wall,path,path,path,path},
                            {wall,path,path,wall,path,path,wall,wall,path,path,wall,path,path,wall,wall,path,wall,path,path,wall,path,wall,path,path,wall,path,wall,wall,wall,wall,path,path,wall,path,path,wall,path,wall,wall,wall,path,path,path,wall,wall,wall,wall,wall},
                            {path,path,path,wall,path,path,wall,wall,path,path,wall,path,path,wall,wall,path,wall,path,path,wall,path,wall,path,path,wall,path,wall,wall,wall,wall,path,path,wall,path,path,wall,path,wall,wall,path,path,path,path,path,path,path,path,path},
                            {path,path,path,wall,path,path,wall,wall,path,path,wall,path,path,wall,wall,path,wall,path,path,wall,path,wall,path,path,wall,wall,wall,wall,wall,wall,path,path,wall,path,path,wall,path,wall,wall,path,path,path,path,path,path,path,path,path},
                            {wall,wall,wall,wall,path,path,wall,wall,path,path,wall,path,path,wall,wall,path,wall,path,path,wall,path,wall,path,path,path,path,path,path,path,path,path,path,wall,path,path,wall,path,wall,wall,path,path,path,wall,wall,wall,wall,wall,wall},
                            {path,path,path,path,path,path,wall,wall,path,path,wall,path,path,wall,wall,wall,wall,path,path,wall,wall,wall,path,path,path,path,path,path,path,path,path,path,wall,path,path,wall,path,wall,wall,path,path,path,wall,path,path,path,path,path},
                            {path,path,path,path,path,path,wall,wall,path,path,wall,path,path,path,path,path,path,path,path,path,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,path,wall,path,path,wall,path,wall,wall,path,path,path,wall,path,path,path,path,path},
                            {wall,wall,wall,wall,wall,wall,wall,wall,path,path,wall,path,path,path,path,path,path,path,path,path,path,wall,path,wall,path,path,path,path,path,wall,path,path,wall,path,path,wall,wall,wall,wall,path,path,path,wall,path,path,wall,wall,wall},
                            {path,path,path,path,path,path,path,path,path,path,wall,path,path,path,path,path,path,path,path,path,path,wall,path,wall,path,wall,wall,wall,path,wall,path,path,wall,path,path,path,path,path,path,path,path,path,wall,path,path,wall,path,path},
                            {path,path,path,path,path,path,path,path,path,path,wall,path,path,wall,path,path,wall,path,wall,wall,wall,wall,path,wall,path,wall,path,wall,path,wall,path,path,wall,path,path,path,path,path,path,path,path,path,wall,path,path,wall,path,path},
                            {wall,wall,wall,wall,wall,wall,wall,wall,path,path,wall,path,path,wall,path,path,wall,path,wall,path,path,path,path,wall,path,wall,path,wall,path,wall,path,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,path,wall,path,wall},
                            {wall,path,path,path,path,wall,wall,wall,path,path,wall,path,path,wall,path,path,wall,path,wall,path,wall,wall,path,wall,path,wall,path,wall,path,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,wall},
                            {wall,path,wall,wall,path,path,wall,wall,path,path,wall,path,path,wall,path,path,wall,path,wall,path,path,wall,path,wall,path,wall,path,wall,path,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,wall},
                            {wall,path,wall,wall,wall,wall,wall,wall,path,path,wall,path,path,wall,path,path,wall,path,wall,wall,wall,wall,path,wall,path,wall,path,wall,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,wall},
                            {wall,path,path,path,path,path,path,path,path,path,wall,path,path,wall,path,path,wall,path,path,path,path,path,path,wall,path,path,path,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall},
                            {wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,path,wall,path,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,wall},
                            {path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,path,path,path,path,path},
                            {path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,path,path,path,path,path},
                            {wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall},
                            {wall,path,wall,path,path,path,path,path,path,wall,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,path,path,path,path,path,path,wall,path,path,path,path,path,path,path,wall,wall,path,path,path,path,path,path,path},
                            {wall,path,wall,path,path,wall,wall,wall,path,wall,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,path,path,path,path,path,path,path,wall,path,path,path,path,path,path,wall,wall,path,path,wall,wall,wall,wall,path},
                            {wall,path,wall,path,path,wall,path,wall,path,wall,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,wall,wall,wall,path,path,wall,wall,wall,wall,wall,path,path,wall,wall,wall,wall,path,wall,wall,path,path,wall,path,path,path,path},
                            {wall,path,wall,path,path,wall,path,wall,path,wall,path,path,path,wall,path,path,path,wall,path,path,wall,path,wall,path,wall,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,wall,wall,path,path,wall,wall,wall,wall,wall},
                            {path,path,wall,path,path,wall,path,wall,path,wall,wall,wall,path,wall,path,wall,path,wall,path,path,wall,path,wall,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,wall,wall,path,path,path,path,path,path,path},
                            {wall,wall,wall,path,path,wall,path,path,path,wall,path,path,path,wall,path,wall,path,wall,path,path,wall,path,wall,path,wall,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,wall,wall,wall,path,wall,wall,wall,wall},
                            {path,path,path,path,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,wall,path,wall,path,path,wall,path,wall,path,wall,path,wall,wall,wall,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,path,path,path,path,path,path,path,path},
                            {path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,path,path,wall,path,wall,path,wall,path,path,path,wall,path,wall,wall,wall,wall,wall,wall,path,path,path,wall,path,path,path,path,path,path,path,path},
                            {wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,wall,path,wall,wall,wall,wall,wall,path,path,path,path,path,path,path,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall},
                            {path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,path,path,path,path,path,wall,wall,wall,wall,wall,wall,path,wall,path,path,path,path,path,path,path,path,path,path},
                            {path,path,path,path,path,path,path,path,path,path,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,wall,path,path,path,path,path,path,path,path,path,path},
                            {wall,wall,wall,wall,wall,wall,wall,wall,wall,path,path,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,wall,path,path,wall,wall,wall,wall,wall,wall},
                            {wall,path,path,path,path,path,path,path,path,path,path,wall,path,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,wall,path,wall,path,path,wall,path,path,path,path,path},
                            {wall,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,path,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,wall,path,wall,path,path,wall,wall,wall,wall,wall,path},
                            {wall,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,wall,path,wall,path,path,path,path,path,path,path,path},
                            {wall,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,wall,wall,wall,path,path,path,path,path,wall},
                            {wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,path,path,path,wall,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,wall,wall,wall,wall,path},
                            {wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,path,path,path,wall,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,path,path,path},
                            {wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,wall,path,wall,wall,path,path,wall,wall,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,path,wall,path,wall,wall,wall,path},
                            {wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,wall,wall,path,path,wall,wall,path,wall,path,wall,path,path,path,path,path,path,path,path,path,path,wall,path,path,wall,path,wall,path,wall,path},
                            {wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,wall,wall,path,path,wall,wall,path,wall,path,wall,path,path,path,path,path,path,path,path,path,path,wall,path,path,wall,path,path,path,wall,path},
                            {wall,path,path,wall,wall,path,wall,wall,path,path,wall,wall,wall,wall,wall,wall,path,wall,path,path,wall,path,path,wall,wall,path,wall,path,wall,path,path,wall,wall,wall,wall,wall,wall,path,path,wall,path,path,wall,wall,wall,wall,wall,path},
                            {wall,path,path,wall,wall,path,path,wall,path,path,wall,path,wall,wall,wall,wall,path,wall,path,path,wall,path,path,path,path,path,path,path,wall,path,path,path,path,path,path,path,wall,path,path,path,path,path,path,path,path,path,path,path},
                            {wall,path,wall,path,path,path,path,wall,path,path,wall,path,path,path,path,path,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall}
                          };

uint8_t *Bigmap2[48][48] = {
                            {wall,path,path,path,wall,wall,wall,wall,path,path,wall,wall,wall,wall,wall,wall,wall,path,wall,wall,wall,wall,wall,wall,wall,path,path,path,wall,wall,path,path,path,wall,path,wall,wall,wall,wall,path,path,wall,path,path,path,wall,path,path},
                            {wall,path,path,path,wall,path,path,wall,path,path,wall,path,path,path,path,path,wall,path,path,path,path,path,wall,wall,wall,path,path,path,wall,wall,path,path,path,wall,path,path,path,path,path,path,path,wall,path,path,path,wall,path,path},
                            {wall,path,path,path,wall,path,path,wall,path,path,wall,path,path,path,path,path,wall,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,path,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,path,path,wall,path,path},
                            {wall,path,path,path,wall,path,path,wall,path,path,wall,path,path,wall,path,path,wall,path,wall,wall,wall,wall,wall,path,path,wall,wall,wall,wall,wall,path,path,path,wall,path,path,path,path,path,path,path,path,path,path,path,wall,path,path},
                            {wall,path,path,path,wall,path,path,wall,path,path,wall,path,path,wall,path,path,wall,path,path,path,path,path,path,path,path,wall,wall,path,path,path,path,path,path,wall,path,path,path,path,path,path,path,path,path,path,path,wall,path,path},
                            {path,path,path,path,wall,path,path,wall,path,path,wall,path,path,wall,path,path,wall,wall,wall,wall,wall,wall,wall,path,path,wall,wall,path,path,path,path,path,path,wall,path,path,path,path,path,path,path,path,path,path,path,wall,path,path},
                            {path,path,path,path,wall,path,path,wall,path,path,wall,path,path,wall,path,path,path,path,path,path,path,wall,wall,path,path,wall,wall,path,path,path,path,path,path,wall,path,path,path,wall,wall,wall,wall,wall,wall,path,path,wall,path,path},
                            {path,path,path,path,wall,path,path,wall,path,path,wall,path,path,wall,path,path,path,path,path,path,path,wall,wall,path,path,wall,wall,path,path,path,wall,wall,wall,wall,path,path,path,wall,path,path,path,path,wall,path,path,wall,path,path},
                            {wall,wall,wall,wall,wall,path,path,wall,path,path,wall,path,path,wall,wall,wall,wall,wall,wall,wall,path,wall,wall,path,path,wall,wall,path,path,path,wall,wall,wall,wall,path,path,path,wall,path,wall,wall,path,wall,path,path,wall,wall,wall},
                            {path,path,path,path,path,path,path,wall,path,path,wall,path,path,wall,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,path,wall,wall,wall,wall,path,path,path,wall,path,wall,wall,path,wall,path,path,path,path,path},
                            {path,path,path,path,path,path,path,wall,path,path,wall,path,path,wall,wall,wall,wall,wall,wall,path,wall,wall,wall,wall,wall,wall,wall,path,path,path,wall,wall,wall,wall,path,path,path,wall,path,wall,wall,path,wall,path,path,path,path,path},
                            {wall,wall,wall,wall,wall,wall,wall,wall,path,path,wall,path,path,path,path,path,path,wall,wall,path,wall,wall,path,path,path,path,wall,path,path,path,path,path,path,path,path,path,path,wall,path,wall,wall,path,wall,path,path,wall,wall,wall},
                            {path,path,path,path,path,path,path,path,path,path,wall,path,path,path,path,path,path,wall,wall,wall,wall,wall,path,path,path,path,wall,path,path,path,path,path,path,path,path,path,path,wall,path,wall,path,path,wall,path,path,wall,path,path},
                            {wall,wall,wall,wall,wall,wall,wall,wall,path,path,wall,wall,path,wall,path,path,path,path,path,path,path,wall,path,path,wall,path,wall,path,path,path,path,path,path,path,path,path,path,wall,path,wall,wall,wall,wall,wall,wall,wall,wall,wall},
                            {path,path,path,path,path,path,path,path,path,path,wall,wall,path,wall,path,path,path,path,path,path,path,wall,path,path,wall,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,path,path,path,path,path,path,path,path,path},
                            {path,path,path,path,path,path,path,path,path,path,wall,wall,path,wall,path,path,path,path,path,path,path,wall,path,path,wall,path,wall,wall,wall,wall,wall,wall,wall,path,path,path,path,wall,path,path,path,path,path,path,path,path,path,path},
                            {wall,wall,wall,wall,wall,path,path,wall,wall,wall,wall,wall,path,wall,path,path,path,wall,wall,wall,wall,wall,path,path,wall,path,path,path,path,path,path,path,path,path,path,path,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall},
                            {wall,wall,wall,wall,wall,path,path,wall,wall,wall,wall,wall,path,wall,path,path,path,wall,wall,wall,wall,wall,path,path,wall,wall,wall,wall,wall,path,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall},
                            {path,wall,path,path,path,path,path,wall,path,path,path,path,path,wall,path,path,path,wall,path,path,path,path,path,path,path,path,wall,path,wall,path,path,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path},
                            {path,wall,path,path,path,path,path,wall,wall,wall,wall,wall,wall,wall,path,path,path,wall,path,path,path,path,path,path,path,path,wall,path,wall,path,path,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path},
                            {path,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,path,path,path,path,path,path,wall,path,wall,path,path,wall,path,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path},
                            {wall,wall,wall,wall,wall,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,path,wall,wall,wall,path,path,wall,path,wall,path,path,wall,path,path,wall,path,path,path,path,path,path,path,path,path,path,path,wall,path},
                            {path,path,path,path,wall,path,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,path,path,wall,wall,wall,path,path,wall,path,path,path,path,wall,path,path,wall,path,path,path,path,path,path,path,path,path,path,path,wall,path},
                            {path,path,path,path,wall,path,path,wall,wall,path,path,path,path,path,path,path,path,path,path,path,path,wall,wall,wall,path,path,wall,path,path,path,path,wall,path,path,wall,path,path,path,path,path,path,path,path,path,path,path,wall,path},
                            {path,path,path,path,wall,path,path,wall,wall,path,path,path,path,path,path,path,path,path,path,path,path,wall,wall,wall,path,path,wall,wall,wall,wall,wall,wall,path,path,wall,path,path,path,wall,path,path,wall,wall,wall,path,path,wall,path},
                            {wall,path,path,path,wall,path,path,wall,wall,path,path,path,path,path,path,path,path,path,path,path,path,wall,wall,wall,path,path,path,path,path,path,path,path,path,path,wall,path,path,path,wall,path,path,path,path,wall,path,path,wall,wall},
                            {wall,path,path,path,wall,path,path,wall,wall,path,path,path,wall,wall,path,path,path,wall,wall,wall,wall,wall,wall,wall,path,path,path,path,path,path,path,path,path,path,wall,path,path,path,wall,path,path,wall,path,wall,path,path,wall,wall},
                            {wall,path,path,path,wall,path,path,wall,wall,path,path,path,wall,wall,path,path,path,wall,wall,wall,wall,wall,wall,wall,path,path,path,path,path,path,path,path,path,path,wall,path,path,path,wall,path,path,wall,path,wall,path,path,wall,wall},
                            {wall,path,path,path,wall,path,path,path,path,path,path,path,wall,wall,path,path,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,path,path,wall,path,path,wall,path,wall,path,path,wall,wall},
                            {wall,path,path,path,wall,path,path,path,path,path,path,path,wall,wall,path,path,path,wall,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,wall,path,wall,path,path,wall,wall},
                            {wall,path,path,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,path,path,wall,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,wall,path,wall,path,path,wall,wall},
                            {path,path,path,path,wall,path,path,path,path,path,path,path,path,path,path,path,path,wall,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,wall,path,wall,path,path,path,path},
                            {path,path,path,path,wall,path,path,path,path,path,path,path,path,path,path,path,path,wall,wall,path,path,path,wall,wall,wall,path,path,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,path,wall,path,wall,path,path,path,path},
                            {wall,wall,path,path,wall,path,path,path,path,path,path,path,path,path,path,path,path,wall,wall,path,path,path,wall,wall,wall,path,path,path,wall,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,wall,wall,wall,wall,wall},
                            {path,wall,path,path,wall,wall,wall,wall,path,wall,wall,wall,path,path,path,wall,wall,wall,wall,path,path,path,wall,wall,wall,path,path,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,path,wall,path,path,path,path,path,path},
                            {wall,wall,wall,wall,wall,wall,wall,wall,path,path,path,wall,path,path,path,wall,wall,wall,wall,path,path,path,wall,wall,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,wall,wall,wall,wall,wall,wall,wall},
                            {path,path,path,path,path,path,path,wall,path,wall,path,wall,path,path,path,wall,wall,wall,wall,path,path,path,wall,wall,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,wall,wall,path,path,path,path,path},
                            {path,path,path,path,path,path,path,wall,path,wall,path,wall,path,path,path,wall,wall,wall,wall,path,path,path,wall,wall,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,wall,wall,path,path,path,path,path},
                            {path,path,path,path,path,path,path,wall,path,wall,path,wall,path,path,path,wall,wall,wall,wall,path,path,path,wall,wall,wall,wall,wall,wall,wall,wall,path,path,path,wall,wall,path,path,path,wall,path,path,wall,wall,path,path,path,path,path},
                            {wall,wall,wall,wall,path,path,path,wall,path,wall,path,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,wall,path,path,path,wall,wall,path,path,path,wall,path,path,wall,wall,wall,wall,wall,wall,wall},
                            {path,path,path,wall,path,path,path,wall,path,wall,path,wall,wall,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,wall,path,path,path,wall,wall,path,path,path,wall,path,path,wall,path,path,path,path,path,path},
                            {path,path,path,wall,path,path,path,wall,path,wall,path,wall,wall,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,wall,path,path,path,wall,wall,path,path,path,wall,path,path,wall,path,path,path,path,path,path},
                            {path,path,path,wall,path,path,path,wall,path,wall,path,path,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,path,path,wall,wall,path,path,path,wall,wall,path,path,path,wall,path,path,wall,path,path,path,path,path,path},
                            {wall,wall,wall,wall,path,path,path,wall,wall,wall,wall,wall,path,path,path,path,path,path,path,path,path,path,path,path,wall,path,path,path,wall,wall,path,path,path,wall,wall,path,path,path,wall,path,path,wall,path,path,path,wall,wall,wall},
                            {path,path,path,path,path,path,path,path,path,path,path,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,wall,path,path,path,wall,wall,path,path,path,wall,wall,path,path,path,wall,path,path,wall,path,path,path,wall,path,path},
                            {path,path,path,path,path,path,path,path,path,path,path,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,wall,path,path,path,wall,wall,path,path,path,wall,path,path,wall,path,path,path,wall,path,path},
                            {path,path,path,path,path,path,path,path,path,path,path,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,wall,path,path,path,wall,wall,path,path,path,wall,path,path,wall,path,path,path,wall,path,path},
                            {wall,path,path,path,wall,wall,wall,wall,path,path,wall,wall,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,path,wall,wall,path,path,path,wall,wall,wall,wall,wall,wall,path,path,wall,path,path,path,wall,path,path},
                          };

uint8_t *map[8][8] = {{wall,wall,wall,path,path,wall,wall,wall},
                      {wall,wall,wall,path,path,wall,wall,wall},
                      {wall,wall,wall,path,path,wall,wall,wall},
                      {path,path,path,path,path,path,path,path},
                      {path,path,path,path,path,path,path,path},
                      {wall,wall,wall,path,path,wall,wall,wall},
                      {wall,wall,wall,path,path,wall,wall,wall},
                      {wall,wall,wall,path,path,wall,wall,wall}};

uint8_t *map2[8][8] = {{wall,wall,wall,path,path,wall,wall,wall},
                      {wall,wall,wall,path,path,wall,wall,wall},
                      {wall,wall,wall,path,path,wall,wall,wall},
                      {path,path,path,path,path,path,path,path},
                      {path,path,path,path,path,path,path,path},
                      {wall,wall,wall,path,path,wall,wall,wall},
                      {wall,wall,wall,path,path,wall,wall,wall},
                      {wall,wall,wall,path,path,wall,wall,wall}};

  uint8_t pat_1[8][8][3] ={     {{0,30,0},{0,28,3},{0,26,5},{0,24,7},{0,22,9},{0,20,11},{0,18,12},{0,16,14}},
                                {{0,28,3},{0,26,5},{0,24,7},{0,22,9},{0,20,11},{0,18,12},{0,16,14},{0,14,16}},
                                {{0,26,5},{0,24,7},{0,22,9},{0,20,11},{0,18,12},{0,16,14},{0,14,16},{0,12,18}},
                                {{0,24,7},{0,22,9},{0,20,11},{0,18,12},{0,16,14},{0,14,16},{0,12,18},{0,10,20}},
                                {{0,22,9},{0,20,11},{0,18,12},{0,16,14},{0,14,16},{0,12,18},{0,10,20},{0,8,22}},
                                {{0,20,11},{0,18,12},{0,16,14},{0,14,16},{0,12,18},{0,10,20},{0,8,22},{0,6,24}},
                                {{0,18,12},{0,16,14},{0,14,16},{0,12,18},{0,10,20},{0,8,22},{0,6,24},{0,4,26}},
                                {{0,16,14},{0,14,16},{0,12,18},{0,10,20},{0,8,22},{0,6,24},{0,4,26},{0,2,28}},
                          };



uint8_t pat_2[8][8][3] ={     {{25,0,0},{25,0,1},{25,0,3},{25,0,5},{25,0,7},{25,0,9},{25,0,10},{25,0,12}},
                              {{25,0,1},{25,0,3},{25,0,5},{25,0,7},{25,0,9},{25,0,10},{25,0,12},{25,0,14}},
                              {{25,0,3},{25,0,5},{25,0,7},{25,0,9},{25,0,10},{25,0,12},{25,0,14},{25,0,16}},
                              {{25,0,5},{25,0,7},{25,0,9},{25,0,10},{25,0,12},{25,0,14},{25,0,16},{25,0,18}},
                              {{25,0,7},{25,0,9},{25,0,10},{25,0,12},{25,0,14},{25,0,16},{25,0,18},{25,0,19}},
                              {{25,0,9},{25,0,10},{25,0,12},{25,0,14},{25,0,16},{25,0,18},{25,0,19},{25,0,20}},
                              {{25,0,10},{25,0,12},{25,0,14},{25,0,16},{25,0,18},{25,0,19},{25,0,20},{25,0,22}},
                              {{25,0,12},{25,0,14},{25,0,16},{25,0,18},{25,0,19},{25,0,20},{25,0,22},{25,0,24}},
                          };
uint8_t pat_3[8][8][3] ={     {{25,0,0},{21,0,0},{17,0,0},{13,0,0},{9,13,0},{5,17,0},{1,21,0},{0,25,0}},
                              {{21,0,0},{17,0,0},{13,0,0},{9,13,0},{5,17,0},{1,21,0},{0,25,0},{0,21,1}},
                              {{17,0,0},{13,0,0},{9,13,0},{5,17,0},{1,21,0},{0,25,0},{0,21,1},{0,17,5}},
                              {{13,0,0},{9,13,0},{5,17,0},{1,21,0},{0,25,0},{0,21,1},{0,17,5},{0,13,9}},
                              {{9,13,0},{5,17,0},{1,21,0},{0,25,0},{0,21,1},{0,17,5},{0,13,9},{0,0,13}},
                              {{5,17,0},{1,21,0},{0,25,0},{0,21,1},{0,17,5},{0,13,9},{0,0,13},{0,0,17}},
                              {{1,21,0},{0,25,0},{0,21,1},{0,17,5},{0,13,9},{0,0,13},{0,0,17},{0,0,21}},
                              {{0,25,0},{0,21,1},{0,17,5},{0,13,9},{0,0,13},{0,0,17},{0,0,21},{0,0,25}},
                          };
void random_spawn(int *ran_res,int max_x,int min_x,int max_y,int min_y){
  int x_axis = rand()%(max_x-min_x+1)+(min_x-1);
  int y_axis = rand()%(max_y-min_y+1)+(min_y-1);
  ran_res[0] = x_axis;
  ran_res[1] = y_axis;
  pc.printf("%d %d\n",ran_res[0],ran_res[1]);

}



void moveMap(uint8_t* map[48][48],int nextCenter[2],int mapsize[2],uint8_t* output[8][8]){
  int row,column;
  for (int i=0;i<8;i++){
    for(int j=0;j<8;j++){
      row = (nextCenter[1]%48)-3+i;
      column = (nextCenter[0]%48)-3+j;
      if (row>=0 && row<mapsize[0]){}
      else if(row<0){row = mapsize[0]+(row%49);}
      else if(row>=mapsize[0]){row = row%48;}
      if(column>=0 && column<mapsize[1]){}
      else if(column<0){column = mapsize[1]+(column%49);}
      else if(column>=mapsize[1]){column = column%48;}
      output[i][j] = map[row][column];
    }
  }
  output[3][3] = character;
}
char buffer[4]={0x00,0x00,0x00,0x00};
char msg[1] = {0x00};
char raw_data[128];
int x_recieve=0,y_recieve=0;
bool is_inLoop = true;
bool serial_en = false;
// void Serial_inter(){
//   // for(int i = 0;i<4;i++){
//   //   raw_data[i] = bt.getc();
//   // }
//     // pc.putc('s');
//     // if(bt.readable()){
//     //   bt.scanf("%s",raw_data );
//     // }
//     if (serial_en){
//       int count_packet = 0;
//       for(int i = 0;i<5;i++){
//         if(bt.readable()){
//           raw_data[i] = bt.getc();
//           count_packet += 1;
//         }
//
//       }
//       // pc.abort_read();
//       // raw_data[0] = bt.getc();
//       // raw_data[1] = bt.getc();
//       // raw_data[2] = bt.getc();
//       // raw_data[3] = bt.getc();
//       bool header = false;
//       int count = 0;
//       if(count_packet>4){
//         for (int i = 0; i < 4; i++) {
//           if(header && raw_data[i]!= 0xFE && raw_data[i]!=0xFF ){
//             dataa[count] = raw_data[i];
//             count += 1;
//           }
//           if(count > 1){
//             break;
//           }
//           if(raw_data[i] == 0xFF){
//             header = true;
//           }
//           if(raw_data[i] == 0xFE){
//             header = true;
//           }
//         }
//       }
//       header = false;
//       if(dataa[0]&0x02){
//         if(dataa[0]&0x04){
//           aby = -dataa[1];
//         }
//         else{
//           aby = dataa[1];
//         }
//       }
//       else{
//         if(dataa[0]&0x04){
//           abx = -dataa[1];
//         }
//         else{
//           abx = dataa[1];
//         }
//       }
//       if(dataa[0]&0x01){
//         is_inLoop = true;
//       }
//       else{
//         is_inLoop = true;
//       }
//       pc.putc(dataa[0]);
//       pc.putc(dataa[1]);
//       pc.abort_write();
//   }
// }
uint8_t *Bigmap[48][48];
void randomMap(uint8_t *output[48][48],uint8_t *map1[48][48],uint8_t *map2[48][48]){
  srand(time(NULL));
  if(rand()%2==0){
    for(int i = 0 ; i<48;i++){
      for(int j = 0; j<48;j++){
        Bigmap[i][j] = map1[i][j];
      }
    }
  }
  else{
    for(int i = 0 ; i<48;i++){
      for(int j = 0; j<48;j++){
        Bigmap[i][j] = map2[i][j];
      }
    }
  }
}
// Bigmap = &Bigmap2;

int main(int argc, char const *argv[]) {
  randomMap(Bigmap,Bigmap1,Bigmap2);
  //bt.attach(&Serial_inter);
  bt.baud(9600);
  pc.baud(9600);
  slave.address(0x70<<1);
  // I2C_StretchClockCmd(I2C2, ENABLE);
  //i2c.frequency(400000);
  color_s.init();
  uint8_t whoami = mpu.whoami();
  if (whoami == 0x71) // WHO_AM_I should always be 0x68
  {
    pc.printf("MPU9250 WHO_AM_I is 0x%x\n\r", whoami);
    pc.printf("MPU9250 is online...\n\r");
    color_s.display(pat_3,4);
    mpu.set_acc_scale(2);
    mpu.calib_acc();
    mpu.calibrateMPU(gyroBias,accBias);
    wait(1);
    pc.printf("gxBias = %f\t",gyroBias[0] );
    pc.printf("gyBias = %f\t",gyroBias[1] );
    pc.printf("gzBias = %f\n",gyroBias[2] );

    pc.printf("axBias = %f\t",accBias[0] );
    pc.printf("axBias = %f\t",accBias[1] );
    pc.printf("axBias = %f\n",accBias[2] );
    mpu.set_acc_scale(2);
    wait(2);
    pc.printf("complete ....\n" );
    wait(2);
  }
  else
   {
     color_s.display(pat_2,4);
    pc.printf("Could not connect to MPU9250: \n\r");
    pc.printf("%#x \n",  whoami);
    while(1);
  }
  int ran_res_eiei[2];

  srand(time(NULL));
  while(1){
    random_spawn(ran_res_eiei,48,8,48,8);
    centerNow[0] = ran_res_eiei[0];
    centerNow[1] = ran_res_eiei[1];
    if (Bigmap[centerNow[1]][centerNow[0]][0]==wall[0] && Bigmap[centerNow[1]][centerNow[0]][1]==wall[1] && Bigmap[centerNow[1]][centerNow[0]][2]==wall[2]){
    }
    else{
      break;
    }
  }
  while(1){
    random_spawn(ran_res_eiei,centerNow[0]+32,centerNow[0]+16,centerNow[1]+32,centerNow[1]+16);
    special_item[0] = ran_res_eiei[0];
    special_item[1] = ran_res_eiei[1];
    if(special_item[0]>47){
      special_item[0] = special_item[0]%48;
    }
    else if(special_item[0]<0){
      special_item[0] = 48-special_item[0]%48;
    }
    if(special_item[1]>47){
      special_item[1] = special_item[1]%48;
    }
    else if(special_item[1]<0){
      special_item[1] = 48-special_item[1]%48;
    }
    if (Bigmap[special_item[1]][special_item[0]][0]==wall[0] && Bigmap[special_item[1]][special_item[0]][1]==wall[1] && Bigmap[special_item[1]][special_item[0]][2]==wall[2]){
    }
    else{
      Bigmap[special_item[1]][special_item[0]] = end_item;
      break;
    }
  }
  // pc.printf("%d %d\n",special_item[0],special_item[1] );

  for(int i = 0;i<15;i++){
    while(1){
      random_spawn(ran_res_eiei,48,8,48,8);
      revert_item_pos[0] = ran_res_eiei[0];
      revert_item_pos[1] = ran_res_eiei[1];
      if (Bigmap[revert_item_pos[1]][revert_item_pos[0]][0]==path[0] && Bigmap[revert_item_pos[1]][revert_item_pos[0]][1]==path[1] && Bigmap[revert_item_pos[1]][revert_item_pos[0]][2]==path[2]){
        Bigmap[revert_item_pos[1]][revert_item_pos[0]] = bad_item;
        break;
      }
      else{}
    }
  }
  for(int i = 0;i<17;i++){
    while(1){
      random_spawn(ran_res_eiei,48,8,48,8);
      increase_time[0] = ran_res_eiei[0];
      increase_time[1] = ran_res_eiei[1];
      if (Bigmap[increase_time[1]][increase_time[0]][0]==path[0] && Bigmap[increase_time[1]][increase_time[0]][1]==path[1] && Bigmap[increase_time[1]][increase_time[0]][2]==path[2]){
        Bigmap[increase_time[1]][increase_time[0]] = good_item;
        break;
      }
      else{}
    }
  }

////////////////////
//Loop start here//
///////////////////
T.start();
  while(1){
    serial_en = true;
    int16_t data[7];
    // mpu.readATandG(data);
    // float ares = mpu.getAres();
    // float ax_mpu = (float)data[0]*ares - accBias[0];
    // float ay_mpu = (float)data[1]*ares - accBias[1];
    mpu.read_acc();
    float ax_mpu = mpu.accelerometer_data[0] - (float)mpu.calib_data[0]/(float)mpu.acc_divider;
    float ay_mpu = mpu.accelerometer_data[1] - (float)mpu.calib_data[1]/(float)mpu.acc_divider;

    int i = slave.receive();
    // printf("%d\n",i);
    // slave.stop();
    // while (i==0){i = slave.receive();}
    // printf("%d\n",i );
    get_recieve(i,msg,buffer,3,1);
    wait_ms(5 );
    // if(i==3){while(i!=1){i = slave.receive();}}
    // else{while(i!=3){i= slave.receive();}}
    i = slave.receive();
    get_recieve(i,msg,buffer,3,1);
    y_recieve = buffer[1];
    x_recieve = buffer[0];
    // printf("%d\n",buffer[2] );
    if(buffer[2]&0x04){x_recieve = -x_recieve;}
    if(buffer[2]&0x02){y_recieve = -y_recieve;}
    if(buffer[2]&0x01){is_win = false; break;}
    // slave.stop();
    int ay = (int8_t)(ay_mpu*100)+y_recieve*1.5;
    int ax = (int8_t)(ax_mpu*100)+x_recieve*1.5;
    if (is_revert){
      if(msg[0]&0x02){}
      else{msg[0] += 0x02;}
      ay = -(ay)*2;
      ax = -(ax)*2;
      t.stop();
      if(t.read() >= 20){
        t.stop();
        t.reset();
        is_revert = false;
      }
      else{
        t.start();
      }
    }

    if (ay<20 && ay>-20){}
    else if (ay>0){
      // if(frameCounty >= 50-(int)(ay*100)){
        if(frameCounty >= 100-ay){
        centerNow[1]++;
        centerNow[1] = centerNow[1]%48;
        frameCounty = 0;
      }
    }
    else if(ay<=0){
      // if(frameCounty >= 50+(int)(ay*100)){
      if(frameCounty >= 100+ay){
        centerNow[1]--;
        if (centerNow[1]<0){centerNow[1] = 48+(centerNow[1]%49);}
        else{centerNow[1] = centerNow[1]%48;}
        frameCounty = 0;
      }
    }
    if (ax<20 && ax>-20){}
    else if (ax>0){
      // if(frameCountx >= 50-(int)(ax*100)){
      if(frameCountx >= 100-ax){
        // pc.printf("%d\n", 150-(int)(ax*200));
        centerNow[0]++;
        centerNow[0] = centerNow[0]%48;
        frameCountx = 0;
      }
    }
    else if(ax<=0){
      // if(frameCountx >= 50+(int)(ax*100)){
      if(frameCountx >= 100+ax){
        // pc.printf("%d\n", 150+(int)(ax*200));
        centerNow[0]=centerNow[0]-1;
        if (centerNow[0]<0){centerNow[0] = 48+(centerNow[0]%49);}
        else{centerNow[0] = centerNow[0]%48;}
        frameCountx = 0;
      }
    }
    if (Bigmap[centerNow[1]][centerNow[0]][0]!=path[0] || Bigmap[centerNow[1]][centerNow[0]][1]!=path[1] || Bigmap[centerNow[1]][centerNow[0]][2]!=path[2]){
      if (Bigmap[centerNow[1]][centerNow[0]][0]==wall[0] && Bigmap[centerNow[1]][centerNow[0]][1]==wall[1] && Bigmap[centerNow[1]][centerNow[0]][2]==wall[2]){
        centerNow[0] = previousCenter[0];
        centerNow[1] = previousCenter[1];
      }
      else if (Bigmap[centerNow[1]][centerNow[0]][0]==end_item[0] && Bigmap[centerNow[1]][centerNow[0]][1]==end_item[1] && Bigmap[centerNow[1]][centerNow[0]][2]==end_item[2]){
        is_win = true;
        if(msg[0]&0x01){}
        else{msg[0] += 0x01;}
        break;
      }
      else if (Bigmap[centerNow[1]][centerNow[0]][0]==bad_item[0] && Bigmap[centerNow[1]][centerNow[0]][1]==bad_item[1] && Bigmap[centerNow[1]][centerNow[0]][2]==bad_item[2]){
        srand(time(NULL));
        int r =rand()%2;
        if(r==0){
          is_revert = true;
          Bigmap[centerNow[1]][centerNow[0]] = path;
          t.reset();
          t.start();
        }
        else if(r==1){
          Bigmap[centerNow[1]][centerNow[0]] = path;
          if(msg[0]&0x04){}
          else{msg[0] += 0x04;}
          }
        }
      else if (Bigmap[centerNow[1]][centerNow[0]][0]==good_item[0] && Bigmap[centerNow[1]][centerNow[0]][1]==good_item[1] && Bigmap[centerNow[1]][centerNow[0]][2]==good_item[2]){
          Bigmap[centerNow[1]][centerNow[0]] = path;
          if(msg[0]&0x08){}
          else{msg[0] += 0x08;}
      }
    }

    // centerNow[0] = 19;
    // centerNow[1] = 19;
    // pc.printf("%d %d\n",ax,ay );
    moveMap(Bigmap,centerNow,mapsize,output);
    color_s.display(output,10);
    frameCountx= (frameCountx+1)%500;
    frameCounty= (frameCounty+1)%500;
    previousCenter[0] = centerNow[0];
    previousCenter[1] = centerNow[1];

    // if(T.read()>=timeout){
    //   is_win = false;
    //   break;
    // }
  }
  while(is_win){
    msg[0] = 0x01;
    printf("0x%x\n",msg[0] );
    int i = slave.receive();
    get_recieve(i,msg,buffer,3,1);
    wait_ms(10);
    color_s.display(pat_3,10);
  }
  while(!is_win){
    color_s.display(pat_1,10);
  }
  return 0;
}
