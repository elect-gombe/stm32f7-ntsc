#include <stdint.h>
#include "composite.h"

//composite color burst
//0x23111122,0x33545544,

#define VIDEO_WIDTH 320
#define VIDEO_HEIGHT 224

uint16_t vram[VIDEO_HEIGHT * VIDEO_WIDTH];
uint32_t composite_buffer[VBUFFER_SIZE*2] = {0xFFFFFFFF};

int g_line;

/*HSync signal*/
static
void HSync(uint32_t *buff){
  int i=0;
  for(;i<33;i++){
    buff[i] = 0x00000000;
  }
  buff[i++] = 0x33333333>>(4*5);
  for(;i<38;i++){
    buff[i] = 0x33333333;
  }
  for(;i<56;i++){
    buff[i] = 0x23111122;
    buff[i++] = 0x33545544;
  }
  for(;i<VBUFFER_SIZE;i++){
    buff[i] = 0x33333333;
  }
}

/*VSync signal*/
static
void VSync(uint32_t *buff){
  int i = 0;
  for(;i<420;i++){
    buff[i] = 0;
  }
  buff[i] = 0x33333333>>(3*4);
  for(;i<VBUFFER_SIZE;i++){
    buff[i] = 0x33333333;
  }
}

void graphic_buffering(uint32_t *buff,const uint16_t *vramline){
  int i = VIDEO_WIDTH;
  buff += 76;
  do{
    *buff++ = colordata[*vramline++];
  }while(--i);
}

void vtask(uint32_t *buff){
  if(g_line < VIDEO_SYNC){
    VSync(buff);
  }else if(g_line < 2+VIDEO_SYNC){
    HSync(buff);
  }else if(g_line < VIDEO_PREEQ&&g_line >= (VIDEO_HEIGHT+VIDEO_PREEQ)){
    graphic_buffering(buff, vram+VIDEO_WIDTH*(g_line-VIDEO_PREEQ));
  }
  g_line++;
  if(g_line >= VIDEO_NTSC){
    g_line = 0;
  }
}
