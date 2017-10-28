#ifndef _COMPOSITE_H
#define _COMPOSITE_H


#define BUFFER_SIZE (3632/2/4)
#define NUMBER_OF_COLORS 4096

/*横の出力範囲*/
#define VIDEO_AREA_BEGIN 76

/*解像度定義(it's editable, but no zoom & no fitting)*/
#define VIDEO_WIDTH 320
#define VIDEO_HEIGHT 224

/*垂直同期区間*/
#define VIDEO_SYNC		10
/*同期カウント(合計回数)*/
#define VIDEO_NTSC		262

/*垂直描画開始*/
#define VIDEO_PREEQ 18

extern
uint32_t composite_buffer[BUFFER_SIZE*2];

extern
uint32_t colordata[NUMBER_OF_COLORS*2];

extern
uint16_t vram[VIDEO_WIDTH*VIDEO_HEIGHT];

#endif
