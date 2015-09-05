/*

  u8g_dev_ssd1331_96x64.c

  Universal 8bit Graphics Library
  
  Copyright (c) 2015, sosman@terratron.com
  Copyright (c) 2013, jamjardavies@gmail.com
  Copyright (c) 2013, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
  
  History:
  Initial version	31 August 2015 sosman@terratron.com.
                           Branched from u8g_dev_ssd1351_128x128.c
  
*/

#include "u8g.h"

#define WIDTH		96
#define HEIGHT		64
#define PAGE_HEIGHT	8

/* Private macro -------------------------------------------------------------*/
#define DRAW_LINE                       0x21
#define DRAW_RECTANGLE                  0x22
#define COPY_WINDOW                     0x23
#define DIM_WINDOW                      0x24
#define CLEAR_WINDOW                    0x25
#define FILL_WINDOW                     0x26
#define DISABLE_FILL                    0x00
#define ENABLE_FILL                     0x01
#define CONTINUOUS_SCROLLING_SETUP      0x27
#define DEACTIVE_SCROLLING              0x2E
#define ACTIVE_SCROLLING                0x2F

#define SET_COLUMN_ADDRESS              0x15
#define SET_ROW_ADDRESS                 0x75
#define SET_CONTRAST_A                  0x81
#define SET_CONTRAST_B                  0x82
#define SET_CONTRAST_C                  0x83
#define MASTER_CURRENT_CONTROL          0x87
#define SET_PRECHARGE_SPEED_A           0x8A
#define SET_PRECHARGE_SPEED_B           0x8B
#define SET_PRECHARGE_SPEED_C           0x8C
#define SET_REMAP                       0xA0
#define SET_DISPLAY_START_LINE          0xA1
#define SET_DISPLAY_OFFSET              0xA2
#define NORMAL_DISPLAY                  0xA4
#define ENTIRE_DISPLAY_ON               0xA5
#define ENTIRE_DISPLAY_OFF              0xA6
#define INVERSE_DISPLAY                 0xA7
#define SET_MULTIPLEX_RATIO             0xA8
#define DIM_MODE_SETTING                0xAB
#define SET_MASTER_CONFIGURE            0xAD
#define DIM_MODE_DISPLAY_ON             0xAC
#define DISPLAY_OFF                     0xAE
#define NORMAL_BRIGHTNESS_DISPLAY_ON    0xAF
#define POWER_SAVE_MODE                 0xB0
#define PHASE_PERIOD_ADJUSTMENT         0xB1
#define DISPLAY_CLOCK_DIV               0xB3
#define SET_GRAY_SCALE_TABLE            0xB8
#define ENABLE_LINEAR_GRAY_SCALE_TABLE  0xB9
#define SET_PRECHARGE_VOLTAGE           0xBB
#define SET_V_VOLTAGE                   0xBE

    //ssd1331_fill_rect(0, 0, 96, 64, 0x0000);
//    ssd1331_clear_screen(0x0000);

static const uint8_t u8g_dev_ssd1331_96x64_init_seq[] PROGMEM = {
	U8G_ESC_CS(0),					/* disable chip */
	U8G_ESC_DLY(50),
	U8G_ESC_ADR(0),					/* instruction mode */
	U8G_ESC_RST(1),					/* do reset low pulse with (1*16)+2 milliseconds */
	U8G_ESC_CS(1),					/* enable chip */
	U8G_ESC_DLY(50),
#if 0 // Sauce
	0xfd,							/* Command Lock */
	0x12,						

	U8G_ESC_ADR(0),					/* instruction mode */
	0xfd,
	0xb1,							/* Command Lock */
#endif

	U8G_ESC_ADR(0),					/* instruction mode */
	DISPLAY_OFF,          //Display Off
	SET_CONTRAST_A,       //Set contrast for color A
	U8G_ESC_255,                     //145 0x91

	SET_CONTRAST_B,       //Set contrast for color B
	U8G_ESC_255,                     //80 0x50

	SET_CONTRAST_C,       //Set contrast for color C
	U8G_ESC_255,                     //125 0x7D

	MASTER_CURRENT_CONTROL,//master current control
	0x06,                     //6

	SET_PRECHARGE_SPEED_A,//Set Second Pre-change Speed For ColorA
	0x64,                     //100

	SET_PRECHARGE_SPEED_B,//Set Second Pre-change Speed For ColorB
	0x78,                     //120

	SET_PRECHARGE_SPEED_C,//Set Second Pre-change Speed For ColorC
	0x64,                     //100

	SET_REMAP,            //set remap & data format
	0x72,                     //0x72              

	SET_DISPLAY_START_LINE,//Set display Start Line
	0x0,

	SET_DISPLAY_OFFSET,   //Set display offset
	0x0,

	NORMAL_DISPLAY,       //Set display mode

	SET_MULTIPLEX_RATIO,  //Set multiplex ratio
	0x3F,

	SET_MASTER_CONFIGURE, //Set master configuration
	0x8E,

	POWER_SAVE_MODE,      //Set Power Save Mode
	0x00,                     //0x00

	PHASE_PERIOD_ADJUSTMENT,//phase 1 and 2 period adjustment
	0x31,                     //0x31

	DISPLAY_CLOCK_DIV,    //display clock divider/oscillator frequency
	0xF0,

	SET_PRECHARGE_VOLTAGE,//Set Pre-Change Level
	0x3A,

	SET_V_VOLTAGE,        //Set vcomH
	0x3E,

	DEACTIVE_SCROLLING,   //disable scrolling

	ENABLE_LINEAR_GRAY_SCALE_TABLE, // Sauce: It's called "gray scale" but it has more to do with gamma correction.

	NORMAL_BRIGHTNESS_DISPLAY_ON,//set display on

	U8G_ESC_DLY(50),
	U8G_ESC_CS(0),					/* disable chip */
	U8G_ESC_END						/* end of sequence */
};

#define u8g_dev_ssd1331_96x64_init_seq u8g_dev_ssd1331_96x64_init_seq

static const uint8_t u8g_dev_ssd1331_96x64_332_column_seq[] PROGMEM = {
	U8G_ESC_CS(1),
	U8G_ESC_ADR(0),

	SET_COLUMN_ADDRESS,
	0x00, WIDTH-1,
	SET_ROW_ADDRESS,
	0x00, HEIGHT-1,
	SET_REMAP,
	0x32,
	
	U8G_ESC_CS(0),
	U8G_ESC_END
};

static const uint8_t u8g_dev_ssd1331_96x64_hicolor_column_seq[] PROGMEM = {
	U8G_ESC_CS(1),
	U8G_ESC_ADR(0),

	SET_COLUMN_ADDRESS,
	0x00, WIDTH-1,
	SET_ROW_ADDRESS,
	0x00, HEIGHT-1,
	SET_REMAP,
	0xb2,
	
	U8G_ESC_CS(0),
	U8G_ESC_END
};

#define RGB332_STREAM_BYTES 8
static uint8_t u8g_ssd1331_stream_bytes[RGB332_STREAM_BYTES*3];

void u8g_ssd1331_to_stream(uint8_t *ptr)
{
  uint8_t cnt = RGB332_STREAM_BYTES;
  uint8_t val;
  uint8_t *dest = u8g_ssd1331_stream_bytes;
  for( cnt = 0; cnt < RGB332_STREAM_BYTES; cnt++ )
  {
      val = *ptr++;
      *dest++ = ((val & 0xe0) >> 2);
      *dest++ = ((val & 0x1c) << 1);
      *dest++ = ((val & 0x03) << 4);
  } 
}


uint8_t u8g_dev_ssd1331_96x64_332_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg)
{
  switch(msg)
    {
    case U8G_DEV_MSG_INIT:
      u8g_InitCom(u8g, dev, U8G_SPI_CLK_CYCLE_50NS);
      u8g_WriteEscSeqP(u8g, dev, u8g_dev_ssd1331_96x64_init_seq);
      break;
      
    case U8G_DEV_MSG_STOP:
      break;
      
    case U8G_DEV_MSG_PAGE_FIRST:
      u8g_WriteEscSeqP(u8g, dev, u8g_dev_ssd1331_96x64_332_column_seq);
      break;
      
    case U8G_DEV_MSG_PAGE_NEXT:
      {
	u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);
	uint8_t *ptr = pb->buf;
	uint8_t page_height;
	int j;
	
	page_height = pb->p.page_y1;
	page_height -= pb->p.page_y0;
	page_height++;
	
	u8g_SetChipSelect(u8g, dev, 1);
	u8g_SetAddress(u8g, dev, 1);
	
	for( j = 0; j < page_height; j++ )
	  {
	    u8g_WriteSequence(u8g, dev, pb->width, ptr);
	    ptr += pb->width;
	  }
	
	u8g_SetChipSelect(u8g, dev, 0);
      }
      
      break;
    case U8G_DEV_MSG_GET_MODE:
      return U8G_MODE_R3G3B2;
    }
  
  return u8g_dev_pb8h8_base_fn(u8g, dev, msg, arg);
}

void u8g_ssd1331_hicolor_to_stream(uint8_t *ptr)
{
  register uint8_t cnt = RGB332_STREAM_BYTES;
  register uint8_t low, high, r, g, b;
  uint8_t *dest = u8g_ssd1331_stream_bytes;
  for( cnt = 0; cnt < RGB332_STREAM_BYTES; cnt++ )
  {
    low = *ptr++;
    high = *ptr++;
    
    r = high & ~7;
    r >>= 2;
    b = low & 31;
    b <<= 1;
    g = high & 7;
    g <<= 3;
    g |= (low>>5)&7;
    
    *dest++ = r;
    *dest++ = g;
    *dest++ = b;
  } 
}

uint8_t u8g_dev_ssd1331_96x64_hicolor_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg)
{
  switch(msg)
    {
    case U8G_DEV_MSG_INIT:
      u8g_InitCom(u8g, dev, U8G_SPI_CLK_CYCLE_50NS);
      u8g_WriteEscSeqP(u8g, dev, u8g_dev_ssd1331_96x64_init_seq);
      break;
    case U8G_DEV_MSG_STOP:
      break;
    case U8G_DEV_MSG_PAGE_FIRST:
      u8g_WriteEscSeqP(u8g, dev, u8g_dev_ssd1331_96x64_hicolor_column_seq);
      break;
    case U8G_DEV_MSG_PAGE_NEXT:
      {
	u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);
	uint8_t i, j;
	uint8_t page_height;
	uint8_t *ptr = pb->buf;
	
	u8g_SetChipSelect(u8g, dev, 1);
	u8g_SetAddress(u8g, dev, 1);
	
	page_height = pb->p.page_y1;
	page_height -= pb->p.page_y0;
	page_height++;
	
	for( j = 0; j < page_height; j++ )
	  {
	    for (i = 0; i < pb->width; i+=RGB332_STREAM_BYTES)
	      {
		u8g_ssd1331_hicolor_to_stream(ptr);
		u8g_WriteSequence(u8g, dev, RGB332_STREAM_BYTES*3, u8g_ssd1331_stream_bytes);
		ptr += RGB332_STREAM_BYTES*2;
	      }	  
	  }
	
	u8g_SetChipSelect(u8g, dev, 0);
      }
      break;    /* continue to base fn */
    case U8G_DEV_MSG_GET_MODE:
      return U8G_MODE_HICOLOR;
    }
  return u8g_dev_pbxh16_base_fn(u8g, dev, msg, arg);
}

uint8_t u8g_dev_ssd1331_96x64_byte_buf[WIDTH*PAGE_HEIGHT] U8G_NOCOMMON ; 

u8g_pb_t u8g_dev_ssd1331_96x64_byte_pb = { {PAGE_HEIGHT, HEIGHT, 0, 0, 0},  WIDTH, u8g_dev_ssd1331_96x64_byte_buf};  
u8g_dev_t u8g_dev_ssd1331_96x64_332_sw_spi = { u8g_dev_ssd1331_96x64_332_fn, &u8g_dev_ssd1331_96x64_byte_pb, U8G_COM_SW_SPI };
u8g_dev_t u8g_dev_ssd1331_96x64_332_hw_spi = { u8g_dev_ssd1331_96x64_332_fn, &u8g_dev_ssd1331_96x64_byte_pb, U8G_COM_HW_SPI };

/* only half of the height, because two bytes are needed for one pixel */
u8g_pb_t u8g_dev_ssd1331_96x64_hicolor_byte_pb = { {PAGE_HEIGHT/2, HEIGHT, 0, 0, 0},  WIDTH, u8g_dev_ssd1331_96x64_byte_buf}; 
u8g_dev_t u8g_dev_ssd1331_96x64_hicolor_sw_spi = { u8g_dev_ssd1331_96x64_hicolor_fn, &u8g_dev_ssd1331_96x64_hicolor_byte_pb, U8G_COM_SW_SPI };
u8g_dev_t u8g_dev_ssd1331_96x64_hicolor_hw_spi = { u8g_dev_ssd1331_96x64_hicolor_fn, &u8g_dev_ssd1331_96x64_hicolor_byte_pb, U8G_COM_HW_SPI };

uint8_t u8g_dev_ssd1331_96x64_4x_byte_buf[WIDTH*PAGE_HEIGHT*4] U8G_NOCOMMON;

u8g_pb_t u8g_dev_ssd1331_96x64_4x_332_byte_pb = { {PAGE_HEIGHT*4, HEIGHT, 0, 0, 0},  WIDTH, u8g_dev_ssd1331_96x64_4x_byte_buf};  
u8g_dev_t u8g_dev_ssd1331_96x64_4x_332_sw_spi = { u8g_dev_ssd1331_96x64_332_fn, &u8g_dev_ssd1331_96x64_4x_332_byte_pb, U8G_COM_SW_SPI };
u8g_dev_t u8g_dev_ssd1331_96x64_4x_332_hw_spi = { u8g_dev_ssd1331_96x64_332_fn, &u8g_dev_ssd1331_96x64_4x_332_byte_pb, U8G_COM_HW_SPI };

u8g_pb_t u8g_dev_ssd1331_96x64_4x_hicolor_byte_pb = { {PAGE_HEIGHT/2*4, HEIGHT, 0, 0, 0},  WIDTH, u8g_dev_ssd1331_96x64_4x_byte_buf}; 
u8g_dev_t u8g_dev_ssd1331_96x64_4x_hicolor_sw_spi = { u8g_dev_ssd1331_96x64_hicolor_fn, &u8g_dev_ssd1331_96x64_4x_hicolor_byte_pb, U8G_COM_SW_SPI };
u8g_dev_t u8g_dev_ssd1331_96x64_4x_hicolor_hw_spi = { u8g_dev_ssd1331_96x64_hicolor_fn, &u8g_dev_ssd1331_96x64_4x_hicolor_byte_pb, U8G_COM_HW_SPI };


/*
U8G_PB_DEV(u8g_dev_ssd1331_96x64_332_sw_spi, WIDTH, HEIGHT, PAGE_HEIGHT, u8g_dev_ssd1331_96x64_332_fn, U8G_COM_SW_SPI);
U8G_PB_DEV(u8g_dev_ssd1331_96x64_332_hw_spi, WIDTH, HEIGHT, PAGE_HEIGHT, u8g_dev_ssd1331_96x64_332_fn, U8G_COM_HW_SPI);

U8G_PB_DEV(u8g_dev_ssd1331_96x64_hicolor_sw_spi, WIDTH, HEIGHT, PAGE_HEIGHT, u8g_dev_ssd1331_96x64_idx_fn, U8G_COM_SW_SPI);
U8G_PB_DEV(u8g_dev_ssd1331_96x64_hicolor_hw_spi, WIDTH, HEIGHT, PAGE_HEIGHT, u8g_dev_ssd1331_96x64_idx_fn, U8G_COM_HW_SPI);
*/
