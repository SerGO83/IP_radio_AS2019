/* vim: set ai et ts=4 sw=4: */
#include "stm32f7xx_hal.h"
#include "ssd1351.h"
#include "main.h"
#include "math.h"

#define COLORSWAP ((color>>8)&0x00FF) | ((color<<8)&0xFF00)
uint16_t videoram[128][128]={0};

static void SSD1351_Select() {
    HAL_GPIO_WritePin(SSD1351_CS_GPIO_Port, SSD1351_CS_Pin, GPIO_PIN_RESET);
}

void SSD1351_Unselect() {
    HAL_GPIO_WritePin(SSD1351_CS_GPIO_Port, SSD1351_CS_Pin, GPIO_PIN_SET);
}

static void SSD1351_Reset() {
    HAL_GPIO_WritePin(SSD1351_RES_GPIO_Port, SSD1351_RES_Pin, GPIO_PIN_SET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(SSD1351_RES_GPIO_Port, SSD1351_RES_Pin, GPIO_PIN_RESET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(SSD1351_RES_GPIO_Port, SSD1351_RES_Pin, GPIO_PIN_SET);
    HAL_Delay(500);
}

/*static*/ void SSD1351_WriteCommand(uint8_t cmd) {
    HAL_GPIO_WritePin(SSD1351_DC_GPIO_Port, SSD1351_DC_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&SSD1351_SPI_PORT, &cmd, sizeof(cmd), HAL_MAX_DELAY);
}

static void SSD1351_WriteData(uint8_t* buff, size_t buff_size) {
    HAL_GPIO_WritePin(SSD1351_DC_GPIO_Port, SSD1351_DC_Pin, GPIO_PIN_SET);

    // split data in small chunks because HAL can't send more then 64K at once
    while(buff_size > 0) {
        uint16_t chunk_size = buff_size > 32768 ? 32768 : buff_size;
        HAL_SPI_Transmit(&SSD1351_SPI_PORT, buff, chunk_size, HAL_MAX_DELAY);
        buff += chunk_size;
        buff_size -= chunk_size;
    }
}

static void SSD1351_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    // column address set
    SSD1351_WriteCommand(0x15); // SETCOLUMN
    {
        uint8_t data[] = { x0 & 0xFF, x1 & 0xFF };
        SSD1351_WriteData(data, sizeof(data));
    }

    // row address set
    SSD1351_WriteCommand(0x75); // SETROW
    {
        uint8_t data[] = { y0 & 0xFF, y1 & 0xFF };
        SSD1351_WriteData(data, sizeof(data));
    }

    // write to RAM
    SSD1351_WriteCommand(0x5C); // WRITERAM
}

void SSD1351_Init() {
    SSD1351_Select();
    SSD1351_Reset();

    // command list is based on https://github.com/adafruit/Adafruit-SSD1351-library

    SSD1351_WriteCommand(0xFD); // COMMANDLOCK
    {
        uint8_t data[] = { 0x12 };
        SSD1351_WriteData(data, sizeof(data));
    }
    SSD1351_WriteCommand(0xFD); // COMMANDLOCK
    {
        uint8_t data[] = { 0xB1 };
        SSD1351_WriteData(data, sizeof(data));
    }
//    SSD1351_WriteCommand(0xAE); // DISPLAYOFF
    SSD1351_WriteCommand(0xB3); // CLOCKDIV
    SSD1351_WriteCommand(0xF1); // 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
    SSD1351_WriteCommand(0xCA); // MUXRATIO
    {
        uint8_t data[] = { 0x7F }; // 127
        SSD1351_WriteData(data, sizeof(data));
    }
    SSD1351_WriteCommand(0xA0); // SETREMAP
    {
        uint8_t data[] = { 0x74 };
        SSD1351_WriteData(data, sizeof(data));
    }
    SSD1351_WriteCommand(0x15); // SETCOLUMN
    {
    	uint8_t data[] = { 0x00, 0x7F };
        SSD1351_WriteData(data, sizeof(data));
    }
    SSD1351_WriteCommand(0x75); // SETROW
    {
        uint8_t data[] = { 0x00, 0x7F };
        SSD1351_WriteData(data, sizeof(data));
    }
    SSD1351_WriteCommand(0xA1); // STARTLINE
    {
        uint8_t data[] = { 0x00 }; // 96 if display height == 96
        SSD1351_WriteData(data, sizeof(data));
    }
    SSD1351_WriteCommand(0xA2); // DISPLAYOFFSET
    {
        uint8_t data[] = { 0x00 };
        SSD1351_WriteData(data, sizeof(data));
    }
    SSD1351_WriteCommand(0xB5); // SETGPIO
    {
        uint8_t data[] = { 0x00 };
        SSD1351_WriteData(data, sizeof(data));
    }
    SSD1351_WriteCommand(0xAB); // FUNCTIONSELECT
    {
        uint8_t data[] = { 0x01 };
        SSD1351_WriteData(data, sizeof(data));
    }
    SSD1351_WriteCommand(0xB1); // PRECHARGE
    {
        uint8_t data[] = { 0x32 };
        SSD1351_WriteData(data, sizeof(data));
    }
    SSD1351_WriteCommand(0xBE); // VCOMH
    {
        uint8_t data[] = { 0x05 };
        SSD1351_WriteData(data, sizeof(data));
    }
    SSD1351_WriteCommand(0xA6); // NORMALDISPLAY (don't invert)
    SSD1351_WriteCommand(0xC1); // CONTRASTABC
    {
        uint8_t data[] = { 0xC8, 0x80, 0xC8 };
        SSD1351_WriteData(data, sizeof(data));
    }
    SSD1351_WriteCommand(0xC7); // CONTRASTMASTER
    {
        uint8_t data[] = { 0x0F };
        SSD1351_WriteData(data, sizeof(data));
    }
    SSD1351_WriteCommand(0xB4); // SETVSL
    {
        uint8_t data[] = { 0xA0, 0xB5, 0x55 };
        SSD1351_WriteData(data, sizeof(data));
    }
    SSD1351_WriteCommand(0xB6); // PRECHARGE2
    {
        uint8_t data[] = { 0x01 };
        SSD1351_WriteData(data, sizeof(data));
    }
    SSD1351_WriteCommand(0xAF); // DISPLAYON

    SSD1351_Unselect();
}

void SSD1351_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    if((x >= SSD1351_WIDTH) || (y >= SSD1351_HEIGHT))
        return;

    SSD1351_Select();

    SSD1351_SetAddressWindow(x, y, x+1, y+1);
    uint8_t data[] = { color >> 8, color & 0xFF };
    SSD1351_WriteData(data, sizeof(data));

    SSD1351_Unselect();
}

static void SSD1351_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor) {
    uint32_t i, b, j;

    SSD1351_SetAddressWindow(x, y, x+font.width-1, y+font.height-1);

    for(i = 0; i < font.height; i++) {
        b = font.data[(ch - 32) * font.height + i];
        for(j = 0; j < font.width; j++) {
            if((b << j) & 0x8000)  {
                uint8_t data[] = { color >> 8, color & 0xFF };
                SSD1351_WriteData(data, sizeof(data));
            } else {
                uint8_t data[] = { bgcolor >> 8, bgcolor & 0xFF };
                SSD1351_WriteData(data, sizeof(data));
            }
        }
    }
}
static void SSD1351_WriteCharVR(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor) {
    uint32_t i, b, j;
		int temp = color;
//    SSD1351_SetAddressWindow(x, y, x+font.width-1, y+font.height-1);

    for(i = 0; i < font.height; i++) {
        b = font.data[(ch - 32) * font.height + i];
        for(j = 0; j < font.width; j++) {
            if((b << j) & 0x8000)  {
//                uint8_t data[] = { color >> 8, color & 0xFF };
//                SSD1351_WriteData(data, sizeof(data));
								color = temp;
								videoram[y+i][x+j] = COLORSWAP;

            } else {
//                uint8_t data[] = { bgcolor >> 8, bgcolor & 0xFF };
//                SSD1351_WriteData(data, sizeof(data));
								color = bgcolor;
								videoram[y+i][x+j] = COLORSWAP;
            }
        }
    }
}

void SSD1351_WriteString(uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor) {
    SSD1351_Select();

    while(*str) {
        if(x + font.width >= SSD1351_WIDTH) {
            x = 0;
            y += font.height;
            if(y + font.height >= SSD1351_HEIGHT) {
                break;
            }

            if(*str == ' ') {
                // skip spaces in the beginning of the new line
                str++;
                continue;
            }
        }

        SSD1351_WriteChar(x, y, *str, font, color, bgcolor);
        x += font.width;
        str++;
    }

    SSD1351_Unselect();
}
void SSD1351_WriteStringVR(uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor) {
//    SSD1351_Select();

    while(*str) {
        if(x + font.width >= SSD1351_WIDTH) {
            x = 0;
            y += font.height;
            if(y + font.height >= SSD1351_HEIGHT) {
                break;
            }

            if(*str == ' ') {
                // skip spaces in the beginning of the new line
                str++;
                continue;
            }
        }

        SSD1351_WriteCharVR(x, y, *str, font, color, bgcolor);
        x += font.width;
        str++;
    }

//    SSD1351_Unselect();
}

void SSD1351_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    // clipping
    if((x >= SSD1351_WIDTH) || (y >= SSD1351_HEIGHT)) return;
    if((x + w - 1) >= SSD1351_WIDTH) w = SSD1351_WIDTH - x;
    if((y + h - 1) >= SSD1351_HEIGHT) h = SSD1351_HEIGHT - y;

    SSD1351_Select();
    SSD1351_SetAddressWindow(x, y, x+w-1, y+h-1);

    uint8_t data[] = { color >> 8, color & 0xFF };
    HAL_GPIO_WritePin(SSD1351_DC_GPIO_Port, SSD1351_DC_Pin, GPIO_PIN_SET);
    for(y = h; y > 0; y--) {
        for(x = w; x > 0; x--) {
            HAL_SPI_Transmit(&SSD1351_SPI_PORT, data, sizeof(data), HAL_MAX_DELAY);
        }
    }

    SSD1351_Unselect();
}

void SSD1351_FillScreen(uint16_t color) {
    SSD1351_FillRectangle(0, 0, SSD1351_WIDTH, SSD1351_HEIGHT, color);
}

void SSD1351_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data) {
    if((x >= SSD1351_WIDTH) || (y >= SSD1351_HEIGHT)) return;
    if((x + w - 1) >= SSD1351_WIDTH) return;
    if((y + h - 1) >= SSD1351_HEIGHT) return;

    SSD1351_Select();
    SSD1351_SetAddressWindow(x, y, x+w-1, y+h-1);
    SSD1351_WriteData((uint8_t*)data, sizeof(uint16_t)*w*h);
    SSD1351_Unselect();
}

void SSD1351_InvertColors(bool invert) {
    SSD1351_Select();
    SSD1351_WriteCommand(invert ? 0xA7 /* INVERTDISPLAY */ : 0xA6 /* NORMALDISPLAY */);
    SSD1351_Unselect();
}

void SSD1351_DrawLine(uint8_t X,uint8_t Y,uint16_t ang,uint16_t len,uint16_t color)
{
	//������� ������
//	int ang = 260;
//	int len = 60;
//	char X = 60;
//	char Y = 60;

	//��������� ����������
	double k,y;
	double dx;
	char sector = 4;
	uint16_t i;

	if ((ang<=45) || (ang>315)) { sector = 0;}
	if ((ang>45) && (ang<=135)) { sector = 1;}
	if ((ang>135) &&(ang<=225)) { sector = 2;}
	if ((ang>225) &&(ang<=315)) { sector = 3;}

	switch (sector) {
	case 0:
		dx = cos (ang * 3.141592/180) * len;
		k =  tan (ang * 3.141592/180);
		for (i = 0; i<dx; i++)
			{
			y = k*i;
			SSD1351_DrawPixel(X + i, round(Y + y), color);
			}
		break;
	case 1:
		dx = cos ((90-ang) * 3.141592/180) * len;
		k =  tan ((90-ang) * 3.141592/180);
		for (i = 0; i<dx; i++)
			{
			y = k*i;
			SSD1351_DrawPixel( round(X + y), Y + i, color);
			}
		break;
	case 2:
		dx = cos (ang * 3.141592/180) * len *(-1);
		k =  tan (ang * 3.141592/180);
		for (i = 0; i<dx; i++)
			{
			y = k*i;
			SSD1351_DrawPixel(X - i, round(Y - y), color);
			}

		break;
	case 3:
		dx = cos ((90-ang) * 3.141592/180) * len * (-1);
		k =  tan ((90-ang) * 3.141592/180);
		for (i = 0; i<dx; i++)
			{
			y = k*i;
			SSD1351_DrawPixel(round(X - y), Y - i, color);
			}
		break;

		default:
			break;
	}
}
void SSD1351_DrawLineVR(uint8_t X,uint8_t Y,uint16_t ang,uint16_t len,uint16_t color)
{
	//������� ������
//	int ang = 260;
//	int len = 60;
//	char X = 60;
//	char Y = 60;

	//��������� ����������
	double k,y;
	double dx;
	char sector = 4;
	uint16_t i;

	if ((ang<=45) || (ang>315)) { sector = 0;}
	if ((ang>45) && (ang<=135)) { sector = 1;}
	if ((ang>135) &&(ang<=225)) { sector = 2;}
	if ((ang>225) &&(ang<=315)) { sector = 3;}

	switch (sector) {
	case 0:
		dx = cos (ang * 3.141592/180) * len;
		k =  tan (ang * 3.141592/180);
		for (i = 0; i<dx; i++)
			{
			y = k*i;
//			SSD1351_DrawPixel(X + i, round(Y + y), color);
			videoram[(uint8_t)(round(Y+y))][X+i] = COLORSWAP;
//			videoram[X+i][(uint8_t)(round(Y+y)+1)] = COLORSWAP;
//			videoram[X+i][(uint8_t)(round(Y+y)-1)] = COLORSWAP;

			}
		break;
	case 1:
		dx = cos ((90-ang) * 3.141592/180) * len;
		k =  tan ((90-ang) * 3.141592/180);
		for (i = 0; i<dx; i++)
			{
			y = k*i;
//			SSD1351_DrawPixel( round(X + y), Y + i, color);
			videoram[Y+i][(uint8_t)(round(X+y))] = COLORSWAP;
//			videoram[(uint8_t)(round(X+y)+1)][Y+i] = COLORSWAP;
//			videoram[(uint8_t)(round(X+y)-1)][Y+i] = COLORSWAP;
			}
		break;
	case 2:
		dx = cos (ang * 3.141592/180) * len *(-1);
		k =  tan (ang * 3.141592/180);
		for (i = 0; i<dx; i++)
			{
			y = k*i;
//			SSD1351_DrawPixel(X - i, round(Y - y), color);
			videoram[(uint8_t)(round(Y-y))] [X-i]= COLORSWAP;
//			videoram[X-i][(uint8_t)(round(Y-y)+1)] = COLORSWAP;
//			videoram[X-i][(uint8_t)(round(Y-y)-1)] = COLORSWAP;

			}

		break;
	case 3:
		dx = cos ((90-ang) * 3.141592/180) * len * (-1);
		k =  tan ((90-ang) * 3.141592/180);
		for (i = 0; i<dx; i++)
			{
			y = k*i;
//			SSD1351_DrawPixel(round(X - y), Y - i, color);
			videoram[Y-i] [(uint8_t)(round(X-y))] = COLORSWAP;
//			videoram[(uint8_t)(round(X-y)+1)][Y-i] = COLORSWAP;
//			videoram[(uint8_t)(round(X-y)-1)][Y-i] = COLORSWAP;
			}
		break;

		default:
			break;
	}
}

void SSD1351_clearVR(){
int i,j;
for (i = 0; i< 128; i++)
	{
	for (j = 0; j<128; j++)
		{
		videoram[i][j] = 0;
		}
	}
}

void SSD1351_FillScreenVR(uint16_t color){
	int i,j;
	for (i = 0; i< 128; i++)
		{
		for (j = 0; j<128; j++)
			{
			videoram[i][j] = COLORSWAP;
			}
		}
	SSD1351_DrawImage(0,0,128,128,(uint16_t*)videoram);
}

void SSD1351_RefreshVR(void){
	SSD1351_DrawImage(0,0,128,128,(uint16_t*)videoram);
}
