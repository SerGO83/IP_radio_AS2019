#include "vs1053.h"
#include "main.h"
//#include "ssd1306_fonts.h"
extern SPI_HandleTypeDef hspi1;
//extern void ssd1351_easyspintf (uint16_t GotoX,uint16_t GotoY, char* str, FontDef Font, uint16_t color);
//char ssd1306_buff[32];

/* Private variables ---------------------------------------------------------*/
#define	__WAIT_DREQ()		while(!HAL_GPIO_ReadPin(SPI3_DREQ_GPIO_Port,SPI3_DREQ_Pin))								// wait DREQ = 1
#define __IS_DREQ()     HAL_GPIO_ReadPin(SPI3_DREQ_GPIO_Port,SPI3_DREQ_Pin)												//read DREQ
#define	__XCS_CLR()			HAL_GPIO_WritePin(SPI3_CS_GPIO_Port,SPI3_CS_Pin,GPIO_PIN_RESET)						// XCS = 0
#define	__XCS_SET()			HAL_GPIO_WritePin(SPI3_CS_GPIO_Port,SPI3_CS_Pin,GPIO_PIN_SET)							// XCS = 1
#define __XRESET_CLR()	HAL_GPIO_WritePin(SPI3_RESET_GPIO_Port,SPI3_RESET_Pin,GPIO_PIN_RESET)					// XRESET = 0
#define __XRESET_SET()	HAL_GPIO_WritePin(SPI3_RESET_GPIO_Port,SPI3_RESET_Pin,GPIO_PIN_SET)					// XRESET = 1
#define __XDCS_CLR()		HAL_GPIO_WritePin(SPI3_DCS_GPIO_Port,SPI3_DCS_Pin,GPIO_PIN_RESET) 					// XDCS = 0
#define __XDCS_SET()		HAL_GPIO_WritePin(SPI3_DCS_GPIO_Port,SPI3_DCS_Pin,GPIO_PIN_SET) 						// XDCS = 1
//#define __MIDI_OFF()		HAL_GPIO_WritePin(MIDI_EN_GPIO_Port,MIDI_EN_Pin,GPIO_PIN_RESET) 				// VS1053_MIDI_EN = 0
//#define __MIDI_ON()			HAL_GPIO_WritePin(MIDI_EN_GPIO_Port,MIDI_EN_Pin,GPIO_PIN_SET) 				// VS1053_MIDI_EN = 1

uint16_t vs1053_buffer;
extern uint32_t pnt_indx_mp3;

//Transmite to command register in vs1053
void vs1053_WriteRegister(uint8_t adress, uint16_t data)
{
	__WAIT_DREQ();
	__XCS_CLR();
	
	uint8_t vs1053_sBuff[4] = {0x02, 0, 0, 0}; //Construct massive for send: 0x02, addres_register, hibite, lowbite
	vs1053_sBuff[1] = adress;
	vs1053_sBuff[2] = (uint8_t)((uint16_t)data>>8);
	vs1053_sBuff[3] = (uint8_t)((uint16_t)data&0x00FF);
	HAL_SPI_Transmit(&hspi1,vs1053_sBuff,4,100); //Transmitt massive
	
	__XCS_SET();
	__WAIT_DREQ();
}

//Read from command register in vs1053
uint16_t vs1053_ReadRegister(uint8_t adress)
{
	__WAIT_DREQ();
	__XCS_CLR();
	
	uint8_t vs1053_sBuffRx[4]; //Construct massive for receive: not care byte, not care byte, hiRxByte, lowRxByte
	uint8_t vs1053_sBuffTx[4] = {0x03, 0, 0, 0}; //Construct massive for send: 0x02, addres_register, not care byte, not care byte
	vs1053_sBuffTx[1] = adress;
	HAL_SPI_TransmitReceive(&hspi1,vs1053_sBuffTx,vs1053_sBuffRx,4,100);
	
	uint16_t dataRx = (vs1053_sBuffRx[2]<<8) | vs1053_sBuffRx[3]; //Get 16bit register from vs1053_sBuffRx
	
	__XCS_SET();
	__WAIT_DREQ();
	
	return dataRx;
}

void vs1053_PlayMp3  (uint8_t *data, uint32_t size) {
	while ( __IS_DREQ()){
			__XCS_SET();
			__XDCS_CLR();
			HAL_SPI_Transmit(&hspi1, &data[pnt_indx_mp3], 32, 100);
			pnt_indx_mp3 +=32;
			if (pnt_indx_mp3>size) 
				{
				HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
				pnt_indx_mp3 = 0;
				}
				__XDCS_SET();
	}
}

//Transmite to data register in vs1053 (in VS1002 native SPI modes)
void vs1053_WriteData(uint8_t *data, uint32_t size)
{
	__WAIT_DREQ();
  __XCS_SET();

	uint32_t i=0;
	while(i<size)
	{
		__WAIT_DREQ();
		__XDCS_CLR();
		HAL_SPI_Transmit(&hspi1, &data[i], 32, 100);
		i += 32;
		__XDCS_SET();
	}

}
 
//This function set 500kBit/s speed spi
void vs1053_setLowSpeedSPI (void)
{
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; //Speed 500kBit/s
	HAL_SPI_Init(&hspi1);
}

//This function set 1MBit/s speed spi
void vs1053_setHiSpeedSPI (void)
{
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; //Speed 1MBit/s
	HAL_SPI_Init(&hspi1);
}

//Start vs1053
void vs1053_start (void)
{
//	ssd1351_easyspintf_easyspintf(0,0,"Reset VS    ", Font_11x18);

	__XRESET_CLR();
	__XCS_CLR();
	__XDCS_SET();

	//From page 12 of datasheet, max SCI reads are CLKI/7. Input clock is 12.288MHz. 
	//Internal clock multiplier is 1.0x after power up. 
	//Therefore, max SPI speed is 1.75MHz. We will use 1MHz to be safe.
	//application note page 30. After a software or hardware reset, first set your SPI clock speed to 2 MHz or lower
	vs1053_setLowSpeedSPI();

	HAL_Delay(1);

	//Bring up VS1053
	__XRESET_SET();
	HAL_Delay(5);
	__WAIT_DREQ();

	 vs1053_WriteRegister(SCI_AUDATA,44101); // 44.1kHz stereo
	//WARNING: But in datasheet 10.2 Hardware Reset haven't this operation! (p.49)
	//applitation note 4.4 Writing to SCI (11, 02)
	vs1053_WriteRegister(SCI_MODE,SM_RESET | SM_SDINEW);
	HAL_Delay(100);

	//Set hi speed
	vs1053_setHiSpeedSPI(); 

//	ssd1306_easyspintf(0,20,"VS ready    ", Font_11x18);
}

//Set midi_run = 0 for off rt midi or midi_run = 1 for run rt midi. If rt midi mode run function return 0xAC45
uint16_t vs1053_midi_mode (uint8_t midi_run)
{
//	if (midi_run)
//		__MIDI_ON();
//	else
//		__MIDI_OFF();
	vs1053_WriteRegister(SCI_MODE,SM_RESET | SM_SDINEW);	
	HAL_Delay(1);
	
	uint16_t vs_audata = vs1053_ReadRegister(SCI_AUDATA);
//	if (vs_audata == 0xAC45)
//	 ssd1306_easyspintf(0,0,"VS MidiMode    ", Font_11x18);
//	else
//	 ssd1306_easyspintf(0,0,"VS NormlMode  ", Font_11x18);
	return vs_audata;
}

//Argument: chanel = 1 - 9; 
void vs1053_midi_note (uint8_t chanel, uint8_t note, uint8_t velocity)
{
	uint8_t midi_massive[6] = {0x90, 0x00, 50, 0x00, 0x7f, 0x00};
	midi_massive[0] = 0x90 + chanel;	//Chanel
	midi_massive[2] = note;						//Note
	midi_massive[4] = velocity;				//Velocity
	vs1053_WriteData (	midi_massive, 6);	
}

//Test volume
void vs1053_sound_loop (uint8_t how_loop)
{
//	ssd1306_easyspintf(0,0,"loopSound", Font_11x18);
	
	//applitation note 4.5 Writing to SCI (01)
	//A good test is to try to switch the volume setting from powersave mode to full volume. 
	//This will cause slight snapping sounds, which can be checked with the earphones.
	for(uint8_t i = 0; i<how_loop ; i ++)
	{
		vs1053_WriteRegister(SCI_VOL,0x0000); //Volume = Max
		HAL_Delay(500);
		vs1053_WriteRegister(SCI_VOL,0xFFFF); //Volume = Mute
		HAL_Delay(500);
			
	}
//	ssd1306_easyspintf(0,0,"             ", Font_11x18);
}

//Test write and read from command register in vs1053
void vs1053_test_sci (void)
{
//	ssd1306_easyspintf(0,0,"Test VS    ", Font_11x18);
				//Read mode
	vs1053_buffer = vs1053_ReadRegister(SCI_MODE);	
//	sprintf(ssd1306_buff,"mode=0x%04X",vs1053_buffer);
//	ssd1306_easyspintf(0,32,ssd1306_buff, Font_7x10);

			//Read status
	vs1053_buffer = vs1053_ReadRegister(SCI_STATUS);	
//	sprintf(ssd1306_buff,"status=0x%04X",vs1053_buffer);
//	ssd1306_easyspintf(0,42,ssd1306_buff, Font_7x10);

			//Read sound register
	vs1053_WriteRegister(SCI_VOL,0xa2f5); //Volume = Mute
	vs1053_buffer = vs1053_ReadRegister(SCI_VOL);
//	sprintf(ssd1306_buff,"volume=0x%04X",vs1053_buffer);
//	ssd1306_easyspintf(0,52,ssd1306_buff, Font_7x10);

	HAL_Delay(500);
	
	//This test sound work
	vs1053_sound_loop (10);
}

//Sinus test
void vs1053_sin_test (uint16_t how_loop)
{
	//datasheet p.63 9.12.1 Sine Test  and p.15 an 4.8 Writing to SDI (11ӳ and 02ӳ new mode)
	vs1053_WriteRegister(0x00, 0x0820);	
		
	uint8_t send1 [] = {0x53, 0xEF, 0x6E, 0xE1, 0x00, 0x00, 0x00, 0x00};
	uint8_t send2 [] = {0x45, 0x78, 0x69, 0x74, 0x00, 0x00, 0x00, 0x00};

  while (how_loop)
  {
		
		//Datasheet ask: send 4 zero, but for 4 zero sin test not work. Because we send 3 zerro.
		vs1053_WriteData(send1, 8);
//		ssd1306_easyspintf(0,35,"snd on       ", Font_11x18);
		HAL_Delay (1000);
		vs1053_WriteData(send2, 6);
	//	ssd1306_easyspintf(0,35,"snd off       ", Font_11x18);
		HAL_Delay (1000);
		
		send1[3] = (((send1[3]>>3)-1)<<3)+1;
		if	(send1[3] == 0x21) send1[3] = 0xF1;
		how_loop--;
	}
	vs1053_WriteData(send1, 7);
}

void vs1053_music_terminator_theme (void)
{
	uint8_t change_instrument[] = {0xc0, 0x00, 1, 0x00}; //11 //14

	vs1053_WriteData (	change_instrument, sizeof(change_instrument));		
	//def - ecf - def - eca - g		
			
	vs1053_midi_note(0,midi_D3,0x7f);
	HAL_Delay(500);
	vs1053_midi_note(0,midi_E3,0x7f);
	HAL_Delay(500);		
	vs1053_midi_note(0,midi_F3,0x7f);

	HAL_Delay(1000);

	vs1053_midi_note(0,midi_E3,0x7f);
	HAL_Delay(500);
	vs1053_midi_note(0,midi_C3,0x7f);
	HAL_Delay(500);		
	vs1053_midi_note(0,midi_F2,0x7f);

	HAL_Delay(2000);

	vs1053_midi_note(0,midi_D3,0x7f);
	HAL_Delay(500);
	vs1053_midi_note(0,midi_E3,0x7f);
	HAL_Delay(500);		
	vs1053_midi_note(0,midi_F3,0x7f);

	HAL_Delay(1000);

	vs1053_midi_note(0,midi_E3,0x7f);
	HAL_Delay(500);
	vs1053_midi_note(0,midi_C3,0x7f);
	HAL_Delay(500);		
	vs1053_midi_note(0,midi_A4,0x7f);

	HAL_Delay(1000);

	vs1053_midi_note(0,midi_G4,0x7f);
	//norm
	HAL_Delay(2000);	

	vs1053_midi_note(0,midi_D3,0x7f);
	HAL_Delay(500);
	vs1053_midi_note(0,midi_E3,0x7f);
	HAL_Delay(500);		
	vs1053_midi_note(0,midi_F3,0x7f);

	HAL_Delay(1000);	

	vs1053_midi_note(0,midi_E3,0x7f);
	HAL_Delay(500);
	vs1053_midi_note(0,midi_C3,0x7f);
	HAL_Delay(500);		
	vs1053_midi_note(0,midi_G2,0x7f);

	HAL_Delay(2200);
	vs1053_midi_note(0,midi_F2,0x7f);
	HAL_Delay(2000);

	vs1053_midi_note(0,midi_D2,0x7f);
	HAL_Delay(500);
	vs1053_midi_note(0,midi_F2,0x7f);
	HAL_Delay(1000);		
	vs1053_midi_note(0,midi_E2,0x7f);

	HAL_Delay(4000);		
}
