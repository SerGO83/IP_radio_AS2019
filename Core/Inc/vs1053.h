#ifndef __vs1053_H
#define __vs1053_H

#include "stm32f7xx_hal.h"

//vs1053 Register
#define SCI_MODE				0x00	//Mode control //After reset = 0x4000, after frimware changes = 0x4800
#define SCI_STATUS			0x01	//Status of VS1053b //After reset = 0x000C
#define SCI_BASS				0x02	//Built-in bass/treble control
#define SCI_CLOCKF			0x03	//Clock freq + multiplier
#define SCI_DECODE_TIME	0x04	//Decode time in seconds
#define SCI_AUDATA			0x05	//Misc. audio data
#define SCI_WRAM				0x06	//RAM write/read
#define SCI_WRAMADDR		0x07	//Base address for RAM write/read
#define SCI_HDAT0				0x08	//Stream header data 0
#define SCI_HDAT1				0x09	//Stream header data 1
#define SCI_AIADDR			0x0A	//Start address of application
#define SCI_VOL					0x0B	//Volume control //after hardware reset the volume is set to full volume
#define SCI_AICTRL0			0x0C	//Application control register 0
#define SCI_AICTRL1			0x0D	//Application control register 1
#define SCI_AICTRL3			0x0E	//Application control register 2
#define SCI_AICTRL4			0x0F	//Application control register 3

//vs1053 bites from Adafruit
#define SM_DIFF 0x0001
#define SM_LAYER12 0x0002
#define SM_RESET 0x0004
#define SM_CANCEL 0x0008
#define SM_EARSPKLO 0x0010
#define SM_TESTS 0x0020
#define SM_STREAM 0x0040
#define SM_SDINEW 0x0800
#define SM_ADPCM 0x1000
#define SM_LINE1 0x4000
#define SM_CLKRANGE 0x8000

// See http://www.vlsi.fi/fileadmin/datasheets/vs1053.pdf Pg 31
#define BANK_DEFAULT 0x00
#define BANK_DRUMS1 0x78
#define BANK_DRUMS2 0x7F
#define BANK_MELODY 0x79

// See http://www.vlsi.fi/fileadmin/datasheets/vs1053.pdf Pg 32 for more!
#define GM1_OCARINA 80

#define MIDI_NOTE_ON  0x90
#define MIDI_NOTE_OFF 0x80
#define MIDI_CHAN_MSG 0xB0
#define MIDI_CHAN_BANK 0x00
#define MIDI_CHAN_VOLUME 0x07
#define MIDI_CHAN_PROGRAM 0xC0

//Note on midi code

#define midi_A0 21
#define midi_B0 23
#define midi_C1 24
#define midi_D1 26
#define midi_E1 28
#define midi_F1 29
#define midi_G1 31

#define midi_A1 33
#define midi_B1 35
#define midi_C2 36
#define midi_D2 38
#define midi_E2 40
#define midi_F2 41
#define midi_G2 43

#define midi_A2 45
#define midi_B2 47
#define midi_C3 48
#define midi_D3 50
#define midi_E3 52
#define midi_F3 53
#define midi_G3 55

#define midi_A3 57
#define midi_B3 59
#define midi_C4 60
#define midi_D4 62
#define midi_E4 64
#define midi_F4 65
#define midi_G4 67

#define midi_A4 69
#define midi_B4 71
#define midi_C5 72
#define midi_D5 74
#define midi_E5 76
#define midi_F5 77
#define midi_G5 79

#define midi_A5 81
#define midi_B5 83
#define midi_C6 84
#define midi_D6 86
#define midi_E6 88
#define midi_F6 89
#define midi_G6 91

#define midi_A6 93
#define midi_B6 95
#define midi_C7 96
#define midi_D7 98
#define midi_E7 100
#define midi_F7 101
#define midi_G7 103

#define midi_A7 105
#define midi_B7 107
#define midi_C8 108


#define midi_A0b 22

#define midi_C1b 25
#define midi_D1b 27

#define midi_F1b 30
#define midi_G1b 32
#define midi_A1b 34

#define midi_C2b 37
#define midi_D2b 39

#define midi_F2b 42
#define midi_G2b 44
#define midi_A2b 46

#define midi_C3b 49
#define midi_D3b 51

#define midi_F3b 54
#define midi_G3b 56
#define midi_A3b 58

#define midi_C4b 61
#define midi_D4b 63

#define midi_F4b 66
#define midi_G4b 68
#define midi_A4b 70

#define midi_C5b 73
#define midi_D5b 75

#define midi_F5b 78
#define midi_G5b 80
#define midi_A5b 82

#define midi_C6b 85
#define midi_D6b 87

#define midi_F6b 90
#define midi_G6b 92
#define midi_A6b 94

#define midi_C7b 97
#define midi_D7b 99

#define midi_F7b 102
#define midi_G7b 104
#define midi_A7b 106

void vs1053_start (void);
void vs1053_setLowSpeedSPI (void);
void vs1053_setHiSpeedSPI (void);
uint16_t vs1053_midi_mode (uint8_t midi_run);
void vs1053_WriteRegister(uint8_t adress, uint16_t data);
uint16_t vs1053_ReadRegister(uint8_t adress);
void vs1053_WriteData(uint8_t *data, uint32_t size);
void vs1053_midi_note (uint8_t chanel, uint8_t note, uint8_t velocity);
void vs1053_sound_loop (uint8_t how_loop);
void vs1053_test_sci (void);
void vs1053_sin_test (uint16_t how_loop);
void vs1053_music_terminator_theme (void);

void vs1053_PlayMp3  (uint8_t *data, uint32_t size);
#endif /* __vs1053_H */

