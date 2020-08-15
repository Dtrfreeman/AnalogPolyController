
#include "main.h"
#include <stdio.h>
#include "math.h"

uint8_t lAttackToVal(uint8_t curVoice,uint16_t gradient,uint16_t limit);

uint8_t lReleaseToVal(uint8_t curVoice,uint16_t gradient,uint16_t limit);

uint8_t fAttackToVal(uint8_t curVoice,uint16_t gradient,uint16_t limit);

uint8_t fReleaseToVal(uint8_t curVoice,uint16_t gradient,uint16_t limit);

void lADSRstep(uint8_t voiceNum,uint16_t * pADSRvals );

void fADSRstep(uint8_t voiceNum,uint16_t * pADSRvals );


struct voice{
	uint8_t noteOn;
	uint8_t noteCode;
	uint16_t finalNoteVal;
	uint8_t lState;
	uint8_t fState;
	uint8_t noteVel;
	uint16_t lEnvCnt;
	uint16_t fEnvCnt;
	uint16_t loudnessVal;
	uint16_t filterVal;
	volatile unsigned int * loudnessChannel;
	volatile unsigned int * filterChannel;
	signed int offsetError;
	double multiConst;
};
