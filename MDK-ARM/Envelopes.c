#include "envelope.h"
volatile struct voice VoiceArray[4];


uint8_t lAttackToVal(uint8_t curVoice,uint16_t gradient,uint16_t limit){//returns 0 once the output reaches value
	
	if(VoiceArray[curVoice].loudnessVal==limit){return(0);}//if at limit itll exit with a state complete code
	
	
	else if((VoiceArray[curVoice].loudnessVal+gradient)<limit){
		VoiceArray[curVoice].loudnessVal+=gradient;
		(*VoiceArray[curVoice].loudnessChannel)=VoiceArray[curVoice].loudnessVal>>6;//scales down to 1024 being max
		return(1);
	}//increases by gradient and exits with state continue code
	
	else if(VoiceArray[curVoice].loudnessVal>limit){
		//if for whatever reason its more than limit itll exit with a state complete code
		VoiceArray[curVoice].loudnessVal=limit;
		(*VoiceArray[curVoice].loudnessChannel)=VoiceArray[curVoice].loudnessVal>>6;
		
		return(0);}
	
		
	else if((VoiceArray[curVoice].loudnessVal+gradient)>=limit){//if once the gradient is added itll exceed the limit itll
		
		VoiceArray[curVoice].loudnessVal=limit;
		(*VoiceArray[curVoice].loudnessChannel)=VoiceArray[curVoice].loudnessVal>>6;
		return(0);
	}
	
	
	return(0);
}
uint8_t lReleaseToVal(uint8_t curVoice,uint16_t gradient,uint16_t limit){//returns 0 once the output reaches limit
	
	if(VoiceArray[curVoice].loudnessVal==limit){return(0);}//if at limit itll exit with a state complete code
	
	
	else if((VoiceArray[curVoice].loudnessVal-gradient)>limit){
		VoiceArray[curVoice].loudnessVal-=gradient;
		(*VoiceArray[curVoice].loudnessChannel)=VoiceArray[curVoice].loudnessVal>>6;//scales down to 1024 being max
		return(1);
	}//increases by gradient and exits with state continue code
	
		
	else if((VoiceArray[curVoice].loudnessVal-gradient)<=limit){//if once the gradient is added itll exceed the limit itll
		
		VoiceArray[curVoice].loudnessVal=limit;
		(*VoiceArray[curVoice].loudnessChannel)=VoiceArray[curVoice].loudnessVal>>6;
		return(0);
	}
	
	
	return(0);
}


uint8_t fAttackToVal(uint8_t curVoice,uint16_t gradient,uint16_t limit){//returns 0 once the output reaches value
	
	if(VoiceArray[curVoice].filterVal==limit){return(0);}//if at limit itll exit witha state complete code
	
	
	else if((VoiceArray[curVoice].filterVal+gradient)<limit){
		VoiceArray[curVoice].filterVal+=gradient;
		(*VoiceArray[curVoice].filterChannel)=VoiceArray[curVoice].filterVal>>6;//scales down to 1024 being max
		return(1);
	}//increases by gradient and exits with state continue code
	
	else if(VoiceArray[curVoice].filterVal>limit){
		//if for whatever reason its more than limit itll exit with a state complete code
		VoiceArray[curVoice].filterVal=limit;
		(*VoiceArray[curVoice].filterChannel)=VoiceArray[curVoice].filterVal>>6;
		
		return(0);}
	
		
	else if((VoiceArray[curVoice].filterVal+gradient)>=limit){//if once the gradient is added itll exceed the limit itll
		
		VoiceArray[curVoice].filterVal=limit;
		(*VoiceArray[curVoice].filterChannel)=VoiceArray[curVoice].filterVal>>6;
		return(0);
	}
	
	
	return(0);
}
uint8_t fReleaseToVal(uint8_t curVoice,uint16_t gradient,uint16_t limit){//returns 0 once the output reaches limit

	if(VoiceArray[curVoice].filterVal==limit){return(0);}//if at limit itll exit with a state complete code
	
	
	else if((VoiceArray[curVoice].filterVal-gradient)>limit){
		VoiceArray[curVoice].filterVal-=gradient;
		(*VoiceArray[curVoice].filterChannel)=VoiceArray[curVoice].filterVal>>6;//scales down to 1024 being max
		return(1);
	}//increases by gradient and exits with state continue code
	
		
	else if((VoiceArray[curVoice].filterVal-gradient)<=limit){//if once the gradient is added itll exceed the limit itll
		
		VoiceArray[curVoice].filterVal=limit;
		(*VoiceArray[curVoice].filterChannel)=VoiceArray[curVoice].filterVal>>6;
		return(0);
	}
	
	
	return(0);
}


/*	    __
			 /  \                         _______
      /    \                       /       \ 
		 /      \_________           	/         \
	  /                 \          /           \
	 /                   \        /             \ 
  /                     \      /               \
-><----><><-><-------><-><----><><-><-----><---><----
0	   1  2  3     4     5   0    1 6    4     5    0

ADSRvals formatted 0:lAttack,1:lDelay,2:lSustain,3:lRelease,4:fAttack,5:fDelay,6:fSustain,7:fRelease,8:fInfluence
*/


void lADSRstep(uint8_t voiceNum,uint16_t * pADSRvals ){
	switch(VoiceArray[voiceNum].lState){
		case (0):{return;}
		
		case (1):{//attack
			if(VoiceArray[voiceNum].lEnvCnt< (*(pADSRvals+1))){
				if(lAttackToVal(voiceNum,*(pADSRvals),0xffff)==0){
					VoiceArray[voiceNum].lState=2;
				}
				VoiceArray[voiceNum].lEnvCnt++;
			}
			else{
					if(VoiceArray[voiceNum].loudnessVal==(*pADSRvals+2)){
						VoiceArray[voiceNum].lState=4;//goes straight into sustain state
					}
					else if(VoiceArray[voiceNum].loudnessVal>(*(pADSRvals+2))){
						VoiceArray[voiceNum].lState=3;}//releases to sustain
					else{VoiceArray[voiceNum].lState=6;}//attacks to sustain	
			}
			if(VoiceArray[voiceNum].noteOn==0){
				VoiceArray[voiceNum].lState=5;
			}
			
			return;
			
		}
		
		case (2):{//once attack has reached peak value
			if(VoiceArray[voiceNum].lEnvCnt>= (*(pADSRvals+1))){
				VoiceArray[voiceNum].lState=3;
			}
			else{VoiceArray[voiceNum].lEnvCnt++;}
			return;
		}
		
		case (3):{//releasing to sustain
			if(lReleaseToVal(voiceNum,*(pADSRvals+3),*(pADSRvals+2))==0){VoiceArray[voiceNum].lState=4;}
			return;
		}
		
		case 4:return;
		case 5:{//releasing to 0
			if(lReleaseToVal(voiceNum,*(pADSRvals+3),0)==0){VoiceArray[voiceNum].lState=0;}
			return;
		}
		
		case 6:{//attacking to sustain after delay
			if(lAttackToVal(voiceNum,*(pADSRvals+3),*(pADSRvals+2))==0){VoiceArray[voiceNum].lState=4;}
		}
		
		
	}


}
void fADSRstep(uint8_t voiceNum,uint16_t * pADSRvals ){
	switch(VoiceArray[voiceNum].fState){
		case 0:{return;}
		
		case 1:{//attack
			if(VoiceArray[voiceNum].fEnvCnt< (*(pADSRvals+5))){
				if(fAttackToVal(voiceNum,*(pADSRvals+4),*(pADSRvals+8))==0){
					VoiceArray[voiceNum].fState=2;
				}
				VoiceArray[voiceNum].fEnvCnt++;
			}
			else{
					if(VoiceArray[voiceNum].filterVal==(*pADSRvals+6)){
						VoiceArray[voiceNum].fState=4;//goes straight into sustain state
					}
					else if(VoiceArray[voiceNum].filterVal>(*(pADSRvals+6))){
						VoiceArray[voiceNum].fState=3;}//releases to sustain
					else{VoiceArray[voiceNum].fState=6;}//attacks to sustain	
			}
			if(VoiceArray[voiceNum].noteOn==0){
				VoiceArray[voiceNum].fState=5;
			}
			
			return;
			
		}
		
		case 2:{//once attack has reached peak value
			if(VoiceArray[voiceNum].fEnvCnt>= (*(pADSRvals+5))){
				VoiceArray[voiceNum].fState=3;
			}
			else{VoiceArray[voiceNum].fEnvCnt++;}
			return;
		}
		
		case 3:{//releasing to sustain
			if(fReleaseToVal(voiceNum,*(pADSRvals+7),*(pADSRvals+6))==0){VoiceArray[voiceNum].fState=4;}
			return;
		}
		
		case 4:return;
		case 5:{//releasing to 0
			if(fReleaseToVal(voiceNum,*(pADSRvals+7),0)==0){VoiceArray[voiceNum].fState=0;}
			return;
		}
		
		case 6:{//attacking to sustain after delay
			if(fAttackToVal(voiceNum,*(pADSRvals+7),*(pADSRvals+6))==0){VoiceArray[voiceNum].fState=4;}
		}
		
		
	}


}
