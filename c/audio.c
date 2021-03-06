#include <machine/spm.h>
#include <stdio.h>
#include "audio.h"

/*
* @file		Audio_setup.c
* @author	Daniel Sanz Ausin s142290 & Fabian Goerge s150957
* @brief	Setting up all the registers in the audio interface	
*/

/*
* @brief		Writes the supplied data to the address register, 
				sets the request signal and waits for the acknowledge signal. 
* @param[in]	addr	the address of which register to write to. 
						Has to be 7 bit long.
* @param[in]	data	the data thats supposed to be written. 
						Has to be 9 Bits long
* @reutrn		returns 0 if successful and a negative number if there was an error. 
*/
int writeToI2C(char* addrC,char* dataC)
{
	int addr = 0;
	int data = 0;

	//Convert binary String of address to int
	for(int i = 0; i < 7; i++)
	{
		addr *= 2;
 		if (*addrC++ == '1') addr += 1;
	}

	//Convert binary String of data to int
	for(int i = 0; i < 9; i++)
	{
		data *= 2;
 		if (*dataC++ == '1') data += 1;
	}

	//Debug info:
	printf("Sending Data: %i to adress %i\n",data,addr);

	*i2cDataReg = data;
	*i2cAdrReg	= addr;
	*i2cReqReg	= 1;


	while(*i2cAckReg == 0)
	{ 
		printf("Waiting ...\n");
		//Maybe input something like a timeout ...
	}
	for (int i = 0; i<200; i++)  { *i2cReqReg=0; }

	printf("success\n");
	
	return 0;
	
	

}

/*
* @brief	Sets the default values
*/
void setup()
{
  /*
  //----------Line in---------------------
  char *addrLeftIn  = "0000000";		
  char *dataLineIn  = "100010111";	//disable Mute, Enable Simultaneous Load, LinVol: 10111 - Set volume to 23 (of 31)										 
  writeToI2C(addrLeftIn,dataLineIn);

  char *addrRigthIn = "0000001";
  writeToI2C(addrRigthIn,dataLineIn);
  */
  
  //----------Headphones---------------------
  char *addrLeftHead  = "0000010";	
  char *dataHeadphone = "001111001";	// disable simultaneous loads, zero cross disable, LinVol: 1111001 (0db)	
  writeToI2C(addrLeftHead,dataHeadphone);

  char *addrRightHead = "0000011";		
  writeToI2C(addrRightHead,dataHeadphone);

  //--------Analogue Audio Path Control-----
  char *addrAnalogue  = "0000100";	
  char *dataAnalogue = "000010010"; //DAC selected, rest disabled, MIC muted, Line input select to ADC
  writeToI2C(addrAnalogue,dataAnalogue);

  
  //--------Digital Audio Path Control-----
  char *addrDigital  = "0000101";	
  char *dataDigital  = "000000000";	//enable soft mute and disable de-emphasis, enable highpass filter
  writeToI2C(addrDigital,dataDigital);
  
  
  //---------Digital Audio Interface Format---------
  char *addrInterface  = "0000111";	
  char *dataInterface = "000010011"; //BCLK not inverted, slave, right-channel-right-data, LRP = A mode for DSP, 16-bit audio, DSP mode	
  writeToI2C(addrInterface,dataInterface);
  
  
  //--------Sampling Control----------------
  char *addrSample  = "0001000";	
  char *dataSample  = "000000000"; //USB mode, BOSR=1, Sample Rate = 44.1 kHz both for ADC and DAC	
  writeToI2C(addrSample,dataSample);

  printf("FINISHED SETUP!\n");
}


/*
* @brief		Writes the supplied data to the audio interface using the correct protocoll. 
* @param[in]	l	left audio data. Has to be <= 16 Bit. 
* @param[in]	r	right audio data. Has to be <= 16 Bit.  
* @reutrn		returns 0 if successful and a negative number if there was an error. 
*/
int setOutputAudio(short l, short r)
{
	*audioDacLReg = l;
	*audioDacRReg = r;
	return 0;
}

/*
* @brief		this will put the audio from the registers into the value of l and r
* @param[in]	l	here the left audio data will be stored. Has to be <= 16 Bit. 
* @param[in]	r	here the right audio data will be stored. Has to be <= 16 Bit.  
* @reutrn		returns 0 if successful and a negative number if there was an error. 
*/
int getInputAudio(short *l, short *r)
{
	*l = *audioAdcLReg;
	*r = *audioAdcRReg;
	return 0;
}

/*
* @brief		It synchroizes with dac the sampling frequency by waiting for the LRC signal to go from low to high. 
*/
void waitSyncDac()
{
		while (*audioDacLrcReg == 0);
		while (*audioDacLrcReg == 1);
}

/*
* @brief		It synchroizes with the adc sampling frequency by waiting for the LRC signal to go from low to high. 
*/
void waitSyncAdc()
{
		while (*audioAdcLrcReg == 0);
		while (*audioAdcLrcReg == 1);
}

/*
* @brief		changes the volume of the audio output. 
* @param[in]	vol 	in db. Has to be between +6 and -73
* @reutrn		returns 0 if successful and a negative number if there was an error. 
*/
int changeVolume(int vol)
{
	//127--1111111 = +6dB	
	//121--1111001 = 0 dB
	//48--0110000 = -73dB							
  if ((vol<-73) || (vol>6)) {
    return -1;	//Invalid input.
  }

	//Since both Left Channel Zero Cross detect and simultaneous Load are disabled this can be send imideatly:
	//LEFT:
	*i2cDataReg = vol + 121;
	*i2cAdrReg	 = 2;
	*i2cReqReg	 = 1;
	while(*i2cAckReg == 0);
	for (int i = 0; i<200; i++)  { *i2cReqReg=0; }
	
	//RIGHT:
	*i2cDataReg = vol + 121;
	*i2cAdrReg	 = 3;
	*i2cReqReg	 = 1;
	while(*i2cAckReg == 0);
	for (int i = 0; i<200; i++)  { *i2cReqReg=0; }

	return 0;
}






