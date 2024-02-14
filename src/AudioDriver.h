/*
 *  AudioDriver.h
 *
 *  Author: Gary Grutzek
 * 	gary@ib-gru.de
 *
 *
 *  Corrections: Junon Matta
 *  Date: 2021/11/18 20:42
 *  New Settings: Set Slave Mode and 24 bits 
 *
 */

#ifndef AUDIODRIVER_H_
#define AUDIODRIVER_H_

#include "Arduino.h"
// include espressif hw files
#include "driver/gpio.h"
#include "driver/i2s.h"


enum i2s_alignment {
	CODEC_I2S_ALIGN = 0,
	CODEC_LJ_RJ_ALIGN
};


class AudioDriver {

public:

	static const int BufferSize=32; // increase to 64 samples to avoid drop outs (at 192 kHz)
	static const float ScaleFloat2Int;
	static const float ScaleInt2Float;

public:

	int setPins(int bitClkPin=26, int lrClkPin=27, int dataOutPin=14, int dataInPin=13, int enablePin=33);

	void setI2sPort(i2s_port_t i2s_port);

    int setFormat(int fs=48000, int channelCount=2, i2s_bits_per_sample_t bitsPerSample=I2S_BITS_PER_SAMPLE_24BIT, 
       i2s_comm_format_t format=I2S_COMM_FORMAT_I2S, int alignment=CODEC_I2S_ALIGN, int mclkFactor=384, 
       int mode=I2S_MODE_SLAVE | I2S_MODE_RX | I2S_MODE_TX);

	bool start();

    // this is a generic I2S setup. If this doesn't match, call setFormat, setPins, etc. seperately
    int setup(int fs=48000, int channelCount=2, int bitClkPin=26, int lrClkPin=27, int dataOutPin=14, int dataInPin=13, int enablePin=33, i2s_port_t i2sPort=I2S_NUM_0);

	bool mute(bool powerDown);

	int readBlock();
	int writeBlock();

    //=======================================================================================
    // default with scale
    //=======================================================================================
	inline int32_t float2Int(float sample) {
		sample *= AudioDriver::ScaleFloat2Int;
		int32_t y = (int32_t)(sample >= 0.5) ? sample + 1 : sample;
        y = constrain(y, -AudioDriver::ScaleFloat2Int, AudioDriver::ScaleFloat2Int-1);
        return (y << 8);
	}

	inline float int2Float(int32_t sample) {
	   return (float)(sample * AudioDriver::ScaleInt2Float); 
	}
 
	inline float readSample(int n, int channel) {
        if (channel == 0)
        return int2Float(i2sReadBuffer[channelCount * n + channel] << lshift);
        else
        return int2Float(i2sReadBuffer[channelCount * n + channel]);
	}

    inline void writeSample(float sample, int n, int channel) {       
         i2sWriteBuffer[channelCount * n + channel] = float2Int(sample);
	}

    // add by Junon
    inline void writeStereoSample(float sampleR, float sampleL, int n) {       
         int32_t R = float2Int(sampleR);
         int32_t L = float2Int(sampleL);
         i2sWriteBuffer[channelCount * n + 0] = L >> lshift;
         i2sWriteBuffer[channelCount * n + 1] = R;
	}
    //=======================================================================================


    //=======================================================================================
    // no scale (raw) add by Junon
    //=======================================================================================
	inline int32_t raw2Int(float sample) {
        int32_t y = (int32_t)sample;         
        return (y << 8);
	}

	inline float int2Raw(int32_t sample) {
	   return (float)sample; 
	}
 
	inline float readRawSample(int n, int channel) {
        if (channel == 0)
        return int2Raw(i2sReadBuffer[channelCount * n + channel] << lshift);
        else
        return int2Raw(i2sReadBuffer[channelCount * n + channel]);
	}

	inline void writeRawStereoSample(float sampleR, float sampleL, int n) {       
        int32_t R = raw2Int(sampleR);
        int32_t L = raw2Int(sampleL);
        i2sWriteBuffer[channelCount * n + 0] = L >> lshift;
        i2sWriteBuffer[channelCount * n + 1] = R;
	}
    //=======================================================================================

   //=======================================================================================

   //=======================================================================================
	inline int readIntSample(int n, int channel) {
        return i2sReadBuffer[channelCount * n + channel];
	}

	inline void writeIntSample(int sample, int n, int channel) {
		i2sWriteBuffer[channelCount * n + channel] = sample;
	}
   //=======================================================================================



protected:

	int fs;
	int channelCount;
	int enablePin;

	int lshift;

	i2s_port_t i2sPort;
	int i2sBufferSize;
	int32_t* i2sReadBuffer;
	int32_t* i2sWriteBuffer;

};



#endif /* AUDIODRIVER_H_ */
