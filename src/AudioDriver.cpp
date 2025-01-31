/*
 *  AudioDriver.cpp
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
#include "AudioDriver.h"

const float AudioDriver::ScaleFloat2Int = 8388608.f; // 2^23 
const float AudioDriver::ScaleInt2Float = 0.0000000004656612873077392578125f; // 1 / 2^31 

// this is a generic I2S setup. If this doesn't match, call setFormat, setPins, etc. seperately
int AudioDriver::setup(int fs, int channelCount, int bitClkPin, int lrClkPin, int dataOutPin, int dataInPin, int enablePin, i2s_port_t i2sPort) {

    int mclk_fs = 384;

	setI2sPort(i2sPort);

    int err = setFormat(fs, channelCount, I2S_BITS_PER_SAMPLE_32BIT, I2S_COMM_FORMAT_I2S, CODEC_LJ_RJ_ALIGN, mclk_fs, I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX);

	err += setPins(bitClkPin, lrClkPin, dataOutPin, dataInPin, enablePin);
	err += start();
	return err;
}


void AudioDriver::setI2sPort(i2s_port_t i2s_port) {
	this->i2sPort = constrain(i2s_port, I2S_NUM_0, I2S_NUM_MAX);
}


int AudioDriver::setFormat(int fs, int channelCount, i2s_bits_per_sample_t bitsPerSample, i2s_comm_format_t format, int alignment, int mclkFactor, int mode)
{
	this->fs = fs;
	this->channelCount = channelCount;
	
	static const i2s_config_t i2s_config = {
			.mode = (i2s_mode_t)(mode),
			.sample_rate = fs,
			.bits_per_sample = bitsPerSample,
			.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
			.communication_format = format,
			.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // high interrupt priority
			.dma_buf_count = 2,
			.dma_buf_len = AudioDriver::BufferSize,
			.use_apll = true,
			.tx_desc_auto_clear = true,
			.fixed_mclk = fs * mclkFactor,
	};

	switch (alignment) {
		case CODEC_I2S_ALIGN: lshift = 8; break;
		case CODEC_LJ_RJ_ALIGN: lshift = 0; break;
		default: lshift = 8; break;
	}

	esp_err_t err = i2s_driver_install(i2sPort, &i2s_config, 0, NULL);

	return err;
}



int AudioDriver::setPins(int bitClkPin, int lrClkPin, int dataOutPin, int dataInPin, int enablePin) {

	// power down (not) pin
	this->enablePin = enablePin;
	pinMode(enablePin, OUTPUT);
	digitalWrite(enablePin, LOW);
	
	static const i2s_pin_config_t pin_config = {
			.bck_io_num = bitClkPin,
			.ws_io_num = lrClkPin,
			.data_out_num = dataOutPin,
			.data_in_num = dataInPin
	};

	esp_err_t err = i2s_set_pin(i2sPort, &pin_config);

	return err;
}


bool AudioDriver::start() {

	// enable I2S master clock on GPIO0 for master mode
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
	WRITE_PERI_REG(PIN_CTRL, READ_PERI_REG(PIN_CTRL) & 0xFFFFFFF0);
	SET_PERI_REG_BITS(PIN_CTRL, CLK_OUT1, 0, CLK_OUT1_S);

	esp_err_t err = i2s_set_sample_rates(i2sPort, fs);

	err += i2s_zero_dma_buffer(i2sPort);
	
	// I2S buffers
	i2sBufferSize =  channelCount * AudioDriver::BufferSize;

	i2sReadBuffer = (int32_t*) calloc(i2sBufferSize, sizeof(int32_t));
	i2sWriteBuffer = (int32_t*) calloc(i2sBufferSize, sizeof(int32_t));

	// wait for stable clock
	delay(400);

    mute(false);
	return true;
}


bool AudioDriver::mute(bool powerDown) {
	digitalWrite(enablePin, !powerDown);
	return true;
}


int AudioDriver::readBlock() {
	uint bytesRead = 0;

	int err = i2s_read(i2sPort, (void*) i2sReadBuffer, i2sBufferSize * sizeof(int32_t), &bytesRead, 500);

	//if (err || bytesRead < (i2sBufferSize * sizeof(int32_t))) {
		//for (int i = 0; i < i2sBufferSize; i++) i2sReadBuffer[i] = 0.f;
		//Serial.print("I2S read error: ");
		//Serial.println(bytesRead);
	//}
	return err;
}


int AudioDriver::writeBlock() {
	uint bytesWritten = 0;

	int err = i2s_write(i2sPort, (const char *) i2sWriteBuffer, channelCount * AudioDriver::BufferSize * sizeof(int32_t), &bytesWritten, 500);

	//if (bytesWritten < 1) {
		//for (int i = 0; i < i2sBufferSize; i++) i2sWriteBuffer[i] = 0.f;
		//Serial.print("I2S write error: ");
		//Serial.println(bytesWritten);
	//}
	return err;
}
