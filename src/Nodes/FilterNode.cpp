/*
 * 	FilterNode
 *  Smoothed LP-, HP-, BP-Filter
 *
 *  Author: Gary Grutzek
 * 	gary@ib-gru.de
 */

#include <Nodes/FilterNode.h>

FilterNode::FilterNode() {
	; // be sure to call begin(fs)
}

FilterNode::FilterNode(int fs, int channelCount) {
	this->begin(fs, channelCount);
}

FilterNode::~FilterNode() {
	if(interpolator) {
		delete interpolator;
	}
}


void FilterNode::begin(int sampleRate, int channelCount) {
	fs = sampleRate;
	interpolator = new Interpolator(fs, 50);

	// reset states
	memset(filterStates, 0, sizeof(filterStates));
	for (int i = 0; i < 5; i++) {
		filterCoeffs[i][kCurrent] = 0;
		filterCoeffs[i][kTarget] = 0;
	}
}



//
void FilterNode::setupFilter(int type, float f0, float q, bool smooth, bool resetStates) {
	//
	// http://www.musicdsp.org/files/Audio-EQ-Cookbook.txt
	//            (b0/a0) + (b1/a0)*z^-1 + (b2/a0)*z^-2
	//     H(z) = ---------------------------------------
	//            1 + (a1/a0)*z^-1 + (a2/a0)*z^-2
	//
	this->f0 = f0;
	this->type = type;
	this->q = q; 
	
    float frequency = f0;
    float AUDIO_SAMPLE_RATE_EXACT = fs; 
    
	// TOD=: gain
	//	float dBgain = 0;
	//  float A = sqrt(pow(10, dBgain / 20));

	float w0 = 0; 
	float cosW0 = 0;
	float sinW0 = 0;
	float alpha = 0;
    double scale = 0;
    float coeff[5];


	switch (type) {
		case LPF: //  H(s) = 1 / (s^2 + s/Q + 1)
		    //int coeff[5];
		    w0 = frequency * (2 * 3.141592654 / AUDIO_SAMPLE_RATE_EXACT);
		    sinW0 = sin(w0);
		    alpha = sinW0 / ((double)q * 2.0);
		    cosW0 = cos(w0);
		    //double scale = 1073741824.0 / (1.0 + alpha);
		    scale = 1.0 / (1.0+alpha); // which is equal to 1.0 / a0
		    /* b0 */ coeff[0] = ((1.0 - cosW0) / 2.0) * scale;
		    /* b1 */ coeff[1] = (1.0 - cosW0) * scale;
		    /* b2 */ coeff[2] = coeff[0];
		    /* a1 */ coeff[3] = (-2.0 * cosW0) * scale;
		    /* a2 */ coeff[4] = (1.0 - alpha) * scale;
			break;
			
		case HPF: //  H(s) = s^2 / (s^2 + s/Q + 1)
			//int coeff[5];
		    w0 = frequency * (2 * 3.141592654 / AUDIO_SAMPLE_RATE_EXACT);
		    sinW0 = sin(w0);
		    alpha = sinW0 / ((double)q * 2.0);
		    cosW0 = cos(w0);
		    scale = 1.0 / (1.0+alpha); // which is equal to 1.0 / a0
		    /* b0 */ coeff[0] = ((1.0 + cosW0) / 2.0) * scale;
		    /* b1 */ coeff[1] = -(1.0 + cosW0) * scale;
		    /* b2 */ coeff[2] = coeff[0];
		    /* a1 */ coeff[3] = (-2.0 * cosW0) * scale;
		    /* a2 */ coeff[4] = (1.0 - alpha) * scale;
			break;
			
		case BPF: //  H(s) = (s/Q) / (s^2 + s/Q + 1)      (constant 0 dB peak gain)
		//int coeff[5];
		     w0 = frequency * (2 * 3.141592654 / AUDIO_SAMPLE_RATE_EXACT);
		     sinW0 = sin(w0);
		     alpha = sinW0 / ((double)q * 2.0);
		     cosW0 = cos(w0);
		     scale = 1.0 / (1.0+alpha); // which is equal to 1.0 / a0
		     /* b0 */ coeff[0] = alpha * scale;
		     /* b1 */ coeff[1] = 0;
		     /* b2 */ coeff[2] = (-alpha) * scale;
		     /* a1 */ coeff[3] = (-2.0 * cosW0) * scale;
		     /* a2 */ coeff[4] = (1.0 - alpha) * scale;
			break;
			
		default:
            for (int i = 0; i < 5; i++) coeff[i] = 0;
			break;
	}
	
	filterCoeffs[cB0][kTarget] = coeff[0];
	filterCoeffs[cB1][kTarget] = coeff[1];
	filterCoeffs[cB2][kTarget] = coeff[2];
	filterCoeffs[cA1][kTarget] = coeff[3];
	filterCoeffs[cA2][kTarget] = coeff[4];
	
	//	ESP_LOGI(TAG, "coeffs: %.5f %.5f %.5f %.5f %.5f %.5f", a0, a1, a2, b0, b1, b2);
	
	if (resetStates) {
		memset(filterStates, 0, sizeof(filterStates));
	}
	
	// calc ramp deltas
	for (int i = 0; i < 5; i++) {
		interpolator->add(&filterCoeffs[i][kCurrent], filterCoeffs[i][kTarget]);
	}
}


// Reconfigura os filtros em tempo de execução, Junon
void FilterNode::resetFilter(int type, float f0, float q, bool smooth, bool resetStates)
{
  if(interpolator) {
	delete interpolator;
  }

  interpolator = new Interpolator(fs, 50);

  // reset states
  memset(filterStates, 0, sizeof(filterStates));
  for (int i = 0; i < 5; i++) {
	filterCoeffs[i][kCurrent] = 0;
	filterCoeffs[i][kTarget] = 0;
  }

  setupFilter(type, f0, q, smooth, resetStates);
}


//
// Apply filter changes
//
void FilterNode::updateFilter(float f0) {
	float f = f0;

    /*	
	// TODO: find smarter solution, unstable when going to low
	switch(type) {
		case HPF: f = constrain(f0, 1.f, fs*0.5f); break;
		case LPF: f = constrain(f0, 50.f, fs*0.5f); break;
		default: f = constrain(f0, 20.f, fs*0.5f); break;
	}
    
    //this->f0 = f;

	// TODO: filter type?
	setupFilter(type, f, q, true, false); // FIXME:    

	// calc ramp deltas? again?
	//for (int i = 0; i < 5; i++) {
	//	interpolator->add(&filterCoeffs[i][kCurrent], filterCoeffs[i][kTarget]);
	//}
    */
}


//
// Apply filter
//
float FilterNode::processSample(float sample, int channel) {
	float x = sample;
	
	// DF 1:
	// y[n] = (b0/a0)*x[n] + (b1/a0)*x[n-1] + (b2/a0)*x[n-2] - (a1/a0)*y[n-1] - (a2/a0)*y[n-2]
	//
	//	float y = filterCoeffs[cB0][kCurrent] * x + filterCoeffs[cB1][kCurrent] * filterStatesX[0] + filterCoeffs[cB2][kCurrent] * filterStatesX[1]
	//	- filterCoeffs[cA1][kCurrent] * filterStates[0] - filterCoeffs[cA2][kCurrent] * filterStates[1];
	//
	//	filterStatesX[1] = filterStatesX[0];
	//	filterStatesX[0] = x;
	//	filterStates[1] = filterStates[0];
	//	filterStates[0] = y;
	//
	// DF 2:
	// v[n] = x[n] - a1/a0 * v[n-1] + a2/a0 * v[n-2];
	// y[n] = b0/a0 * v[n] + b1/a0 * v[n-1] + b2 * v[n-2];
	
	//	  float v = x - filterCoeffs[cA1][kCurrent] * filterStates[0] - filterCoeffs[cA2][kCurrent] * filterStates[1];
	//	  float y = filterCoeffs[cB0][kCurrent] * v + filterCoeffs[cB1][kCurrent] * filterStates[0] + filterCoeffs[cB2][kCurrent] * filterStates[1];
	//	  filterStates[1] = filterStates[0];
	//	  filterStates[0] = v;
	
	// Transposed DF 2:
	float y = filterCoeffs[cB0][kCurrent] * x + filterStates[0][channel];
	filterStates[0][channel] = filterCoeffs[cB1][kCurrent] * x - filterCoeffs[cA1][kCurrent] * y + filterStates[1][channel];
	filterStates[1][channel] = filterCoeffs[cB2][kCurrent] * x - filterCoeffs[cA2][kCurrent] * y;
	
	interpolator->process();
	return y;
}
