#pragma once

#include <stdint.h>

bool uploadWave( 
    const char* url, const char* user, const char* pass,
    const char* filename, 
    const int16_t* data, unsigned nsamples, 
    unsigned sample_rate, unsigned nchannels=1 );
