/**
 * @file        uploadWAV.cpp
 * @project     test_Skainet
 * @author      Bernd Waldmann (you@domain.com)
 * @date        2025-10-06
 * tabsize  4
 * 
 * This Revision: $Id: uploadWAV.cpp 1910 2025-11-04 11:12:38Z  $
 */

/*
   Copyright (C) 2025 Bernd Waldmann

   This Source Code Form is subject to the terms of the Mozilla Public License, 
   v. 2.0. If a copy of the MPL was not distributed with this file, You can 
   obtain one at http://mozilla.org/MPL/2.0/

   SPDX-License-Identifier: MPL-2.0
*/

/**
 * @brief   upload an array of audio samples to an FTP server, as a WAV file
 */

#include <WiFi.h>
#include <SPIFFS.h>
#include <FTPClient.h>      // MIT license, https://github.com/exocet22/TinyFTPClient
#include "wav_header.h"
#include "uploadWAV.h"


static FTPClient ftpClient;


static pcm_wav_header_t wavHdr = {
    .descriptor_chunk = {
        .chunk_id = {'R','I','F','F'},
        .chunk_size = 4,
        .chunk_format = {'W','A','V','E'}
    },
    .fmt_chunk = {
        .subchunk_id = {'f','m','t',' '},
        .subchunk_size = 16,
        .audio_format = 1,
        .num_of_channels = 1,
        .sample_rate = 16000,
        .byte_rate = 16000 * 1 * sizeof(int16_t),
        .block_align = 1 * sizeof(int16_t),
        .bits_per_sample = 1 * sizeof(int16_t) * 8
    },
    .data_chunk = {
        .subchunk_id = {'d','a','t','a'},
        .subchunk_size = 0
    }
};


/**
 * @brief upload array of `nsamples` audio samples in `data` to 
 * FTP server `url`, authenticate with `user`and `pass`. in WAV file format chunk,
 * set `sample_rate`and `nchannels`
 * 
 * @param url       FTP server name, like "myserver.local"
 * @param user      username for authentication with FTP server
 * @param pass      password for authentication with FTP server
 * @param filename  filename to use
 * @param data      pointer to audio samples (int16_t)
 * @param nsamples  number of audio samples
 * @param sample_rate  sample rate in Hz
 * @param nchannels number of interleaved audio channels, typically 1 or 2
 * @return true     if upload was successful
 * @return false     if some error occured
 */
bool uploadWave( 
    const char* url,    
    const char* user, const char* pass,
    const char* filename, 
    const int16_t* data, unsigned nsamples, 
    unsigned sample_rate, unsigned nchannels )
{
    size_t nbytes = nsamples * sizeof(int16_t);

    wavHdr.fmt_chunk.num_of_channels = nchannels;
    wavHdr.fmt_chunk.sample_rate = sample_rate;
    wavHdr.fmt_chunk.byte_rate = sample_rate * nchannels * sizeof(int16_t);
    wavHdr.data_chunk.subchunk_size = nbytes;

    uint32_t t_wav_start = millis();

    if  (!ftpClient.open(url,21,user,pass)) {
        log_e("Can't connect to FTP server");
        return false;
    }
    ftpClient.write_file( filename, (uint8_t*)&wavHdr, sizeof wavHdr);
    ftpClient.append_file( filename, (uint8_t*)data, nbytes );
    ftpClient.close();
    log_i("WAV file written, %u ms", millis()-t_wav_start);

    return true;
}
