/*
    PiFmRds - FM/RDS transmitter for the Raspberry Pi
    Copyright (C) 2014 Christophe Jacquet, F8FTK
    
    See https://github.com/ChristopheJacquet/PiFmRds
    
    rds_wav.c is a test program that writes a RDS baseband signal to a WAV
    file. It requires libsndfile.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
    fm_mpx.c: generates an FM multiplex signal containing RDS plus possibly
    monaural or stereo audio.
*/

#include <sndfile.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <math.h>

#include "rds.h"


#define PI 3.141592654


#define FIR_HALF_SIZE 30 
#define FIR_SIZE (2*FIR_HALF_SIZE-1)

#define MAX_STATIONS 5
// 15 DMA channels are usable on the RPi (0..14)
#define DMA_CHANNELS    15

float carrier_38[] = {0.0, 0.8660254037844386, 0.8660254037844388, 1.2246467991473532e-16, -0.8660254037844384, -0.8660254037844386};

float carrier_19[] = {0.0, 0.5, 0.8660254037844386, 1.0, 0.8660254037844388, 0.5, 1.2246467991473532e-16, -0.5, -0.8660254037844384, -1.0, -0.8660254037844386, -0.5};
    
int phase_38 = 0;
int phase_19 = 0;

struct channel {
    float downsample_factor;
    float *audio_buffer;
    int audio_index;
    int audio_len;
    float audio_pos;
    SNDFILE *inf;
    size_t length;
    int num_channels; // i.e. mono or stereo
    
    float fir_buffer_mono[FIR_SIZE];
    float fir_buffer_stereo[FIR_SIZE];
    int fir_index;
    // coefficients of the low-pass FIR filter
    float low_pass_fir[FIR_HALF_SIZE];
};

// One control structure per channel
static struct channel channels[DMA_CHANNELS];


float *alloc_empty_buffer(size_t length) {
    float *p = malloc(length * sizeof(float));
    if(p == NULL)
        return NULL;
    
    bzero(p, length * sizeof(float));
    
    return p;
}

int
mpx_init_channel(int station) {    
    channels[station].audio_index = 0;
    channels[station].audio_len = 0;
    memset(channels[station].fir_buffer_mono, 0, sizeof(channels[station].fir_buffer_mono));
    memset(channels[station].fir_buffer_stereo, 0, sizeof(channels[station].fir_buffer_stereo));
    channels[station].fir_index = 0;
    
    return 1;
}


int fm_mpx_open(char *filename, size_t len, int station) {
    mpx_init_channel(station);
    
    channels[station].length = len;
    
    if(filename != NULL) {
            
        // Open the input file
        SF_INFO sfinfo;
            
        // stdin or file on the filesystem?
        if(filename[0] == '-') {
            if(! (channels[station].inf = sf_open_fd(fileno(stdin), SFM_READ, &sfinfo, 0))) {
                fprintf(stderr, "Error: could not open stdin for audio input.\n");
                printf("Error: could not open stdin for audio input.\n");
                return -1;
            } else {
                printf("Using stdin for audio input.\n");
            }
        } else {
            channels[station].inf = sf_open(filename, SFM_READ, &sfinfo);
            if(!channels[station].inf) {
                fprintf(stderr, "Error: could not open input file %s.\n", filename);
                printf("Error: could not open input file %s. \n", filename);
                return -1;
            } else {
                printf("Using audio file: %s for channel %d\n", filename, station);
            }
        }
                
        int in_samplerate = sfinfo.samplerate;
        channels[station].downsample_factor = 228000. / in_samplerate;
        
        printf("Input: %d Hz, upsampling factor: %.2f\n", in_samplerate, channels[station].downsample_factor);

        channels[station].num_channels = sfinfo.channels;
        if(channels[station].num_channels > 1) {
            printf("%d channels, generating stereo multiplex.\n", channels[station].num_channels);
        } else {
            printf("1 channel, monophonic operation.\n");
        }
        
        
        // Create the low-pass FIR filter
        float cutoff_freq = 15000 * .8;
        if(in_samplerate/2 < cutoff_freq)
            cutoff_freq = in_samplerate/2 * .8;
        
        
        
        channels[station].low_pass_fir[FIR_HALF_SIZE-1] = 2 * cutoff_freq / 228000 /2;
        // Here we divide this coefficient by two because it will be counted twice
        // when applying the filter

        // Only store half of the filter since it is symmetric
        for(int i=1; i<FIR_HALF_SIZE; i++) {
            channels[station].low_pass_fir[FIR_HALF_SIZE-1-i] = 
                sin(2 * PI * cutoff_freq * i / 228000) / (PI * i)      // sinc
                * (.54 - .46 * cos(2*PI * (i+FIR_HALF_SIZE) / (2*FIR_HALF_SIZE)));
                                                                  // Hamming window
        }
        printf("Created low-pass FIR filter for audio channels, with cutoff at %.1f Hz\n", cutoff_freq);
        
        /*
        for(int i=0; i<FIR_HALF_SIZE; i++) {
            printf("%.5f ", low_pass_fir[i]);
        }
        printf("\n");
        */
            
        channels[station].audio_pos = channels[station].downsample_factor;
        channels[station].audio_buffer = alloc_empty_buffer(channels[station].length * channels[station].num_channels); // Buffer length is 10,000 for stereo, 5,000 for mono.
        if(channels[station].audio_buffer == NULL) 
            return -1;
    
    } // end if(filename != NULL)
    else {
        printf("Error: no audio found.\n");
        channels[station].inf = NULL;
        // inf == NULL indicates that there is no audio
    }
    
    return 0;
}


// samples provided by this function are in 0..10: they need to be divided by
// 10 after.
int fm_mpx_get_samples(float *mpx_buffer, int station) {
        
    get_rds_samples(mpx_buffer, channels[station].length);
    
    if (station < 0) {
        printf("Error: Improper station. Enter a real integer between 1 and %d\n", MAX_STATIONS);
        return -1;
    }

    if(channels[station].inf == NULL) { // If there is no audio, stop here 
        printf("Fatal: No audio found for channel %d.\n", station);
        return 0;
    }
    
    for(int i = 0; i < channels[station].length; i++) {
        
        if(channels[station].audio_pos >= channels[station].downsample_factor) {
            channels[station].audio_pos -= channels[station].downsample_factor;
            
            if(channels[station].audio_len == 0) { 
                for(int j=0; j<2; j++) {                    // Loop for one attempt and one retry
                    channels[station].audio_len = sf_read_float(channels[station].inf, channels[station].audio_buffer, channels[station].length);              // Read "length" bits from file "inf" and store in "audio_buffer", store num of bits read in "audio_len"
                    if (channels[station].audio_len < 0) {         // Handle error from sf_read_float
                        fprintf(stderr, "Error reading audio\n");
                        return -1;
                    }
                    if (channels[station].audio_len < channels[station].length) {                                // End of file has been reached
                        printf("Attempting to rewind file.\n");
                        if(sf_seek(channels[station].inf, 0, SEEK_SET) < 0 ) {                 // Rewind back to file beginning
                            fprintf(stderr, "Could not rewind in audio file, terminating\n");
                            return -1;
                        }
                        else
                        {
                            printf("File rewound successfully.\n");
                            
                            
                            /* Since audio_len < length, we know the buffer is not full. We already rewound the file to its beginning,
                             * now we have to fill in the remaining empty buffer registers with the bytes at the beginning of the file
                             * until the buffer is full, then we proceed as normal.
                             */
                            int left_to_cover = ((int) channels[station].length) - ((int)channels[station].audio_len);
                            int register_left_off_at = ((int)channels[station].audio_len) - 1;
                            channels[station].audio_len = ((int)channels[station].audio_len) + sf_read_float(channels[station].inf, &(channels[station].audio_buffer[register_left_off_at]), left_to_cover);
                            break;
                        }
                    }
                }
                channels[station].audio_index = 0;  // Var used to keep track of position in audio array
            } else {
                channels[station].audio_index += channels[station].num_channels;        // For mono data, variables will increment by one.
                channels[station].audio_len -= channels[station].num_channels;          // For stereo data, variables must increment by 2, since both channel bytes appear as a pair in memory.
            }
        }

        // First store the current sample(s) into the FIR filter's ring buffer
        if(channels[station].num_channels == 0) {
            channels[station].fir_buffer_mono[channels[station].fir_index] = channels[station].audio_buffer[channels[station].audio_index];
        } else {
            // In stereo operation, generate sum and difference signals
            channels[station].fir_buffer_mono[channels[station].fir_index] = 
                channels[station].audio_buffer[channels[station].audio_index] + channels[station].audio_buffer[channels[station].audio_index + 1];
            channels[station].fir_buffer_stereo[channels[station].fir_index] = 
                channels[station].audio_buffer[channels[station].audio_index] - channels[station].audio_buffer[channels[station].audio_index + 1];
        }
        
        channels[station].fir_index++;
        if(channels[station].fir_index >= FIR_SIZE) 
            channels[station].fir_index = 0;
        
        /* Now apply the FIR low-pass filter.
         * As the FIR filter is symmetric, we do not multiply all 
         * the coefficients independently, but two-by-two, thus reducing
         * the total number of multiplications by a factor of two
         */
        float out_mono = 0;    // Store filtered mono signal
        float out_stereo = 0;  // Store filtered stereo signal  
        int ifbi = channels[station].fir_index;  // ifbi = increasing FIR Buffer Index
        int dfbi = channels[station].fir_index;  // dfbi = decreasing FIR Buffer Index
        for(int fi=0; fi<FIR_HALF_SIZE; fi++) {  // fi = Filter Index

            // Decrement dfbi to read buffer backwards, and reset dfbi if end of buffer is reached.
            dfbi--;
            if(dfbi < 0) 
                dfbi = FIR_SIZE-1;

            // Calculate the filtered audio signal by multiplying the filter coefficient by the sum of
            // two samples from the FIR buffer.
            out_mono += channels[station].low_pass_fir[fi] * (channels[station].fir_buffer_mono[ifbi] + channels[station].fir_buffer_mono[dfbi]);

            // Do the same for stereo
            if(channels[station].num_channels > 1)
                out_stereo += channels[station].low_pass_fir[fi] * (channels[station].fir_buffer_stereo[ifbi] + channels[station].fir_buffer_stereo[dfbi]);

            // Increment ifbi to read buffer forwards, and reset if out of bounds
            ifbi++;
            if(ifbi >= FIR_SIZE) 
                ifbi = 0;
        }
        // End of FIR filter
        

        // Create data buffer
        mpx_buffer[i] = 
            mpx_buffer[i] +    // RDS data samples are currently in mpx_buffer
            4.05*out_mono;     // Unmodulated monophonic (or stereo-sum) signal
            
        if(channels[station].num_channels > 1) {
            mpx_buffer[i] +=
                4.05 * carrier_38[phase_38] * out_stereo + // Stereo difference signal
                .9*carrier_19[phase_19];                  // Stereo pilot tone

            phase_19++;
            phase_38++;
            if(phase_19 >= 12) phase_19 = 0;
            if(phase_38 >= 6) phase_38 = 0;
        }
            
        channels[station].audio_pos++;   
        
    }
    
    return 0;
}


int fm_mpx_close() {
    
    for(int i = 0; i < DMA_CHANNELS; i++) {
        if(channels[i].inf == NULL || channels[i].audio_buffer == NULL)
            continue;
        if(sf_close(channels[i].inf))
            fprintf(stderr, "Error closing audio file");
        free(channels[i].audio_buffer);
    }
    
    return 0;
}
