#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "taps.h"

#define BLOCK 100*1024*1024
#define PACKET_SIZE 44

int decimate = 1;
int filter = 0;

const static float *taps = taps_200e3_51;

uint32_t array_to_32(int8_t *arr) {
    return (((uint32_t)arr[3] & 0xFF)<<(3*8))|(((uint32_t)arr[2] & 0xFF)<<(2*8))|(((uint32_t)arr[1] & 0xFF)<<(1*8))|((uint32_t)arr[0] & 0xFF);
}

int gcd(int m, int n)
{
        int tmp;
        while(m) {
            tmp = m;
            m = n % m;
            n = tmp;
        }
        return n;
}

int lcm(int m, int n)
{
        return m / gcd(m, n) * n;
}

int conv(const float *taps, int ntaps, const int16_t *signal, int len_signal, int16_t *output) {
    int i,j;
    float x;
    for(i=0;i<len_signal-ntaps;i++) {
        x = 0.0f;
        for(j=0;j<ntaps;j++) {
            x += signal[i+j] * taps[ntaps-j-1];
        }
        output[i] = (int16_t)x;
    }
    return len_signal-ntaps;
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        printf("<input> <output>\n");
        return -1;
    }
    FILE *fin = fopen(argv[1], "rb");
    if (!fin) {
        printf("Failed to open input file: %s\n", argv[1]);
        return -1;
    }

    FILE *fout = fopen(argv[2], "wb");
    if (!fout) {
        printf("Failed to open output file: %s\n", argv[2]);
        return -1;
    }

    char *sync_file = malloc(strlen(argv[2])+10);
    if (!sync_file) {
        printf("malloc failed\n");
        return -1;
    }

    sprintf(sync_file, "%s.sync", argv[2]);
    FILE *fsync = fopen(sync_file, "wb");
    if (!fout) {
        printf("Failed to open sync output file: %s\n", sync_file);
        return -1;
    }

    unsigned int block_size = BLOCK - BLOCK%lcm(PACKET_SIZE, lcm(decimate, TAPS_LENGTH));
    printf("Block size: %d\n", block_size);

    int8_t *block8 = malloc(block_size*sizeof(int8_t));
    if (!block8) {
        printf("malloc failed\n");
        return -1;
    }
    int16_t *block = malloc(block_size*sizeof(int16_t));
    if (!block) {
        printf("malloc failed\n");
        return -1;
    }
    int16_t *block_filtered = malloc(block_size*sizeof(int16_t));
    if (!block_filtered) {
        printf("malloc failed\n");
        return -1;
    }
    int16_t *block_out = malloc(2*block_size*sizeof(int16_t));
    if (!block_out) {
        printf("malloc failed\n");
        return -1;
    }
    uint32_t *syncs = malloc(block_size*sizeof(uint32_t));
    if (!syncs) {
        printf("malloc failed\n");
        return -1;
    }

    //Read header
    {
        int res;
        char magic[4];
        //Magic, should be "FMCW"
        res = fread(magic, 1, 4, fin);
        if (res != 4) {
            printf("Failed to read input file\n");
            return -1;
        }
        if (strncmp("FMCW", magic, 4) != 0) {
            printf("Invalid header, exiting\n");
            return -1;
        }
        int version;
        int header_size;
        double sample_rate;
        res = fread(&version, 4, 1, fin);
        res = fread(&header_size, 4, 1, fin);
        res = fread(&sample_rate, 8, 1, fin);
        if (res != 1) {
            printf("Failed to read header\n");
            return -1;
        }
        printf("Sample rate: %f\n", sample_rate);
        printf("New sample rate: %f\n", sample_rate/decimate);
        header_size = header_size-4-4-4-8;

        //Copy header to output
        char *header = malloc(header_size);
        if (!header) {
            printf("malloc failed\n");
            return -1;
        }
        res = fread(header, 1, header_size, fin);
        if (res != header_size) {
            printf("Failed to read header\n");
            return -1;
        }
        fwrite(magic, 1, 4, fout);
        fwrite(&version, 4, 1, fout);
        fwrite(&header_size, 4, 1, fout);
        sample_rate = sample_rate/decimate;
        fwrite(&sample_rate, 8, 1, fout);
        fwrite(header, 1, header_size, fout);
    }

    int read_size;
    int i, j;
    int fsamples;
    unsigned int stored = 0;
    unsigned int sample_counter = 0;
    unsigned int last_sync = 0;
    while (1) {
        unsigned int sync_counter = 0;
        int read = block_size - stored*2;
        // Read must be aligned to packet size
        read = read - read%PACKET_SIZE;
        if ( !(read_size = fread(block8, 1, read, fin)) ) {
            // EOF
            break;
        }

        // Attach the 2 LSB bits to right samples
        int read_samples = 0;
        int sync_phase = 1;
        for(i=0;i<read_size/PACKET_SIZE;i++) {
            for(j=0;j<31;j++) {
                sample_counter++;
                // Store bits as bytes for easier access
                uint8_t sync = !!(array_to_32(block8+(i*PACKET_SIZE+40)) & (1 << j));
                if (sync_phase != sync) {
                    sync_phase = sync;
                    if ( sync_phase == 0) {
                        syncs[sync_counter++] = sample_counter-last_sync;
                        last_sync = sample_counter;
                    }
                }
                int d1 = !!(array_to_32(block8+(i*PACKET_SIZE+32)) & (1 << j));
                int d0 = !!(array_to_32(block8+(i*PACKET_SIZE+36)) & (1 << j));
                block[stored+read_samples] = (block8[i*PACKET_SIZE+j]<<2) | (d1 << 1) | d0;
                char sign = block8[i*PACKET_SIZE+j] & (1 << 7);
                // Sign extend
                if (sign) {
                    block[stored+read_samples] |= 0xFC00;
                }
                read_samples++;
            }
        }
        if (filter) {
            fsamples = conv(taps, TAPS_LENGTH, block, read_samples, block_filtered);
            // Move unused samples to beginning of the block
            for(i=fsamples;i<read_samples;i++) {
                block[i-fsamples] = block[i];
            }
            stored = read_samples-fsamples;

            // Decimate sync words
            for(i=0;i<sync_counter;i++) {
                syncs[i] = (uint32_t)roundf(((double)syncs[i])/decimate);
            }

            // Decimate samples
            int aligned_samples = fsamples/decimate;
            for(i=0;i<aligned_samples;i++) {
                int acc = 0;
                for(j=0;j<decimate;j++) {
                    acc += block_filtered[i*decimate+j];
                }
                block_out[i] = (int16_t)acc;
            }

            //printf("stored %d\n", fsamples/decimate - aligned_samples);
            fwrite(block_out, 2, aligned_samples, fout);
            fwrite(syncs, 4, sync_counter, fsync);
        } else {
            fwrite(block, 2, read_samples, fout);
            fwrite(syncs, 4, sync_counter, fsync);
        }
        printf("%d %d %d\n", read_size, fsamples, stored);
    }
    free(block);
    free(block_filtered);
    free(block_out);
    free(syncs);
    fclose(fin);
    fclose(fout);
    fclose(fsync);
    return 0;
}
