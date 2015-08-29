#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define BLOCK 1024*1024

int decimate = 5;
int filter = 1;

//10.2e6 sample rate
//1e6 cutoff
//Order 75
const static float taps[] = {
-0.000493656697454,
-0.00013255116767,
0.000328953247674,
0.000768284177902,
0.00101983945803,
0.000911971775533,
0.000346352730748,
-0.000608452414895,
-0.0016647344791,
-0.00236024152727,
-0.00221917225251,
-0.000988097786741,
0.00115237730846,
0.00352037245284,
0.0050927547003,
0.00488405010035,
0.00242400594025,
-0.00186781082839,
-0.00661587040672,
-0.0098436104867,
-0.0096952578402,
-0.00529127640339,
0.00262851392347,
0.0116035845836,
0.0180603854265,
0.0185326912,
0.011127412773,
-0.00329816541075,
-0.020761227265,
-0.03486815785,
-0.0385386262414,
-0.0262734430007,
0.00375572663871,
0.0485351170886,
0.10044913874,
0.148982508572,
0.183443860716,
0.19590490101,
0.183443860716,
0.148982508572,
0.10044913874,
0.0485351170886,
0.00375572663871,
-0.0262734430007,
-0.0385386262414,
-0.03486815785,
-0.020761227265,
-0.00329816541075,
0.011127412773,
0.0185326912,
0.0180603854265,
0.0116035845836,
0.00262851392347,
-0.00529127640339,
-0.0096952578402,
-0.0098436104867,
-0.00661587040672,
-0.00186781082839,
0.00242400594025,
0.00488405010035,
0.0050927547003,
0.00352037245284,
0.00115237730846,
-0.000988097786741,
-0.00221917225251,
-0.00236024152727,
-0.0016647344791,
-0.000608452414895,
0.000346352730748,
0.000911971775533,
0.00101983945803,
0.000768284177902,
0.000328953247674,
-0.00013255116767,
-0.000493656697454,
};
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

    unsigned int block_size = BLOCK - BLOCK%lcm(40, lcm(decimate, sizeof(taps)));
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
    int16_t *block_out = malloc(block_size*sizeof(int16_t));
    if (!block_out) {
        printf("malloc failed\n");
        return -1;
    }
    int read_size;
    int i, j;
    int fsamples;
    unsigned int stored = 0;
    uint32_t sync;
    while (1) {
        int read = block_size - stored*2;
        // Read must be aligned to packet size
        read = read - read%40;
        if ( !(read_size = fread(block8, 1, read, fin)) ) {
            // EOF
            break;
        }

        // Attach the 2 LSB bits to right samples
        int read_samples = 0;
        for(i=0;i<read_size/40;i++) {
            for(j=0;j<31;j++) {
                int d1 = !!(array_to_32(block8+(i*40+32)) & (1 << j));
                int d0 = !!(array_to_32(block8+(i*40+36)) & (1 << j));
                block[stored+read_samples] = (block8[i*40+j]<<2) | (d1 << 1) | d0;
                char sign = block8[i*40+j] & (1 << 7);
                // Sign extend
                if (sign) {
                    block[stored+read_samples] |= 0xFC00;
                }
                read_samples++;
            }
        }
        if (filter) {
            fsamples = conv(taps, sizeof(taps)/sizeof(taps[0]), block, read_samples, block_filtered);
            // Move unused samples to beginning of the block
            for(i=fsamples;i<read_samples;i++) {
                block[i-fsamples] = block[i];
            }
            stored = read_samples-fsamples;
            // Decimate
            for(i=0;i<fsamples/decimate;i++) {
                int acc = 0;
                for(j=0;j<decimate;j++) {
                    acc += block_filtered[i*decimate+j];
                }
                block_out[i] = (int16_t)acc;
            }
            fwrite(block_out, 2, fsamples/decimate, fout);
        } else {
            fwrite(block, 2, read_samples, fout);
        }
        printf("%d %d %d\n", read_size, fsamples/decimate, stored);
    }
    free(block);
    free(block_filtered);
    free(block_out);
    fclose(fin);
    fclose(fout);
    return 0;
}
