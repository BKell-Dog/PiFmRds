/*
 * PiFmRds - FM/RDS transmitter for the Raspberry Pi
 * Copyright (C) 2014, 2015 Christophe Jacquet, F8FTK
 * Copyright (C) 2012, 2015 Richard Hirst
 * Copyright (C) 2012 Oliver Mattos and Oskar Weigl
 *
 * See https://github.com/ChristopheJacquet/PiFmRds
 *
 * PI-FM-RDS: RaspberryPi FM transmitter, with RDS. 
 *
 * This file contains the VHF FM modulator. All credit goes to the original
 * authors, Oliver Mattos and Oskar Weigl for the original idea, and to
 * Richard Hirst for using the Pi's DMA engine, which reduced CPU usage
 * dramatically.
 *
 * I (Christophe Jacquet) have adapted their idea to transmitting samples
 * at 228 kHz, allowing to build the 57 kHz subcarrier for RDS BPSK data.
 *
 * To make it work on the Raspberry Pi 2, I used a fix by Richard Hirst
 * (again) to request memory using Broadcom's mailbox interface. This fix
 * was published for ServoBlaster here:
 * https://www.raspberrypi.org/forums/viewtopic.php?p=699651#p699651
 *
 * Never use this to transmit VHF-FM data through an antenna, as it is
 * illegal in most countries. This code is for testing purposes only.
 * Always connect a shielded transmission line from the RaspberryPi directly
 * to a radio receiver, so as *not* to emit radio waves.
 *
 * ---------------------------------------------------------------------------
 * These are the comments from Richard Hirst's version:
 *
 * RaspberryPi based FM transmitter.  For the original idea, see:
 *
 * http://www.icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter
 *
 * All credit to Oliver Mattos and Oskar Weigl for creating the original code.
 * 
 * I have taken their idea and reworked it to use the Pi DMA engine, so
 * reducing the CPU overhead for playing a .wav file from 100% to about 1.6%.
 *
 * I have implemented this in user space, using an idea I picked up from Joan
 * on the Raspberry Pi forums - credit to Joan for the DMA from user space
 * idea.
 *
 * The idea of feeding the PWM FIFO in order to pace DMA control blocks comes
 * from ServoBlaster, and I take credit for that :-)
 *
 * This code uses DMA channel 0 and the PWM hardware, with no regard for
 * whether something else might be trying to use it at the same time (such as
 * the 3.5mm jack audio driver).
 *
 * I know nothing much about sound, subsampling, or FM broadcasting, so it is
 * quite likely the sound quality produced by this code can be improved by
 * someone who knows what they are doing.  There may be issues realting to
 * caching, as the user space process just writes to its virtual address space,
 * and expects the DMA controller to see the data; it seems to work for me
 * though.
 *
 * NOTE: THIS CODE MAY WELL CRASH YOUR PI, TRASH YOUR FILE SYSTEMS, AND
 * POTENTIALLY EVEN DAMAGE YOUR HARDWARE.  THIS IS BECAUSE IT STARTS UP THE DMA
 * CONTROLLER USING MEMORY OWNED BY A USER PROCESS.  IF THAT USER PROCESS EXITS
 * WITHOUT STOPPING THE DMA CONTROLLER, ALL HELL COULD BREAK LOOSE AS THE
 * MEMORY GETS REALLOCATED TO OTHER PROCESSES WHILE THE DMA CONTROLLER IS STILL
 * USING IT.  I HAVE ATTEMPTED TO MINIMISE ANY RISK BY CATCHING SIGNALS AND
 * RESETTING THE DMA CONTROLLER BEFORE EXITING, BUT YOU HAVE BEEN WARNED.  I
 * ACCEPT NO LIABILITY OR RESPONSIBILITY FOR ANYTHING THAT HAPPENS AS A RESULT
 * OF YOU RUNNING THIS CODE.  IF IT BREAKS, YOU GET TO KEEP ALL THE PIECES.
 *
 * NOTE ALSO:  THIS MAY BE ILLEGAL IN YOUR COUNTRY.  HERE ARE SOME COMMENTS
 * FROM MORE KNOWLEDGEABLE PEOPLE ON THE FORUM:
 *
 * "Just be aware that in some countries FM broadcast and especially long
 * distance FM broadcast could get yourself into trouble with the law, stray FM
 * broadcasts over Airband aviation is also strictly forbidden."
 *
 * "A low pass filter is really really required for this as it has strong
 * harmonics at the 3rd, 5th 7th and 9th which sit in licensed and rather
 * essential bands, ie GSM, HAM, emergency services and others. Polluting these
 * frequencies is immoral and dangerous, whereas "breaking in" on FM bands is
 * just plain illegal."
 *
 * "Don't get caught, this GPIO use has the potential to exceed the legal
 * limits by about 2000% with a proper aerial."
 *
 *
 * As for the original code, this code is released under the GPL.
 *
 * Richard Hirst <richardghirst@gmail.com>  December 2012
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sndfile.h>

#include "rds.h"
#include "fm_mpx.h"
#include "control_pipe.h"

#include "mailbox.h"
#define MBFILE            DEVICE_FILE_NAME    /* From mailbox.h */

#if (RASPI)==1
#define PERIPH_VIRT_BASE 0x20000000
#define PERIPH_PHYS_BASE 0x7e000000
#define DRAM_PHYS_BASE 0x40000000
#define MEM_FLAG 0x0c
#define PLLFREQ 500000000.
#elif (RASPI)==2
#define PERIPH_VIRT_BASE 0x3f000000
#define PERIPH_PHYS_BASE 0x7e000000
#define DRAM_PHYS_BASE 0xc0000000
#define MEM_FLAG 0x04
#define PLLFREQ 500000000.
#elif (RASPI)==4
#define PERIPH_VIRT_BASE 0xfe000000  // We use this peripheral address because we are on a BCM2711 in "Low Peripheral" mode (see doc pg. 5).
#define PERIPH_PHYS_BASE 0x7e000000
#define DRAM_PHYS_BASE 0xc0000000
#define MEM_FLAG 0x04
#define PLLFREQ 750000000.
#else
#error Unknown Raspberry Pi version (variable RASPI)
#endif

#define NUM_SAMPLES        50000
#define NUM_CBS            (NUM_SAMPLES * 2)
#define SUBSIZE            1
#define DATA_SIZE          5000

// 15 DMA channels are usable on the RPi (0..14)
#define DMA_CHANNELS    15

// Define Broadcom DMA flags
#define BCM2711_DMA_NO_WIDE_BURSTS      (1<<26)
#define BCM2711_DMA_WAIT_RESP           (1<<3)
#define BCM2711_DMA_D_DREQ              (1<<6)
#define BCM2711_DMA_PER_MAP(x)          ((x)<<16)
#define BCM2711_DMA_END                 (1<<1)
#define BCM2711_DMA_RESET               (1<<31)
#define BCM2711_DMA_INT                 (1<<2)

// Memory Addresses
#define DMA_BASE_OFFSET        0x00007000
#define DMA_LEN                0x84
#define DMA_CHANNEL_INC        0x100
#define PWM_BASE_OFFSET        0x0020C000
#define PWM_LEN                0x28
#define CLK_BASE_OFFSET        0x00101000
#define CLK_LEN                0xA8
#define GPIO_BASE_OFFSET       0x00200000
#define GPIO_LEN               0x100

// Each DMA channel has 3 writeable registers:
#define DMA_CS                 (0x00/4)
#define DMA_CONBLK_AD          (0x04/4)
#define DMA_DEBUG              (0x20/4)

// GPIO Memory Addresses
#define GPIO_FSEL0           (0x00/4)
#define GPIO_SET0            (0x1c/4)
#define GPIO_CLR0            (0x28/4)
#define GPIO_LEV0            (0x34/4)
#define GPIO_PULLEN          (0x94/4)
#define GPIO_PULLCLK         (0x98/4)

// PWM Memory Addresses
#define PWM_CTL              (0x00/4)
#define PWM_DMAC             (0x08/4)
#define PWM_RNG1             (0x10/4)
#define PWM_FIFO             (0x18/4)

#define DMA_VIRT_BASE        (PERIPH_VIRT_BASE + DMA_BASE_OFFSET)
#define PWM_VIRT_BASE        (PERIPH_VIRT_BASE + PWM_BASE_OFFSET)
#define CLK_VIRT_BASE        (PERIPH_VIRT_BASE + CLK_BASE_OFFSET)
#define GPIO_VIRT_BASE       (PERIPH_VIRT_BASE + GPIO_BASE_OFFSET)
#define PCM_VIRT_BASE        (PERIPH_VIRT_BASE + PCM_BASE_OFFSET)

#define PWM_PHYS_BASE        (PERIPH_PHYS_BASE + PWM_BASE_OFFSET)
#define PCM_PHYS_BASE        (PERIPH_PHYS_BASE + PCM_BASE_OFFSET)
#define GPIO_PHYS_BASE       (PERIPH_PHYS_BASE + GPIO_BASE_OFFSET)

#define PWM_CTL              (0x00/4)
#define PWM_DMAC             (0x08/4)
#define PWM_RNG1             (0x10/4)
#define PWM_FIFO             (0x18/4)

#define PWMCLK_CNTL          40
#define PWMCLK_DIV           41

// Variables for control of general purpose clock (GPCLK) on GPIO pins
// See BCM2711 Documentation 5.4.2
#define CM_GP0CTL           (0x7e101070)
#define CM_GP0DIV           (0x7e101074)
#define CM_GP1CTL           (0x7e101078)
#define CM_GP1DIV           (0x7e10107c)
#define CM_GP2CTL           (0x7e101080)
#define CM_GP2DIV           (0x7e101084)

#define GPCLK0_CNTL         (0x70/4)
#define GPCLK0_DIV          (0x74/4)
#define GPCLK1_CNTL         (0x78/4)
#define GPCLK1_DIV          (0x7c/4)
#define GPCLK2_CNTL         (0x80/4)
#define GPCLK2_DIV          (0x84/4) 

#define PWMCTL_MODE1        (1<<1)
#define PWMCTL_PWEN1        (1<<0)
#define PWMCTL_CLRF         (1<<6)
#define PWMCTL_USEF1        (1<<5)

#define PWMDMAC_ENAB        (1<<31)
// I think this means it requests as soon as there is one free slot in the FIFO
// which is what we want as burst DMA would mess up our timing.
#define PWMDMAC_THRSHLD     ((15<<8)|(15<<0))

#define GPFSEL0             (0x00/4)
#define GPFSEL1             (0x04/4)

// The deviation specifies how wide the signal is. Use 25.0 for WBFM
// (broadcast radio) and about 3.5 for NBFM (walkie-talkie style radio)
#define DEVIATION        25.0

// Max number of stations allowable
#define MAX_STATIONS 5

#define BUS_TO_PHYS(x) ((x)&~0xC0000000)

// Default subcycle time
#define SUBCYCLE_TIME_US_DEFAULT 20000
// Subcycle minimum. We kept seeing no signals and strange behavior of the RPi
#define SUBCYCLE_TIME_US_MIN 3000
// Default pulse-width-increment-granularity
#define PULSE_WIDTH_INCREMENT_GRANULARITY_US_DEFAULT 10


// DMA Control Block Data Structure (p40 of datasheet): 8 words (256 bits)
typedef struct {
    uint32_t info;   // TI: transfer information
    uint32_t src;    // SOURCE_AD
    uint32_t dst;    // DEST_AD
    uint32_t length; // TXFR_LEN: transfer length
    uint32_t stride; // 2D stride mode
    uint32_t next;   // NEXTCONBK
    uint32_t pad[2]; // _reserved_
} dma_cb_t;

// Main control structure per channel, used for multiple DMA channels
struct channel {
    uint8_t *virtbase;
    
    char *audio_file = NULL;
    dma_cb_t cb[NUM_CBS];
    uint32_t sample[NUM_SAMPLES];
    volatile uint32_t *dma_reg;
    volatile uint32_t *clk_reg;
    uint32_t carrier_freq = 107900000;
    uint32_t freq_ctl = 0;
    
    // RDS Variables
    char *ps = NULL;
    char *rt = "PiFmRds: live FM-RDS transmission from the RaspberryPi";
    uint16_t pi = 0x1234;
    uint16_t ps_count = 0;
    uint16_t ps_count2 = 0;
    int varying_ps = 0;
    char myps[9] = {0};
    
    // TX variables / Data structures for baseband data
    float data[DATA_SIZE];
    int data_len = 0;
    int data_index = 0;
    uint32_t last_cb = 0U;
};

// One control structure per possible channel
static struct channel channels[DMA_CHANNELS];


// This struct defines and stores the data needed for a mailbox request to interface with memory
static struct {
    int handle;            /* From mbox_open() */
    unsigned mem_ref;      /* From mem_alloc() */
    unsigned bus_addr;     /* From mem_lock()  */
    uint8_t *virt_addr;    /* From mapmem()    */
} mbox;

static volatile uint32_t *pwm_reg;
static volatile uint32_t *clk_reg;
static volatile uint32_t *dma_reg;
static volatile uint32_t *gpio_reg;

#define PAGE_SIZE    4096
#define PAGE_SHIFT    12
#define NUM_PAGES    ((sizeof(struct channel) + PAGE_SIZE - 1) >> PAGE_SHIFT)
static uint8_t _is_setup = 0;
unsigned memory_size = NUM_PAGES * 4096;

// Very short delay as demanded per datasheet
static void
udelay(int us)
{
    struct timespec ts = { 0, us * 1000 };

    nanosleep(&ts, NULL);
}

// Terminate is triggered by signals
static void
terminate(int num)
{
    // Stop outputting and generating the clock.
    if (clk_reg && gpio_reg && mbox.virt_addr) {
        // Set GPIO4 to be an output (instead of ALT FUNC 0, which is the clock).
        gpio_reg[GPFSEL0] = (gpio_reg[GPFSEL0] & ~(7 << 12)) | (1 << 12);

        // Disable the clock generator.
        clk_reg[GPCLK0_CNTL] = 0x5A;
    }

    if (dma_reg && mbox.virt_addr) {
        dma_reg[DMA_CS] = BCM2711_DMA_RESET;
        udelay(10);
    }
    
    if (channels[1].dma_reg && mbox.virt_addr) {
        channels[1].dma_reg[DMA_CS] = BCM2711_DMA_RESET;
        udelay(10);
    }
    
    fm_mpx_close();
    close_control_pipe();

    if (mbox.virt_addr != NULL) {
        unmapmem(mbox.virt_addr, NUM_PAGES * 4096);
        mem_unlock(mbox.handle, mbox.mem_ref);
        mem_free(mbox.handle, mbox.mem_ref);
    }

    printf("Terminating: cleanly deactivated the DMA engine and killed the carrier.\nError code %d\n", num);
    
    exit(num);
}

// Shutdown with an error message. Returns EXIT_FAILURE for convenience.
// if soft_fatal is set to 1, a call to `fatal(..)` will not shut down
// PWM/DMA activity (used in the Python wrapper).
static void
fatal(char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    terminate(0);
}

// Memory mapping
static uint32_t
mem_virt_to_phys(void *virt)
{
    uint32_t offset = (uint8_t *)virt - mbox.virt_addr;

    return mbox.bus_addr + offset;
}

static uint32_t
mem_phys_to_virt(uint32_t phys)
{
    return phys - (uint32_t)mbox.bus_addr + (uint32_t)mbox.virt_addr;
}

// Peripherals memory mapping
static void *
map_peripheral(uint32_t base, uint32_t len)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    void * vaddr;

    if (fd < 0)
        fatal("Failed to open /dev/mem: %m.\n");
        
    vaddr = mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, base);
    
    if (vaddr == MAP_FAILED){
        printf("Error Number: %d.\n", errno);
        printf("Error Message: %s.\n", strerror(errno));
        fatal("Failed to map peripheral at 0x%08x: %m.\n", base);
    }
    close(fd);

    return vaddr;
}


// Returns a pointer to the control block of this channel in DMA memory
uint8_t*
get_cb(int channel)
{
    return channels[channel].virtbase + (sizeof(uint32_t) * channels[channel].num_samples);
}

// Reset this channel to original state (all samples=0, all cbs=clr0)
int
clear_channel(int channel)
{
    int i;
    uint32_t phys_gpclr0 = 0x7e200000 + 0x28;
    dma_cb_t *cbp = (dma_cb_t *) get_cb(channel);
    uint32_t *dp = (uint32_t *) channels[channel].virtbase;

    if (!channels[channel].virtbase)
        fatal("Error: channel %d has not been initialized with 'init_channel(..)'\n", channel);

    // First we have to stop all currently enabled pulses
    for (i = 0; i < channels[channel].num_samples; i++) {
        cbp->dst = phys_gpclr0;
        cbp += 2;
    }

    // Let DMA do one cycle to actually clear them
    udelay(channels[channel].subcycle_time_us);

    // Finally set all samples to 0 (instead of gpio_mask)
    for (i = 0; i < channels[channel].num_samples; i++) {
        *(dp + i) = 0;
    }

    return EXIT_SUCCESS;
}

int tx(uint32_t carrier_freq, char *audio_files[], int stations) {
    

    // Setup
    for (int station = 0; station < stations; station++) {
        // Initialize the baseband generator
        if(fm_mpx_open(channels[station].audio_file, DATA_SIZE, station) < 0)
        {
            printf("Error when trying to open file(s). \n");
            return 1;
        }
        
        // By this point a data stream from the specified file has been initialized in the fm_mpx_open function.
        // Later we will send the array "data" into that class to be populated with music samples.
    
        printf("Starting to transmit on %3.1f MHz.\n", channels[station].carrier_freq / 1e6);
    }

    // Transmission
    for (int station = 0; station < stations; station++) {
        
        // Default (varying) PS
        if(channels[station].varying_ps) {
            if(channels[station].ps_count == 512) {
                snprintf(channels[station].myps, 9, "%08d", channels[station].ps_count2);
                set_rds_ps(channels[station].myps);
                channels[station].ps_count2++;
            }
            if(channels[station].ps_count == 1024) {
                set_rds_ps("RPi-Live");
                channels[station].ps_count = 0;
            }
            channels[station].ps_count++;
        }
        
        usleep(5000);

        // Calculate the number of free slots left in the DMA buffer
        uint32_t cur_cb = mem_phys_to_virt(channels[station].dma_reg[DMA_CONBLK_AD]);
        int last_sample = (last_cb - (uint32_t)channels[station].virtbase)/ (sizeof(dma_cb_t) * 2);
        int this_sample = (cur_cb - (uint32_t)channels[station].virtbase) / (sizeof(dma_cb_t) * 2);
        int free_slots = this_sample - last_sample;

        if (free_slots < 0)
            free_slots += NUM_SAMPLES;

        while (free_slots >= SUBSIZE) {
            // get more baseband samples if necessary
            if(channels[station].data_len == 0) {
                if(fm_mpx_get_samples(channels[station].data, 1) < 0 ) {
                    printf("Something went horribly wrong while fetching samples.\n");
                    terminate(0);
                }
                channels[station].data_len = DATA_SIZE;
                channels[station].data_index = 0;
            }
            
            float dval = channels[station].data[channels[station].data_index] * (DEVIATION / 10.);
            channels[station].data_index++;
            channels[station].data_len--;

            int intval = (int)((floor)(dval));
            //int frac = (int)((dval - (float)intval) * SUBSIZE);

            channels[station].sample[last_sample++] = (0x5A << 24 | channels[station].freq_ctl) + intval; //(frac > j ? intval + 1 : intval);
            if (last_sample == NUM_SAMPLES)
                last_sample = 0;

            free_slots -= SUBSIZE;
        }
        last_cb = (uint32_t)channels[station].virtbase + last_sample * sizeof(dma_cb_t) * 2;
        
        if (station == stations - 1)
            station = 0;
    }

    return 0;
}

static void
setup_sighandlers(void)
{
    // Catch all signals possible - it is vital we kill the DMA engine
    // on process exit!
    for (int i = 0; i < 64; i++) {
        struct sigaction sa;

        memset(&sa, 0, sizeof(sa));
        sa.sa_handler = terminate;
        sigaction(i, &sa, NULL);
    }
}

// Setup a channel with a specific subcycle time. After that pulse-widths can be
// added at any time.
int
init_channel(int channel_num, int subcycle_time_us)
{
    log_debug("Initializing channel %d...\n", channel_num);
    if (_is_setup == 0)
        fatal("Error: you need to call `setup(..)` before initializing channels\n");
    if (channel_num > DMA_CHANNELS-1)
        fatal("Error: maximum channel is %d (requested channel %d)\n", DMA_CHANNELS-1, channel_num);
    if (channels[channel_num].virtbase)
        fatal("Error: channel %d already initialized.\n", channel_num);
    if (subcycle_time_us < SUBCYCLE_TIME_US_MIN)
        fatal("Error: subcycle time %dus is too small (min=%dus)\n", subcycle_time_us, SUBCYCLE_TIME_US_MIN);

    // Setup Data
    channels[channel_num].subcycle_time_us = subcycle_time_us;
    channels[channel_num].num_samples = NUM_SAMPLES;
    channels[channel_num].width_max = channels[channel_num].num_samples - 1;
    channels[channel_num].num_cbs = channels[channel_num].num_samples * 2;
    channels[channel_num].num_pages = ((channels[channel_num].num_cbs * 32 + channels[channel_num].num_samples * 4 + \
                                       PAGE_SIZE - 1) >> PAGE_SHIFT);

    // Initialize channel
    if (init_virtbase(channel_num) == EXIT_FAILURE)
        return EXIT_FAILURE;
    if (make_pagemap(channel_num) == EXIT_FAILURE)
        return EXIT_FAILURE;
    if (init_ctrl_data(channel_num) == EXIT_FAILURE)
        return EXIT_FAILURE;
    return EXIT_SUCCESS;
}


void
init_control_blocks(uint32_t carrier_freq, int stations)
{
    for (int i = 0; i < DMA_CHANNELS && i < stations; i++)
    {
        channels[i].virtbase = mbox.virt_addr + (memory_size * i);
        dma_cb_t *cbp = channels[i].cb;
        uint32_t phys_sample_dst = CM_GP0DIV + (i * 0x8);
        uint32_t phys_pwm_fifo_addr = PWM_PHYS_BASE + 0x18;
        
        // Calculate the frequency control word (Clock freq / Desired freq)
        // The fractional part is stored in the lower 12 bits
        freq_ctl = ((float)(PLLFREQ / channels[i].carrier_freq)) * ( 1 << 12 );
        
        for (int j = 0; j < NUM_SAMPLES; j++)
        {
            channels[i].sample[j] = 0x5a << 24 | freq_ctl; // Silence
            
            // Set up a control block for a future DMA transfer of audio data (data will be filled in below)
            cbp->info = BCM2711_DMA_NO_WIDE_BURSTS | BCM2711_DMA_WAIT_RESP;
            cbp->src = mem_virt_to_phys(channels[i].sample + j);
            cbp->dst = phys_sample_dst;
            cbp->length = 4;
            cbp->stride = 0;
            cbp->next = mem_virt_to_phys(cbp + 1);
            cbp++;
            
            // Set up a future control block for a delay.
            cbp->info = BCM2711_DMA_NO_WIDE_BURSTS | BCM2711_DMA_WAIT_RESP | BCM2711_DMA_D_DREQ | BCM2711_DMA_PER_MAP(5);
            cbp->src = mem_virt_to_phys(channels[i].virtbase);
            cbp->dst = phys_pwm_fifo_addr;
            cbp->length = 4 - stations;
            cbp->stride = 0;
            cbp->next = mem_virt_to_phys(cbp + 1);
            cbp++;
        }
        cbp--;
        cbp->next = mem_virt_to_phys(channels[i].virtbase); // Here we reset the 'next' val of the last cb to be the address
    }                                                                     // of the first cb (mbox.virt_addr), so we make an infinite loop.
}

void init_rds(uint16_t pi, char *ps, char *rt, char *control_pipe)
{
    // Initialize the RDS modulator
    set_rds_pi(pi);
    set_rds_rt(rt);
    ps_count = 0;
    ps_count2 = 0;
    varying_ps = 0;
    
    if(ps) {
        set_rds_ps(ps);
        printf("PI: %04X, PS: \"%s\".\n", pi, ps);
    } else {
        printf("PI: %04X, PS: <Varying>.\n", pi);
        varying_ps = 1;
    }
    printf("RT: \"%s\"\n", rt);
    
    // Initialize the control pipe reader
    if(control_pipe) {
        if(open_control_pipe(control_pipe) == 0) {
            printf("Reading control commands on %s.\n", control_pipe);
        } else {
            printf("Failed to open control pipe: %s.\n", control_pipe);
            control_pipe = NULL;
        }
    }
}


// Initialize PWM or PCM hardware once for all channels (10MHz)
static void
init_hardware(int stations, float ppm)
{
    // Here we define the rate at which we want to update the GPCLK control 
    // register using PWM.
    //
    // Set the range to 2 bits. PLLD is at 500 MHz, therefore to get 228 kHz
    // we need a divisor of 500000000 / 2000 / 228 = 1096.491228
    //
    // This is 1096 + 2012*2^-12 theoretically.
    //
    // However the fractional part may have to be adjusted to take the actual
    // frequency of your Pi's oscillator into account. For example on my Pi,
    // the fractional part should be 1916 instead of 2012 to get exactly 
    // 228 kHz. However RDS decoding is still okay even at 2012.
    //
    // So we use the 'ppm' parameter to compensate for the oscillator error
    float divider = (PLLFREQ/(2000*228*(1.+ppm/1.e6)));
    uint32_t idivider = (uint32_t) divider;
    uint32_t fdivider = (uint32_t) ((divider - idivider)*pow(2, 12));
    printf("ppm corr is %.4f, divider is %.4f (%d + %d*2^-12) [nominal 1096.4912].\n", ppm, divider, idivider, fdivider);
    
    // Initialize GPCLK and PWM
    pwm_reg[PWM_CTL] = 0;                           // Disable PWM Module
    udelay(10);
    clk_reg[PWMCLK_CNTL] = 0x5A000006;              // Source = PLLD (500MHz)
    udelay(100);
    clk_reg[PWMCLK_DIV] = 0x5A000000 | (idivider<<12) | fdivider;    // Set PWM div to give 228 kHz
    udelay(100);
    clk_reg[PWMCLK_CNTL] = 0x5A000016;              // Set clock source to PLLD and enable + apply a MASH filter of 1
    udelay(100);
    pwm_reg[PWM_RNG1] = 2;                          // Set PWM range register to 2, which determines PWM frequency.
    udelay(10);
    pwm_reg[PWM_DMAC] = PWMDMAC_ENAB | PWMDMAC_THRSHLD; // Set DMA enabled and set DMA request threshold.
    udelay(10);
    pwm_reg[PWM_CTL] = PWMCTL_CLRF;                 // Clear PWM control register
    udelay(10);
    pwm_reg[PWM_CTL] = PWMCTL_USEF1 | PWMCTL_PWEN1; // Set PWM control register to use FIFO mode 1 and enable PWM channel 1.
    
    // Initialise the DMA channels
    for (int i = 0; i < DMA_CHANNELS && i < stations; i++)
    {
        channels[i].dma_reg[DMA_CS] = BCM2711_DMA_RESET;                        // Reset DMA control and status (CS) register
        udelay(10);
        channels[i].dma_reg[DMA_CS] = BCM2711_DMA_INT | BCM2711_DMA_END;        // Set DMA CS register to enable interrupts and end-of-transfer detection
        channels[i].dma_reg[DMA_CONBLK_AD] = mem_virt_to_phys(channels[i].cb);  // Set address for control block (in physical mem, not virtual)
        channels[i].dma_reg[DMA_DEBUG] = 7;                                     // Clear any debug error flags
        channels[i].dma_reg[DMA_CS] = 0x10880001;                               // Set DMA to begin, at mid priority, i.e. to wait for outstanding writes to complete
    }
    
    // See Note 1 below for GPIO explained
    gpio_reg[GPFSEL0] = (gpio_reg[GPFSEL0] & ~(7 << 12)) | (4 << 12); // GPIO4 needs to be ALT FUNC 0 to output the clock GPCLK0
    gpio_reg[GPFSEL1] = (gpio_reg[GPFSEL1] & ~(7 << 15)) | (4 << 15); // GPIO5 needs to be ALT FUNC 0 to output GPCLK1
 
    // Program GPCLK0 and GPCLK1 to use MASH setting 1, so fractional dividers work
    // 0x5A is CLK password, 6 specifies PLLD as CLK source, 1 << 9 sets MASH to mode 1, 1 << 4 sets enable. 
    clk_reg[GPCLK0_CNTL] = 0x5A << 24 | 6;
    udelay(100);
    clk_reg[GPCLK0_CNTL] = 0x5A << 24 | 1 << 9 | 1 << 4 | 6;
    clk_reg[GPCLK1_CNTL] = 0x5A << 24 | 6;
    udelay(100);
    clk_reg[GPCLK1_CNTL] = 0x5A << 24 | 1 << 9 | 1 << 4 | 6;
}


// setup(..) needs to be called once and starts the PWM timer. Delay hardware
// and pulse-width-increment-granularity is set for all DMA channels and cannot
// be changed during runtime due to hardware mechanics (specific PWM timing).
int setup(uint32_t carrier_freq, int stations, uint16_t pi, char *ps, char *rt, float ppm, char *control_pipe)
{
    if (_is_setup == 1)
        fatal("Error: setup(..) has already been called before\n");

    // Catch all kind of kill signals
    setup_sighandlers();

    // Initialize common stuff
    dma_reg = map_peripheral(DMA_VIRT_BASE, DMA_LEN);
    pwm_reg = map_peripheral(PWM_VIRT_BASE, PWM_LEN);
    clk_reg = map_peripheral(CLK_VIRT_BASE, CLK_LEN);
    gpio_reg = map_peripheral(GPIO_VIRT_BASE, GPIO_LEN);
    if (pwm_reg == NULL || dma_reg == NULL || clk_reg == NULL || gpio_reg == NULL)
        fatal("ERROR: One of the peripheral registers is NULL.");
        
    // DMA channel 0 begins at 0xfe007000, channel 1 at 0xfe007100. Since the first address is 
    // already mapped, we can access the location 100 registers down from DMA base to access channel 1.
    for (int i = 0; i < DMA_CHANNELS && i < stations; i++)
    {
        channels[i].dma_reg = &dma_reg[DMA_CHANNEL_INC * i];
    }
    
    // Use the mailbox interface to the VC to ask for physical memory.
    mbox.handle = mbox_open();
    if (mbox.handle < 0)
        fatal("Failed to open mailbox. Check kernel support for vcio / BCM2711 mailbox.\n");
    printf("Allocating physical memory: size = %d     ", stations * NUM_PAGES * 4096);
    if(! (mbox.mem_ref = mem_alloc(mbox.handle, stations * memory_size, 4096, MEM_FLAG))) { // We multiply by var "stations" to increase memory size
        fatal("Could not allocate memory.\n");
    }
    // TODO: How do we know that succeeded?
    printf("mem_ref = %u     ", mbox.mem_ref);
    if(! (mbox.bus_addr = mem_lock(mbox.handle, mbox.mem_ref))) {
        fatal("Could not lock memory.\n");
    }
    printf("bus_addr = %x     ", mbox.bus_addr);
    if(! (mbox.virt_addr = mapmem(BUS_TO_PHYS(mbox.bus_addr), stations * memory_size))) {
        fatal("Could not map memory.\n");
    }
    printf("virt_addr = %p\n", mbox.virt_addr);
        
    // Start PWM/PCM timing activity
    init_hardware(stations, ppm);
    
    // Setup control block architecture
    init_control_blocks(carrier_freq, stations);
    
    // Setup RDS data
    init_rds(pi, ps, rt, control_pipe);

    _is_setup = 0x1;
    return EXIT_SUCCESS;
}


int main(int argc, char **argv) {
    char *audio_files[MAX_STATIONS];
    int stations = 0;
    char *control_pipe = NULL;
    uint32_t carrier_freq = 107900000;
    char *ps = NULL;
    char *rt = "PiFmRds: live FM-RDS transmission from the RaspberryPi";
    uint16_t pi = 0x1234;
    float ppm = 0;
    
    
    // Parse command-line arguments
    for(int i=1; i<argc; i++) {
        char *arg = argv[i];
        char *param = NULL;
        
        if(arg[0] == '-' && i+1 < argc) 
            param = argv[i+1];
        
        if((strcmp("-wav", arg)==0 || strcmp("-audio", arg)==0) && param != NULL) {
            
            // There can be up to five audio files submitted, parse args until next tack command is found.
            for (int j = i + 1; j < argc; j++) {
                char *next_arg = argv[j];
                if(next_arg == NULL || next_arg[0] == '-')
                    break;
                else {
                    audio_files[stations] = next_arg;
                    stations++;
                    i++;
                }
            }
        } else if(strcmp("-freq", arg)==0 && param != NULL) {
            i++;
            carrier_freq = 1e6 * atof(param);
            if(carrier_freq < 76e6 || carrier_freq > 108e6)
                fatal("Incorrect frequency specification. Must be in megahertz, of the form 107.9, between 76 and 108.\n");
        } else if(strcmp("-pi", arg)==0 && param != NULL) {
            i++;
            pi = (uint16_t) strtol(param, NULL, 16);
        } else if(strcmp("-ps", arg)==0 && param != NULL) {
            i++;
            ps = param;
        } else if(strcmp("-rt", arg)==0 && param != NULL) {
            i++;
            rt = param;
        } else if(strcmp("-ppm", arg)==0 && param != NULL) {
            i++;
            ppm = atof(param);
        } else if(strcmp("-ctl", arg)==0 && param != NULL) {
            i++;
            control_pipe = param;
        } else {
            fatal("Unrecognised argument: %s.\n"
            "Syntax: pi_fm_rds [-freq freq] [-audio file] [-ppm ppm_error] [-pi pi_code]\n"
            "                  [-ps ps_text] [-rt rt_text] [-ctl control_pipe]\n", arg);
        }
    }
    
    int setup_success = setup(carrier_freq, stations, pi, ps, rt, ppm, control_pipe);
    
    int errcode = tx(carrier_freq, audio_files, stations, control_pipe);
    
    terminate(errcode);
}



/* Note 1: On GPIO pin assignments
 * 
 * The documentation in the BCM2711 manual seems convoluted, and assigning a function to a GPIO pin may
 * seem esoteric at first. You'll see on page 77 (sec 5.3) that each GPIO can have up to six alternate
 * functions. We want to set it to ALT0 in our cases, i.e. the GPCLK. You'll also see on page 66 (sec 
 * 5.2) a list of registers with offsets which control the GPIO pin in question, and that there are six
 * options at the top, GPFSEL0 ... GPFSEL5. These are 32-bit wide slots in which each trio of bits
 * represents a pin function. Since there are 58 pins, and each pin has 6 possible functions, we need
 * three bits (2^3 = 8 [six functions + input and output]) to represent all functions. So in total we
 * need 58 * 3 = 174 bits to fully represent all the GPIO pins and functions. Each GPFSEL is 32 bits long,
 * so we need six registers to hold all bit trios (174 / 32 = 5.4375 -> 6 registers). If we wanted to 
 * assign GPIO 11 to its first function (FUNC 0), we would write the bits 100 or 0x04 (see doc pg 67) to
 * registers 5:3 at GPIO_BASE + GPFSEL1 = 0x7e200004. So in reality we would have to create a bitmask
 * for whatever is already there and add our assignment to it, as such.
 *       
 *      For example, GPFSEL1 contains   11 100 010 110 110 010 000 100 100 010 101
 *      Bitmask:                        11 111 111 111 111 111 111 111 111 000 111
 *                                                                      [GPIO 11]
 *      Bitmask & GPFSEL1:              11 100 010 110 110 010 000 100 100 000 101
 *      Desired function:                                                  100 000
 *      (Bitmask & GPFSEL1) | Desired:  11 100 010 110 110 010 000 100 100 100 101
 * 
 * Thus we have inserted our desired data for GPIO 11 without disturbing any other data, and can write
 * back to GPIO_BASE + GPFSEL1 as a 32 bit whole.
*/
