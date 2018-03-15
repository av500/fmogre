/*
 * File:   FMOgre_DMA_Main.c for FMOgre_44 (44-pin surface-mount) board
 * Author: wsy
 *
 * Created on August 16, 2013, 7:10 AM
 * 
 *  This file is Copyright 2013-2015 William S. Yerazunis,
 *  It is licensed CC-BY-SA-NC
 *
 *  (creative commons, attribute source, share alike, no commercial use without other license)
 */

/* edited by Timo Rozendal, all these edits are mentioned with TIMO EDIT in the comments
 * Reasons for editing: 
 * - sync was connected to another port on the latest design (RA9  in stead of RB9)
 * - rerouted v/oct input to an7 to make use of the trimmer in the latest design
 * - enabling external voltage reference (requires new edits in circuit)
 * - using an8 for pitch knob input (was needed since an0 is used for voltage reference)
 * 
 * these edits still need to be incorporated in the other comments
 */

/*
 *    Steps in the process:
 *
 *  - Initialize the IO pins:
 *     - analog outputs:
 *          10(DAC1RP : primary output), 11(DAC1RN), 14(DAC1LP), 15(DAC1LN) (RB12,13,14,15)
 *     - analog inputs:
 *          AN0 (19, RA0):  exponential pitch jack
 *          AN1 (20, RA1):  operator feedback jack
 *          AN2 (21, RB0):  freq mod jack
 *          AN3 (22, RB1):  phase mod jack
 *          AN4 (23, RB2):  phase mod index knob
 *          AN5 (24, RB3):  fm mod index knob
 *          AN6 (25, RC0):  operator feedback knob
 *          AN7 (26, RC1):  pitch knob
 *     - digital inputs:
 *          RA2 (30):   (also crystal in)
 *          RA3 (31):   (also crystal out)  ???  if low, use RA2 as 7.37MHz osc in ???
 *          RA8 (32):   Switch 1 (external 100K pullup, AKA lfo switch LFOSWITCH
 *          RB4 (33):   Switch 2 (external 100K pullup, AKA phase/resolution RESOLSWITCH
 *          RA4 (34):   Switch 3 (external 100K pullup)
 *          pin 17,28,40: Vdd (+3.3v)
 *          RB5 (41): / PGED3 (5Vtol)  idle heartbeat - each Hz is ~1 MIP available
 *          RB6 (42): / PGEC3 (5Vtol)  negative frequency
 *          RB7 (43):         (5Vtol)  negative phase
 *          RB8 (44):         (5Vtol)  Sync Out (25% duty cycle pulse train)
 *          RB9  (1):         (5Vtol)  Sync In
 *          pins 6, 16, 29, 39: Vss (ground)
 *          pin 7: Vcap (core voltage stabilizer)
 *          pin 21: PGED2 / RB10 (5Vtol) - used for programming
 *          pin 22: PGEC2 / RB11 (5Vtol) - used for programming
 *          23-26 = RP, RN, LP, and LN analog out = see above
 *          pin 27: AVss
 *          pin 28: AVdd
*/

#include <stdio.h>
#include <p33FJ128GP804.h>
#include "dsp.h"
#include <stdint.h>
#include <string.h>

// DSPIC33FJ64GP802 Configuration Bit Settings

#include <p33Fxxxx.h>

// FBS
#pragma config BWRP = WRPROTECT_OFF	// Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH	// Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM	// Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF	// Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH	// Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM	// Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF	// General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF	// General Segment Code Protection (User program memory is not code-protected)

_FOSCSEL(FNOSC_PRIPLL & IESO_ON)
_FOSC   (POSCMD_XT & OSCIOFNC_OFF & IOL1WAY_OFF & FCKSM_CSECME)

// FWDT
#pragma config WDTPOST = PS32768	// Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128	// WDT Prescaler (1:128)
#pragma config WINDIS = OFF	// Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF	// Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)
// FPOR
#pragma config FPWRT = PWR16	// POR Timer Value (16ms)
#pragma config ALTI2C = OFF	// Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
// FICD
#pragma config ICS = PGD2	// Comm Channel Select (Communicate on PGC2/EMUC2 and PGD2/EMUD2)
#pragma config JTAGEN = OFF	// JTAG Port Enable (JTAG is Disabled)

//     Now for our actual application specific stuff

#define LFOSWITCH        PORTAbits.RA8
#define RESOLUTIONSWITCH PORTBbits.RB4
#define SAMPLESWITCH     PORTAbits.RA4

// FM state variables
static volatile unsigned long curbasephase;
static volatile long curpitchval;
static volatile long curpitchincr;
static volatile long curfreqmod;
static volatile long cvpm_dv, old_cvpm, older_cvpm, cvpm_predicted; 
static volatile long curphasemod;
static volatile long curaltphasemod;
static volatile long curfbgain;
static volatile char oldhardsync;

//  to make stuffing the ADC values easier, we use some DEFINEs:
static volatile unsigned int cvdata[16] __attribute__ ((space(dma), aligned(256)));

//#define REVISION1

#ifdef REVISION1
  #define cvpitch                  (cvdata[7])	// TIMO EDIT: was 0
  #define cvfb                     (cvdata[1])
  #define cvfm                     (cvdata[2])
  #define cvpm                     (cvdata[3])
  #define cvpmknob    ((long)4095 - cvdata[4])
  #define cvfmknob    ((long)4095 - cvdata[5])
  #define cvfbknob    ((long)4095 - cvdata[6])
  #define cvpitchknob ((long)4095 - cvdata[0])	// TIMO EDIT: was 7
#else
  #define cvpitch     (cvdata[7])		// TIMO EDIT: was 0
  #define cvfb        (cvdata[1])
  #define cvfm        (cvdata[2])
  #define cvpm        (cvdata[3])
  #define cvpmknob    (cvdata[4])
  #define cvfmknob    (cvdata[5])
  #define cvfbknob    (cvdata[6])
  #define cvpitchknob (cvdata[8])		// TIMO EDIT: was 0 and before that 7
#endif

#define SAMPLE_BUF_LEN   4096

// Test point defines
#define TESTPOINT1 PORTCbits.RC6
#define TESTPOINT2 PORTCbits.RC7
#define TESTPOINT3 PORTCbits.RC8

// Unless we use __attribute__ ((far)) we would need to use large memory model for this to work.
__attribute__ ((far)) static unsigned int sample_buf[SAMPLE_BUF_LEN];
static volatile unsigned sample_index = 0;

// Pins RB5, RB6, and RB7 are the LED port.
// Pin  RB8 is SYNC OUT, pin RB9 is SYNC IN.
#define SYNC_IN (PORTAbits.RA9)			// TIMO edit  RB9->RA9

// The wavetable for sine waves are in wavetable.h
#include "wavetable.h"

// oscillator phase: 32 bit unsigned int, used as follows:
//   2^31 to 2^19 - used for output (12 bits into the LUT, same as the DX7 had).
//   2^18 to 2^ 0 - fractional phase, not used but carried over which maintains phase coherency
//
// By using this format, it is unnecessary to rectify phase wrapping in
// either directions - overflow and underflow can be IGNORED. The latter also
// makes this do thru-zero FM work correctly.

void __attribute__ ((__interrupt__, __auto_psv__)) _DAC1LInterrupt(void)
{
}

void __attribute__ ((__interrupt__, __auto_psv__)) _DAC1RInterrupt(void)
{
	// The analog outputs are all driven from this interrupt.
	// Whenever the DAC finishes and creates space in the FIFO, this
	// interrupt fires off to fill that space in the FIFO.  Since
	// the DAC itself is clocked by a PLL from the crystal, the result
	// is a very stable tone generation IFF the input freq. and phase
	// are solid and unchanging (they aren't- the ADCs are a bit noisy)

	IFS4bits.DAC1RIF = 0;	// clear the interrupt
	TESTPOINT2 = 1;		// show we're in DAC1RInterrupt

//#define TEST_ADC_DAC_PASSTHROUGH
#ifdef TEST_ADC_DAC_PASSTHROUGH
	DAC1LDAT = cvfm << 4;
	DAC1RDAT = cvpm << 4;
	TESTPOINT2 = 0;
	return;
#endif
	// Store the AD channel 1 (wavesampling) into the playback buffer
	// Note we store this in 12 bit (0-4095) mode; when we do our
	// windowing multiply we get it out to 16 bits
	sample_buf[sample_index] = (((unsigned long) 4095) - cvfb);
	if (PORTAbits.RA9 == 0) { // is SYNC / FREEZE on or off? // TIMO edit  RB9->RA9
		sample_index++;
	};		
	if (sample_index >= SAMPLE_BUF_LEN)
		sample_index = 0;

	// CVPITCHINCR could also be calculated in the mainline.  We used to do it here
	// but it ate too much CPU and slowed down the ADC interrupt.

	curbasephase = curbasephase + curpitchincr;

	// Calculate the linear frequency modulation.  Used to do it here at interrupt,
	// now do it in the mainline to speed up the interrupt.
	// Add in linear frequency to the pitch base phase.
//#define INTERRUPT_CURFREQMOD
#ifdef INTERRUPT_CURFREQMOD
	curfreqmod = cvfmknob * (((long) 2047 - cvfm) << 6;
#endif
	curbasephase = curbasephase + curfreqmod;

	unsigned long truncated_phase = (curbasephase >> 20) & 0x00000FFF;

//#define RAW_PLUS_LINEAR_FM_CHECK
#ifdef RAW_PLUS_LINEAR_FM_CHECK
	DAC1RDAT = sine_table[truncated_phase];
	DAC1LDAT = sine_table[truncated_phase];
	return;
#endif
	// sinevalue is the output of the basephase->sine conversion BUT it's
	// not used directly.  Instead, it's the intermediate for the operator
	// feedback, and used to generate the final (actual) FM phase.
	// Because it's synchronous with curbasephase, we *do* calculate it at
	// interrupt time (fortunately, it's fast)
	unsigned long sinevalue = SAMPLESWITCH ? sample_buf[truncated_phase]
	                                       : sine_table[truncated_phase];
	
	// used to calculate curphasemod here, but now do it in mainline (again, to
	// minimize time spent in the interrupt and thus not slow down the ADC stream)
//#define INTERRUPT_CURPHASEMOD
#ifdef INTERRUPT_CURPHASEMOD
	curphasemod = (((cvpm - 2048) * cvpmknob) >> 12);
#endif
	// FM + Feedback on one, FM + PM + FB on other

	// HW change: normal FB input to 3V3 and use absolute value if a bipolar CV is patched:
	long _cvfb = abs(2048 - cvfb) * 2;
	long fbgain = ((long)(SAMPLESWITCH ? 4096 : _cvfb) * cvfbknob) >> 12;

	long fbphase = (((sinevalue - 32767) * fbgain) >> 12);
	unsigned long final_phase_fm_feedback = (truncated_phase + fbphase) & 0x00000FFF;

	// calculate the phase modulation.   Can do here or baseline.
#define INTERRUPT_CURPHASEMOD
#ifdef INTERRUPT_CURPHASEMOD
#define DEADBAND 0
	// new method- keep current, old, and older.   
	if (cvpm != old_cvpm) {
		older_cvpm = old_cvpm; cvpm_predicted = older_cvpm; old_cvpm = cvpm;
	} else {
		cvpm_predicted = (cvpm_predicted + old_cvpm) >> 1;	// use this for 26.4 KHz max freq 
	}

	curphasemod   = (((long) 2047 - cvpm_predicted) * cvpmknob) >> 10;
#endif
	unsigned long final_phase_fm_fb_pm = 0x00000FFF & (
			// native base phase
			final_phase_fm_feedback + (RESOLUTIONSWITCH ? 0 : curphasemod));
	
	// all done - stuff the DAC output registers.
	DAC1RDAT = sine_table[0x00000FFF & final_phase_fm_feedback];

	// Next section calculates the DAC1L output.  This can be
	// from the sine table or the incoming sample, and then
	// resolution reduced.
	unsigned int phase = 0xfff & (final_phase_fm_fb_pm + 2048);
	
	// dist is the distance between read and write point, range is 0-2048
	unsigned int dist1 = (abs(final_phase_fm_fb_pm - sample_index)); 
	             dist1 = (dist1) > 256 ? 16 : (dist1 >> 4);
	unsigned int dist2 = 16 - dist1; 

	long dac1la = SAMPLESWITCH ? sample_buf[final_phase_fm_fb_pm] * dist1 + sample_buf[phase] * dist2
	                           : sine_table[0X00000fff & (final_phase_fm_fb_pm)]; 
	long dac1lb = 0;
	 
	if (RESOLUTIONSWITCH) {
		// DANGER DANGER DANGER!!! Use muldiv decimation only with DAC dividers 
		// of 6 or greater!   Otherwise, the interrupt cannot keep up with the
		// demand of the DAC oversampling hardware and you'll underrun which makes
		// lots of negative-going pulses.  Not A Good Thing!
		dac1lb = dac1la;
#define MULDIV_DECIMATION
#ifdef MULDIV_DECIMATION
		// next step:  decimation / resolution reduction - uses integer divide
		// then multiply by same amount to reduce the resolution.
		int shift = curaltphasemod >> 8; dac1lb = (((dac1la - 32768) / shift) * shift) + 32768;
#endif

//#define SHIFT_DECIMATION
#ifdef SHIFT_DECIMATION
		int shift = curaltphasemod >> 8; dac1lb = (((dac1la - 32768) >> shift) << shift) + 32768;
#endif

//#define TABLE_MASK_DECIMATION
#ifdef TABLE_MASK_DECIMATION
		// work in progress.   Not really satisfactory yet.
		dac1lb = dac1la & highorderbittable[((4095 - cvpm) * cvpmknob) >> 14];
#endif
	}

	DAC1LDAT = RESOLUTIONSWITCH ? dac1lb : dac1la;

	// hard sync out - tried doing this in base loop, too much jitter.  
	// Even here, there's a lag of about 0.2 millisecond in the DAC path
	// due to the 256x oversampling versus the direct path here for SYNC OUT
	// Note that Hard Sync IN changes if we're in granular/sampling mode to
	// become _FREEZE_INPUT_SAMPLES_
	if (SAMPLESWITCH) {
		PORTBbits.RB8 = sample_index < 0x000000FF;
	} else {
		PORTBbits.RB8 = final_phase_fm_feedback > 0x00000800;
	}

	TESTPOINT2 = 0; 
	return;
}

static void setup_pins(void)
{
	// set up which pins are digital in and out
	TRISA = 0x0;		// Inputs on RA0-1 (ADC 0 and 1)
	TRISAbits.TRISA0 = 1; 
	TRISAbits.TRISA1 = 1; 
	TRISAbits.TRISA4 = 1;	// RA4 is switch 3(sine versus sample)
	TRISAbits.TRISA8 = 1;	// RA8 is switch 1 (VCO versus LFO)
	TRISAbits.TRISA9 = 1;	// Pin RA9 is SYNC IN. TIMO EDIT
	
	TRISC = 0x0;		// Inputs on RC0-1 (ADC 6 and 7)
	TRISCbits.TRISC0 = 1; 
	TRISCbits.TRISC1 = 1;
#ifndef REVISION1
	TRISCbits.TRISC2 = 1;	// TIMO EDIT (to use an8 in stead of an0)
#endif
	TRISB = 0x0;		// Inputs on RB0-3 (ADC 2,3,4,5), RB8-9 (switches)RB12-15 (DACs)
	TRISBbits.TRISB0 = 1; 
	TRISBbits.TRISB1 = 1;
	TRISBbits.TRISB2 = 1;
	TRISBbits.TRISB3 = 1; 
	TRISBbits.TRISB4 = 1;	// RB4 is switch 2
				// Pins RB5, RB6, and RB7 are the LED ports 
	TRISBbits.TRISB5 = 0;	// RB5 is heartbeat (final out positive); we toggle TRIS.
	PORTBbits.RB5    = 1; 
	TRISBbits.TRISB6 = 0;	// RB6 is negative frequency
	PORTBbits.RB6    = 1; 
	TRISBbits.TRISB7 = 0;	// RB7 is negative phase
	PORTBbits.RB7    = 1; 
	TRISBbits.TRISB8 = 0;	// Pin RB8 is SYNC OUT,
	//TRISBbits.TRISB9 = 1;  // Pin RB9 is not SYNC IN. TIMO EDIT
	TRISBbits.TRISB12 = 1; 
	TRISBbits.TRISB13 = 1; 
	TRISBbits.TRISB14 = 1; 
	TRISBbits.TRISB15 = 1;
	
	// use port RC6 to see stuff like ADC and DAC interrupts, so output...
	TRISCbits.TRISC6 = 0;	// Test Point 1
	TRISCbits.TRISC7 = 0;	// Test Point 2
	TRISCbits.TRISC8 = 0;	// Test Point 3   
}

static void setup_dac(void)
{
	// set up the DACs
	DAC1CONbits.DACEN = 1;	// enable the audio dac
	DAC1CONbits.AMPON = 1;	// enable the output amplifier
	DAC1CONbits.FORM  = 0;	// unsigned data (0 = unsigned data)
	
	// CHANGE THE DACFDIV TO CHANGE OUTPUT SAMPLING SPEED!!!
	// The following pitch-DACFDIV values are relative, of course.
	// (Note, this is not sample RATE.  This is sine wave rendered)
	// 8 = 14.7 KHz max pitch
	// 7 = 16.5 KHz max pitch
	// 6 = 18.9 KHz max pitch, 50 KHz phase update, fastest for MULDIV 
	// 5 = 22.0 KHz max pitch, 50 KHz phase update
	// 4 = 26.4 KHz max pitch, 50 KHz phase update
	// 3 = 33.0 KHz max pitch, 33 KHz phase update
	// 2 = 44.07 kHz max pitch, 28 KHz phase update 
	// 1 .... antialias malfunction.  Don't use this.
	// To be honest, it sounds really good at either 3 or 4 to me if you want
	// dogs to hear it, and just fine at 6, which allows MULDIV-decimation.
	DAC1CONbits.DACFDIV = 6;	// (was 3, then 6, then 8) divide Fosc to drive 
					// interpolator. 3 = 80.078KHz @ 82MHz F_VCO
	DAC1STAT = 0xFFFF;		// everything on
	DAC1STATbits.LITYPE = 1;	// 0 means interrupt on Left LIFO not full
	DAC1STATbits.RITYPE = 1;	// 0 means interrupt on Right LIFO not full
	DAC1STATbits.LMVOEN = 0;	// left channel midpoint output off
	DAC1STATbits.RMVOEN = 0;	// right channel midpoint output off
	
	// set up DMA channel 0 for the ADC 
	DMA0CONbits.CHEN  = 0;	// Disable DMA channel 0 during setup
	DMA0CONbits.SIZE  = 0;	// 0 = word transfers
	DMA0CONbits.DIR   = 0;	// 0 = read from peripheral, write to RAM
	DMA0CONbits.HALF  = 0;	// 0 = wait for full transfer before interrupt
	DMA0CONbits.NULLW = 0;	// 0 = normal write 
	DMA0CONbits.AMODE = 2;	// 2 = peripheral indirect addressing
	DMA0CONbits.MODE  = 0;	// 0 = continuous transfer, no ping-pong
	DMA0PAD = (int) &ADC1BUF0;	// source the data from the ADC
	DMA0CNT = 0;		// How many transfers (DMA0CNT=0 --> 1 transfer)
	DMA0REQbits.IRQSEL = 13;// ADC1DAT0 is #13 (see manual table 8.1)

	// DMA buffer (note that the pointer passed to the DMA engine is
	// NOT the same as the CPU address of the DMA buffer; the two have
	// overlapping but not zero-originned address spaces.)
	DMA0STA = __builtin_dmaoffset(&cvdata);	// DMA engine's buffer address - not CPU's
	DMA0STB = 0;		// No DMA secondary register
	
	IFS0bits.DMA0IF = 0;	// Clear DMA0's interrupt flag
	IEC0bits.DMA0IE = 0;	// Clear the DMA interrupt enable
	
	DMA0CONbits.CHEN = 1;	// Enable the DMA channel.   Off you go!
}

static void setup_adc(void)
{
	// Set up the ADCs
	AD1CON1bits.ADON = 0;	// AD converter off while configuring
	AD1CON1bits.AD12B = 1;	// 1 = 12-bit mode
	AD1CON1bits.FORM = 0;	// output as unsigned 12 bit integer
	AD1CON1bits.SSRC = 2;	// TIMER3 ends sampling and starts conversion
	AD1CON1bits.ASAM = 1;	// Sample again right after conversion
	AD1CON1bits.ADSIDL = 0;	// keep it on even during idle
	AD1CON1bits.ADDMABM = 0;// Module provides real addr (1) or s/g (0) to DMA
	AD1CON1bits.DONE = 0;	// set ADC to not be done yet (HW sets to 1 on completion)
	AD1CON2bits.CSCNA = 1;	// 1 = scan inputs, 0 = do not scan inputs
#ifdef REVISION1
	AD1CON2bits.VCFG = 0;	// nothing fancy, swing is Vss to Vdd  // TIMO EDIT, was 0, changed it to 1 for external positive ref
#else
	AD1CON2bits.VCFG = 1;
#endif
	AD1CON2bits.CHPS = 0;	// use just 1 ADC channel
	AD1CON2bits.SMPI = 7;	// (was 7)increment DMA adddress after N+1 converts
	AD1CON2bits.BUFM = 0;	// always start buffer fill at 0
	AD1CON2bits.ALTS = 0;	// always use sample A
	
	// ADC clock must nominally be at least 117 nS long per cycle
	AD1CON3bits.ADRC = 0;	// 0 = sysclock; 1 = use the ADC's internal RC clock
	AD1CON3bits.ADCS = 8;	// (was 7, then 4, then 2)  divide 80 mHz by this plus 1
	
	//AD1CON3bits.SAMC = 1;      //  Auto sample time bits (ignored here)
	AD1CON4bits.DMABL = 0;	// DMA buffer length 1 word per analog input
	AD1CHS0bits.CH0NB = 0;	// channel B negative input is Vref-
	AD1CHS0bits.CH0SB = 0;	// channel B pos input is input AD0
	AD1CHS0bits.CH0NA = 0;	// channel A negative input is Vref-
	AD1CHS0bits.CH0SA = 0;	// channel A positive input is input AD0
#ifdef REVISION1
	AD1CSSL  = 0x00FF;	// AN0-7 enabled for scan
	AD1PCFGL = 0xFF00;	// enable ADC on low 8 AD pins.
#else
	AD1CSSL  = 0x01FE;	// AN0-7 enabled for scan	// TIMO EDIT was 00FF (to use an8 in stead of an0)
	AD1PCFGL = 0xFE00;	// enable ADC on low 8 AD pins.	// TIMO EDIT was FF00 (to use an8 in stead of an0)
#endif
	IFS0bits.AD1IF   = 0;	// Clear the AD interrupt flag
	IEC0bits.AD1IE   = 0;	// Do not enable AD interrupt 
	AD1CON1bits.ADON = 1;	// turn on the ADC
	
	// Set up the TIMER3 to keep kicking the ADC.   We use TIMER3 because
	// only timer3 has a direct wire to kick the ADC into sample/convert.
	TMR3 = 0x0000;
	
	// PR3 is the scale factor for the ADC converter.  400 KHz
	// sounds pretty good (that's update at 50 KHz per input), PR3=100.
	// Yes, there is crosstalk from one channel to another.  But it's 
	// not improved by slowing down the ADC (yes, I've tested this all the way
	// down to 8 KHz sample rate and going slower doesn't help; even more
	// amazingly it doesn't sound that bad at all even though the scope trace
	// is utter crud.)
	PR3             = 100;	//  100 = prescale to 400 KHz
	T3CONbits.TSIDL = 0;	// keep timing in idle
	IFS0bits.T3IF   = 0;	// Clear T3 interrupt
	IEC0bits.T3IE   = 0;	// Disable T3 interrupt
	T3CONbits.TON   = 1;	// start the timer
	
	// zero the ADC inputs in preparation for those that aren't updated
	memset((void*)cvdata, 0, sizeof(cvdata));
}

static void setup_clocks(void)
{
	// Configure Oscillator to operate the device at ~40MHz
	// Fosc = Fin  *   M      / (N1 * N2), Fcy = Fosc / 2
	// Fosc = 4.0M * (80 + 2) / ( 2 * 2 ) = 82.00Mhz

	// Current crystal is 4 MHz so the below actually clocks at 82MHz/41 MIPS
	CLKDIVbits.PLLPRE  = 0;	 // N2 = 2
	PLLFBD             = 80; // M  = 80 + 2 = 82, set this to 78 for an accurate 80 MIPS
	CLKDIVbits.PLLPOST = 0;	 // N1 = 2
				 // 4MHz xtal + 0/80/0 yields ~41 MHz inst speed (measured)

	// Disable Watch Dog Timer
	RCONbits.SWDTEN = 0;
	
	// Wait for PLL to lock
	while (OSCCONbits.LOCK != 1) {
	};
	
	//   Set up aux oscillator channel
	ACLKCONbits.SELACLK  = 0;	// aux oscillator from main F_osc;
	ACLKCONbits.APSTSCLR = 6;	// divide by 2
	ACLKCONbits.ASRCSEL  = 0;	// use primary clock as source (but doesn't matter)
}

int main(int argc, char **argv) 
{
	setup_clocks();
	
	setup_pins();
	
	setup_dac();
	
	setup_adc();

	// enable the DAC interrupt
	IEC4bits.DAC1RIE  = 1;	// enable the right channel DAC FIFO interrupt
	IPC19bits.DAC1RIP = 5;	// set the FIFO interrupt priority (7 is max)
	
	// The main "WHILE" loop - this does those things that don't have
	// to be absolutely real time (that is, servicing the ADC and DAC
	// interrupts.   Things like calculations of various things that are
	// dependent on the ADC and are needed every DAC tick but don't
	// necessarily have to change every DAC tick are calculated here.
	// These are things like the RB5 "hearbeat" LED, RB6 neg Freq, RB7 neg phase,
	// RB8 and RB9 hard sync in and hard sync out, and the internals:
	// curpitchval, curpitchincr, and curphasemod.
	while (1) {	
	
//#define HEARTBEAT_CPU
#ifdef HEARTBEAT_CPU
		static long int tick = 0;
		PORTBbits.RB5 = 0x1 & (tick++ >> 17);	// at >>17, and 25 instructions counting loop
							// each flash = ~3 MIP of slack
#endif

#define HEARTBEAT_WAVEPHASE
#ifdef HEARTBEAT_WAVEPHASE
		// Turns out that if you just toggle a bit fast the LED doesn't
		// respond AT ALL (RC issues - the 220 ohm ballast resistance and the LED's
		// capacitance form an RC filter and the output voltage never gets high
		// enough to turn on the LED).  So we use TRIS (tristate) instead.
		//      Use this for normal operation (on in first 1/2 of wave)
		TRISBbits.TRISB5 = 0x7FF < (0x00000FFF & (curbasephase >> 20));
#endif

//#define HEARTBEAT_NOTINTERRUPT
#ifdef HEARTBEAT_NOTINTERRUPT
		PORTCbits.RC7 = 1; 
		PORTCbits.RC7 = 0;
#endif

#define BASELINE_CVPITCHINCR
#ifdef BASELINE_CVPITCHINCR
		curpitchval = cvpitchknob + ((long) 2047 - cvpitch);
		// use exp_table to get exp freq resp, or if RA8 (aka FLOSWITCH) is turned off
		// then we're in LFO mode and we use the value rightshifted 12 bits (very low
		// frequency).
		long exp_val = exp_table[(curpitchval < 0) ? 0 : (curpitchval > 0x00000FFF) ? 0x00000FFF : curpitchval];
		if( LFOSWITCH ) {
			exp_val >>= 12;
		}
		curpitchincr = exp_val;
#endif
		// We can calculate linear frequency modulation here.
#define BASELINE_CURFREQMOD
#ifdef BASELINE_CURFREQMOD
		curfreqmod = cvfmknob * (((long) 2047) - cvfm) << 6;

		// Output RB6 high if we have negative frequency.  Note because
		// of capacitance issues, we ALWAYS have RB6 on, but rather
		// just turn on and off the tristate (TRISbits) for RB6
		PORTBbits.RB6 = 1; TRISBbits.TRISB6 = curpitchincr + curfreqmod < 0 ? 0 : 1;
#endif

//#define BASELINE_CURPHASEMOD    
#ifdef BASELINE_CURPHASEMOD
		curphasemod    = ((((long) 2047 - cvpm) * cvpmknob) >> 10); 
		curaltphasemod =  (((long) 2047 - cvpm) * cvpmknob) + 256;
#endif
		curaltphasemod =  (((long) 2047 - cvpm) * cvpmknob) + 256;
		
		// Output RB7 driven high if we have negative phase.   Note that
		// because of capacitance issues, we can't just output the bit; we
		// have to change the tristate (TRISbits) to hi-Z the output.
		PORTBbits.RB7 = 1; TRISBbits.TRISB7 = curpitchincr + (curphasemod << 13) < 0 ? 0 : 1;

#define BASELINE_HARDSYNC
#ifdef BASELINE_HARDSYNC
		// Do we have a HARD SYNC IN request on pin RB9?
		// GROT GROT GROT note that this hacky hysteresis could / should
		// really be done in an interrupt.  Maybe later....
		if (PORTAbits.RA9 == 1 && SAMPLESWITCH == 0) { // TIMO EDIT RB9->RA9
			if (oldhardsync == 0) {
				curbasephase = 0; 
				oldhardsync = 1;
			}
		} else {
			oldhardsync = 0;
		}
#endif

//#define BASELINE_CURFBGAIN
#ifdef BASELINE_CURFBGAIN
		curfbgain = ((4095 - cvfb) * cvfbknob) >> 12;
#endif
	}
}
