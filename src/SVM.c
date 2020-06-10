/********************************************************************************
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Incorporated ("Microchip") retains all ownership and 
* intellectual property rights in the code accompanying this message and in all 
* derivatives hereto.  You may use this code, and any derivatives created by 
* any person or entity by or on your behalf, exclusively with Microchip's
* proprietary products.  Your acceptance and/or use of this code constitutes 
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO 
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A 
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S 
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE, WHETHER 
* IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), 
* STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, 
* PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF 
* ANY KIND WHATSOEVER RELATED TO THE CODE, HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN 
* ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT 
* ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO 
* THIS CODE, SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO 
* HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and 
* determining its suitability.  Microchip has no obligation to modify, test, 
* certify, or support the code.
*
* REVISION HISTORY:
*                                                                     
  ***************************************************************************/

 /****************************************************************************
 *                                                                      
 *    Author: Steve Bowling                                             
 *                                                                     
 *    Filename:       svm.c					                           *
 *    Date:           3/2/04                                           
 *    File Version:   1.00                                             
 *                                                                     
 *    Tools used:    Compiler  -> 1.10                                 
 *                                                                     
 *    Linker File:    p30f4012.gld                                     
 *                                                                     
 *                                                                     
 *****************************************************************************
 *	Code Description
 *  
 *  This file implements 3-phase space vector modulation.  
 *****************************************************************************/

#include "p30fxxxx.h"
#include "svm.h"

//---------------------------------------------------------------------
// These are the definitions for various angles used in the SVM 
// routine.  A 16-bit unsigned value is used as the angle variable.
// The SVM algorithm determines the 60 degree sector
#define	VECTOR1	0				// 0 degrees
#define	VECTOR2	0x2aaa			// 60 degrees
#define	VECTOR3	0x5555			// 120 degrees
#define	VECTOR4	0x8000			// 180 degrees
#define	VECTOR5	0xaaaa			// 240 degrees
#define	VECTOR6	0xd555			// 300 degrees
#define	SIXTY_DEG	0x2aaa

//---------------------------------------------------------------------

// This is the maximum value that may be passed to the SVM 
// function without overmodulation.  This limit is equivalent
// to 0.866, which is sqrt(3)/2.
#define VOLTS_LIMIT	28300

// This sinewave lookup table has 171 entries.  (1024 points per
// electrical cycle -- 1024*(60/360) = 171)
// The table covers 60 degrees of the sine function.

int sinetable[] = //__attribute__((far,section(".const,r")))
{0,201,401,602,803,1003,1204,1404,1605,1805,
2005,2206,2406,2606,2806,3006,3205,3405,3605,3804,4003,4202,4401,4600,
4799,4997,5195,5393,5591,5789,5986,6183,6380,6577,6773,6970,7166,7361,
7557,7752,7947,8141,8335,8529,8723,8916,9109,9302,9494,9686,9877,10068,
10259,10449,10639,10829,11018,11207,11395,11583,11771,11958,12144,
12331,12516,12701,12886,13070,13254,13437,13620,13802,13984,14165,
14346,14526,14706,14885,15063,15241,15419,15595,15772,15947,16122,
16297,16470,16643,16816,16988,17159,17330,17500,17669,17838,18006,
18173,18340,18506,18671,18835,18999,19162,19325,19487,19647,19808,
19967,20126,20284,20441,20598,20753,20908,21062,21216,21368,21520,
21671,21821,21970,22119,22266,22413,22559,22704,22848,22992,23134,
23276,23417,23557,23696,23834,23971,24107,24243,24377,24511,24644,
24776,24906,25036,25165,25293,25420,25547,25672,25796,25919,26042,
26163,26283,26403,26521,26638,26755,26870,26984,27098,27210,27321,
27431,27541,27649,27756,27862,27967,28071,28174,28276,28377};

//---------------------------------------------------------------------
// The function SVM() determines which sector the input angle is
// located in.  Then, the modulation angle is normalized to the current
// 60 degree sector.  Two angles are calculated from the normalized 
// angle.  These two angles determine the times for the T1 and T2
// vectors.  The T1 and T2 vectors are then scaled by the modulation
// amplitude and the duty cycle range.  Finally, the T0 vector time
// is the time left over in the PWM counting period.
// The SVM() function then calculates three duty cycle values based
// on the T0, T1, and T2 times.  The duty cycle calculation depends
// on the 
// appropriate duty cycle values depending on the type of SVM to be
// generated.
//---------------------------------------------------------------------

void SVM(int volts, unsigned int angle)
{
// These variables hold the normalized sector angles used to find
// t1, t2.
unsigned int angle1, angle2;

// These variables hold the space vector times.
unsigned int half_t0,t1,t2,tpwm;

// Calculate the total PWM count period, which is twice the value
// in the PTPER register.
tpwm = PTPER << 1;

// Limit volts input to avoid overmodulation.
if(volts > VOLTS_LIMIT) volts = VOLTS_LIMIT;

if(angle < VECTOR2)
	{
	angle2 = angle - VECTOR1;		// Reference SVM angle to the current 
                                    // sector
	angle1 = SIXTY_DEG - angle2;	// Calculate second angle referenced to 
                                    // sector

	t1 = sinetable[(unsigned char)(angle1 >> 6)];	// Look up values from 
                                                    // table.
	t2 = sinetable[(unsigned char)(angle2 >> 6)];

	// Scale t1 to by the volts variable.
	t1 = ((long)t1*(long)volts) >> 15;
	// Scale t1 for the duty cycle range.
	t1 = ((long)t1*(long)tpwm) >> 15;
	// Scale t2 time
	t2 = ((long)t2*(long)volts) >> 15;
	t2 = ((long)t2*(long)tpwm) >> 15;

	half_t0 = (tpwm - t1 - t2) >> 1;		// Calculate half_t0 null time from 
                                            // period and t1,t2
	
	// Calculate duty cycles for Sector 1  (0 - 59 degrees)
	PDC1 = t1 + t2 + half_t0;
	PDC2 = t2 + half_t0;
	PDC3 = half_t0;
	}

else if(angle < VECTOR3)
	{
	angle2 = angle - VECTOR2;		// Reference SVM angle to the current 
                                    // sector
	angle1 = SIXTY_DEG - angle2;	// Calculate second angle referenced to 
                                    // sector

	t1 = sinetable[(unsigned char)(angle1 >> 6)];	// Look up values from 
                                                    // table.
	t2 = sinetable[(unsigned char)(angle2 >> 6)];

	// Scale t1 to by the volts variable.
	t1 = ((long)t1*(long)volts) >> 15;
	// Scale t1 for the duty cycle range.
	t1 = ((long)t1*(long)tpwm) >> 15;
	// Scale t2 time
	t2 = ((long)t2*(long)volts) >> 15;
	t2 = ((long)t2*(long)tpwm) >> 15;

	half_t0 = (tpwm - t1 - t2) >> 1;		// Calculate half_t0 null time from
                                            // period and t1,t2
	
	// Calculate duty cycles for Sector 2  (60 - 119 degrees)
	PDC1 = t1 + half_t0;
	PDC2 = t1 + t2 + half_t0;
	PDC3 = half_t0;
	}

else if(angle < VECTOR4)
	{
	angle2 = angle - VECTOR3;		// Reference SVM angle to the current 
                                    // sector
	angle1 = SIXTY_DEG - angle2;	// Calculate second angle referenced to 
                                    // sector

	t1 = sinetable[(unsigned char)(angle1 >> 6)];	// Look up values from 
                                                    // table.
	t2 = sinetable[(unsigned char)(angle2 >> 6)];

	// Scale t1 to by the volts variable.
	t1 = ((long)t1*(long)volts) >> 15;
	// Scale t1 for the duty cycle range.
	t1 = ((long)t1*(long)tpwm) >> 15;
	// Scale t2 time
	t2 = ((long)t2*(long)volts) >> 15;
	t2 = ((long)t2*(long)tpwm) >> 15;

	half_t0 = (tpwm - t1 - t2) >> 1;		// Calculate half_t0 null time from
                                            // period and t1,t2
	
	// Calculate duty cycles for Sector 3  (120 - 179 degrees)
	PDC1 = half_t0;
	PDC2 = t1 + t2 + half_t0;
	PDC3 = t2 + half_t0;
	}

else if(angle < VECTOR5)		
	{
	angle2 = angle - VECTOR4;		// Reference SVM angle to the current 
                                    // sector
	angle1 = SIXTY_DEG - angle2;	// Calculate second angle referenced to 
                                    // sector

	t1 = sinetable[(unsigned char)(angle1 >> 6)];	// Look up values from 
                                                    // table.
	t2 = sinetable[(unsigned char)(angle2 >> 6)];

	// Scale t1 to by the volts variable.
	t1 = ((long)t1*(long)volts) >> 15;
	// Scale t1 for the duty cycle range.
	t1 = ((long)t1*(long)tpwm) >> 15;
	// Scale t2 time
	t2 = ((long)t2*(long)volts) >> 15;
	t2 = ((long)t2*(long)tpwm) >> 15;

	half_t0 = (tpwm - t1 - t2) >> 1;		// Calculate half_t0 null time from
                                            // period and t1,t2
	
	// Calculate duty cycles for Sector 4  (180 - 239 degrees)
	PDC1 = half_t0;
	PDC2 = t1 + half_t0;
	PDC3 = t1 + t2 + half_t0;
	}

else if(angle < VECTOR6)
	{
	angle2 = angle - VECTOR5;		// Reference SVM angle to the current
                                    // sector
	angle1 = SIXTY_DEG - angle2;	// Calculate second angle referenced to 
                                    // sector

	t1 = sinetable[(unsigned char)(angle1 >> 6)];	// Look up values from 
                                                    // table.
	t2 = sinetable[(unsigned char)(angle2 >> 6)];

	// Scale t1 to by the volts variable.
	t1 = ((long)t1*(long)volts) >> 15;
	// Scale t1 for the duty cycle range.
	t1 = ((long)t1*(long)tpwm) >> 15;
	// Scale t2 time
	t2 = ((long)t2*(long)volts) >> 15;
	t2 = ((long)t2*(long)tpwm) >> 15;

	half_t0 = (tpwm - t1 - t2) >> 1;		// Calculate half_t0 null time from
                                            // period and t1,t2
	
	// Calculate duty cycles for Sector 5  (240 - 299 degrees)
	PDC1 = t2 + half_t0;
	PDC2 = half_t0;
	PDC3 = t1 + t2 + half_t0;
	}

else
	{
	angle2 = angle - VECTOR6;		// Reference SVM angle to the current 
                                    // sector
	angle1 = SIXTY_DEG - angle2;	// Calculate second angle referenced to 
                                    // sector

	t1 = sinetable[(unsigned char)(angle1 >> 6)];	// Look up values from 
                                                    // table.
	t2 = sinetable[(unsigned char)(angle2 >> 6)];

	// Scale t1 to by the volts variable.
	t1 = ((long)t1*(long)volts) >> 15;
	// Scale t1 for the duty cycle range.
	t1 = ((long)t1*(long)tpwm) >> 15;
	// Scale t2 time
	t2 = ((long)t2*(long)volts) >> 15;
	t2 = ((long)t2*(long)tpwm) >> 15;

	half_t0 = (tpwm - t1 - t2) >> 1;		// Calculate half_t0 null time from
                                            // period and t1,t2
	
	// Calculate duty cycles for Sector 6  ( 300 - 359 degrees )
	PDC1 = t1 + t2 + half_t0;
	PDC2 = half_t0;
	PDC3 = t1 + half_t0;
	}
}			// end SVM()

