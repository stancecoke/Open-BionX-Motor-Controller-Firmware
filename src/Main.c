// Useful advice! http://www.microchip.com/forums/m790211.aspx
// FCY must be defined before the includes or things go wrong.
// For some odd reason, the config tool does not set this based
// on the MCU + Oscillator selected
// by stancecoke


//#define SYS_FREQ        104000000L          //aus dem Blink-Beispiel 13MHz Quarz /2 * 16PLL
//#define FCY             SYS_FREQ/4          //aus dem Blink-Beispiel
// DSPIC30F6015 Configuration Bit Settings

// 'C' source line config statements
// Nachgebaut aus den Werten der originalen Firmware, nur FCKSMEN passt nicht (Origial xb11)
// FOSC

#pragma config FOSFPR = HS2_PLL16       // Oscillator (HS2 w/PLL 16x)
#pragma config FCKSMEN = CSW_FSCM_ON    // Clock Switching and Monitor (Sw Enabled, Mon Enabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64           // POR Timer Value (64ms)
#pragma config BODENV = BORV27          // Brown Out Voltage (2.7V)
#pragma config BOREN = PBOR_ON          // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_LO      // Low-side PWM Output Polarity (Active Low)
#pragma config HPOL = PWMxH_ACT_HI      // High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN       // PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FBS
#pragma config BWRP = WR_PROTECT_BOOT_OFF// Boot Segment Program Memory Write Protect (Boot Segment Program Memory may be written)
#pragma config BSS = NO_BOOT_CODE       // Boot Segment Program Flash Memory Code Protection (No Boot Segment)
#pragma config EBS = NO_BOOT_EEPROM     // Boot Segment Data EEPROM Protection (No Boot EEPROM)
#pragma config RBS = NO_BOOT_RAM        // Boot Segment Data RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WR_PROT_SEC_OFF   // Secure Segment Program Write Protect (Disabled)
#pragma config SSS = NO_SEC_CODE        // Secure Segment Program Flash Memory Code Protection (No Secure Segment)
#pragma config ESS = NO_SEC_EEPROM      // Secure Segment Data EEPROM Protection (No Segment Data EEPROM)
#pragma config RSS = NO_SEC_RAM         // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = GSS_OFF            // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

//*****************************************************************************
// Device Config /end (paste config between these two)
//*****************************************************************************


#include <xc.h>
#include <libpic30.h>
#include <uart.h>
#include <stdio.h>
#include <adc10.h>
#include "svm.h"


//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
/*
 //aus dem BLDC Beispiel, ersetzt durch Pragmas
_FOSC(CSW_FSCM_OFF & XT_PLL16);
_FWDT(WDT_OFF);
_FBORPOR(PBOR_ON & BORV20 & PWRT_64 & MCLR_EN);
*/

// 	Hurst Motor Terminals | MC LV PICDEM Board Connection
// -----------------------|---------------------------------
//	Ground Phase ---------|-- G
//	Phase Red    ---------|-- M1
//	Phase Black  ---------|-- M2
//	Phase White  ---------|-- M3
//	Hall White   ---------|-- HA
//	Hall Brown   ---------|-- HB
//	Hall Green   ---------|-- HC

typedef signed int SFRAC16;

#define CLOSED_LOOP      // if defined the speed controller will be enabled
#define PHASE_ADVANCE    // for extended speed ranges this should be defined

#define FCY  26000000	 // xtal = 13Mhz/2; PLLx16 /4 -> 26 MIPS, Schon oben aus dem Blink-Beispiel definiert
#define FPWM 20000		 // 20 kHz, so that no audible noise is present.
#define _10MILLISEC	 10  // Used as a timeout with no hall effect sensors
                         // transitions and Forcing steps according to the
                         // actual position of the motor
#define _100MILLISEC 100 // after this time has elapsed, the motor is
                         // consider stalled and it's stopped
#define _1000MILLISEC 1000

// These Phase values represent the base Phase value of the sinewave for each
// one of the sectors (each sector is a translation of the hall effect sensors
// reading 
#define PHASE_ZERO 	57344
#define PHASE_ONE	((PHASE_ZERO + 65536/6) % 65536)
#define PHASE_TWO	((PHASE_ONE + 65536/6) % 65536)
#define PHASE_THREE	((PHASE_TWO + 65536/6) % 65536)
#define PHASE_FOUR	((PHASE_THREE + 65536/6) % 65536)
#define PHASE_FIVE	((PHASE_FOUR + 65536/6) % 65536)

#define MAX_PH_ADV_DEG 40  // This value represents the maximum allowed phase
                           // advance in electrical degrees. Set a value from
                           // 0 to 60. This value will be used to calculate
                           // phase advance only if PHASE_ADVANCE is defined

// This is the calculation from the required phase advance to the actual
// value to be multiplied by the speed of the motor. So, if PHASE_ADVANCE is
// enabled, a certain amount of shit angle will be added to the generated
// sine wave, up to a maximum of the specified value on MAX_PH_ADV_DEG. This
// maximum phase shift will be present when the MeasuredSpeed variable is a 
// fractional 1.0 (for CW) or -1.0 (for CCW).
#define MAX_PH_ADV 		(int)(((float)MAX_PH_ADV_DEG / 360.0) * 65536.0)

#define HALLA	1	// Connected to RB3
#define HALLB	2	// Connected to RB4
#define HALLC	4	// Connected to RB5
#define CW	0		// Counter Clock Wise direction
#define CCW	1		// Clock Wise direction
#define SWITCH_S2	(!PORTDbits.RD8) // Push button S2 "dir" connector on BionX PCB

// Period Calculation
// Period = (TMRClock * 60) / (RPM * Motor_Poles)
// For example>
// Motor_Poles = 10
// RPM = 6000 (Max Speed)
// Period = ((20,000,000 / 64) * 60) / (6000 * 10) = 312.5
// RPM = 60 (Min Speed)
// Period = ((20,000,000 / 64) * 60) / (60 * 10) = 31250

#define MINPERIOD	313		// For 6000 max rpm and 10 poles motor
#define MAXPERIOD	31250	// For 60 min rpm and 10 poles motor

// Use this MACRO when using floats to initialize signed 16-bit fractional 
// variables
#define SFloat_To_SFrac16(Float_Value)	\
        ((Float_Value < 0.0) ? (SFRAC16)(32768 * (Float_Value) - 0.5) \
        : (SFRAC16)(32767 * (Float_Value) + 0.5))

void InitADC10(void);	// Initialization of ADC used for Speed Command
void InitMCPWM(void);	// Initialization for PWM at 20kHz, Center aligned, 
                        // Complementary mode with 1 us of deadtime
void InitTMR1(void);	// Initialization for TIMER1 used for speed control 
                        // and motor stalled protection
void InitTMR3(void);	// Initialization for TIMER3 used as a timebase 
                        // for the two input capture channels
void InitUserInt(void);	// This function initializes all ports 
                        // (inputs and outputs) for the application
void InitICandCN(void);	// Initializes input captures and change notification, 
                        // used for the hall sensor inputs
void RunMotor(void);	// This function initializes all variables 
                        // and interrupts used for starting and running 
                        // the motor
void StopMotor(void);	// This function clears all flags, and stops anything
                        // related to motor control, and also disables PWMs
void SpeedControl(void);     // This function contains all ASM and C operations
                             // for doing the PID Control loop for the speed
void ForceCommutation(void); // When motor is to slow to generate interrupts 
                             // on halls, this function forces a commutation
void ChargeBootstraps(void); // At the begining of the motor operation, the 
                             // bootstrap caps are charged with this function

// Constants used for properly energizing the motor depending on the 
// rotor's position
int PhaseValues[6] = {PHASE_ZERO, PHASE_ONE, PHASE_TWO, PHASE_THREE, PHASE_FOUR, PHASE_FIVE}; 

// In the sinewave generation algorithm we need an offset to be added to the
// pointer when energizing the motor in CCW. This is done to compensate an
// asymetry of the sinewave
int PhaseOffset = 4100;
char Buf[80];

int i=0, j=0, ADCValues[3];
int __C30_UART = 2; //leitet Printbefehl auf UART2 um.
/* This is UART2 transmit ISR */

// Flags used for the application
struct 
{
	unsigned MotorRunning	:1;  // This bit is 1 if motor running
	unsigned unused			:15;
}Flags;
 
unsigned int Phase;	// This variable is incremented by the PWM interrupt
                    // in order to generate a proper sinewave. Its value
                    // is incremented by a value of PhaseInc, which
                    // represents the frequency of the generated sinewave
signed int PhaseInc; // Delta increments of the Phase variable, calculated 
                     // in the TIMER1 interrupt (each 1 ms) and used in 
                     // the PWM interrupt (each 50 us)
signed int PhaseAdvance; // Used for extending motor speed range. This value
                         // is added directly to the parameters passed to the
                         // SVM function (the sine wave generation subroutine)
unsigned int HallValue;	 // This variable holds the hall sensor input readings
unsigned int Sector;  // This variables holds present sector value, which is 
                      // the rotor position
unsigned int LastSector; // This variable holds the last sector value. This 
                         // is critical to filter slow slew rate on the Hall
                         // effect sensors hardware
unsigned int MotorStalledCounter = 0; // This variable gets incremented each 
                                      // 1 ms, and is cleared everytime a new
                                      // sector is detected. Used for 
                                      // ForceCommutation and MotorStalled 
                                      // protection functions

// This array translates the hall state value read from the digital I/O to the
// proper sector.  Hall values of 0 or 7 represent illegal values and therefore
// return -1.
char SectorTable[] = {-1,4,2,3,0,5,1,-1};

unsigned char Current_Direction;	// Current mechanical motor direction of 
                                    // rotation Calculated in halls interrupts
unsigned char Required_Direction;	// Required mechanical motor direction of 
                                    // rotation, will have the same sign as the
									// ControlOutput variable from the Speed 
                                    // Controller

// Variables containing the Period of half an electrical cycle, which is an 
// interrupt each edge of one of the hall sensor input
unsigned int PastCapture, ActualCapture, Period; 
// Used as a temporal variable to perform a fractional divide operation in 
// assembly
SFRAC16 _MINPERIOD = MINPERIOD - 1;

SFRAC16 MeasuredSpeed, RefSpeed;	// Actual and Desired speeds for the PID 
                            // controller, that will generate the error
SFRAC16 ControlOutput = 0;	// Controller output, used as a voltage output, 
                            // use its sign for the required direction

// Absolute PID gains used by the controller. Position form implementation of 
// a digital PID. See SpeedControl subroutine for details
SFRAC16 Kp = SFloat_To_SFrac16(0.1);   // P Gain
SFRAC16 Ki = SFloat_To_SFrac16(0.01);  // I Gain
SFRAC16 Kd = SFloat_To_SFrac16(0.000); // D Gain

// Constants used by the PID controller, since a MAC operation is used, the 
// PID structure is changed (See SpeedControl() Comments)
SFRAC16 ControlDifference[3] \
        __attribute__((__space__(xmemory), __aligned__(4)));
SFRAC16 PIDCoefficients[3]   \
        __attribute__((__space__(ymemory), __aligned__(4)));

// Used as a temporal variable to perform a fractional divide operation in 
// assembly
SFRAC16 _MAX_PH_ADV = MAX_PH_ADV;

/*********************************************************************
  Function:        void __attribute__((__interrupt__)) _T1Interrupt (void)

  PreCondition:    The motor is running and is generating hall effect sensors
                   interrupts. Also, the actual direction of the motor used
                   in this interrupt is assumed to be previously calculated.
 
  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        In this ISR the Period, Phase Increment and MeasuredSpeed
                   are calculated based on the input capture of one of the
                   halls. The speed controller is also called in this ISR
                   to generate a new output voltage (ControlOutput). The 
                   Phase Advance is calculated based on the maximum allowed
                   phase advance (MAX_PH_ADV) and the actual speed of the 
                   motor. The last thing done in this ISR is the forced 
                   commutation, which happens each time the motor doesn't
                   generate a new hall interrupt after a programmed period 
                   of time. If the timeout for generating hall ISR is too much
                   (i.e. 100 ms) the motor is then stopped.

  Note:            The MeasuredSpeed Calculation is made in assembly to take 
                   advantage of the signed fractional division.
********************************************************************/

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt (void)
{
	IFS0bits.T1IF = 0;
    j++;
    
	Period = ActualCapture - PastCapture;  // This is an UNsigned substraction
                                           // to get the Period between one 
                                           // hall effect sensor transition

    // These operations limit the Period value to a range from 60 to 6000 rpm
	if (Period < (unsigned int)MINPERIOD)  // MINPERIOD or 6000 rpm
		Period = MINPERIOD;
	else if (Period > (unsigned int)MAXPERIOD) // MAXPERIOD or 60 rpm
		Period = MAXPERIOD;

    // PhaseInc is a value added to the Phase variable to generate the sine
    // voltages. 1 electrical degree corresponds to a PhaseInc value of 184,
    // since the pointer to the sine table is a 16bit value, where 360 Elec
    // Degrees represents 65535 in the pointer. 
    // __builtin_divud(Long Value, Int Value) is a function of the compiler
    // to do Long over Integer divisions.
	PhaseInc = __builtin_divud(512000UL, Period);	// Phase increment is used
								 					// by the PWM isr (SVM)

    // This subroutine in assembly calculates the MeasuredSpeed using 
    // fractional division. These operations in assembly perform the following
    // formula:
    //                   MINPERIOD (in fractional) 
    //  MeasuredSpeed = ---------------------------
    //                    Period (in fractional)
    //
    { int divr;
	__asm__ volatile("repeat #17\n\t"
                         "divf %1,%2\n\t"
                          :  /* output */ "=a"(divr)  
                                     :  /* input */ "r"(_MINPERIOD),
                                                    "e"(Period)
                                     );
       MeasuredSpeed = divr;
    } 

    // MeasuredSpeed sign adjustment based on current motor direction of 
    // rotation
	if (Current_Direction == CCW)
		MeasuredSpeed = -MeasuredSpeed;

    // The following values represent the MeasuredSpeed values from the 
    // previous operations:
    //
	// CONDITION        RPM          SFRAC16      SINT      HEX
	// Max Speed CW  -> 6000 RPM  -> 0.996805  -> 32663  -> 0x7F97
	// Min Speed CW  -> 60 RPM    -> 0.009984  -> 327    -> 0x0147
	// Min Speed CCW -> -60 RPM   -> -0.009984 -> -327   -> 0xFEB9
	// Max Speed CCW -> -6000 RPM -> -0.996805 -> -32663 -> 0x8069

	SpeedControl(); // Speed PID controller is called here. It will use 
                    // MeasuredSpeed, RefSpeed, some buffers and will generate
                    // the new ControlOutput, which represents a new amplitude
                    // of the sinewave that will be generated by the SVM 
                    // subroutine.

#ifdef PHASE_ADVANCE
	// Calculate Phase Advance Based on Actual Speed and MAX_PH_ADV define
        // The following assembly instruction perform the following formula
        // using fractional multiplication:
        // 
        // PhaseAdvance = MAX_PH_ADV * MeasuredSpeed
        //

#if !defined(__C30_VERSION__) || (__C30_VERSION__ < 200) || (__C30_VERSION__ == 300) || defined(TEST_ASM)
        {  register int wreg4 asm("w4") = _MAX_PH_ADV;
           register int wreg5 asm("w5") = MeasuredSpeed;

           asm volatile("mpy %0*%1, A" : /* no outputs */ 
                                       : "r"(wreg4), "r"(wreg5));
           asm volatile("sac A, %0" : "=r"(PhaseAdvance));
        }
#else
        {  register int a_reg asm("A");

           a_reg = __builtin_mpy(_MAX_PH_ADV, MeasuredSpeed, 0,0,0,0,0,0);
           PhaseAdvance = __builtin_sac(a_reg, 0);

        }
#endif
           
#endif

	MotorStalledCounter++;	// We increment a timeout variable to see if the
                            // motor is too slow (not generating hall effect
                            // sensors interrupts frequently enough) or if
                            // the motor is stalled. This variable is cleared
                            // in halls ISRs
    if ((MotorStalledCounter % _10MILLISEC) == 0)
	{
		ForceCommutation();	// Force Commutation if no hall sensor changes 
                            // have occured in specified timeout.
	}
	else if (MotorStalledCounter >= _1000MILLISEC)
	{
		StopMotor(); // Stop motor is no hall changes have occured in 
                     // specified timeout
	}
	return;
}

/*********************************************************************
  Function:        void __attribute__((__interrupt__)) _CNInterrupt (void)

  PreCondition:    The inputs of the hall effect sensors should have low pass
                   filters. A simple RC network works.
 
  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        This interrupt represent Hall A ISR. Hall A -> RB3 -> CN5.
                   This is generated by the input change notification CN5.
                   The purpose of this ISR is to Calculate the actual 
                   mechanical direction of rotation of the motor, and to adjust
                   the Phase variable depending on the sector the rotor is in.

  Note 1:          The sector is validated in order to avoid any spurious
                   interrupt due to a slow slew rate on the halls inputs due to
                   hardware filtering.

  Note 2:          For Phase adjustment in CCW, an offset is added to 
                   compensate non-symetries in the sine table used.
********************************************************************/

// Hall 1 und 2 beide auf Change Notification Pins, daher dritte Interruptroutine nicht benötigt, aber Richtungserkennung muß aufgebohrt werden

void __attribute__((interrupt, no_auto_psv)) _CNInterrupt (void)
{
	IFS0bits.CNIF = 0;	// Clear interrupt flag
	HallValue = (unsigned int)((PORTD >> 5) & 0x0007);	// Read halls from RD5 to RD7
	Sector = SectorTable[HallValue];	// Get Sector from table, SectorTable[] = {-1,4,2,3,0,5,1,-1};

    // This MUST be done for getting around the HW slow rate
	if (Sector != LastSector)	
	{
		// Since a new sector is detected, clear variable that would stop 
        // the motor if stalled.
		MotorStalledCounter = 0;

		// Motor current direction is computed based on Sector
		if ((LastSector == 4 && Sector==5) 
                || (LastSector == 5 && Sector==0)
                || (LastSector == 1 && Sector==2)
                || (LastSector == 2 && Sector==3))	
			Current_Direction = CW;
		else
			Current_Direction = CCW;

        // Motor commutation is actually based on the required direction, not
        // the current dir. This allows driving the motor in four quadrants
		if (Required_Direction == CW)	
		{
			Phase = PhaseValues[Sector];
		}
		else
		{
			// For CCW an offset must be added to compensate difference in 
            // symmetry of the sine table used for CW and CCW
			Phase = PhaseValues[(Sector + 3) % 6] + PhaseOffset;
		}		
        
        //printf("CNInterrupt, %d, %d, %d, %d\r\n",(unsigned int)((PORTD >> 5) & 0x0007), LastSector, Sector, RefSpeed);
        LastSector = Sector; // Update last sector
        //Hier erst mal Printfunktion einbauen um richtige Übergänge für Richtungserkennung zu testen.
	}

	return;
}

/*********************************************************************
  Function:        void __attribute__((__interrupt__)) _IC7Interrupt (void)

  PreCondition:    The inputs of the hall effect sensors should have low pass
                   filters. A simple RC network works.
 
  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        This interrupt represent Hall B ISR. Hall B -> RB4 -> IC7.
                   This is generated by the input Capture Channel IC7.
                   The purpose of this ISR is to Calculate the actual Period
                   between hall effect sensor transitions, calculate the actual
                   mechanical direction of rotation of the motor, and also to
                   adjust the Phase variable depending on the sector the rotor
                   is in.

  Note 1:          The sector is validated in order to avoid any spurious
                   interrupt due to a slow slew rate on the halls inputs due to
                   hardware filtering.

  Note 2:          For Phase adjustment in CCW, an offset is added to 
                   compensate non-symetries in the sine table used.
********************************************************************/
// Hall 3 auf Pin RD5 = IC6 Input Capture 6, muß noch rausfinden, zu welchem Timer der gehört...
void __attribute__((interrupt, no_auto_psv)) _IC6Interrupt (void)
{
	IFS1bits.IC6IF = 0;	// Cleat interrupt flag
	HallValue = (unsigned int)((PORTD >> 5) & 0x0007);	// Read halls
	Sector = SectorTable[HallValue];	// Get Sector from table

    // This MUST be done for getting around the HW slow rate
	if (Sector != LastSector)
	{
		// Calculate Hall period corresponding to half an electrical cycle
		PastCapture = ActualCapture;
		ActualCapture = IC6BUF; //4 mal lesen wie im Datenblatt beschrieben.
		IC6BUF;
		IC6BUF;
		IC6BUF;

		// Since a new sector is detected, clear variable that would stop 
        // the motor if stalled.
		MotorStalledCounter = 0;

		// Motor current direction is computed based on Sector
		if ((LastSector == 3 && Sector==4) 
                || (LastSector == 0 && Sector==1))
			Current_Direction = CW;
		else
			Current_Direction = CCW;

        // Motor commutation is actually based on the required direction, not
        // the current dir. This allows driving the motor in four quadrants
		if (Required_Direction == CW)
		{
			Phase = PhaseValues[Sector];
		}
		else
		{
			// For CCW an offset must be added to compensate difference in 
            // symmetry of the sine table used for CW and CCW
			Phase = PhaseValues[(Sector + 3) % 6] + PhaseOffset;
		}
		
        //printf("IC6Interrupt,  %d, %d, %d, %d\r\n",(unsigned int)((PORTD >> 5) & 0x0007), LastSector, Sector, RefSpeed);
        LastSector = Sector; // Update last sector
        //hier auch erst mal ausdrucken für richtige Sektoren.
	}

	return;
}

/*********************************************************************
  Function:        void __attribute__((__interrupt__)) _IC8Interrupt (void)

  PreCondition:    The inputs of the hall effect sensors should have low pass
                   filters. A simple RC network works.
 
  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        This interrupt represent Hall C ISR. Hall C -> RB5 -> IC8.
                   This is generated by the input Capture Channel IC8.
                   The purpose of this ISR is to Calculate the actual 
                   mechanical direction of rotation of the motor, and to adjust
                   the Phase variable depending on the sector the rotor is in.

  Note 1:          The sector is validated in order to avoid any spurious
                   interrupt due to a slow slew rate on the halls inputs due to
                   hardware filtering.

  Note 2:          For Phase adjustment in CCW, an offset is added to 
                   compensate non-symetries in the sine table used.
********************************************************************/
/* wird nicht benötigt, da Hallsensor auf anderem Pin
void __attribute__((interrupt, no_auto_psv)) _IC8Interrupt (void)
{	
	IFS1bits.IC8IF = 0;	// Cleat interrupt flag
	HallValue = (unsigned int)((PORTB >> 3) & 0x0007);	// Read halls
	Sector = SectorTable[HallValue];	// Get Sector from table

    // This MUST be done for getting around the HW slow rate
	if (Sector != LastSector)
	{
		// Since a new sector is detected, clear variable that would stop 
        // the motor if stalled.
		MotorStalledCounter = 0;

		// Motor current direction is computed based on Sector
		if ((Sector == 1) || (Sector == 4))
			Current_Direction = CCW;
		else
			Current_Direction = CW;
		
        // Motor commutation is actually based on the required direction, not
        // the current dir. This allows driving the motor in four quadrants
		if (Required_Direction == CW)
		{
			Phase = PhaseValues[Sector];
		}
		else
		{
			// For CCW an offset must be added to compensate difference in 
            // symmetry of the sine table used for CW and CCW
			Phase = PhaseValues[(Sector + 3) % 6] + PhaseOffset;
		}
		LastSector = Sector; // Update last sector
	}

	return;
}
*/
/*********************************************************************
  Function:        void __attribute__((__interrupt__)) _PWMInterrupt (void)

  PreCondition:    None.
 
  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        in this ISR the sinewave is generated. If the current motor
                   direction of rotation is different from the required 
                   direction then the motor is operated	in braking mode and 
                   step commutation is performed. Once both directions are 
                   equal then the sinewave is fed into the motor windings.
                   If PHASE_ADVANCE is defined, a value corresponding to the
                   multiplication of the actual speed * maximum phase advance
                   is added to the sine wave phase to produce the phase shift

  Note:            None.
********************************************************************/

void __attribute__((interrupt, no_auto_psv)) _PWMInterrupt (void)
{
	IFS2bits.PWMIF = 0;	// Clear interrupt flag

	if (Required_Direction == CW)
	{
		if (Current_Direction == CW)
			Phase += PhaseInc;    // Increment Phase if CW to generate the 
                                  // sinewave only if both directions are equal
		// If Required_Direction is CW (forward) POSITIVE voltage is applied
		#ifdef PHASE_ADVANCE
		SVM(ControlOutput, Phase + PhaseAdvance);	// PhaseAdvance addition
													// produces the sinewave
													// phase shift
		#else
		SVM(ControlOutput, Phase);
		#endif
	}
	else
	{
		if (Current_Direction == CCW)	
			Phase -= PhaseInc;      // Decrement Phase if CCW to generate
									// the sinewave only if both 
									// directions are equal
		// If Required_Direction is CCW (reverse) NEGATIVE voltage is applied
		#ifdef PHASE_ADVANCE
		SVM(-(ControlOutput+1), Phase + PhaseAdvance);// PhaseAdvance addition
													  // produces the sinewave
													  // phase shift
		#else
		SVM(-(ControlOutput+1), Phase);
		#endif
	}
	return;
}

/*********************************************************************
  Function:        void __attribute__((__interrupt__)) _ADCInterrupt (void)

  PreCondition:    None.
 
  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        The ADC interrupt loads the reference speed (RefSpeed) with
                   the respective value of the POT. The value will be a signed
                   fractional value, so it doesn't need any scaling.

  Note:            None.
********************************************************************/

void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt (void)
{
	IFS0bits.ADIF = 0;	// Clear interrupt flag
	RefSpeed = ADCBUF0; // Read POT value to set Reference Speed
	return;
}

//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX


void __attribute__((__interrupt__, no_auto_psv)) _U2TXInterrupt(void)

{
 IFS1bits.U2TXIF = 0;
}


// This is UART2 receive ISR done
void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void)
{
 IFS1bits.U2RXIF = 0;
// Read the receive buffer till atleast one or more character can be read 
 while( DataRdyUART2())
 {
  
 Buf[i] = ReadUART2();
 i++;
 if(i>20){
     i=0;
     //printf("Buffer full\r\n");
  }
} 
}
/*wird nicht mehr benötigt, da ADC über Timer getriggert.
unsigned int read_analog_channel(int channel)
{
    ADCHS = channel;          // Select the requested channel
    ADCON1bits.SAMP = 1;      // start sampling
    __delay32(30);            // 1us delay @ 30 MIPS
    ADCON1bits.SAMP = 0;      // start Converting
    while (!ADCON1bits.DONE); // Should take 12 * Tad = 1.2us
    return ADCBUF0;
}*/
int main(void) {
    
  /* _TRISD8 = 0; // set D3 to output alter Teil aus Blink und UART-Beispiel
    
    _LATD8 = 1;
    // Make RD0-3 digital outputs
    TRISD = 0b0000;
 
    // Configure analog inputs
    TRISB = 0x01FF;      // Port B all inputs
    ADPCFG = ENABLE_AN3_ANA&ENABLE_AN4_ANA&ENABLE_AN5_ANA;     // Lowest 8 PORTB pins are analog inputs
    ADCON1 = 0;          // Manually clear SAMP to end sampling, start conversion
    ADCON2 = 0;          // Voltage reference from AVDD and AVSS
    ADCON3 = 0x0005;     // Manual Sample, ADCS=5 -> Tad = 3*Tcy = 0.1us
    ADCON1bits.ADON = 1; // Turn ADC ON
    */
    /* Data to be transmitted using UART communication module */
char Txdata[30];
/* Holds the value of baud register */
unsigned int baudvalue;
/* Holds the value of uart config reg */
unsigned int U2MODEvalue;
/* Holds the information regarding uart
TX & RX interrupt modes */
unsigned int U2STAvalue;
/* Turn off UART1module */
 CloseUART2();
/* Configure uart1 receive and transmit interrupt */
 ConfigIntUART2(UART_RX_INT_EN & UART_RX_INT_PR6 &
 UART_TX_INT_DIS & UART_TX_INT_PR2);
/* Configure UART1 module to transmit 8 bit data with one stopbit.
Also Enable loopback mode */
 baudvalue = 10; //(FCY/(16*Baudrate))-1
 U2MODEvalue = UART_EN & UART_IDLE_CON &
 UART_DIS_WAKE & UART_EN_LOOPBACK &
 UART_EN_ABAUD & UART_NO_PAR_8BIT &
 UART_1STOPBIT;
 U2STAvalue = UART_INT_TX_BUF_EMPTY &
 UART_TX_PIN_NORMAL &
 UART_TX_ENABLE & UART_INT_RX_3_4_FUL &
 UART_ADR_DETECT_DIS &
 UART_RX_OVERRUN_CLEAR;
 OpenUART2(U2MODEvalue, U2STAvalue, baudvalue);
    //welcome message
    sprintf(Txdata, "BionX OSF v0.1\r\n");
    putsUART2 ((unsigned int *)Txdata);
    /* Wait for transmission to complete */
    while(BusyUART2());

//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
    InitUserInt();	// Initialize User Interface I/Os
	InitADC10();	// Initialize ADC to be signed fractional
	InitTMR1();		// Initialize TMR1 for 1 ms periodic ISR
	InitTMR3();		// Initialize TMR3 for timebase of capture
	InitICandCN();	// Initialize Hall sensor inputs ISRs	
	InitMCPWM();	// Initialize PWM @ 20 kHz, center aligned, 1 us of 
                    // deadtime
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX       
     

/* Read all the data remaining in receive buffer which are unread 
 while(DataRdyUART1())
 {
 (*( Receivedddata)++) = ReadUART1() ;
 }*/

 

    while(1)
    {   
        if ((!Flags.MotorRunning))RunMotor();
 //XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
	/*if ((SWITCH_S2) && (!Flags.MotorRunning))
		{
			while(SWITCH_S2);
			RunMotor();	// Run motor if push button is pressed and motor is
                        // stopped
		}
		else if ((SWITCH_S2) && (Flags.MotorRunning))
		{
			while(SWITCH_S2);
			StopMotor();// Stop motor if push button is pressed and motor is 
                        // running
		}*/
        
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX       
       /*    alter Teil aus Blink und UART Beispiel
        _LATD8 = 0;
    
       // printf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d\r\n", PORTD&1,PORTD&(1<<10),PORTD&(1<<11),PORTD&(1<<3),PORTD&(1<<4),PORTD&(1<<5),PORTD&(1<<6),PORTD&(1<<7),PORTD&(1<<8),PORTD&(1<<9));
       for(j=0; j <= 3; j++){
        __delay_ms(300);
        ADCValues[j]=read_analog_channel(j+3);
        printf("%d, %d, %d\r\n",ADCValues[0],ADCValues[1],ADCValues[2]);
       }
        _LATD8 = 1;
        __delay_ms(300);
                
        ClrWdt();
        */
    
    if(j>50){
        j=0;
        printf("Mainloop, %d, %d, %d, %d, %d\r\n",(unsigned int)((PORTD >> 5) & 0x0007), LastSector, Sector, RefSpeed, Period );
    }
    }
    
 /* Turn off UART module */
    CloseUART2();
    return 0;
} //end of main loop

/*********************************************************************
  Function:        void ChargeBootstraps(void)

  PreCondition:    None.
 
  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        In the topology used, it is necessary to charge the 
                   bootstrap caps each time the motor is energized for the 
                   first time after an undetermined amount of time. 
                   ChargeBootstraps subroutine turns ON	the lower transistors
                   for 10 ms to ensure voltage on these caps, and then it 
                   transfers the control of the outputs to the PWM module.

  Note:            None.
********************************************************************/

void ChargeBootstraps(void)
{
	/*wird nicht benötigt, da Halfbridge Driver S2003 vorhanden.
     * unsigned int i;
	OVDCON = 0x0015;	// Turn ON low side transistors to charge
	for (i = 0; i < 33330; i++) // 10 ms Delay at 20 MIPs
		;
	PWMCON2bits.UDIS = 1;
	PDC1 = PTPER;	// Initialize as 0 voltage
	PDC2 = PTPER;	// Initialize as 0 voltage
	PDC3 = PTPER;	// Initialize as 0 voltage
	OVDCON = 0x3F00;	// Configure PWM0-5 to be governed by PWM module
	PWMCON2bits.UDIS = 0;
	return;
    */
}

/*********************************************************************
  Function:        void RunMotor(void)

  PreCondition:    None.
 
  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        Call this subroutine when first trying to run the motor and
                   the motor is previously stopped. RunMotor will charge 
                   bootstrap caps, will initialize application variables, and
                   will enable all ISRs.

  Note:            None.
********************************************************************/

void RunMotor(void)
{
	ChargeBootstraps();
	// init variables
	ControlDifference[0] = 0;	// Error at K	(most recent)
	ControlDifference[1] = 0;	// Error at K-1
	ControlDifference[2] = 0;	// Error at K-2	(least recent)
	PIDCoefficients[0] = Kp + Ki + Kd;	// Modified coefficient for using MACs
	PIDCoefficients[1] = -(Ki + 2*Kd);	// Modified coefficient for using MACs
	PIDCoefficients[2] = Kd;			// Modified coefficient for using MACs
	
	TMR1 = 0;			// Reset timer 1 for speed control
	TMR3 = 0;			// Reset timer 3 for speed measurement
	ActualCapture = MAXPERIOD; 	// Initialize captures for minimum speed 
								//(60 RPMs)
	PastCapture = 0;

	// Initialize direction with required direction
	// Remember that ADC is not stopped.
	HallValue = (unsigned int)((PORTD >> 5) & 0x0007);	// Read halls
	LastSector = Sector = SectorTable[HallValue];	// Initialize Sector 
                                                    // variable

	// RefSpeed's sign will determine if the motor should be run at CW 
    // (+RefSpeed) or CCW (-RefSpeed) ONLY at start up, since when the motor 
    // has started, the required direction will be set by the control output
    // variable to be able to operate in the four quadrants
	if (RefSpeed < 0)
	{
		ControlOutput = 0;	// Initial output voltage
		Current_Direction = Required_Direction = CCW;
		Phase = PhaseValues[(Sector + 3) % 6] + PhaseOffset;
	}
	else
	{
		ControlOutput = 0;	// Initial output voltage
		Current_Direction = Required_Direction = CW;
		Phase = PhaseValues[Sector];
	}

	MotorStalledCounter = 0;	// Reset motor stalled protection counter
	// Set initial Phase increment with minimum value. This will change if a 
    // costing operation is required by the application
	PhaseInc = __builtin_divud(512000UL, MAXPERIOD);	

	// Clear all interrupts flags
	IFS0bits.T1IF = 0;	// Clear timer 1 flag
	IFS0bits.CNIF = 0;	// Clear interrupt flag
	IFS1bits.IC7IF = 0;	// Clear interrupt flag
	IFS1bits.IC8IF = 0;	// Clear interrupt flag
	IFS2bits.PWMIF = 0;	// Clear interrupt flag

	// enable all interrupts
	__asm__ volatile ("DISI #0x3FFF");
	IEC0bits.T1IE = 1;	// Enable interrupts for timer 1
	IEC0bits.CNIE = 1;	// Enable interrupts on CN5 Braucht es hier für CN15 CN16 was anderes?!
	IEC1bits.IC6IE = 1;	// Enable interrupts on IC6 
	//IEC1bits.IC8IE = 1;	// Enable interrupts on IC8 wird nicht gebraucht, da Hall nicht an IC8
	IEC2bits.PWMIE = 1;	// Enable PWM interrupts
        DISICNT = 0;

	Flags.MotorRunning = 1;	// Indicate that the motor is running
	return;
}

/*********************************************************************
	
*********************************************************************/

/*********************************************************************
  Function:        void StopMotor(void)

  PreCondition:    None.
 
  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        Call this subroutine whenever the user want to stop the 
                   motor. This subroutine will clear interrupts properly, and
                   will also turn OFF all PWM channels.

  Note:            None.
********************************************************************/

void StopMotor(void)
{
	OVDCON = 0x0000;	// turn OFF every transistor
	
	// disable all interrupts
	__asm__ volatile ("DISI #0x3FFF");
	IEC0bits.T1IE = 0;	// Disable interrupts for timer 1
	IEC0bits.CNIE = 0;	// Disable interrupts on CN5
	IEC1bits.IC6IE = 0;	// Disable interrupts on IC7
	//IEC1bits.IC8IE = 0;	// Disable interrupts on IC8
	IEC2bits.PWMIE = 0;	// Disable PWM interrupts
        DISICNT = 0;

	Flags.MotorRunning = 0;	// Indicate that the motor has been stopped
	return;
}

/*********************************************************************
  Function:        void SpeedControl(void)

  PreCondition:    None.
 
  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        This subroutine implements a PID in assembly using the MAC
                   instruction of the dsPIC.

  Note:            None.
********************************************************************/

/*

                                             ----   Proportional
                                            |    |  Output
                             ---------------| Kp |-----------------
                            |               |    |                 |
                            |                ----                  |
Reference                   |                                     --- 
Speed         ---           |           --------------  Integral | + | Control   -------
     --------| + |  Error   |          |      Ki      | Output   |   | Output   |       |
             |   |----------|----------| ------------ |----------|+  |----------| Plant |--
        -----| - |          |          |  1 - Z^(-1)  |          |   |          |       |  |
       |      ---           |           --------------           | + |           -------   |
       |                    |                                     ---                      |
       | Measured           |         -------------------  Deriv   |                       |
       | Speed              |        |                   | Output  |                       |
       |                     --------| Kd * (1 - Z^(-1)) |---------                        |
       |                             |                   |                                 |
       |                              -------------------                                  |
       |                                                                                   |
       |                                                                                   |
        -----------------------------------------------------------------------------------

   ControlOutput(K) = ControlOutput(K-1) 
                    + ControlDifference(K) * (Kp + Ki + Kd)
                    + ControlDifference(K-1) * (-Ki - 2*Kd)
                    + ControlDifference(K-2) * Kd

   Using PIDCoefficients:
   PIDCoefficients[0] = Kp + Ki + Kd
   PIDCoefficients[1] = -(Ki + 2*Kd)
   PIDCoefficients[2] = Kd
   and leting:
   ControlOutput -> ControlOutput(K) and ControlOutput(K-1)
   ControlDifference[0] -> ControlDifference(K)
   ControlDifference[1] -> ControlDifference(K-1)
   ControlDifference[2] -> ControlDifference(K-2)

   ControlOutput = ControlOutput
                 + ControlDifference[0] * PIDCoefficients[0]
                 + ControlDifference[1] * PIDCoefficients[1]
                 + ControlDifference[2] * PIDCoefficients[2]

   This was implemented using Assembly with signed fractional and saturation enabled
   with MAC instruction
*/
//#define __C30_VERSION__ 100
#if !defined(__C30_VERSION__) || (__C30_VERSION__ < 200)
void SpeedControl(void)
{ 	register SFRAC16 *ControlDifferencePtr asm("w8") = ControlDifference;
	register SFRAC16 *PIDCoefficientsPtr asm("w10") = PIDCoefficients;
	register SFRAC16 x_prefetch asm("w4");
	register SFRAC16 y_prefetch asm("w5");

	CORCONbits.SATA = 1; 	// Enable Saturation on Acc A
	// Calculate most recent error with saturation, no limit checking required
	__asm__ volatile ("LAC %0, A" : /* no outputs */ : "r"(RefSpeed));
	__asm__ volatile ("LAC %0, B" : /* no outputs */ : "r"(MeasuredSpeed));
	__asm__ volatile ("SUB A");
	__asm__ volatile ("SAC A, [%0]" : /* no outputs */ : 
                                          "r"(ControlDifferencePtr));
	// Prepare MAC Operands
	__asm__ volatile ("MOVSAC A, [%0]+=2, %2, [%1]+=2, %3" :
                                /* outputs */ "+r"(ControlDifferencePtr),
                                              "+r"(PIDCoefficientsPtr),
                                              "=r"(x_prefetch),
                                              "=r"(y_prefetch));
	__asm__ volatile ("LAC %0, A" : /* no outpus */ : "r"(ControlOutput));			// Load Acc with last output
	// Perform MAC
	__asm__ volatile ("REPEAT #2\n\t"
	                  "MAC %2*%3, A, [%0]+=2, %2, [%1]+=2, %3" :
                                /* outputs */ "+r"(ControlDifferencePtr),
                                              "+r"(PIDCoefficientsPtr),
                                              "+r"(x_prefetch),
                                              "+r"(y_prefetch));
	// Store result in ControlOutput with saturation
	__asm__ volatile ("SAC A, %0" : "=r"(ControlOutput));
	CORCONbits.SATA = 0; 	// Disable Saturation on Acc A
	// Store last 2 errors
	ControlDifference[2] = ControlDifference[1];
	ControlDifference[1] = ControlDifference[0];

	// If CLOSED_LOOP is undefined (running open loop) overide ControlOutput
	// with value read from the external potentiometer
	#ifndef CLOSED_LOOP
		ControlOutput = RefSpeed;
	#endif

	// ControlOutput will determine the motor required direction
	if (ControlOutput < 0)
		Required_Direction = CCW;
	else
		Required_Direction = CW;
	return;
}
#else
	// In version 2.0 and greater; there is no need to claim specific
	// registers, new constraint letters allow the compiler to make a
	// choice (also, new builtins mean you don't have to resort to
	// assembly for this kind of operation, see the next function


void SpeedControl(void) {
        SFRAC16 *ControlDifferencePtr = ControlDifference;
        SFRAC16 *PIDCoefficientsPtr = PIDCoefficients;
        SFRAC16 x_prefetch;
        SFRAC16 y_prefetch;

		register int reg_a asm("A");
		register int reg_b asm("B");

		CORCONbits.SATA = 1;    // Enable Saturation on Acc A

	#if __C30_VERSION__ == 320
	#error "This Demo is not supported with v3.20"
	#endif

	#if __C30_VERSION__ < 320

		reg_a = __builtin_lac(RefSpeed,0);
		reg_b = __builtin_lac(MeasuredSpeed,0);
		reg_a = __builtin_subab();
		*ControlDifferencePtr = __builtin_sac(reg_a,0);
		reg_a = __builtin_movsac(&ControlDifferencePtr, &x_prefetch, 2,
				 &PIDCoefficientsPtr, &y_prefetch, 2, 0);
		reg_a = __builtin_lac(ControlOutput, 0);
		reg_a = __builtin_mac(x_prefetch,y_prefetch,
                              &ControlDifferencePtr, &x_prefetch, 2,
                              &PIDCoefficientsPtr, &y_prefetch, 2, 0);
		reg_a = __builtin_mac(x_prefetch,y_prefetch,
                              &ControlDifferencePtr, &x_prefetch, 2,
                              &PIDCoefficientsPtr, &y_prefetch, 2, 0);
		reg_a = __builtin_mac(x_prefetch,y_prefetch,
                              &ControlDifferencePtr, &x_prefetch, 2,
                              &PIDCoefficientsPtr, &y_prefetch, 2, 0);
        ControlOutput = __builtin_sac(reg_a, 0);

	#else

		//https://www.mikrocontroller.net/attachment/169069/SinusoidalBLDC_v1.2.c
        reg_a = __builtin_lac(RefSpeed,0);
		reg_b = __builtin_lac(MeasuredSpeed,0);
		reg_a = __builtin_subab(reg_a,reg_b);
		*ControlDifferencePtr = __builtin_sac(reg_a,0);
		__builtin_movsac(&ControlDifferencePtr, &x_prefetch, 2, &PIDCoefficientsPtr, &y_prefetch, 2, 0, reg_b);
		reg_a = __builtin_lac(ControlOutput, 0);

		reg_a = __builtin_mac(reg_a,x_prefetch,y_prefetch,
                              &ControlDifferencePtr, &x_prefetch, 2,
                              &PIDCoefficientsPtr, &y_prefetch, 2, 0, reg_b);
		reg_a = __builtin_mac(reg_a,x_prefetch,y_prefetch,
                              &ControlDifferencePtr, &x_prefetch, 2,
                              &PIDCoefficientsPtr, &y_prefetch, 2, 0, reg_b);
		reg_a = __builtin_mac(reg_a,x_prefetch,y_prefetch,
                              &ControlDifferencePtr, &x_prefetch, 2,
                              &PIDCoefficientsPtr, &y_prefetch, 2, 0, reg_b);

    	ControlOutput = __builtin_sac(reg_a, 0);

	#endif

        CORCONbits.SATA = 0;    // Disable Saturation on Acc A
        // Store last 2 errors
        ControlDifference[2] = ControlDifference[1];
        ControlDifference[1] = ControlDifference[0];

        // If CLOSED_LOOP is undefined (running open loop) overide ControlOutput
        // with value read from the external potentiometer
        #ifndef CLOSED_LOOP
                ControlOutput = RefSpeed;
        #endif

        // ControlOutput will determine the motor required direction
        if (ControlOutput < 0)
                Required_Direction = CCW;
        else
                Required_Direction = CW;
        return;
}
#endif


/*********************************************************************
  Function:        void ForceCommutation(void)

  PreCondition:    None.
 
  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        This function is called each time the motor doesn't 
                   generate	hall change interrupt, which means that the motor
                   running too slow or is stalled. If it is stalled, the motor
                   is stopped, but if it is only slow, this function is called
                   and forces a commutation based on the actual hall sensor 
                   position and the required direction of rotation.

  Note:            None.
********************************************************************/

void ForceCommutation(void)
{
	HallValue = (unsigned int)((PORTD >> 5) & 0x0007);	// Read halls
	Sector = SectorTable[HallValue];	// Read sector based on halls
	if (Sector != -1)	// If the sector is invalid don't do anything
	{
		// Depending on the required direction, a new phase is fetched
		if (Required_Direction == CW)
		{
			// Motor is required to run forward, so read directly the table
			Phase = PhaseValues[Sector];
		}
		else
		{
			// Motor is required to run reverse, so calculate new phase and 
            // add offset to compensate asymmetries
			Phase = PhaseValues[(Sector + 3) % 6] + PhaseOffset;
		}
	}
	return;
}

/*********************************************************************
  Function:        void InitADC10(void)

  PreCondition:    None.
 
  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        Below is the code required to setup the ADC registers for:
	               1. 1 channel conversion (in this case RB2/AN2)
                   2. PWM trigger starts conversion
                   3. Pot is connected to CH0 and RB2/AN2
                   4. The data format will be signed fractional		

  Note:            None.
********************************************************************/

void InitADC10(void)
{
    printf("ADC init\r\n");
	//ADPCFG = 0x0038;				// RB3, RB4, and RB5 are digital Keine digitalen Signale auf Port B
	ADCON1 = 0x036E;				// PWM starts conversion
									// Signed fractional conversions
	ADCON2 = 0x0000;				 									
	ADCHS = 0x000A;					// Pot is connected to AN10, AN10 liegt an Headern über 47kOhm Widerstand
									 
	ADCON3 = 0x0003;				 
	IFS0bits.ADIF = 0;				// Clear ISR flag
	IEC0bits.ADIE = 1;				// Enable interrupts
	
	ADCON1bits.ADON = 1;			// turn ADC ON
	return;
}

/*********************************************************************
  Function:        void InitMCPWM(void)

  PreCondition:    None.
 
  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        InitMCPWM, intializes the PWM as follows:
                   1. FPWM = 20000 hz
                   2. Complementary PWMs with center aligned
                   3. Set Duty Cycle to 0 for complementary, which is half the
                      period
                   4. Set ADC to be triggered by PWM special trigger
                   5. Configure deadtime to be 1 us	

  Note:            None.
********************************************************************/

void InitMCPWM(void)
{
	printf("PWM init\r\n");
    TRISE = 0x0100;	// PWM pins as outputs, and FLTA as input
	PTPER = (FCY/FPWM - 1) >> 1;	// Compute Period based on CPU speed and 
                                    // required PWM frequency (see defines)
	OVDCON = 0x0000;	// Disable all PWM outputs.
	DTCON1 = 0x0010;	// ~1 us of dead time
	PWMCON1 = 0x0077;	// Enable PWM output pins and configure them as 
                        // complementary mode		 
	PDC1 = PTPER;	    // Initialize as 0 voltage
	PDC2 = PTPER;	    // Initialize as 0 voltage
	PDC3 = PTPER;	    // Initialize as 0 voltage
	SEVTCMP = 1;	    // Enable triggering for ADC
	PWMCON2 = 0x0F02;	// 16 postscale values, for achieving 20 kHz
	PTCON = 0x8002;		// start PWM as center aligned mode
	return;				 
}

/*********************************************************************
  Function:        void InitICandCN(void)

  PreCondition:    None.
 
  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        Configure Hall sensor inputs, one change notification and 
                   two input captures. on IC7 the actual capture value is used
                   for further period calculation

  Note:            None.
********************************************************************/

void InitICandCN(void)
{
    printf("Hall interrupt init\r\n");
	//Hall A -> CN5. Hall A is only used for commutation.
	//Hall B -> IC7. Hall B is used for Speed measurement and commutation.
	//Hall C -> IC8. Hall C is only used for commutation.
	
	// Init Input change notification 5
	TRISD |= 0x70;		// Ensure that hall connections are inputs 
	CNPU1 = 0;	    	// Disable all CN pull ups
	CNEN1 = 0x4000;		// Enable CN15
    CNEN2 = 0x0001;		// Enable CN15
	IFS0bits.CNIF = 0;	// Clear interrupt flag

	// Init Input Capture 6
	IC6CON = 0x0001;	// Input capture every edge with interrupts and TMR3, ICTMR = 0 für Timer3
	IFS1bits.IC6IF = 0;	// Clear interrupt flag

	// Init Input Capture 8
	//IC8CON = 0x0001;	// Input capture every edge with interrupts and TMR3
	//IFS1bits.IC8IF = 0;	// Clear interrupt flag

	return;
}

/*********************************************************************
  Function:        void InitTMR1(void)

  PreCondition:    None.
 
  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        Initialization of timer 1 as a periodic interrupt each 1 ms
                   for speed control, motor stalled protection which includes:
                   forced commutation if the motor is too slow, or motor 
                   stopped if the motor is stalled.

  Note:            None.
********************************************************************/

void InitTMR1(void)
{
    printf("Timer1 init\r\n");
	T1CON = 0x0020;			// internal Tcy/64 clock
	TMR1 = 0;
	PR1 = 313;				// 1 ms interrupts for 20 MIPS ! Passt natürlich nicht zu 106 MIPS
	T1CONbits.TON = 1;		// turn on timer 1 
	return;
}

/*********************************************************************
  Function:        void InitTMR3(void)

  PreCondition:    None.
 
  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        Initialization of timer 3 as the timebase for the capture 
                   channels for calculating the period of the halls.

  Note:            None.
********************************************************************/

void InitTMR3(void)
{
	printf("Timer3 init\r\n");
    T3CON = 0x0020;			// internal Tcy/64 clock
	TMR3 = 0;
	PR3 = 0xFFFF;
	T3CONbits.TON = 1;		// turn on timer 3 
	return;
}

/*********************************************************************
  Function:        void InitUserInt(void)

  PreCondition:    None.
 
  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        Initialization of the IOs used by the application. The IOs 
                   for the PWM and for the ADC are initialized in their 
                   respective peripheral initialization subroutine.

  Note:            None.
********************************************************************/

void InitUserInt(void)
{
	printf("UserInt init\r\n");
    _TRISD8 = 1;	// dir RD8 as input
	// Analog pin for POT already initialized in ADC init subroutine
	//PORTF = 0x0008;		// RS232 Initial values UART in main initialisiert
	//TRISF = 0xFFF7;		// TX as output
	return;
}

// End of SinusoidalBLDC v1.1.c