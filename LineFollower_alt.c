/* 
 * File:   LineFollower.c
 * Authors: Jesse Batstone and Matt Dioso
 *
 * Created on September 7, 2016, 11:08 AM
 */

#include <stdio.h>
#include <stdlib.h>
#include <plib.h>   // INT_CHANGE_NOTICE_VECTOR
#include <stdbool.h>

void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) OpticalSensorInterruptServiceRoutine() {
    if (PORTReadBits(IOPORT_B, BIT_1)) {
            SetDCOC3PWM(400);
            SetDCOC2PWM(700);
    }
    else {
            SetDCOC3PWM(700);
            SetDCOC2PWM(400);
    }
    
}
/* all ports have names:                        */
/* e.g. IOPORT_A IOPORT_B, etc.                 */
/* all port bits have names:                    */
/* e.g. BIT_0, BIT_1,... BIT_14, BIT_15, etc.   */
/* led 1    port RB10 */
/* led 2    port RB11 */
/* led 3    port RB12 */
/* led 4    port RB13 */

/*
 * This program is to control the motors on the MRK-LINE Robot, using feedback
 * from the optical sensors in the line-following accessory.
 * Here are the pertinent port connections:
 *      Left wheel (assuming sensors are on front bumper)
 *          DIR - RD6
 *          EN -  RD2   active high
 *          SA -  RD10  SA and SB are quadrature encoded feedback
 *          SB -  RC2
 *      Right wheel
 *          DIR - RD7
 *          EN -  RD1
 *          SA -  RD9
 *          SB -  RC1
 *      Optical Sensors
 *          S1 - RB0    S1 is rightmost sensor
 *          S2 - RB1    S2 is middle right sensor
 *          S3 - RB2    S3 is middle left sensor
 *          S4 - RB3    S4 is leftmost sensor
 */
int main(int argc, char** argv) {
    unsigned int Right_SA, Right_SB, Left_SA, Left_SB, temp;
    unsigned int RSA[64], RSB[64], LSA[64], LSB[64];
    unsigned int RSA_index, RSB_index, LSA_index, LSB_index = 0;
    unsigned int time_left, time_right, velocity_left, velocity_right;
    unsigned int timeValuersa, prevTimeValuersa, timeValuersb, prevTimeValuersb, timeValuelsa, prevTimeValuelsa, timeValuelsb, prevTimeValuelsb;
    bool rightsaChange = FALSE;
    bool rightsbChange = FALSE;
    bool leftsaChange = FALSE;
    bool leftsbChange = FALSE;
    const int timePeriod = 65535;
    const int saDistance = 5.875;

    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
    INTEnableInterrupts();

        //Configure ports for onboard LEDs as outputs
    PORTSetPinsDigitalOut(IOPORT_B, BIT_10 | BIT_11 | BIT_12 | BIT_13);
        //Configure ports for motor control as outputs
    PORTSetPinsDigitalOut(IOPORT_D, BIT_1 | BIT_2 | BIT_6 | BIT_7);
    // Useful functions: (see PeripheralLibraries pdf file for more)
    //  PORTSetPinsDigitalOut(IOPORT_B, BIT_10 | BIT_11 | BIT_12 | BIT_13);
    //  PORTSetPinsDigitalIn(IOPORT_B, BIT_10 | BIT_11 | BIT_12 | BIT_13);
    //  PORTClearBits(IOPORT_B, BIT_10 | BIT_11 | BIT_12 | BIT_13);     // clear all bits
    //  PORTSetBits(IOPORT_B, BIT_10 | BIT_11 | BIT_12 | BIT_13);       // set all bits
    //  PORTToggleBits(IOPORT_B, BIT_10 | BIT_11 | BIT_12 | BIT_13);    // toggle state of the bits

    // Configure built in buttons as inputs
    // On ProMX4 butons are on RA6 and RA7
    PORTSetPinsDigitalIn(IOPORT_A, BIT_6 | BIT_7);
    // Configure Optical Sensors as inputs
    PORTSetPinsDigitalIn(IOPORT_B, BIT_0 | BIT_1 | BIT_2 | BIT_3);
    // Configure Motor feedback as inputs
    PORTSetPinsDigitalIn(IOPORT_C, BIT_1 | BIT_2);
    PORTSetPinsDigitalIn(IOPORT_D, BIT_9 | BIT_10);
    // Useful function:  see DigitalIO project for more
    // PORTReadBits(IOPORT_A, BIT_6);   // read the state of button on RA6

    PORTSetBits(IOPORT_D, BIT_6);       // Left wheel DIR=1
    PORTClearBits(IOPORT_D, BIT_7);     // Right wheel DIR=0
    PORTClearBits(IOPORT_D, BIT_2);     // Left wheel EN=0
    PORTClearBits(IOPORT_D, BIT_1);     // Left wheel EN=0

    OSCConfig(OSC_POSC_PLL, OSC_PLL_MULT_20, OSC_PLL_POST_8, OSC_FRC_POST_1);

    OpenTimer2(T2_ON | T2_IDLE_STOP | T2_GATE_OFF | T2_PS_1_1, 1000);
    // RD1, the right wheel's EN pin, doubles as OC2
    // RD2, the left wheel's EN pin, doubles as OC3
    OpenOC2(OC_ON | OC_IDLE_STOP | OC_TIMER_MODE16 | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 00, 00 );
    OpenOC3(OC_ON | OC_IDLE_STOP | OC_TIMER_MODE16 | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 00, 00 );

    /* STEP 2. setup the change notice options */
    mCNOpen(CN_ON | CN_IDLE_STOP, CN3_ENABLE,CN_PULLUP_DISABLE_ALL );
    /* STEP 3. read port(s) to clear mismatch */
    int value = mPORTBRead();
    /* STEP 4. clear change notice interrupt flag */
    ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_2);
    /* STEP 5. enable multi-vector interrupts */
    INTEnableSystemMultiVectoredInt();

    OpenTimer3(T3_ON|T3_IDLE_STOP|T3_GATE_OFF|T3_PS_1_32,65535);

    SetDCOC3PWM(600);
    SetDCOC2PWM(600);

    Right_SA = PORTReadBits(IOPORT_D, BIT_9);
    Right_SB = PORTReadBits(IOPORT_C, BIT_1);
    Left_SA = PORTReadBits(IOPORT_D, BIT_10);
    Left_SB = PORTReadBits(IOPORT_C, BIT_2);

    //
     unsigned int position[64][2] = 0;  //arbitrary
     unsigned int orientation[64][2] = 0;   //pointing in -x direction, arbitrary
     orientation[0][0] = -1; orientation[0][1] = 0;
     signed int posRow = 0;
     signed int posCol = 0;
     signed int oriRow = 0;
     signed int oriCol = 0;

    bool roundtripnotdone = TRUE;
    bool SChange = FALSE;
    while (roundtripnotdone) {
        if (rightsaChange) {
    //       record timer in right SA array;
            prevTimeValuersa = timeValuersa;
            timeValuersa = RSA[RSA_index];
            SChange=TRUE;
            rightsaChange = FALSE;
        }
        if (rightsbChange) {
    //       record timer in right SB array;
            prevTimeValuersb = timeValuersb;
            timeValuersb = RSB[RSB_index];
            SChange=TRUE;
            rightsbChange = FALSE;
        }
        if (leftsaChange) {
    //       record timer in left SA array;
            prevTimeValuelsa = timeValuelsa;
            timeValuelsa = LSA[LSA_index];
            SChange=TRUE;
            leftsaChange = FALSE;
        }
        if (leftsbChange) {
    //       record timer in left SB array;
            prevTimeValuelsb = timeValuelsb;
            timeValuelsb = LSB[LSB_index];
            SChange=TRUE;
            leftsbChange = FALSE;
        }
        if(SChange){
            SChange=FALSE;
            time_right= (timeValuersa - prevTimeValuersa)* 65535;
            time_left = (timeValuelsa - prevTimeValuelsa)* 65535;
            velocity_left = saDistance/time_left;
            velocity_right = saDistance/time_right;
            if (velocity_right > velocity_left) {
    //          r (to the left) = d/((velocity_right/velocity_left)-1);
    //          position(new) = position(previous) +
    //             orientation(previous)*[distance*(r+d/2)/(r))*(1-distance/r),distance*(r+d/2)/(r))*(distance/r)]';
    //          orientation(new) = orientation(previous) + [distance*(r+d/2)/(r))*(1-distance/r),-distance*(r+d/2)/(r))*(distance/r)]
             }
    //       else if (velocity_left > velocity_right){
    //          r (to the right) = d/((velocity_left/velocity_right)-1);
    //          position(new) = position(previous) +
    //             orientation(previous)*[distance*(r+d/2)/(r))*(1-distance/r),-distance*(r+d/2)/(r))*(distance/r)]';
    //          orientation(new) = orientation(previous) + [distance*(r+d/2)/(r))*(1-distance/r),distance*(r+d/2)/(r))*(distance/r)]
    //       }
    //       else {
    //          position(new) = position(previous) + distance*orientation(previous);
    //          orientation(new) = orientation(previous);
    //       }
    //       if ||position(new) - position(0)|| < 2 inches
    //          roundtripnotdone = FALSE;
        }
    }
    // traverse position array from beginning to find entry where the x coordinate
    //    is again approximately 0, call it n
    // arc_length = y difference;
    // increment to position(1), decrement from position(n)
    // arc_width = x difference;
    // area = area + arc_length*arc_width;
    // continue process until end points meet;
    // go back to original arc and continue in the other direction until done.
    // display area on OLED controller or blink it slowly on the LEDs or ...
    //
    while (1) // continuous loop
    {
        if (Right_SA != (temp = PORTReadBits(IOPORT_D, BIT_9)) ) {
            Right_SA = temp;
            RSA[RSA_index] = ReadTimer3();
            RSA_index++;
            rightsaChange = TRUE;
        }
        if (Right_SB != (temp = PORTReadBits(IOPORT_C, BIT_1) )) {
            Right_SB = temp;
            RSB[RSB_index] = ReadTimer3();
            RSB_index++;
            rightsbChange = TRUE;
        }
        if (Left_SA != (temp = PORTReadBits(IOPORT_D, BIT_10) )) {
            Left_SA = temp;
            LSA[LSA_index] = ReadTimer3();
            LSA_index++;
            leftsaChange = TRUE;
        }
        if (Left_SB != (temp = PORTReadBits(IOPORT_C, BIT_2) )) {
            Left_SB = temp;
            LSB[LSB_index] = ReadTimer3();
            LSB_index++;
            leftsbChange = TRUE;
        }
    }

    return (EXIT_SUCCESS);
}

