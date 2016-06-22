/* Use instructions:
 * After reseting the controller, press RB0 to enter Build Mode
 * While in Build Mode, the LED will flicker
 * After pressing RB1 the controller will collect data and set corner point
 * After the corner point was succefully set the LED will flash and start
 *      to flicker again
 * Once all 4 corner points have been set, the LED will flash multiple times
 * Pressing RB0 while in Build mode will exit build mode
 * If the controller is not in build mode, it will constantly check borders
 * If the LED flashes, it is outside the border.
 *      (the flash simulates a shock being administered)
 *
 * Inputs - RB0 and RB1
 * Outputs - RD7 (green LED)
 */

#include<pic18.h>
#include"Utilities.h"


unsigned int A2D_Read(void);
void InitializeInterrupts(void);
void InitializePorts(void);
void ResetTMR0Countdown(void);
void WaitForPingBreak(void);
void OutputFourCornerPointsToSerial(void);
void FlashLED(int numberOfTimes);
void SetNewPoint(triPingData newPoint);

bool CheckIfInsideCornerBorder(int cornerNumber, triPingData pingData);
bool CheckIfInsideBorders(triPingData pingData);

triPingData RetrieveNextPingsData();

bool didReceivePing = false;
bool isPingBreak = false;
bool isInBuildMode = false;
bool areAllCornersSet = false;

bool wasButton0Pressed = false;
bool wasButton1Pressed = false;

char pingCounter = 0;
char array_index = 0;

int lightCounter = 0;
int numberOfCorners = 0;
//int maxNumberOfCorners = 8;

triPingData cornerPoints[8];
triPingData mostRecentPingData;

// all interrupts are handled here
void interrupt IntServe(void) {
    if(INT0IF){
        // debouncing test - see if noise or actual button press
        for(int i = 0; i < 32000; i++);
        if(RB0){
            wasButton0Pressed = true;
        }
        else { wasButton0Pressed = false; }

        INT0IF = 0;
    }
    if(INT1IF){
        // debouncing test - see if noise or actual button press
        for(int i = 0; i < 32000; i++);
        if(RB1){
            wasButton1Pressed = true;
        }
        else { wasButton1Pressed = false; }
        
        INT1IF = 0;
    }
    if(INT2IF){
        didReceivePing = true;
        pingCounter++;

        if(pingCounter == 3) {
            ResetTMR0Countdown();
            isPingBreak = true;
        }
        else if(pingCounter > 3 ) {
            pingCounter = 0;
        }

        // clear interrupt flag
        INT2IF = 0;
    }

    if(TMR0IF){
        // reset isPingBreak after 1 second has passed from ping counter == 3
        if(isPingBreak){
            pingCounter = 0;
            isPingBreak = false;
        }


        //clear interrupt flag and reset to 1 second
        TMR0IF = 0;
        ResetTMR0Countdown();
    }
}

void main(void) {
    InitializeInterrupts();
    InitializePorts();

    // main loop
    while(1){
        /* if RB0
         *  wait for button
         *  set flag for build
         * if build_flag
         *  turn light on
         *  if RB1
         *      turn light off
         *      wait for button
         *      RetrieveNextPingsData()
         *      SetNewPoint()
         *  if RB0
         *      wait for RB0
         *      clear flag for build
         * RetrieveNextPingsData()
         * CheckIfInsideBorders()
         */

        if(wasButton0Pressed) {
            wasButton0Pressed = false;
            isInBuildMode = true;
        }
        if(isInBuildMode) {
            //flicker light
            if(lightCounter++ > 1000){ RD7 = !RD7; lightCounter = 0; }

            if(wasButton1Pressed) {
                //turn light off
                wasButton1Pressed = false;
                RD7 = 0;
                mostRecentPingData = RetrieveNextPingsData();
                SetNewPoint(mostRecentPingData);
//                FlashLED(numberOfCorners);
            }
                   
            if(wasButton0Pressed) {
                wasButton0Pressed = false;
                isInBuildMode = false;
                RD7 = 0;
            }
        }
        else {
            mostRecentPingData = RetrieveNextPingsData();

            if(!CheckIfInsideBorders(mostRecentPingData)){
                RD7 = 1;
                for(int i = 0; i < 32000; i++);
                for(int i = 0; i < 32000; i++);
                for(int i = 0; i < 32000; i++);
                RD7 = 0;
            }
        }
    }
    return;
}

// initialize all PIC variables for the interrupts
void InitializeInterrupts(void) {
    GIE = 1; //Global interrupt enable
    PEIE = 1; //Timer interrupt enable

    //turn on rising edge interrupt on RB0 (INT0)
    INTEDG0 = 1; //Set to rising edge interrupt
    INT0IE = 1; //int0 enable 1/2
    TRISB0 = 1; //int0 enable 2/2
    
    //turn on rising edge interrupt on RB1 (INT1)
    INTEDG1 = 1; //Set to rising edge interrupt
    INT1IE = 1; //int1 enable 1/3
    INT1IP = 1; //int1 enable 2/3
    TRISB1 = 1; //int1 enable 3/3

    //turn on rising edge interrupt on RB2 (INT2)
    INTEDG2 = 1; //Set to rising edge interrupt
    INT2IE = 1; //int2 enable 1/3
    INT2IP = 1; //int2 enable 2/3
    TRISB2 = 1; //int2 enable 3/3

    //turn on Timer 0 interrupt
    T0CS = 0; //timer 0 clock, use osc/4
    T0CON = 0x87; //Timer 0 prescaler, PS = 256
    TMR0 = -9766; //set to 1 sec for PS = 256
    TMR0ON = 1; //Timer 0 enable 1/3
    TMR0IE = 1; //Timer 0 enable 2/3
    TMR0IP = 1; //Timer 0 enable 3/3

    //turn on uart interrupt
    //set baud rate to 9600 for 10MHz osc
    TXIE = 0;
    RCIE = 0;
    BRGH = 0;
    BRG16 = 1;
    SYNC = 0;
    SPBRG = 64;
    TXSTA = 0x22;
    RCSTA = 0x90;

    return;
}

void InitializePorts(void) {
    //set input/output
    //0x00 is output, 0xff is input
    TRISA = 0xff;
    TRISB = 0xff;
    TRISC = 0xc0; //set RC6 and RC7 to input for uart
    TRISD = 0x00;
    TRISE = 0x0f;

    //clear ports
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    PORTE = 0;

    //Set Ports B,C,D,E to digital, Set Port A to analog
    ADCON0 = 0x01; // Analog on channel 0, RA0
    ADCON1 = 0x07; // Analog/Binary Select
    ADCON2 = 0x85; // ADC Clock Settings

    return;
}


void ResetTMR0Countdown(void){
    TMR0 = -9766;
    return;
}

// sets the next corner point
void SetNewPoint(triPingData newPoint){
    cornerPoints[numberOfCorners] = newPoint;

    numberOfCorners++;
    
    //cap the number of corners to 8
    if(numberOfCorners > (8-1)){ numberOfCorners = 8; }

    FlashLED(numberOfCorners);

    return;
}

//checks the three parameters against the stored data, returns if inside
bool CheckIfInsideCornerBorder(int cornerNumber, triPingData pingData){
    bool isInsideMasterPostRange = false;
    bool isInsideSlave1PostRange = false;
    bool isInsideSlave2PostRange = false;

    if(pingData.masterValue > cornerPoints[cornerNumber].masterValue) { isInsideMasterPostRange = true; }

    if(pingData.slave1Value > cornerPoints[cornerNumber].slave1Value) { isInsideSlave1PostRange = true; }

    if(pingData.slave2Value > cornerPoints[cornerNumber].slave2Value) { isInsideSlave2PostRange = true; }

    return (isInsideMasterPostRange && isInsideSlave1PostRange && isInsideSlave2PostRange);
}

// checks each corner point's data set to see if the dog is inside the bounds
bool CheckIfInsideBorders(triPingData pingData){
//    OutputFourCornerPointsToSerial();
    // if no corners are set, there is no border
    if (numberOfCorners == 0) { return true; }
    
    bool insideBorder = false;
    for (int cornerNumber = 0; cornerNumber < numberOfCorners; cornerNumber++){
        if (CheckIfInsideCornerBorder(cornerNumber, pingData)) {
            insideBorder = true;
        }
    }
    return insideBorder;
    
    /* retrieve the 3 pings of data from the posts store in variables
     * if any of the 4 points's data sets is inside
     *      then the dog is inside, clear a flag and break from loop
     * if the dog isn't in any of the areas
     *      then the dog is outside all of them, set flag
     * if the flag is set then turn on LED
     */
}

// retrieves the ping data
triPingData RetrieveNextPingsData() {
    triPingData readValues;
    WaitForPingBreak();
    while(pingCounter <= 3){
        if(didReceivePing){
            didReceivePing = false;
            if(pingCounter == 1) {
                readValues.masterValue = A2D_Read();
            }
            else if(pingCounter == 2) {
                readValues.slave1Value = A2D_Read();
            }
            else if(pingCounter == 3) {
                readValues.slave2Value = A2D_Read();
                break;
            }
        }
    }
    return readValues;
}

// outputs all corner points to serial (for debugging)
void OutputAllCornerPointsToSerial(){
    for(int i = 0; i < numberOfCorners; i++) {
        OutputToSerial(cornerPoints[i].masterValue);
        OutputToSerial(cornerPoints[i].slave1Value);
        OutputToSerial(cornerPoints[i].slave2Value);

        while(!TRMT);
        TXREG = 10; // line feed
        while(!TRMT);
        TXREG = 13; // carriage return
    }
    return;
}

// waits for the ping break to start and stop (used before collecting values)
void WaitForPingBreak(){
    while(!isPingBreak); // wait for ping break flag
    while(isPingBreak); // wait for ping break flag to clear
    return;
}

/* Generic Point Creation - (max of 8 points)
 *  keep track of the number of points that are entered
 *  when new point is added, store data into (numberOfPoints++)
 *                                  (use variable++ to increment after use)
 *  when checking, use for loop until at numberOfPoints
 *      if numberOfPoints is not set (i.e. 0), then do nothing
 *
 *  when setting a new point
 *      flash LED once for each point stored
 */

// flashes the output LED for numberOfTimes
void FlashLED(int numberOfTimes){
    for(int i=0; i < numberOfTimes; i++){
        RD7 = 0;
        for(int i = 0; i < 32000; i++){ RD7 = 1; }
        for(int i = 0; i < 32000; i++){ RD7 = 0; }
    }
    return;
}