//tri-post controller/dog collar

//needs to be finished being debugged
//not used in finished project for this reason

//timer interrupts on posts and collar/controller calibrated for 10MHz Clock

//listen for tri-post signal, able to find distances between posts,
    //determine position on a virtual grid, able to build a border, 
    //and determine if within border

//takes 4 A2D readings per received tri-post signal then averages the readings
//needs debug, getting inconsistant averages

//location finding using triangle method
//location output to serial in feet
//indicates if location is within border by turning on LED connected to RD7 if
    //location is outside the border and outputting to serial: 1 for inside,
    //0 for outside

//Pictorial diagram notes
    //Post 1 is Master
    //Post 2 is Slave1
    //Post 3 is Slave2
    //r12 is labeled DistanceMS1
    //r13 is labeled DistanceMS2
    //r1d is labeled DistanceToMaster
    //r2d is labeled DistanceToSlave1
    //r3d is labeled DistanceToSlave2
    //Cosine Theta 1-4 are labeled Cosine Theta 0-3

//Finding distances between posts notes
    //stand at master when pressing RB0
    //Pressing RB0 brings user from main to distance finding function

//Border notes
    //max of 8 corners
    //convex polygons only
    //place corners in a clockwise fashion starting w/ the bottom left corner
        //examples to be given so there is no confusion
    //Pressing RB1 brings user from main to corner placement function
    //Pressing RB1 again places a corner
    //Pressing RB0 returns user to main

//include files
#include<pic18.h>
#include<math.h>

//globals
char BorderCreated = 0;
char CornerPlaced = 0;
char CornerX5LeftOfCornerX6 = 0;
char CornerX4LeftOfCornerX5 = 0;
char CornerX4LeftOfCornerX3 = 0;
char CornerX7LeftOfCornerX8 = 0;
char CornerX6LeftOfCornerX7 = 0;
char CornerX3LeftOfCornerX4 = 0;
char CornerX3LeftOfCornerX2 = 0;
char CornerX2LeftOfCornerX1 = 0;
char CornerX2RightOfCornerX3 = 0;
char DistanceBetweenPostsFound = 0;
char i = 0;//for loop index variable
char InsideBorder = 1;
int InsideBorder_int = 0;
char NumberOfCorners = 0;
char ReadCount = 0;
char ThreeReadingsExist = 0;

unsigned int Read[4] = {0,0,0,0};
unsigned int ReadFromMaster = 0;
double ReadFromMaster_double = 0;
unsigned int ReadFromSlave1 = 0;
double ReadFromSlave1_double = 0;
unsigned int ReadFromSlave2 = 0;
double ReadFromSlave2_double =0;

long int DistanceMS1 = 0;
long int DistanceMS2 = 0;

double BorderSlope[8] = {0,0,0,0,0,0,0,0};
double CornerX[8] = {0,0,0,0,0,0,0,0};
double CornerY[8] = {0,0,0,0,0,0,0,0};
double CosineTheta[4] = {0,0,0,0};
double Location[2] = {0,0};
double Location_feet[2] = {0,0};
int Location_feet_int[2] = {0,0};
double YBorderLine[8] = {0,0,0,0,0,0,0,0};
double YIntercept[8] = {0,0,0,0,0,0,0,0};

double DistanceToMaster = 0;
long int DistanceToMaster_lint = 0;
double DistanceToSlave1 = 0;
long int DistanceToSlave1_lint = 0;
double DistanceToSlave2 = 0;
long int DistanceToSlave2_lint = 0;
double PowerFromMaster = 0;
double PowerFromSlave1 = 0;
double PowerFromSlave2 = 0;

//placeholder variables
long int Denominator = 0;
double Denominator_double = 0;
//reduces number of divide operations

long int Numerator = 0;
double Numerator_double = 0;
//used w/ denominator in divide most operations

double Exponent = 0;
//condenses exponent for pow function to one number


//constants
const double ANTENNA_GAIN = 1.7;//antenna gain in dB
const double A2D_LEVELS_PER_VOLT = 320;
const double METER_TO_FOOT_CONVERSION_RATIO = 3.28084;
const double PI = 3.14159;
const double POWER_EQUATION_SLOPE_VOLTAGE = 62.3;
const double POWER_EQUATION_Y_INTERCEPT = -137.5;
const double TRANSMISSION_POWER = -27;//transmission power in dB
const double WAVELENGTH = .7177;//signal wavelength in meters

double POWER_EQUATION_SLOPE_A2D = 0;//set in InitializePorts subroutine

//declare subroutines
unsigned int A2D_Read(void);
unsigned int AverageReadings(void);

int AbsoluteValue_int(int num);

double AbsoluteValue(double num);

void BorderCreation(void);
void BuildBorder(void);
void CalculateCosines(void);
void CalculatePowerRecievedFromPosts(void);
void CheckHowManyCornersExist(void);
void CheckIfThreeReadingsExist(void);
void CheckIfInsideOfBorder(void);
void CheckIfInsideOfBorderThreeSides(void);
void CheckIfInsideOfBorderFourSides(void);
void CheckIfInsideOfBorderFiveSides(void);
void CheckIfInsideOfBorderSixSides(void);
void CheckIfInsideOfBorderSevenSides(void);
void CheckIfInsideOfBorderEightSides(void);
void ClearCosines(void);
void ClearDistances(void);
void ClearINT2Flags(void);
void ClearOrientationVariables(void);
void ClearPlaceholderVariables(void);
void ClearReadings(void);
void ClearReadingsBeingAveraged(void);
void ClearTMR0Flags(void);
void ClearTMR1Flags(void);
void ConvertLocationToFeet(void);
void ConvertPowerReceivedTodB(void);
void FindDistanceBetweenPosts(void);
void FindDistanceToPosts(void);
void FindLocation(void);
void IndicateIfWithinBorder(void);
void InitializeInterrupts(void);
void InitializePorts(void);
void OutputReadings(void);
void OutputToSerial(int DATA);
void PlaceCorner(void);
void ResetReadCounter(void);
void ResetTMR0Countdown(void);
void TakeReadings(void);
void TurnOffBorderViolationIndicationLight(void);
void TurnOnBorderViolationIndicationLight(void);


//isr
void interrupt IntServe(void){
    if(INT2IF){
        ResetTMR0Countdown();
        if(!ReadFromMaster){
            TakeReadings();
            ReadFromMaster = AverageReadings();
        }
        if(!ReadFromSlave1){
            TakeReadings();
            ReadFromSlave1 = AverageReadings();
        }
        if(!ReadFromSlave2){
            TakeReadings();
            ReadFromSlave2 = AverageReadings();
        }
        ReadCount++;
        ClearINT2Flags();
    }
    if(TMR0IF){
        ClearReadings();
        ResetReadCounter();
        ClearTMR0Flags();
    }
}

//main
void main(void){
    InitializePorts();
    InitializeInterrupts();
    while(1){//main loop
        if(RB0 && !DistanceBetweenPostsFound){
            FindDistanceBetweenPosts();
        }
        if(RB1 && !BorderCreated){
            BorderCreation();
        }
            
        CheckIfThreeReadingsExist();
        if(ThreeReadingsExist){
            FindLocation();
            CheckIfInsideOfBorder();

            OutputToSerial(Location_feet_int[0]);
            OutputToSerial(Location_feet_int[1]);
            OutputToSerial(InsideBorder_int);
            while(!TRMT); TXREG = 13; //return to beginning of line
            while(!TRMT); TXREG = 10; //next line
            
            IndicateIfWithinBorder();
        }
        
    }
}


//subroutines
unsigned int A2D_Read(void){
    ADCON0 = 0x01;//select channel 0 for a2d reading
    GODONE = 1;//start conversions
    while(GODONE);//wait until done (approx 8us)
    return(ADRES);
}

unsigned int AverageReadings(void){
    unsigned int average = 0;
    unsigned int sum = 0;
    for(i=0;i<4;i++){
        sum = sum + Read[i];
    }
    average = (sum >> 2);//bit shift for faster divide by 4
    ClearReadingsBeingAveraged();
    return average;
}

int AbsoluteValue_int(int num){
    if (num < 0){
        return -num;
    }
    else{
        return num;
    }
}

double AbsoluteValue(double num){
    if (num < 0){
        return -num;
    }
    else{
        return num;
    }
}

void BorderCreation(void){
    do{
       while(RB1);
       while(!RB0){
           if(RB1){
               PlaceCorner();
           }
       }
       BuildBorder();
    }while(!BorderCreated);
    BorderCreated = 0;
}

void BuildBorder(void){
    CheckHowManyCornersExist();
    
    //find slopes
    for(i=0;i<=(NumberOfCorners-2);i++){
        ClearPlaceholderVariables();
        Numerator_double = CornerY[i] - CornerY[i+1];
        Denominator_double = CornerX[i] - CornerX[i+1];
        BorderSlope[i] = Numerator_double/Denominator_double;
    }

    ClearPlaceholderVariables();
    Numerator_double = CornerY[NumberOfCorners-1] - CornerY[1];
    Denominator_double = CornerY[NumberOfCorners-1] - CornerY[1];
    BorderSlope[(NumberOfCorners-1)] = Numerator_double/Denominator_double;

    //find y intercepts
    for(i=0;i<=(NumberOfCorners-1);i++){
        YIntercept[i] = CornerY[i] - (CornerX[i]*BorderSlope[i]);
    }
    
    BorderCreated = 1;
}

void CalculateCosines(void){
    //calculate cosine theta using law of cosines
    //converting between long ints and doubles for faster calculation

    //clear previously calculated cosine values
    ClearCosines();
    
    //calculate cosine theta 0
    ClearPlaceholderVariables();
    Numerator = (DistanceMS1*DistanceMS1) +
            (DistanceToMaster_lint*DistanceToMaster_lint)
            - (DistanceToSlave1_lint*DistanceToSlave1_lint);
    Denominator = DistanceToMaster_lint*DistanceMS1*2;
    Numerator_double = (double)Numerator;
    Denominator_double = (double)Denominator;
    CosineTheta[0] = Numerator_double/Denominator_double;

    //calculate cosine theta 1
    ClearPlaceholderVariables();
    Numerator = (DistanceMS1*DistanceMS1) +
            (DistanceToSlave1_lint*DistanceToSlave1_lint)
            - (DistanceToMaster_lint*DistanceToMaster_lint);
    Denominator = DistanceToSlave1_lint*DistanceMS1*2;
    Numerator_double = (double)Numerator;
    Denominator_double = (double)Denominator;
    CosineTheta[1] = Numerator_double/Denominator_double;

    //calculate cosine theta 2
    ClearPlaceholderVariables();
    Numerator = (DistanceMS2*DistanceMS2) +
            (DistanceToSlave2_lint*DistanceToSlave2_lint)
            - (DistanceToMaster_lint*DistanceToMaster_lint);
    Denominator = DistanceToSlave2_lint*DistanceMS2*2;
    Numerator_double = (double)Numerator;
    Denominator_double = (double)Denominator;
    CosineTheta[2] = Numerator_double/Denominator_double;

    //calculate cosine theta 3
    ClearPlaceholderVariables();
    Numerator = (DistanceMS2*DistanceMS2) +
            (DistanceToMaster_lint*DistanceToMaster_lint)
            - (DistanceToSlave2_lint*DistanceToSlave2_lint);
    Denominator = DistanceToMaster_lint*DistanceMS2*2;
    Numerator_double = (double)Numerator;
    Denominator_double = (double)Denominator;
    CosineTheta[3] = Numerator_double/Denominator_double;
}

void CalculatePowerRecievedFromPosts(void){
    //Calculate Power Received from a2d readings
    
    //convert to doubles for accurate calculations
    ReadFromMaster_double = (double)ReadFromMaster;
    ReadFromSlave1_double = (double)ReadFromSlave1;
    ReadFromSlave2_double = (double)ReadFromSlave2;
    //equation from linear portion of power received vs Vrssi from data sheet
        //and a2d to voltage conversion
    //gives power in dBm
    PowerFromMaster = POWER_EQUATION_SLOPE_A2D*ReadFromMaster_double 
            + POWER_EQUATION_Y_INTERCEPT;
    PowerFromSlave1 = POWER_EQUATION_SLOPE_A2D*ReadFromSlave1_double 
            + POWER_EQUATION_Y_INTERCEPT;
    PowerFromSlave2 = POWER_EQUATION_SLOPE_A2D*ReadFromSlave2_double 
            + POWER_EQUATION_Y_INTERCEPT;

    OutputReadings();
    ClearReadings();
    //ConvertPowerReceivedTodB();
}

void CheckHowManyCornersExist(void){
    for(i=0;i<=7;i++){
        if(CornerX[i] != 0){
            NumberOfCorners++;
        }
    }
}

void CheckIfThreeReadingsExist(void){
    if((ReadFromMaster)&&(ReadFromSlave1)&&(ReadFromSlave2)){
        ThreeReadingsExist = 1;
    }
    else{
        ThreeReadingsExist = 0;
    }
}

void CheckIfInsideOfBorder(void){
    if(NumberOfCorners == 3){
        CheckIfInsideOfBorderThreeSides();
    }
    if(NumberOfCorners == 4){
        CheckIfInsideOfBorderFourSides();
    }
    if(NumberOfCorners == 5){
        CheckIfInsideOfBorderFiveSides();
    }
    if(NumberOfCorners == 6){
        CheckIfInsideOfBorderSixSides();
    }
    if(NumberOfCorners == 7){
        CheckIfInsideOfBorderSevenSides();
    }
    if(NumberOfCorners == 8){
        CheckIfInsideOfBorderEightSides();
    }
    
    //convert to int for output to serial
    InsideBorder_int = (int)InsideBorder;
}

void CheckIfInsideOfBorderThreeSides(void){
    //calculate border lines
    for(i=0;i<=2;i++){
        YBorderLine[i] = BorderSlope[i]*Location[0] + YIntercept[i];
    }

    //check where corners are in relation to each other
    if(CornerX[1] < CornerX[0]){
        CornerX2LeftOfCornerX1 = 1;
    }
    else if (CornerX[1] > CornerX[2]){
        CornerX2RightOfCornerX3 = 1;
    }

    //determine if Location is in border
    if(CornerX2LeftOfCornerX1){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] > Location[1]) 
                && (YBorderLine[3] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    else if(CornerX2RightOfCornerX3){
        if((YBorderLine[0] > Location[1]) && (YBorderLine[1] < Location[1]) 
                && (YBorderLine[3] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    else{//CornerX[2] between CornerOnex and CornerX[3]
        if((YBorderLine[0] > Location[1]) && (YBorderLine[1] > Location[1]) 
                && (YBorderLine[3] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    
    ClearOrientationVariables();
}

void CheckIfInsideOfBorderFourSides(void){
    //calculate border lines
    for(i=0;i<=3;i++){
        YBorderLine[i] = BorderSlope[i]*Location[0] + YIntercept[i];
    }

    //check where corners are in relation to each other
    if(CornerX[1] < CornerX[0]){
        CornerX2LeftOfCornerX1 = 1;
    }
    if(CornerX[2] < CornerX[3]){
        CornerX3LeftOfCornerX4 = 1;
    }

    //check if Location is inside the border
    if(CornerX2LeftOfCornerX1 && CornerX3LeftOfCornerX4){
        if((YBorderLine[1] > Location[1]) && (YBorderLine[3] > Location[1]) &&
            (YBorderLine[3] < Location[1]) && (YBorderLine[0] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    else if(!CornerX2LeftOfCornerX1 && CornerX3LeftOfCornerX4){
        if((YBorderLine[0] > Location[1]) && (YBorderLine[1] > Location[1]) &&
            (YBorderLine[3] > Location[1]) && (YBorderLine[3] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    else if(!CornerX2LeftOfCornerX1 && !CornerX3LeftOfCornerX4){
        if((YBorderLine[0] > Location[1]) && (YBorderLine[1] > Location[1]) &&
            (YBorderLine[3] < Location[1]) && (YBorderLine[3] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    else if(CornerX2LeftOfCornerX1 && CornerX3LeftOfCornerX4){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] > Location[1]) &&
            (YBorderLine[3] > Location[1]) && (YBorderLine[3] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    
    ClearOrientationVariables();
}

void CheckIfInsideOfBorderFiveSides(void){
    //calculate border lines
    for(i=0;i<=4;i++){
        YBorderLine[i] = BorderSlope[i]*Location[0] + YIntercept[i];
    }

    //check where corners are in relation to each other
    if(CornerX[1] > CornerX[0]){
        CornerX2LeftOfCornerX1 = 1;
    }
    if(CornerX[2] > CornerX[1]){
        CornerX3LeftOfCornerX2 = 1;
    }
    if(CornerX[2] > CornerX[3]){
        CornerX3LeftOfCornerX4 = 1;
    }
    if(CornerX[3] > CornerX[4]){
        CornerX4LeftOfCornerX5 = 1;
    }

    //determine if Location is inside the border
    if(CornerX2LeftOfCornerX1 && CornerX3LeftOfCornerX2
            && CornerX4LeftOfCornerX5){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] < Location[1]) &&
              (YBorderLine[3] > Location[1]) && (YBorderLine[3] > Location[1]) 
               && (YBorderLine[4] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    else if(CornerX2LeftOfCornerX1 && !CornerX3LeftOfCornerX2
            && CornerX4LeftOfCornerX5){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] > Location[1]) &&
              (YBorderLine[3] > Location[1]) && (YBorderLine[3] > Location[1]) 
               && (YBorderLine[4] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }

    }

    else if(CornerX2LeftOfCornerX1 && !CornerX3LeftOfCornerX2
            && !CornerX4LeftOfCornerX5){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] > Location[1]) &&
             (YBorderLine[3] > Location[1]) &&
            (YBorderLine[3] < Location[1]) && (YBorderLine[4] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    else if(!CornerX2LeftOfCornerX1 && !CornerX3LeftOfCornerX4
            && !CornerX4LeftOfCornerX5){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] > Location[1]) &&
              (YBorderLine[3] < Location[1]) && (YBorderLine[3] < Location[1])
              && (YBorderLine[4] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    else if(!CornerX2LeftOfCornerX1 && !CornerX3LeftOfCornerX2
            && !CornerX4LeftOfCornerX5 && CornerX3LeftOfCornerX4){
        if((YBorderLine[0] > Location[1]) && (YBorderLine[1] > Location[1]) &&
              (YBorderLine[3] > Location[1]) && (YBorderLine[3] > Location[1]) 
              && (YBorderLine[4] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
        else if(!CornerX2LeftOfCornerX1 && !CornerX3LeftOfCornerX4
            && !CornerX4LeftOfCornerX5){
        if((YBorderLine[0] > Location[1]) && (YBorderLine[1] > Location[1]) &&
              (YBorderLine[3] < Location[1]) && (YBorderLine[3] < Location[1]) 
              && (YBorderLine[4] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }

    ClearOrientationVariables();
}

void CheckIfInsideOfBorderSixSides(void){
    //calculate border lines
    for(i=0;i<=5;i++){
        YBorderLine[i] = BorderSlope[i]*Location[0] + YIntercept[i];
    }

    //check where corners are in relation to each other
    if(CornerX[1] < CornerX[0]){
        CornerX2LeftOfCornerX1 = 1;
    }
    if(CornerX[2] < CornerX[1]){
        CornerX3LeftOfCornerX2 = 1;
    }
    if(CornerX[3] < CornerX[4]){
        CornerX4LeftOfCornerX5 = 1;
    }
    if(CornerX[4] < CornerX[5]){
        CornerX5LeftOfCornerX6 = 1;
    }

    //check if Location is inside of the border
    if(CornerX2LeftOfCornerX1 && !CornerX3LeftOfCornerX2
            && CornerX4LeftOfCornerX5 && !CornerX5LeftOfCornerX6){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] > Location[1]) &&
           (YBorderLine[3] > Location[1]) && (YBorderLine[3] > Location[1]) 
          &&(YBorderLine[4] < Location[1]) && (YBorderLine[5] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    else if(CornerX2LeftOfCornerX1 && CornerX3LeftOfCornerX2
              && CornerX4LeftOfCornerX5 && !CornerX5LeftOfCornerX6){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] < Location[1]) 
           && (YBorderLine[3] > Location[1]) && (YBorderLine[3] > Location[1]) 
          &&(YBorderLine[4] < Location[1]) && (YBorderLine[5] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    else if(CornerX2LeftOfCornerX1 && CornerX3LeftOfCornerX2
              && !CornerX4LeftOfCornerX5 && !CornerX5LeftOfCornerX6){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] < Location[1]) 
           && (YBorderLine[3] > Location[1]) && (YBorderLine[3] < Location[1]) 
          &&(YBorderLine[4] < Location[1]) && (YBorderLine[5] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    else if(CornerX2LeftOfCornerX1 && CornerX3LeftOfCornerX2
              && CornerX4LeftOfCornerX5 && CornerX5LeftOfCornerX6){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] < Location[1]) 
           && (YBorderLine[3] > Location[1]) && (YBorderLine[3] > Location[1]) 
          &&(YBorderLine[4] > Location[1]) && (YBorderLine[5] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
        else if(CornerX2LeftOfCornerX1 && !CornerX3LeftOfCornerX2
              && !CornerX4LeftOfCornerX5 && !CornerX5LeftOfCornerX6){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] > Location[1]) 
           && (YBorderLine[3] > Location[1]) && (YBorderLine[3] < Location[1]) 
          &&(YBorderLine[4] < Location[1]) && (YBorderLine[5] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    else if(!CornerX2LeftOfCornerX1 && !CornerX3LeftOfCornerX2
              && !CornerX4LeftOfCornerX5 && !CornerX5LeftOfCornerX6){
        if((YBorderLine[0] > Location[1]) && (YBorderLine[1] > Location[1]) 
           && (YBorderLine[3] > Location[1]) && (YBorderLine[3] < Location[1]) 
          &&(YBorderLine[4] < Location[1]) && (YBorderLine[5] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }

    ClearOrientationVariables();
}

void CheckIfInsideOfBorderSevenSides(void){
    //calculate border lines
    for(i=0;i<=6;i++){
        YBorderLine[i] = BorderSlope[i]*Location[0] + YIntercept[i];
    }

    //check where corners are in relation to each other
    if(CornerX[1] < CornerX[0]){
        CornerX2LeftOfCornerX1 = 1;
    }
    if(CornerX[2] < CornerX[1]){
        CornerX3LeftOfCornerX2 = 1;
    }
    if(CornerX[3] < CornerX[4]){
        CornerX4LeftOfCornerX5 = 1;
    }
    if(CornerX[3] < CornerX[2]){
        CornerX4LeftOfCornerX3 = 1;
    }
    if(CornerX[4] < CornerX[5]){
        CornerX5LeftOfCornerX6 = 1;
    }
    if(CornerX[5] < CornerX[6]){
        CornerX6LeftOfCornerX7 = 1;
    }

    //determine if Location is inside the border
    //one
    if(CornerX2LeftOfCornerX1 && !CornerX3LeftOfCornerX2
            && !CornerX4LeftOfCornerX3 && CornerX4LeftOfCornerX5
            && CornerX5LeftOfCornerX6 && !CornerX6LeftOfCornerX7){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] > Location[1]) 
           && (YBorderLine[3] > Location[1]) && (YBorderLine[3] > Location[1]) 
           && (YBorderLine[4] > Location[1]) && (YBorderLine[5] < Location[1]) 
             && (YBorderLine[6] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    //two
    else if(CornerX2LeftOfCornerX1 && CornerX3LeftOfCornerX2
            && !CornerX4LeftOfCornerX3 && CornerX4LeftOfCornerX5
            && CornerX5LeftOfCornerX6 && !CornerX6LeftOfCornerX7){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] < Location[1]) 
           && (YBorderLine[3] > Location[1]) && (YBorderLine[3] > Location[1]) 
           && (YBorderLine[4] > Location[1]) && (YBorderLine[5] < Location[1]) 
             && (YBorderLine[6] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    //three
    else if(CornerX2LeftOfCornerX1 && CornerX3LeftOfCornerX2
            && CornerX4LeftOfCornerX3
            && CornerX5LeftOfCornerX6 && !CornerX6LeftOfCornerX7){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] < Location[1]) 
           && (YBorderLine[3] < Location[1]) && (YBorderLine[3] > Location[1]) 
           && (YBorderLine[4] > Location[1]) && (YBorderLine[5] < Location[1]) 
             && (YBorderLine[6] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    //four
    else if(CornerX2LeftOfCornerX1 && CornerX3LeftOfCornerX2
            && !CornerX4LeftOfCornerX5
            && !CornerX5LeftOfCornerX6 && !CornerX6LeftOfCornerX7){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] < Location[1]) 
           && (YBorderLine[3] < Location[1]) && (YBorderLine[3] > Location[1]) 
           && (YBorderLine[4] > Location[1]) && (YBorderLine[5] < Location[1]) 
             && (YBorderLine[6] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    //five
    else if(CornerX2LeftOfCornerX1 && CornerX3LeftOfCornerX2
            && !CornerX4LeftOfCornerX3 && CornerX4LeftOfCornerX5
            && !CornerX5LeftOfCornerX6 && !CornerX6LeftOfCornerX7){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] < Location[1]) 
           && (YBorderLine[3] > Location[1]) && (YBorderLine[3] > Location[1]) 
           && (YBorderLine[4] < Location[1]) && (YBorderLine[5] < Location[1]) 
             && (YBorderLine[6] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    //six
    else if(CornerX2LeftOfCornerX1 && CornerX3LeftOfCornerX2
            && !CornerX4LeftOfCornerX3 && CornerX4LeftOfCornerX5
            && CornerX5LeftOfCornerX6 && CornerX6LeftOfCornerX7){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] < Location[1]) 
           && (YBorderLine[3] > Location[1]) && (YBorderLine[3] > Location[1]) 
           && (YBorderLine[4] > Location[1]) && (YBorderLine[5] > Location[1]) 
             && (YBorderLine[6] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    //seven
    else if(CornerX2LeftOfCornerX1 && CornerX3LeftOfCornerX2
            && CornerX4LeftOfCornerX3 
            && CornerX5LeftOfCornerX6 && CornerX6LeftOfCornerX7){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] < Location[1]) 
           && (YBorderLine[3] < Location[1]) && (YBorderLine[3] > Location[1]) 
           && (YBorderLine[4] > Location[1]) && (YBorderLine[5] > Location[1])
             && (YBorderLine[6] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    //eight
    else if(!CornerX2LeftOfCornerX1 && !CornerX3LeftOfCornerX2
            && !CornerX4LeftOfCornerX5
            && !CornerX5LeftOfCornerX6 && !CornerX6LeftOfCornerX7){
        if((YBorderLine[0] > Location[1]) && (YBorderLine[1] > Location[1]) 
           && (YBorderLine[3] > Location[1]) && (YBorderLine[3] < Location[1]) 
           && (YBorderLine[4] < Location[1]) && (YBorderLine[5] < Location[1])
             && (YBorderLine[6] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    //nine
    else if(!CornerX2LeftOfCornerX1 && !CornerX3LeftOfCornerX2
            && !CornerX4LeftOfCornerX3 && CornerX4LeftOfCornerX5
            && !CornerX5LeftOfCornerX6 && !CornerX6LeftOfCornerX7){
        if((YBorderLine[0] > Location[1]) && (YBorderLine[1] > Location[1]) 
           && (YBorderLine[3] > Location[1]) && (YBorderLine[3] > Location[1]) 
           && (YBorderLine[4] < Location[1]) && (YBorderLine[5] < Location[1]) 
             && (YBorderLine[6] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    //ten
    else if(CornerX2LeftOfCornerX1 && !CornerX3LeftOfCornerX2
            && !CornerX4LeftOfCornerX5
            && !CornerX5LeftOfCornerX6 && !CornerX6LeftOfCornerX7){
        if((YBorderLine[0] < Location[1]) &&  (YBorderLine[1] > Location[1]) 
           && (YBorderLine[3] > Location[1]) && (YBorderLine[3] < Location[1]) 
           && (YBorderLine[4] < Location[1]) && (YBorderLine[5] < Location[1]) 
             && (YBorderLine[6] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    //eleven
    else if(CornerX2LeftOfCornerX1 && CornerX3LeftOfCornerX2
            && CornerX4LeftOfCornerX3
            && !CornerX5LeftOfCornerX6 && !CornerX6LeftOfCornerX7){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] < Location[1]) 
           && (YBorderLine[3] < Location[1]) && (YBorderLine[3] > Location[1]) 
           && (YBorderLine[4] < Location[1]) && (YBorderLine[5] < Location[1]) 
             && (YBorderLine[6] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    
    ClearOrientationVariables();
}

void CheckIfInsideOfBorderEightSides(void){
    //calculate border lines
    for(i=0;i<=7;i++){
        YBorderLine[i] = BorderSlope[i]*Location[0] + YIntercept[i];
    }

    //check where corners are in relation to each other
    if(CornerX[1] < CornerX[0]){
        CornerX2LeftOfCornerX1 = 1;
    }
    if(CornerX[2] < CornerX[1]){
        CornerX3LeftOfCornerX2 = 1;
    }
    if(CornerX[3] < CornerX[2]){
        CornerX4LeftOfCornerX3 = 1;
    }
    if(CornerX[4] < CornerX[5]){
        CornerX5LeftOfCornerX6 = 1;
    }
    if(CornerX[5] < CornerX[6]){
        CornerX6LeftOfCornerX7 = 1;
    }
    if(CornerX[6] < CornerX[7]){
        CornerX7LeftOfCornerX8 = 1;
    }
    
    //check if Location is within the border
    //one
    if(CornerX2LeftOfCornerX1 && CornerX3LeftOfCornerX2
           && !CornerX4LeftOfCornerX3 && CornerX5LeftOfCornerX6
           && !CornerX6LeftOfCornerX7 && !CornerX7LeftOfCornerX8){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] < Location[1]) 
           && (YBorderLine[2] > Location[1]) && (YBorderLine[3] > Location[1]) 
         &&(YBorderLine[4] > Location[1]) && (YBorderLine[5] < Location[1]) &&
           (YBorderLine[6] < Location[1]) && (YBorderLine[7] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    //two
    else if(CornerX2LeftOfCornerX1 && CornerX3LeftOfCornerX2
           && CornerX4LeftOfCornerX3 && CornerX5LeftOfCornerX6
           && !CornerX6LeftOfCornerX7 && !CornerX7LeftOfCornerX8){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] < Location[1]) 
           && (YBorderLine[3] < Location[1]) && (YBorderLine[3] > Location[1]) 
         &&(YBorderLine[4] > Location[1]) && (YBorderLine[5] < Location[1]) &&
           (YBorderLine[6] < Location[1]) && (YBorderLine[7] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    //three
    else if(CornerX2LeftOfCornerX1 && CornerX3LeftOfCornerX2
           && !CornerX4LeftOfCornerX3 && !CornerX5LeftOfCornerX6
           && !CornerX6LeftOfCornerX7 && !CornerX7LeftOfCornerX8){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] < Location[1]) 
           && (YBorderLine[3] > Location[1]) && (YBorderLine[3] > Location[1]) 
         &&(YBorderLine[4] < Location[1]) && (YBorderLine[5] < Location[1]) &&
           (YBorderLine[6] < Location[1]) && (YBorderLine[7] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    //four
    else if(CornerX2LeftOfCornerX1 && CornerX3LeftOfCornerX2
           && CornerX4LeftOfCornerX3 && !CornerX5LeftOfCornerX6
           && !CornerX6LeftOfCornerX7 && !CornerX7LeftOfCornerX8){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] < Location[1]) 
           && (YBorderLine[3] < Location[1]) && (YBorderLine[3] > Location[1]) 
         &&(YBorderLine[4] < Location[1]) && (YBorderLine[5] < Location[1]) &&
           (YBorderLine[6] < Location[1]) && (YBorderLine[7] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    //five
    else if(CornerX2LeftOfCornerX1 && !CornerX3LeftOfCornerX2
           && !CornerX4LeftOfCornerX3 && CornerX5LeftOfCornerX6
           && CornerX6LeftOfCornerX7 && !CornerX7LeftOfCornerX8){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] > Location[1]) 
           && (YBorderLine[3] > Location[1]) && (YBorderLine[3] > Location[1])
         &&(YBorderLine[4] > Location[1]) && (YBorderLine[5] > Location[1]) &&
           (YBorderLine[6] < Location[1]) && (YBorderLine[7] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    //six
    else if(CornerX2LeftOfCornerX1 && CornerX3LeftOfCornerX2
            && !CornerX4LeftOfCornerX3 && CornerX5LeftOfCornerX6
            && CornerX6LeftOfCornerX7 && !CornerX7LeftOfCornerX8){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] < Location[1]) 
           && (YBorderLine[3] > Location[1]) && (YBorderLine[3] > Location[1])
         &&(YBorderLine[4] > Location[1]) && (YBorderLine[5] > Location[1]) &&
           (YBorderLine[6] < Location[1]) && (YBorderLine[7] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    //seven
    else if(CornerX2LeftOfCornerX1 && CornerX3LeftOfCornerX2
            && CornerX4LeftOfCornerX3 && CornerX5LeftOfCornerX6
            && CornerX6LeftOfCornerX7 && !CornerX7LeftOfCornerX8){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] < Location[1]) 
           && (YBorderLine[3] < Location[1]) && (YBorderLine[3] > Location[1]) 
         &&(YBorderLine[4] > Location[1]) && (YBorderLine[5] > Location[1]) &&
           (YBorderLine[6] < Location[1]) && (YBorderLine[7] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    //eight
    else if(CornerX2LeftOfCornerX1 && CornerX3LeftOfCornerX2
            && CornerX4LeftOfCornerX3 && CornerX5LeftOfCornerX6
            && CornerX6LeftOfCornerX7 && CornerX7LeftOfCornerX8){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] < Location[1]) 
           && (YBorderLine[3] < Location[1]) && (YBorderLine[3] > Location[1]) 
         &&(YBorderLine[4] > Location[1]) && (YBorderLine[5] > Location[1]) &&
           (YBorderLine[6] > Location[1]) && (YBorderLine[7] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    //nine
    else if(!CornerX2LeftOfCornerX1 && !CornerX3LeftOfCornerX2
           && !CornerX4LeftOfCornerX3 && !CornerX5LeftOfCornerX6
           && !CornerX6LeftOfCornerX7 && !CornerX7LeftOfCornerX8){
        if((YBorderLine[0] > Location[1]) && (YBorderLine[1] > Location[1]) 
           && (YBorderLine[3] > Location[1]) && (YBorderLine[3] > Location[1])
         &&(YBorderLine[4] < Location[1]) && (YBorderLine[5] < Location[1]) &&  
           (YBorderLine[6] < Location[1]) && (YBorderLine[7] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    //ten
    else if(!CornerX2LeftOfCornerX1 && !CornerX3LeftOfCornerX2
            && !CornerX4LeftOfCornerX3 && CornerX5LeftOfCornerX6
            && CornerX6LeftOfCornerX7 && !CornerX7LeftOfCornerX8){
        if((YBorderLine[0] > Location[1]) && (YBorderLine[1] > Location[1]) 
           && (YBorderLine[3] > Location[1]) && (YBorderLine[3] > Location[1]) 
         &&(YBorderLine[4] > Location[1]) && (YBorderLine[5] > Location[1]) &&
           (YBorderLine[6] < Location[1]) && (YBorderLine[7] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    //eleven
    else if(CornerX2LeftOfCornerX1 && !CornerX3LeftOfCornerX2
            && !CornerX4LeftOfCornerX3 && CornerX5LeftOfCornerX6
            && CornerX6LeftOfCornerX7 && CornerX7LeftOfCornerX8){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] > Location[1]) 
           && (YBorderLine[3] > Location[1]) && (YBorderLine[3] > Location[1]) 
         &&(YBorderLine[4] > Location[1]) && (YBorderLine[5] > Location[1]) &&
           (YBorderLine[6] > Location[1]) && (YBorderLine[7] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    //twelve
    else if(!CornerX2LeftOfCornerX1 && !CornerX3LeftOfCornerX2
            && !CornerX4LeftOfCornerX3 && CornerX5LeftOfCornerX6
            && CornerX6LeftOfCornerX7 && CornerX7LeftOfCornerX8){
        if((YBorderLine[0] > Location[1]) && (YBorderLine[1] > Location[1]) 
           && (YBorderLine[3] > Location[1]) && (YBorderLine[3] > Location[1]) 
         &&(YBorderLine[4] > Location[1]) && (YBorderLine[5] > Location[1]) &&
           (YBorderLine[6] > Location[1]) && (YBorderLine[7] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    //thirteen
    else if(CornerX2LeftOfCornerX1 && !CornerX3LeftOfCornerX2
           && !CornerX4LeftOfCornerX3 && !CornerX5LeftOfCornerX6
           && !CornerX6LeftOfCornerX7 && !CornerX7LeftOfCornerX8){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] > Location[1]) 
           && (YBorderLine[3] > Location[1]) && (YBorderLine[3] > Location[1])
         &&(YBorderLine[4] < Location[1]) && (YBorderLine[5] < Location[1]) &&
           (YBorderLine[6] < Location[1]) && (YBorderLine[7] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    //fourteen
    else if(CornerX2LeftOfCornerX1 && !CornerX3LeftOfCornerX2
           && !CornerX4LeftOfCornerX3 && CornerX5LeftOfCornerX6
           && !CornerX6LeftOfCornerX7 && !CornerX7LeftOfCornerX8){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] > Location[1]) 
          && (YBorderLine[3] > Location[1]) && (YBorderLine[3] > Location[1]) 
        &&(YBorderLine[4] > Location[1]) && (YBorderLine[5] < Location[1]) &&
          (YBorderLine[6] < Location[1]) && (YBorderLine[7] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    //fifteen
    else if(CornerX2LeftOfCornerX1 && CornerX3LeftOfCornerX2
            && !CornerX4LeftOfCornerX3 && CornerX5LeftOfCornerX6
            && CornerX6LeftOfCornerX7 && CornerX7LeftOfCornerX8){
        if((YBorderLine[0] < Location[1]) && (YBorderLine[1] < Location[1]) 
          && (YBorderLine[3] > Location[1]) && (YBorderLine[3] > Location[1]) 
        &&(YBorderLine[4] > Location[1]) && (YBorderLine[5] > Location[1]) &&
          (YBorderLine[6] > Location[1]) && (YBorderLine[7] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    //sixteen
    else if(!CornerX2LeftOfCornerX1 && !CornerX3LeftOfCornerX2
           && !CornerX4LeftOfCornerX3 && CornerX5LeftOfCornerX6
           && !CornerX6LeftOfCornerX7 && !CornerX7LeftOfCornerX8){
        if((YBorderLine[0] > Location[1]) && (YBorderLine[1] > Location[1]) 
           && (YBorderLine[3] > Location[1]) && (YBorderLine[3] > Location[1])
         &&(YBorderLine[4] > Location[1]) && (YBorderLine[5] < Location[1]) &&
           (YBorderLine[6] < Location[1]) && (YBorderLine[7] < Location[1])){
            InsideBorder = 1;
        }
        else{
            InsideBorder = 0;
        }
    }
    
    ClearOrientationVariables();
}

void ClearINT2Flags(void){
    INT2IF = 0;//clear INT2 flag
}

void ClearCosines(void){
    for(i = 0;i <= 3;i++){
        CosineTheta[i] = 0;
    }
}

void ClearDistances(void){
    DistanceToMaster = 0;
    DistanceToMaster_lint = 0;
    DistanceToSlave1 = 0;
    DistanceToSlave1_lint = 0;
    DistanceToSlave2 = 0;
    DistanceToSlave2_lint = 0;
}

void ClearOrientationVariables(void){
CornerX5LeftOfCornerX6 = 0;
CornerX4LeftOfCornerX5 = 0;
CornerX4LeftOfCornerX3 = 0;
CornerX7LeftOfCornerX8 = 0;
CornerX6LeftOfCornerX7 = 0;
CornerX3LeftOfCornerX4 = 0;
CornerX3LeftOfCornerX2 = 0;
CornerX2LeftOfCornerX1 = 0;
CornerX2RightOfCornerX3 = 0;
}

void ClearPlaceholderVariables(void){
    Denominator = 0;
    Numerator = 0;
    Numerator_double = 0;
    Denominator_double = 0;
    Exponent = 0;
}

void ClearReadings(void){
    ReadFromMaster = 0;
    ReadFromSlave1 = 0;
    ReadFromSlave2 = 0;
    ReadFromMaster_double = 0;
    ReadFromSlave1_double = 0;
    ReadFromSlave2_double = 0;
    ThreeReadingsExist = 0;
}

void ClearReadingsBeingAveraged(void){
    for(i=0;i<4;i++){
        Read[i] = 0;
    }
}

void ClearTMR0Flags(void){
    TMR0 = -9766;//reset timer
    TMR0IF = 0;//clear Timer 0 flag
}

void ConvertLocationToFeet(void){
    //convert to feet
    Location_feet[0] = Location[0]*METER_TO_FOOT_CONVERSION_RATIO;
    Location_feet[1] = Location[1]*METER_TO_FOOT_CONVERSION_RATIO;
   
    //convert to int for output
    Location_feet_int[0] = (int)Location_feet[0];
    Location_feet_int[1] = (int)Location_feet[1];
}

void ConvertPowerReceivedTodB(void){
    //Subtract 30 to convert from dBm to dB
    PowerFromMaster = PowerFromMaster - 30;
    PowerFromSlave1 = PowerFromSlave1 - 30;
    PowerFromSlave2 = PowerFromSlave2 - 30;
}

void FindDistanceBetweenPosts(void){
    do{
        CheckIfThreeReadingsExist();
        if(ThreeReadingsExist){
            FindDistanceToPosts();
            DistanceMS1 = DistanceToSlave1_lint;
            DistanceMS2 = DistanceToSlave2_lint;
            DistanceBetweenPostsFound = 1;
            ClearReadings();
        }
    }while(!DistanceBetweenPostsFound);
}

void FindDistanceToPosts(void){
    CalculatePowerRecievedFromPosts();
    ClearDistances();

    //Distances calculated using friis equation
    //calculate distance to master
    ClearPlaceholderVariables();
    Numerator_double = PowerFromMaster - TRANSMISSION_POWER - 2*ANTENNA_GAIN;
    Exponent = Numerator_double*0.05;//multiply by .05 for faster divide by 20
    Denominator_double = 4*PI*pow(10,Exponent);
    DistanceToMaster = WAVELENGTH/Denominator_double;

    //calculate distance to slave 1
    ClearPlaceholderVariables();
    Numerator_double = PowerFromSlave1 - TRANSMISSION_POWER - 2*ANTENNA_GAIN;
    Exponent = Numerator_double*0.05;//multiply by .05 for faster divide by 20
    Denominator_double = 4*PI*pow(10,Exponent);
    DistanceToSlave1 = WAVELENGTH/Denominator_double;

    //calculate distance to slave 2
    ClearPlaceholderVariables();
    Numerator_double = PowerFromSlave2 - TRANSMISSION_POWER - 2*ANTENNA_GAIN;
    Exponent = Numerator_double*0.05;//multiply by .05 for faster divide by 20
    Denominator_double = 4*PI*pow(10,Exponent);
    DistanceToSlave2 = WAVELENGTH/Denominator_double;

    //convert to long ints for faster calculations in CalculateCosines
    DistanceToMaster_lint = (long int)DistanceToMaster;
    DistanceToSlave1_lint = (long int)DistanceToSlave1;
    DistanceToSlave2_lint = (long int)DistanceToSlave2;
}

void FindLocation(void){
    FindDistanceToPosts();
    CalculateCosines();

    //Calculate Location[0], x
    //calculate magnitude
    Location[0] = DistanceToMaster*AbsoluteValue(CosineTheta[0]);
    //calculate sign
    if((DistanceToSlave1*AbsoluteValue(CosineTheta[1])) > DistanceMS1){
        Location[0] = -Location[0];
    }

    //Calculate Location[1], y
    //calculate magnitude
    Location[1] = DistanceToMaster*AbsoluteValue(CosineTheta[3]);
    //calculate sign
    if((DistanceToSlave2*AbsoluteValue(CosineTheta[2])) > DistanceMS2){
        Location[1] = -Location[1];
    }
    
    ConvertLocationToFeet();
}

void IndicateIfWithinBorder(void){
    if(!InsideBorder){
        TurnOnBorderViolationIndicationLight();
    }
     else{
        TurnOffBorderViolationIndicationLight();
    }
}

void InitializeInterrupts(void){
    GIE = 1;//Global interrupt enable
    PEIE = 1;//Timer interrupt enable

    //turn on rising edge interrupt on RB2 (INT2)
    INTEDG2 = 1;//Set to rising edge interrupt
    INT2IE = 1;//int2 enable 1/3
    INT2IP = 1;//int2 enable 2/3
    TRISB2 = 1;//int2 enable 3/3

    //turn on Timer 0 interrupt
    T0CS = 0;//timer 0 clock, use osc/4
    T0CON = 0x87;//Timer 0 prescaler, PS = 256
    TMR0 = -9766;//set to 1 sec for PS = 256
    TMR0ON = 1;//Timer 0 enable 1/3
    TMR0IE = 1;//Timer 0 enable 2/3
    TMR0IP = 1;//Timer 0 enable 3/3
    
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
}

void InitializePorts(void){
    //set input/output
    //0x00 is output, 0xff is input
    TRISA = 0xff;
    TRISB = 0xff;
    TRISC = 0xc0;//set RC6 and RC7 to input for uart
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

    POWER_EQUATION_SLOPE_A2D=POWER_EQUATION_SLOPE_VOLTAGE/A2D_LEVELS_PER_VOLT;
}

void OutputReadings(void){
    OutputToSerial(ReadFromMaster);
    OutputToSerial(ReadFromSlave1);
    OutputToSerial(ReadFromSlave2);
    while(!TRMT); TXREG = 13; //return to beginning of line
    while(!TRMT); TXREG = 10; //next line
}

void OutputToSerial(int DATA){
   unsigned char A[5];
   if(DATA < 0) { //output '-' if the value is negative
        while(!TRMT);
        TXREG = 45;//ascii # for -
       }
   for (i=0; i<5; i++) { //get 4 decimal places stored into array
      A[i] = (unsigned char)AbsoluteValue_int(DATA) % 10;
      DATA = AbsoluteValue_int(DATA) / 10;
      }
   for (i=5; i>0; i--) {
       while(!TRMT);
       TXREG = A[i-1] + 48; //Output ASCII number
   }

   while(!TRMT);
   TXREG = ' ';
}

void PlaceCorner(void){
    do{
        CheckIfThreeReadingsExist();
        
        if(ThreeReadingsExist){
            FindLocation();
            
            if(CornerX[0] == 0){
                CornerX[0] = Location[0];
                CornerY[0] = Location[1];
            }
            else if(CornerX[1] == 0){
                CornerX[1] = Location[0];
                CornerY[1] = Location[1];
            }
            else if(CornerX[2] == 0){
                CornerX[2] = Location[0];
                CornerY[2] = Location[1];
            }
            else if(CornerX[3] == 0){
                CornerX[3] = Location[0];
                CornerY[3] = Location[1];
            }
            else if(CornerX[4] == 0){
                CornerX[4] = Location[0];
                CornerY[4] = Location[1];
            }
            else if(CornerX[5] == 0){
                CornerX[5] = Location[0];
                CornerY[5] = Location[1];
            }
            else if(CornerX[6] == 0){
                CornerX[6] = Location[0];
                CornerY[6] = Location[1];
            }
            else if(CornerX[7] == 0){
                CornerX[7] = Location[0];
                CornerY[7] = Location[1];
            }
            CornerPlaced = 1;
        }
    }while(!CornerPlaced);
    CornerPlaced = 0;//reset for next corner placement
}

void ResetReadCounter(void){
    ReadCount = 0;
}

void ResetTMR0Countdown(void){
    TMR0 = -9766;//reset timer 0 countdown timer
}

void TakeReadings(void){
    RD1 = 1;//pin 20, debug line
    //check if all readings are taken while still receiving xmit from posts
    for(i=0;i<4;i++){
        Read[i] = A2D_Read();
    }
    RD1 = 0;//pin 20
}

void TurnOffBorderViolationIndicationLight(void){
    RD7 = 0;//turn off LED on RD7
}

void TurnOnBorderViolationIndicationLight(void){
    RD7 = 1;//turn on LED on RD7
}