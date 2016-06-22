//Tripost-master_post
//Ping at constant timed rate
#include<pic18.h>

//globals
char InterruptCounter = 0;


//declare subroutine
void InitializePorts(void);
void InitializeISR(void);
void InitializeTransceiver(void);
void SetTransceiverSignal(void);
void ClearTransceiverSignal(void);
void IncrementInterruptCounter(void);
void ClearTMR0Flags(void);
void SetToTransmit(void);

//isr
void interrupt IntServe(void){
    if(TMR0IF){

        if(InterruptCounter == 0){
            SetTransceiverSignal();
        }
        else if(InterruptCounter == 1){
            ClearTransceiverSignal();
        }

        IncrementInterruptCounter();

        ClearTMR0Flags();
    }
}


//main
void main(void){
    InitializePorts();
    InitializeISR();
    InitializeTransceiver();
    while(1){
        //Do nothing - Wait for interrupts
    }
}

//subroutines

void InitializePorts(void){
    //Set input/output
    //0 is output, 255 is input
    TRISA = 0;
    TRISB = 0;
    TRISC = 0;
    TRISD = 0;
    TRISE = 0;

    //Clear ports
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    PORTE = 0;

    //Set ports to digital
    ADCON0 = 0x00; // Analog disabled
    ADCON1 = 0x0F; // All input binary
    ADCON2 = 0x87; // ADC Clock Settings (not used)
}

void InitializeISR(void){
    //Enable bits for timer 0
    TMR0ON = 1;//timer 0 enable 1/3
    TMR0IE = 1;//timer 0 enable 2/3
    TMR0IP = 1;//timer 0 enable 3/3
    PEIE = 1;//timer interrupt enable
    GIE = 1;//global interrupt enable

    T0CS = 0; //Timer Oscillator Select, run off crystal
    T0CON = 0x85; //timer 0 prescaler, PS = 64
    TMR0 = -4883; //Set to 1/8th of a second for prescaler 64
}

void InitializeTransceiver(void){
    SetToTransmit();
}

void SetTransceiverSignal(void){
    RD7 = 1; //Turn on tranceiver's signal
}

void ClearTransceiverSignal(void){
    RD7 = 0; //Turn off tranceiver's signal
}

void IncrementInterruptCounter(void){
    InterruptCounter =  (InterruptCounter+1) & 0x0f; //Increment, reset at 16
}

void ClearTMR0Flags(void){
    TMR0IF = 0; //Clear timer flag
    TMR0 = -4883; //reset timer
}

void SetToTransmit(void){
    RD6 = 1; //Set tranceiver to transmit mode
}