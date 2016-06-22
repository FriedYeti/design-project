// Tripost-Slave_one
// Ping after master
#include<pic18.h>

// globals
char PingCounter = 0;
char WaitResetCounter = 0;
char TMR0WaitFlag = 1;
char i = 0;

// constants
const char SLAVE_NUMBER = 1;

// declare subroutine
void InitializePorts(void);
void InitializeInterrupts(void);
void InitializeTransceiver(void);
void SetToReceive(void);
void SetToTransmit(void);
void ClearINT2Flags(void);
void ClearTMR0Flags(void);
void TransmitSignal(void);
void TMR0CountdownReset(void);

// isr
void interrupt IntServe(void){
    if(INT2IF)
    {
        PingCounter++;
        WaitResetCounter = 0;
        ClearINT2Flags();
    }
    if(TMR0IF){
        TMR0WaitFlag = 0;
        WaitResetCounter++;
        if(WaitResetCounter >= 5){
            WaitResetCounter = 0;
            PingCounter = 0;
        }
        ClearTMR0Flags();
    }
}

//main
void main(void){
    InitializePorts();
    InitializeInterrupts();
    InitializeTransceiver();
    while(1){ // main loop
        if(PingCounter == 1) { RD2 = 1; }
        else {RD2 = 0; }

        if(PingCounter == 2) { RD3 = 1; }
        else { RD3 = 0; }

        if(PingCounter == SLAVE_NUMBER){
            SetToTransmit();

            // wait for rest of input signal
            TMR0CountdownReset();
            TMR0WaitFlag = 1;
            while(TMR0WaitFlag);

            // wait extra 1/8th of second to send
            TMR0WaitFlag = 1;
            while(TMR0WaitFlag);

            TransmitSignal();
            SetToReceive();
            for(i = 0;i<4;i++){
                TMR0WaitFlag = 1;
                while(TMR0WaitFlag);
            }
        }
    }
}

//subroutines
void InitializePorts(void){
    //Set input/output
    //0 is output 255 is input
    TRISA = 0;
    TRISB = 255;
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

void InitializeInterrupts(void){
    GIE = 1;//Global interupt enable
    PEIE = 1;//timer interrupt enable

    //turn on rising edge interrupt on RB2 (INT2)
    INTEDG2 = 1;//Set to rising edge interrupt
    INT2IE = 1;//int2 enable 1/3
    INT2IP = 1;//int2 enable 2/3
    TRISB2 = 1;//int2 enable 3/3

    //turn on timer 0
    TMR0ON = 1;//timer 0 enable 1/3
    TMR0IP = 1;//timer 0 enable 2/3
    TMR0IE = 1;//timer 0 enable 3/3
    T0CS = 0;//use osc/4
    T0CON = 0x85;//timer 0 prescaler, set to 64
    TMR0 = -4883;//set to 1/8 sec for prescaler 64
}

void SetToReceive(void){
    RD6 = 0;//set transceiver to receive mode
    RA1 = 1;//turn on switch on line c
}

void SetToTransmit(void){
    RA1 = 0;//turn off switch on line c
    RD6 = 1;//set transceiver to transmit mode
}

void InitializeTransceiver(void){
    SetToReceive();
}

void ClearINT2Flags(void){
    INT2IF = 0;
}

void ClearTMR0Flags(void){
    TMR0IF = 0;
    TMR0 = -4883;
}

void TMR0CountdownReset(void){
    TMR0 = -4883;
}

void TransmitSignal(void){
    RD7 = 1;
    TMR0CountdownReset();
    TMR0WaitFlag = 1;
    while(TMR0WaitFlag);
    RD7 = 0;
}