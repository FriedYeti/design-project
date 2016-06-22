/* 
 * File:   Utilities.h
 * Author: carlbaumann
 *
 * Created on April 22, 2016, 8:25 AM
 */

#ifndef UTILITIES_H
#define	UTILITIES_H

/**********Utility Types/Constants***********/

#define false 0
#define true 1

typedef struct {
    int masterValue;
    int slave1Value;
    int slave2Value;
} triPingData;

typedef int bool;


/**************Utility Functions****************/
void OutputToSerial(int data);
int AbsoluteValue_int(int num);
unsigned int A2D_Read(void);


void OutputToSerial(int data){
   unsigned char A[5];
   if(data < 0) { //output '-' if the value is negative
        while(!TRMT);
        TXREG = 45;
       }
   for (int i=0; i<5; i++) { //get 4 decimal places stored into array
      A[i] = (unsigned char)AbsoluteValue_int(data) % 10;
      data = AbsoluteValue_int(data) / 10;
      }
   for (int i=5; i>0; i--) {
       while(!TRMT);
       TXREG = A[i-1] + 48; //Output ASCII number
   }

   while(!TRMT);
   TXREG = ' ';
   return;
}

int AbsoluteValue_int(int num){
    if (num < 0){
        return -num;
    }
    else{
        return num;
    }
}

// reads and returns the value from the A2D converter
unsigned int A2D_Read(void) {
    ADCON0 = 0x01; //select channel 0 for a2d reading
    GODONE = 1; //start conversions
    while (GODONE); //wait until done (approx 8us)
    return (ADRES);
}


#endif	/* UTILITIES_H */

