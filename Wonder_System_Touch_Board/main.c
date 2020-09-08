/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//*****************************************************************************
// Development main.c for MSP430FR2633, MSP430FR2533, MSP430FR2632, and
// MSP430FR2532.
//
// This starter application initializes the CapTIvate touch library
// for the touch panel specified by CAPT_UserConfig.c/.h via a call to
// CAPT_appStart(), which initializes and calibrates all sensors in the
// application, and starts the CapTIvate interval timer.
//
// Then, the capacitive touch interface is driven by calling the CapTIvate
// application handler, CAPT_appHandler().  The application handler manages
// whether the user interface (UI) is running in full active scan mode, or
// in a low-power wake-on-proximity mode.
//
// The CapTIvate application handler will return true if proximity was
// detected on any of the sensors in the application, which is used here
// to control the state of LED2. LED1 is set while the background loop enters
// the handler, and is cleared when the background loop leaves the handler.
//
// \version 1.83.00.05
// Released on May 15, 2020
//
//*****************************************************************************

#include <msp430.h>                      // Generic MSP430 Device Include
#include "driverlib.h"                   // MSPWare Driver Library
#include "captivate.h"                   // CapTIvate Touch Software Library
#include "CAPT_App.h"                    // CapTIvate Application Code
#include "CAPT_BSP.h"                    // CapTIvate EVM Board Support Package

#define ON 1
#define OFF 0

#define HIGH 1
#define LOW 0

#define TX_n 2

#define LED_A(mode) P2OUT=(mode==0)?(P2OUT|GPIO_PIN5):(P2OUT&~GPIO_PIN5)
#define LED_B(mode) P2OUT=(mode==0)?(P2OUT|GPIO_PIN6):(P2OUT&~GPIO_PIN6)
#define LED_C(mode) P3OUT=(mode==0)?(P3OUT|GPIO_PIN0):(P3OUT&~GPIO_PIN0)
#define LED_D(mode) P1OUT=(mode==0)?(P1OUT|GPIO_PIN0):(P1OUT&~GPIO_PIN0)
#define LED_E(mode) P2OUT=(mode==0)?(P2OUT|GPIO_PIN2):(P2OUT&~GPIO_PIN2)
#define LED_T(mode) P1OUT=(mode==0)?(P1OUT|GPIO_PIN7):(P1OUT&~GPIO_PIN7)

#define LED_START(mode) P2OUT=(mode==0)?(P2OUT|GPIO_PIN0):(P2OUT&~GPIO_PIN0)
#define LED_UP(mode) P2OUT=(mode==0)?(P2OUT|GPIO_PIN1):(P2OUT&~GPIO_PIN1)
#define LED_DOWN(mode) P1OUT=(mode==0)?(P1OUT|GPIO_PIN6):(P1OUT&~GPIO_PIN6)
#define LED_AUT(mode) P2OUT=(mode==0)?(P2OUT|GPIO_PIN7):(P2OUT&~GPIO_PIN7)
#define LED_GRAY(mode) P3OUT=(mode==0)?(P3OUT|GPIO_PIN1):(P3OUT&~GPIO_PIN1)
#define LED_MODE_SWITCH(mode) P2OUT=(mode==0)?(P2OUT|GPIO_PIN3):(P2OUT&~GPIO_PIN3)

#define LOCAL_IIC_INT(vol) P1OUT=(vol==1)?(P1OUT|GPIO_PIN1):(P1OUT&~GPIO_PIN1)


enum btn{start=0,up,down,aut,gray,mode_switch,null};
enum slide{s1=0,s2,s3,s4};
enum touch_state{released=0,pressed};
enum led{led_a=0x1000,led_b=0x2000,led_c=0x0800,led_d=0x0400,led_e=0x0200,led_start=0x0020,led_up=0x0010,led_down=0x0008,led_aut=0x0004,led_gray=0x0002,led_mode_switch=0x0001};

volatile uint16_t TXData    [2]={0x00,0x00};
volatile uint16_t RXData    [2]={0x00,0x00};
volatile unsigned char RXData_Cnt=0;
volatile unsigned char TXData_Cnt=0;

extern uint16_t sliderSensor_E00_RawCnts[CAPT_SELF_FREQ_CNT];
extern uint16_t sliderSensor_E01_RawCnts[CAPT_SELF_FREQ_CNT];
extern uint16_t sliderSensor_E02_RawCnts[CAPT_SELF_FREQ_CNT];
extern uint16_t sliderSensor_E03_RawCnts[CAPT_SELF_FREQ_CNT];

extern uint16_t keypadSensor_E00_RawCnts[CAPT_SELF_FREQ_CNT];
extern uint16_t keypadSensor_E01_RawCnts[CAPT_SELF_FREQ_CNT];
extern uint16_t keypadSensor_E02_RawCnts[CAPT_SELF_FREQ_CNT];
extern uint16_t keypadSensor_E03_RawCnts[CAPT_SELF_FREQ_CNT];
extern uint16_t keypadSensor_E04_RawCnts[CAPT_SELF_FREQ_CNT];
extern uint16_t keypadSensor_E05_RawCnts[CAPT_SELF_FREQ_CNT];

uint16_t * keypadSensor_list[6]={keypadSensor_E00_RawCnts,keypadSensor_E01_RawCnts,keypadSensor_E02_RawCnts,keypadSensor_E03_RawCnts,keypadSensor_E04_RawCnts,keypadSensor_E05_RawCnts};
uint16_t * slide_list[4]={sliderSensor_E00_RawCnts,sliderSensor_E01_RawCnts,sliderSensor_E02_RawCnts,sliderSensor_E03_RawCnts};

extern tSensor sliderSensor;
extern tSensor keypadSensor;
uint16_t position=0;
uint16_t position_old=0;
uint8_t button=null;
uint16_t slide_cnt=0;
uint16_t btn_cnt=0;

uint16_t button_state[6]={released};
uint16_t button_state_old[6]={released};
uint16_t slide_state=released;
uint16_t slide_new_state=0;
uint16_t slide_old_state=0;
uint16_t led_state=0;
uint16_t led_state_old=0;

uint32_t i;
uint16_t state[14];

void initClockTo16MHz()
{
    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    __bis_SR_register(SCG0);    // disable FLL
    CSCTL3 |= SELREF__REFOCLK;  // Set REFO as FLL reference source
    CSCTL0 = 0;                 // clear DCO and MOD registers
    CSCTL1 &= ~(DCORSEL_7);     // Clear DCO frequency select bits first
    CSCTL1 |= DCORSEL_5;        // Set DCO = 16MHz
    CSCTL2 = FLLD_0 + 487;      // set to fDCOCLKDIV = (FLLN + 1)*(fFLLREFCLK/n)
                                //                   = (487 + 1)*(32.768 kHz/1)
                                //                   = 16 MHz

    __delay_cycles(3);
    __bic_SR_register(SCG0);                        // enable FLL
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));      // FLL locked

    CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;
}

void I2C_Slave_Init(){
    UCB0CTLW0 = UCSWRST;
    UCB0CTLW0 |= UCMODE_3 | UCSYNC;
    UCB0I2COA0 = 0x48 | UCOAEN;
    UCB0CTLW0 &= ~UCSWRST;                    // clear reset register
    UCB0IE |= UCRXIE | UCSTPIE;
}

void send_data(){
    if((button!=null)||(position^position_old)){
        LOCAL_IIC_INT(1);
        __delay_cycles(16);//1MHz
        LOCAL_IIC_INT(0);
        UCB0IE |= UCTXIE;
        UCB0IE &= ~UCRXIE;
    }

}

void init_led(){
    LED_A(ON);
    LED_B(ON);
    LED_C(ON);
    LED_D(ON);
    LED_E(ON);
    LED_T(ON);
    LED_START(ON);
    LED_UP(ON);
    LED_DOWN(ON);
    LED_AUT(ON);
    LED_GRAY(ON);
    LED_MODE_SWITCH(ON);
    __delay_cycles(0x800000>>1);
    LED_A(OFF);
    LED_B(OFF);
    LED_C(OFF);
    LED_D(OFF);
    LED_E(OFF);
    LED_T(OFF);
    LED_START(OFF);
    LED_UP(OFF);
    LED_DOWN(OFF);
    LED_AUT(OFF);
    LED_GRAY(OFF);
    LED_MODE_SWITCH(OFF);
}

void main(void)
{
    //
    // Initialize the MCU
    // BSP_configureMCU() sets up the device IO and clocking
    // The global interrupt enable is set to allow peripherals
    // to wake the MCU.
    //
    WDTCTL = WDTPW | WDTHOLD;
    BSP_configureMCU();
    init_led();
    initClockTo16MHz();
    I2C_Slave_Init();

    __bis_SR_register(GIE);

    //
    // Start the CapTIvate application
    //


    CAPT_appStart();

    //
    // Background Loop
    //

    while(1)
    {
        //
        // Run the captivate application handler.
        // Set LED1 while the app handler is running,
        // and set LED2 if proximity is detected
        // on any sensor.
        //
//      LED1_ON;
        if(CAPT_appHandler()==true){
//           button = 0xff;
//           if((position>200)&&(position!=0xffff)){
//           if((keypadSensor_list[start][0]<100)||(keypadSensor_list[up][0]<100)||(keypadSensor_list[down][0]<100)||(keypadSensor_list[aut][0]<100)||(keypadSensor_list[gray][0]<100)||(keypadSensor_list[mode_switch][0]<100)){
//            button = CAPT_getDominantButton(&keypadSensor);
            for(i=start;i!=null;i++){
                __no_operation();
                __no_operation();
                __no_operation();
               if(keypadSensor_list[i][0]<200){
                   button = i;
                   if(button_state[i]==released){
                       button_state[i]=pressed;
                       switch(i){
                       case start:
                           TXData[0] = 0x0020;
               //              __no_operation();
               //              __no_operation();
               //              __no_operation();
                             break;
                       case up:
                           TXData[0] = 0x0010;
                           break;
                       case down:
                           TXData[0] = 0x0008;
                           break;
                       case aut:
                           TXData[0] = 0x0004;
                           break;
                       case gray:
                           TXData[0] = 0x0002;
                           break;
                       case mode_switch:
                           TXData[0] = 0x0001;
                           break;
                       default:
                           break;
                   }
                       send_data();
                   }
               }else{
                   if(button_state[i]^button_state_old[i]){
                       TXData[0] = 0x00;
                       send_data();
                       button_state[i] = released;
                       button_state_old[i] = button_state[i];
                   }
               }
            }

               btn_cnt++;
            __delay_cycles(0x0080000>>2);
            LED1_OFF;
//            send_data();
//           }
           if(((slide_list[s1][0]<200)||(slide_list[s2][0]<200)||(slide_list[s3][0]<200)||(slide_list[s4][0]<200))){
               position = CAPT_getSensorPosition(&sliderSensor);
               if(position!=0xffff){
               slide_cnt++;
//               if(position^position_old){
//                   LOCAL_IIC_INT(1);
//                   LOCAL_IIC_INT(0);
//                   UCB0IE |= UCTXIE;
//                   UCB0IE &= ~UCRXIE;
//               }
               send_data();
               __delay_cycles(0x0080000>>2);
               LED1_OFF;
               }

//               send_data();
           }
        }
        else{
            if(button_state[start]==pressed){
                button_state[start]=released;
                send_data();
            }
            if(button_state[up]==pressed){
                button_state[up]=released;
                send_data();
            }
            if(button_state[down]==pressed){
                button_state[down]=released;
                send_data();
            }
            if(button_state[aut]==pressed){
                button_state[aut]=released;
                send_data();
            }
            if(button_state[gray]==pressed){
                button_state[gray]=released;
                send_data();
            }
            if(button_state[mode_switch]==pressed){
                button_state[mode_switch]=released;
                send_data();
            }


            LED1_ON;
        }
//      LED1_OFF;
        if(led_state^led_state_old){
            if(led_state&led_start){
                LED_START(ON);
            }else{
                LED_START(OFF);
            }
            if(led_state&led_up){
                LED_UP(ON);
            }else{
                LED_UP(OFF);
            }
            if(led_state&led_down){
                LED_DOWN(ON);
            }else{
                LED_DOWN(OFF);
            }
            if(led_state&led_aut){
                LED_AUT(ON);
            }else{
                LED_AUT(OFF);
            }
            if(led_state&led_gray){
                LED_GRAY(ON);
            }else{
                LED_GRAY(OFF);
            }
            if(led_state&led_mode_switch){
                LED_MODE_SWITCH(ON);
            }else{
                LED_MODE_SWITCH(OFF);
            }

            if(led_state&led_a){
                LED_A(ON);
            }else{
                LED_A(OFF);
            }
            if(led_state&led_b){
                LED_B(ON);
            }else{
                LED_B(OFF);
            }
            if(led_state&led_c){
                LED_C(ON);
            }else{
                LED_C(OFF);
            }
            if(led_state&led_d){
                LED_D(ON);
            }else{
                LED_D(OFF);
            }
            if(led_state&led_e){
                LED_E(ON);
            }else{
                LED_E(OFF);
            }

            led_state_old=led_state;
        }
        //
        // This is a great place to add in any
        // background application code.
        //
        __no_operation();

        //
        // End of background loop iteration
        // Go to sleep if there is nothing left to do
        //
        CAPT_appSleep();

    } // End background loop
} // End main()

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_B0_VECTOR
__interrupt void USCIB0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B0_VECTOR))) USCIB0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG))
  {
    case USCI_NONE: state[0]++;break;                  // Vector 0: No interrupts
    case USCI_I2C_UCALIFG:  state[1]++;break;           // Vector 2: ALIFG
    case USCI_I2C_UCNACKIFG:
        state[2]++;
        break;         // Vector 4: NACKIFG
    case USCI_I2C_UCSTTIFG:  state[3]++;break;          // Vector 6: STTIFG
    case USCI_I2C_UCSTPIFG:                 // Vector 8: STPIFG
      state[4]++;
      UCB0IFG &= ~(UCTXIFG0);
//      LOCAL_IIC_INT(1);
     if(RXData_Cnt==2){
         led_state&=RXData[0];
         led_state&=(RXData[1]<<8);
         led_state|=RXData[0];
         led_state|=(RXData[1]<<8);
         RXData_Cnt = 0;
     }
      break;
    case USCI_I2C_UCRXIFG3:  state[5]++;break;          // Vector 10: RXIFG3
    case USCI_I2C_UCTXIFG3:  state[6]++;break;          // Vector 14: TXIFG3
    case USCI_I2C_UCRXIFG2:  state[7]++;break;          // Vector 16: RXIFG2
    case USCI_I2C_UCTXIFG2:  state[8]++;break;          // Vector 18: TXIFG2
    case USCI_I2C_UCRXIFG1:  state[9]++;break;          // Vector 20: RXIFG1
    case USCI_I2C_UCTXIFG1:  state[10]++;break;          // Vector 22: TXIFG1
    case USCI_I2C_UCRXIFG0:                 // Vector 24: RXIFG0
           RXData[RXData_Cnt] = UCB0RXBUF;                            // Get RX data
           RXData_Cnt++;
           UCB0IE |= UCRXIE;
           UCB0IE &= ~UCTXIE;
           state[11]++;
           break;
//        }
    case USCI_I2C_UCTXIFG0:
//        switch(button){
//        case start:
////            TXData[0] = 0x0020;
////              __no_operation();
////              __no_operation();
////              __no_operation();
//              break;
//        case up:
//            TXData[0] = 0x0010;
//            break;
//        case down:
//            TXData[0] = 0x0008;
//            break;
//        case aut:
//            TXData[0] = 0x0004;
//            break;
//        case gray:
//            TXData[0] = 0x0002;
//            break;
//        case mode_switch:
//            TXData[0] = 0x0001;
//            break;
//        default:
//            break;
//    }
       button=null;
       if(position_old ^ position){
           TXData[1] = position;
           position_old = position;
       }
//       if((position_old == position) && (button==null)){
//       UCB0IE |= UCRXIE;
//       UCB0IE &= ~UCTXIE;
//       LOCAL_IIC_INT(1);
//       }
       if(TXData_Cnt<TX_n){
           UCB0TXBUF = TXData[TXData_Cnt];
           TXData_Cnt++;
       }else{
           UCB0IE |= UCRXIE;
           UCB0IE &= ~UCTXIE;
           TXData_Cnt = 0;
//           LOCAL_IIC_INT(1);
       }
       state[12]++;
       break;                               // Vector 26: TXIFG0
//    case USCI_I2C_UCBCNTIFG: break;         // Vector 28: BCNTIFG
//    case USCI_I2C_UCCLTOIFG: break;         // Vector 30: clock low timeout
//    case USCI_I2C_UCBIT9IFG: break;         // Vector 32: 9th bit
    default:  state[13]++;break;
  }
}
