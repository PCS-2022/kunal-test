// PIC16F1509 Configuration Bit Settings

// 'C' source line config statements
/*

 * TRISC=0b00001111
 * TRISA=0b00001011

 * TRISB=0b10101111
 */
// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover Mode (Internal/External Switchover Mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <xc.h>




//#include <xc.h>
//#include <pic18f46k22.h>
#define hi 1
#define lo 0
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <pic16f1508.h>
#include	<stdio.h>
#define _XTAL_FREQ  16000000

#define  addpin8 PORTCbits.RC5
#define  addpin7 PORTCbits.RC4
#define  addpin6 PORTCbits.RC3
#define  addpin5 PORTCbits.RC2
#define  addpin4 PORTCbits.RC1
#define  addpin3 PORTCbits.RC0
#define  addpin2 PORTAbits.RA4
#define  addpin1 PORTAbits.RA5




#define en_485 LATCbits.LATC7

union {

    struct {
        unsigned char bit0 : 1; //
        unsigned char bit1 : 1; //
        unsigned char bit2 : 1; //
        unsigned char bit3 : 1; //
        unsigned char bit4 : 1; //
        unsigned char bit5 : 1; //
        unsigned char bit6 : 1; //
        unsigned char bit7 : 1; //
    } charu1;
    unsigned char charu8;

} char2bit;
unsigned char bit15;
unsigned char bit15;

union {

    struct {
        unsigned char lsb;
        unsigned char msb;

    } charu8;
    unsigned int intu16;

} inttochar;

union {

    struct {
        unsigned char byte0;
        unsigned char byte1;
        unsigned char byte2;
        unsigned char byte3;
    } charu8;
    unsigned long int intu32;

} int32tou8;

struct {
    unsigned char tim_int_ind : 1; //
    unsigned char rxdone : 1; //
    unsigned char rxcrccorrect : 1; //
    unsigned char togg : 1; //
    unsigned char opto2 : 1; //
    unsigned char opto3 : 1; //
    unsigned char opto4 : 1; //
    unsigned char blkcomm : 1; //
    unsigned char rxexist : 1; //
    unsigned char bit9 : 1; //
    unsigned char bit10 : 1; //
    unsigned char bit11 : 1; //
    unsigned char bit12 : 1; // 
    unsigned char bit13 : 1; //
    unsigned char bit14 : 1; //
    unsigned char bit15 : 1; //

} cond;

struct {
    unsigned char nodisp : 1; //
    unsigned char rlyoff : 1; //
    unsigned char bit2 : 1; //
    unsigned char bit3 : 1; //
    unsigned char bit4 : 1; //
    unsigned char bit5 : 1; //
    unsigned char bit6 : 1; //
    unsigned char bit7 : 1; //
} once;






unsigned int cnt, cnt1, cnt2,actval;

unsigned char i2c_add, chk, lastadd, laststat, rxcnt, recvcnt, addval, ipval, data2sendcnt, i2cdelaycnt,datavalidval,adchconfig,adch1cnt,adch2cnt,adchval;
unsigned volatile char int1cnt, rx_cnt, rx_data, txchkcnt, rd1cnt, rd2cnt, txlen2, txlen3, txcnt;
unsigned char batcnt;

unsigned volatile int intcnt, logtim;
unsigned char *condstr;

unsigned long int wdtcnt,longactval,adch1val,adch2val,avgadch1val,avgadch2val,addadch1val,addadch2val;


unsigned char rxrecvarr[10] = {0};
unsigned char rxdataarr[10] = {0};
unsigned char txcommstr[10] = {0};
unsigned char tx1str[20] = {0};

//unsigned char rd1str[20] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0xC4, 0x0B};

//unsigned char rd2str[] = {0xff,0x04,0x04,0x05};
//unsigned char tempdata[20];
//unsigned char i2cdata[10];
//#define irqPin

void I2CInit(void) {
    TRISB4 = 1; /* SDA and SCL as input pin */
    TRISB6 = 1; /* these pins can be configured either i/p or o/p */
    SSP1STAT |= 0x80; /* Slew rate disabled */
    SSP1CON1 = 0x28; /* SSPEN = 1, I2C Master mode, clock = FOSC/(4 * (SSPADD + 1)) */
    SSP1ADD = 199; /* 10Khz @ 8Mhz Fosc */
}

void I2CStart() {
    SSP1CON2bits.SEN = 1; /* Start condition enabled */
    i2cdelaycnt = 0;
    while (SSP1CON2bits.SEN) /* automatically cleared by hardware */ {
        i2cdelaycnt++;
        __delay_us(50);
        if (i2cdelaycnt > 100) {
            i2cdelaycnt = 0;
            break;
        }
    }
    /* wait for start condition to finish */
}

void I2CStop() {
    SSP1CON2bits.PEN = 1; /* Stop condition enabled */
    i2cdelaycnt = 0;
    while (SSP1CON2bits.PEN) /* Wait for stop condition to finish */ {
        i2cdelaycnt++;
        __delay_us(50);
        if (i2cdelaycnt > 100) {
            i2cdelaycnt = 0;
            break;
        }
    }
    /* PEN automatically cleared by hardware */
}

void I2CRestart() {
    SSP1CON2bits.RSEN = 1; /* Repeated start enabled */
    i2cdelaycnt = 0;
    while (SSP1CON2bits.RSEN) /* wait for condition to finish */ {
        i2cdelaycnt++;
        __delay_us(50);
        if (i2cdelaycnt > 100) {
            i2cdelaycnt = 0;
            break;
        }
    }
}

void I2CAck() {
    SSP1CON2bits.ACKDT = 0; /* Acknowledge data bit, 0 = ACK */
    SSP1CON2bits.ACKEN = 1; /* Ack data enabled */
    i2cdelaycnt = 0;
    while (SSP1CON2bits.ACKEN) /* wait for ack data to send on bus */ {
        i2cdelaycnt++;
        __delay_us(50);
        if (i2cdelaycnt > 100) {
            i2cdelaycnt = 0;
            break;
        }
    }
}

void I2CNak() {
    SSP1CON2bits.ACKDT = 1; /* Acknowledge data bit, 1 = NAK */
    SSP1CON2bits.ACKEN = 1; /* Ack data enabled */
    i2cdelaycnt = 0;
    while (SSP1CON2bits.ACKEN) /* wait for ack data to send on bus */ {
        i2cdelaycnt++;
        __delay_us(50);
        if (i2cdelaycnt > 100) {
            i2cdelaycnt = 0;
            break;
        }
    }
}

void I2CWait() {
    i2cdelaycnt = 0;
    while (((SSP1CON2 & 0x1F) == 0x1f) || ((SSP1STAT & 0x04) == 0x04)) {
        i2cdelaycnt++;
        __delay_us(100);
        if (i2cdelaycnt > 100) {
            i2cdelaycnt = 0;
            break;
        }
    }

    /* wait for any pending transfer */
}

void I2CSend(unsigned char dat) {
    SSP1BUF = dat; /* Move data to SSPBUF */
    i2cdelaycnt = 0;
    while (SSP1STATbits.BF) /* wait till complete data is sent from buffer */ {
        i2cdelaycnt++;
        __delay_us(100);
        if (i2cdelaycnt > 100) {
            i2cdelaycnt = 0;
            break;
        }
    }
    I2CWait(); /* wait for any pending transfer */
}

unsigned char I2CRead(void) {
    unsigned char temp;
    /* Reception works if transfer is initiated in read mode */
    SSP1CON2bits.RCEN = 1; /* Enable data reception */
    i2cdelaycnt = 0;
    while (!SSP1STATbits.BF) /* wait for buffer full */ {
        i2cdelaycnt++;
        __delay_us(100);
        if (i2cdelaycnt > 100) {
            i2cdelaycnt = 0;
            break;
        }
    }
    temp = SSP1BUF; /* Read serial buffer and store in temp register */
    I2CWait(); /* wait to check any pending transfer */
    return temp; /* Return the read data from bus */
}

unsigned int MODBUS_CRC16_v1(const unsigned char *buf, unsigned int len) {
    unsigned int crc = 0xFFFF;
    unsigned int i = 0;
    char bit1 = 0;

    for (i = 0; i < len; i++) {
        crc ^= buf[i];

        for (bit1 = 0; bit1 < 8; bit1++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

void uart2_init() {
    BAUDCON = 0X00; //BAUD RATE SET
    SPBRG = 25;
    ANSELB = 0X00; // DATASHEET RECOMMENDS
    //    TRISCbits.TRISC7 = 1;
    //    TRISCbits.TRISC6 = 1;
    TRISBbits.TRISB5 = 1;
    TRISBbits.TRISB7 = 0;
}

void tx(unsigned char bb) {
    uart2_init();
    RCSTA = 0X90; //RCSTA1<<8 SPEN=1;   RECEIVE STATUS & CONTROL REGISTER BITS CONFIGURED (RCSTAX=...)
    TXSTA = 0X20; //TXSTA<<6, TXEN=1, SYNC=0, TRANSMIT STATUS & CONTROL REGISTER BITS CONFIGURE,


    TXREG = bb;
    txchkcnt = 0;
    while (TXSTAbits.TRMT == 0) {
        __delay_us(200);
        txchkcnt++;
        if (txchkcnt > 100) {
            txchkcnt = 0;
            break;
        }
    }

}

void txmul_fn(unsigned char *txdatastr1, unsigned char txlen1) {


    while (txlen1) {
        TXREG = *txdatastr1;
        txlen1--;
        txdatastr1++;
        txchkcnt = 0;
        while (TXSTAbits.TRMT == 0) {
            __delay_us(200);
            txchkcnt++;
            if (txchkcnt > 100) {
                txchkcnt = 0;
                break;
            }
        }

    }

}

void rxint() {
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
    PIE1bits.RCIE = 1;
    PIR1bits.RCIF = 0;
    uart2_init();
    TXSTA = 0X20;
    RCSTA = 0B10010000;


}

void time_int() {
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.TMR0IE = 1;
    INTCONbits.TMR0IF = 0;
    //T0CON=0b00001000;
    OPTION_REG = 0x87;
    TMR0 = 0X00;
    //TMR0H=0X00;
    // T0CONbits.TMR0ON=1;

}

void mcp3422_fn(unsigned char ch) {
    I2CStart();
    I2CSend(0xD0);
    if(ch==1)
    {
    I2CSend(0x9e);
//    cond.togg=0;
    }
    else if(ch==2)
    {
        I2CSend(0xbe);
//        I2CSend(0x9e);
//    cond.togg=1;
    }
    I2CRestart();
    I2CSend(0xD1);


    tx1str[0] = I2CRead(); // read weekDay and return Positive ACK
    if (tx1str[0] > 0xfd) {
        tx1str[0] = tx1str[0]&0x01;
    } else {
        tx1str[0] = tx1str[0] + 0x02;
    }

    I2CAck();
    tx1str[1] = I2CRead(); // read Date and return Positive ACK
    I2CAck();
    tx1str[2] = I2CRead(); // read Month and return Positive ACK
    I2CAck();
    adchconfig = I2CRead(); // read Year and return Negative/No ACK
    I2CNak();
    I2CStop();
    
   
    
    
}

void __interrupt() INTERRUPT_InterruptManager(void) {
    //void interrupt time(void)
    //{


    if (INTCONbits.TMR0IF == 1) {

        INTCONbits.TMR0IF = 0;

        cond.tim_int_ind = 1;
        TMR0 = 0X60;
        recvcnt++;
        if (recvcnt > 3) {
            recvcnt = 0;
            rxcnt = 0;

            if (cond.rxexist == 1) {
                cond.rxexist = 0;

                rxdataarr[0] = rxrecvarr[0];
                rxdataarr[1] = rxrecvarr[1];
                rxdataarr[2] = rxrecvarr[2];
                rxdataarr[3] = rxrecvarr[3];
                rxdataarr[4] = rxrecvarr[4];
                rxdataarr[5] = rxrecvarr[5];
                rxdataarr[6] = rxrecvarr[6];
                rxdataarr[7] = rxrecvarr[7];
                cond.rxdone = 1;


                rxrecvarr[0] = 0;
                rxrecvarr[1] = 0;
                rxrecvarr[2] = 0;
                rxrecvarr[3] = 0;
                rxrecvarr[4] = 0;
                rxrecvarr[5] = 0;
                rxrecvarr[6] = 0;
                rxrecvarr[7] = 0;
            }
            //      if(cnt1==30000)
            //      {
            //          
            //          monf=0;
            //      }

        }
    }
    if (PIR1bits.RCIF == 1) {
        PIR1bits.RCIF = 0;
        if (RCSTAbits.OERR == 1) {
            RCSTAbits.CREN = 0;
            __delay_us(1);
            RCSTAbits.CREN = 1;
        }

        rx_data = RCREG;


        recvcnt = 0;

        rxrecvarr[rxcnt] = rx_data;
        rxcnt++;
        if (rxcnt > 7) {
            rxcnt = 0;
        }
        cond.rxexist = 1;

    }



}

void main() {
    // start the SPI library:
    __delay_ms(1);
    OSCCON = 0B01111010;
    __delay_ms(20);
    ANSELC = 0;

    ANSELA = 0;

    ANSELB = 0;
    TRISC = 0x7f;
    TRISA = 0xff;

    TRISB = 0xff;


    //LATC=0xff;
    LATA = 0xff;
    LATC = 0x7f;
    LATB = 0xff;
    rxint();
    I2CInit();
    //    LATA4 = 0;

    //LATA4 = 1;
    //  I2C_Slave_Init(0x30);
    addval = 1;

    en_485 = 1;

    time_int();
//    strcpy(tx1str, "kunal");
    txmul_fn(tx1str, 6);
    adchval=1;
    while (1) {


        //        if (cond.rxdone == 1) {/** if their is a data in receiver **/
        //
        //            cond.rxdone = 0;
        //            //            if (cond.echo == 0) {
        //            inttochar.intu16 = MODBUS_CRC16_v1(rxdataarr, 6);
        //            if ((rxdataarr[6] == inttochar.charu8.lsb)&&(rxdataarr[7] == inttochar.charu8.msb)&&(rxdataarr[1] == 0x03)) {
        //                /** if crc correct and 0x03 is second byte **/
        //                cond.rxcrccorrect = 1;
        //            }
        //            //            }
        //        }
        //        if (cond.rxcrccorrect == 1) {
        //            if (rxdataarr[0] == addval) {
        //              
        //
        //                __delay_ms(5);
        //              
        //                tx1str[0] = addval;
        //                tx1str[1] = 0x03;
        //                tx1str[2] = 0x02;
        //                inttochar.intu16 = data2send[data2sendcnt];
        //                tx1str[3] = inttochar.charu8.msb;
        //                tx1str[4] = inttochar.charu8.lsb;
        //                inttochar.intu16 = MODBUS_CRC16_v1(tx1str, 5);
        //                tx1str[5] = inttochar.charu8.lsb;
        //                tx1str[6] = inttochar.charu8.msb;
        //                memset(txcommstr, 0, sizeof (txcommstr));
        //                memcpy(txcommstr, tx1str, 7);
        //                txlen2 = 7;
        //                txcnt = 0;
        //            }
        //            cond.rxcrccorrect = 0;
        //        }
        //        if (txlen2 == 0) {
        //            //            txcnt = 0;
        //            //            memset(txcommstr, 0, sizeof (txcommstr));
        //            //            memcpy(txcommstr, tx1str, 7);
        //            //            txlen2 = 7;
        //            txcnt = 0;
        //            __delay_ms(1);
        //            en_485 = 0;
        //
        //        } else {
        //            if (en_485 == 0) {
        //                en_485 = 1;
        //
        //                __delay_ms(1);
        //            }
        //
        //            tx(txcommstr[txcnt]);
        //            txcnt++;
        //            txlen2--;
        //        }


        if (cond.tim_int_ind == 1) {
            cond.tim_int_ind = 0;
//            __delay_ms(100);
            mcp3422_fn(adchval);
            if((adchconfig==0x1e)&&(adchval==1))
    {
         addadch1val = adch1val + addadch1val;
         addadch1val = adch1val + addadch1val;
            adch1cnt++;
            if (adch1cnt > 9) {
                adch1cnt = 0;
                avgadch1val = (addadch1val) / 10;
                addadch1val = 0;
            }
    }
    else
    {
        cond.togg=~cond.togg;
    }
           
          
            
            tx1str[4] = 0x0d;
            tx1str[5] = 0x0a;
            txmul_fn(tx1str, 6);
            int32tou8.charu8.byte3=0;
            int32tou8.charu8.byte2=tx1str[0];
            int32tou8.charu8.byte1=tx1str[1];
            int32tou8.charu8.byte0=tx1str[2];
            longactval=int32tou8.intu32*1024/2621440;
            actval=longactval+200;
            memset(tx1str,0,sizeof(tx1str));
//            sprintf(tx1str,"v=%d\r\n",actval);
            tx1str[0]=((actval/100)%10)+'0';
            tx1str[1]=((actval/10)%10)+'0';
            tx1str[2]=(actval%10)+'0';
            tx1str[3]='0';
            tx1str[4] = 0x0d;
            tx1str[5] = 0x0a;
            txmul_fn(tx1str, 6);
            memset(tx1str,0,sizeof(tx1str));
           
            //            memset

            //        LATA2=~LATA2;
            //         __delay_ms(2000);



        }

    }
}

