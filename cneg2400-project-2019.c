#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "driverlib/qei.h"
#include "driverlib/rom.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

#define PWM_FREQUENCY 55
#define Velocity 200

/*#define initPWML 500
#define initPWMR 500

#define P_left 1
#define I_left 1
#define D_left 1

#define P_right 1
#define I_right 1
#define D_right 1
*/

/*PID for sample 1
#define initPWML 500
#define initPWMR 400

#define P_left 250
#define I_left 20
#define D_left 16

#define P_right 110
#define I_right 20
#define D_right 10
*/

/* PID for sample 2
#define initPWML 470
#define initPWMR 500

#define P_left 260
#define I_left 40
#define D_left 10

#define P_right 220
#define I_right 30
#define D_right 10
*/

//PID for sample 3
#define initPWML 500
#define initPWMR 450

#define P_left 250
#define I_left 20
#define D_left 30

#define P_right 230
#define I_right 20
#define D_right 30

#define MaxAccu 50
#define MinAccu -50

#define DIV  230
#define LEFT90 330
#define RIGHT90 380

//#define GRID 1395
int GRID=4000;
int cntInitVal=7000;

double deltaLeft,diffLeft,diff2Left,lastLeft,last2Left,accuLeft;
double deltaRight,diffRight,diff2Right,lastRight,last2Right,accuRight;

volatile int qeiPositionL,qeiPositionR;
volatile int qeiVelocityL,qeiVelocityR;

volatile int VL[100],VR[100],vcnt;

volatile int intCnt,stepCnt,ms;
volatile int sensor_val,LidarDist[11],temp_sen,temp_pos;
volatile int phase=1;
volatile int Run,Rotate;
volatile int chkpos=0;

volatile int Load;
volatile int PWMClock;
volatile int AdjustLeft;
volatile int AdjustRight;
volatile char sen,pos;

#define NEWLINE UARTCharPut(UART0_BASE, 0x0a); UARTCharPut(UART0_BASE, 0x0d)
#define NEWLINE1 UARTCharPut(UART1_BASE, 0x0a); UARTCharPut(UART1_BASE, 0x0d)

void initSystem(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

    // Enable UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Initialize UART
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // configure SW1 and SW2 GPIOF PF0 & PF4
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
    GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3|GPIO_PIN_4);     // configure 3,4 as output (A,B)
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2|GPIO_PIN_3);     // configure PB6,7 as output (C,D)
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2|GPIO_PIN_3); // configure PD2 and PD3 as output pins
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4|GPIO_PIN_5); // configure PE4 and PE5 as output pins

    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2);
}

void initQEI(void) {
    // Enable QEI Peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);

    //Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    //Set Pins to be PHA0 and PHB0
    GPIOPinConfigure(GPIO_PD6_PHA0);
    GPIOPinConfigure(GPIO_PD7_PHB0);
    //Set Pins to be PHA1 and PHB1
    GPIOPinConfigure(GPIO_PC5_PHA1);
    GPIOPinConfigure(GPIO_PC6_PHB1);

    //Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7.
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 |  GPIO_PIN_7);
    //Set GPIO pins for QEI. PhA1 -> PC5, PhB1 ->PC6.
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 |  GPIO_PIN_6);

    //DISable peripheral and int before configuration
    QEIDisable(QEI0_BASE);
    QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
    QEIDisable(QEI1_BASE);
    QEIIntDisable(QEI1_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

    // Configure quadrature encoder, use an arbitrary top limit of 10000
    //QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 10000);
    //QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 10000);
    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_CLOCK_DIR | QEI_CONFIG_NO_SWAP), 10000);
    QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_CLOCK_DIR | QEI_CONFIG_NO_SWAP), 10000);
    //Initial Disable the QEI Velocity
    QEIVelocityDisable(QEI0_BASE);
    QEIVelocityDisable(QEI1_BASE);

    //Configure the QEI velocity
    QEIVelocityConfigure(QEI0_BASE,QEI_VELDIV_1,8000000);
    QEIVelocityConfigure(QEI1_BASE,QEI_VELDIV_1,8000000);

    // Enable the QEI velocity
    QEIVelocityEnable(QEI0_BASE);
    QEIVelocityEnable(QEI1_BASE);

    // Enable the quadrature encoder.
    QEIEnable(QEI0_BASE);
    QEIEnable(QEI1_BASE);

    //Set initial position to 0
    QEIPositionSet(QEI0_BASE, 0);
    QEIPositionSet(QEI1_BASE, 0);
}

void initPWM(void) {
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PD0_M1PWM0);
    GPIOPinConfigure(GPIO_PD1_M1PWM1);

    PWMClock = SysCtlClockGet() / 64;
    Load = (PWMClock / PWM_FREQUENCY) - 1;
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, Load);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, Load);

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, AdjustLeft * Load / 1000);
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, AdjustRight * Load / 1000);
    PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_1);
}

void initTimer(void){
    uint32_t ui32Period;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    ui32Period = SysCtlClockGet() / 1000;     // Period = 1mS
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period -1);

    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntMasterEnable();

    TimerEnable(TIMER0_BASE, TIMER_A);
}

void delayMS(int ms) {
    SysCtlDelay((SysCtlClockGet()/1000)*ms);
}

int  print(char *p) {
    while(*p!='\0') {
        UARTCharPut(UART0_BASE, (*p++));;
   }
   return(0);
}

int  print1(char *p) {
    while(*p!='\0') {
        UARTCharPut(UART1_BASE, (*p++));;
   }
   return(0);
}

void putint(int count) {
    UARTCharPut(UART0_BASE,'0' + count/10000);
    UARTCharPut(UART0_BASE,'0' + (count/1000) % 10);
    UARTCharPut(UART0_BASE,'0' + (count/100) % 10);
    UARTCharPut(UART0_BASE,'0' + (count/10) % 10);
    UARTCharPut(UART0_BASE,'0' + count % 10);
}

void putint1(int count) {
    UARTCharPut(UART1_BASE,'0' + count/10000);
    UARTCharPut(UART1_BASE,'0' + (count/1000) % 10);
    UARTCharPut(UART1_BASE,'0' + (count/100) % 10);
    UARTCharPut(UART1_BASE,'0' + (count/10) % 10);
    UARTCharPut(UART1_BASE,'0' + count % 10);
}

void fb_check_position(void){
      if(qeiPositionL<=(cntInitVal-GRID)) {
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x0);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x0);
        if(Run==1) Run=2;
        else if(Run==3) Run=0;
      }
      if(qeiPositionR<=(cntInitVal-GRID)) {
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0x0);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0x0);
        if(Run==1) Run=3;
        else if(Run==2) Run=0;
      }
}

void l_check_position(void){
      if(qeiPositionL<=(cntInitVal-LEFT90)) {
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x0);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x0);
        if(Run==1) Run=2;
        else if(Run==3) Run=0;
      }
      if(qeiPositionR<=(cntInitVal-LEFT90)) {
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0x0);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0x0);
        if(Run==1) Run=3;
        else if(Run==2) Run=0;
      }
}

void r_check_position(void){
      if(qeiPositionL<=(cntInitVal-RIGHT90)) {
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x0);
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x0);
        if(Run==1) Run=2;
        else if(Run==3) Run=0;
      }
      if(qeiPositionR<=(cntInitVal-RIGHT90)) {
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0x0);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0x0);
        if(Run==1) Run=3;
        else if(Run==2) Run=0;
      }
}

void stop(void){
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x0);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x0);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0x0);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0x0);
    Run = 0;
}

void forward(void) {
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x0);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0x0);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5);
    Run = 1;
}

void backward(void) {
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x0);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0x0);
    Run = 1;
}

void rotate_left(void) {
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x0);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0x0);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5);
    Run = 1;
}

void rotate_right(void) {
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x0);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0x0);
    Run = 1;
}


void stepMotor_stop(void) {
    Rotate = 0;
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0x0);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0x0);
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0x0);
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0x0);
}

int getSensor(void){

    sen = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);
    if(sen!=0) sen=1;

    return(sen);
}

int getPos(void){

    pos = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2);
    //if(pos!=0) pos=1; //for old position ir sensor
    if(pos==0) pos=1;

    return(pos);
}

void delayms(int t){
    ms=0;
    while(ms<t){}
}

void cal_pos(void){
    temp_pos = getPos();
      if(temp_pos==1) {
        Rotate=1;
        while(temp_pos==1){
          temp_pos = getPos();
        }
        stepMotor_stop();

        Rotate=2;
        while(temp_pos==0){
          temp_pos = getPos();
        }
        stepMotor_stop();
      }
      else {
        Rotate=2;
        while(temp_pos==0){
          temp_pos = getPos();
        }
        stepMotor_stop();
      }
}

int sensor_scan(void){

      sensor_val = 0;

      sensor_val = LidarDist[10]+2*LidarDist[9]+4*LidarDist[8]+8*LidarDist[7]+16*LidarDist[6]+32*LidarDist[5];
      sensor_val += 64*LidarDist[4]+128*LidarDist[3]+256*LidarDist[2]+512*LidarDist[1]+1024*LidarDist[0];

      putint1(sensor_val);
      NEWLINE1;
      return sensor_val;
}

void forward_dist(int dist){
    GRID = dist;
    forward();
    chkpos=1;
    QEIPositionSet(QEI0_BASE, cntInitVal);
    QEIPositionSet(QEI1_BASE, cntInitVal);
}

int main(void)
{
    char cin;
    int i;

    initSystem();
    initTimer();
    initQEI();


    NEWLINE;
        print("=============================================================="); NEWLINE;
        print("*               CUHK CSE CENG2400 (2019)                     *"); NEWLINE;
        print("*                                                            *"); NEWLINE;
        print("*                    Test Program                            *"); NEWLINE;
        print("=============================================================="); NEWLINE;
        print("*                                                            *"); NEWLINE;
        print("*  f - forward 40 cm;      b - backward 40 cm                *"); NEWLINE;
        print("*                                                            *"); NEWLINE;
        print("*  l - rotate left;        r - rotate right                  *"); NEWLINE;
        print("*                                                            *"); NEWLINE;
        print("*  v - test sensor         F - continuous forward            *"); NEWLINE;
        print("=============================================================="); NEWLINE;
    NEWLINE;

    NEWLINE1;
    print1("=============================================================="); NEWLINE1;
    print1("*               CUHK CSE CENG2400 (2019)                     *"); NEWLINE1;
    print1("*                                                            *"); NEWLINE1;
    print1("*                    Test Program                            *"); NEWLINE1;
    print1("=============================================================="); NEWLINE1;
    print1("*                                                            *"); NEWLINE1;
    print1("*  f - forward 40 cm;      b - backward 40 cm                *"); NEWLINE1;
    print1("*                                                            *"); NEWLINE1;
    print1("*  l - rotate left;        r - rotate right                  *"); NEWLINE1;
    print1("*                                                            *"); NEWLINE1;
    print1("*  v - test sensor         F - continuous forward            *"); NEWLINE1;
    print1("=============================================================="); NEWLINE1;
    NEWLINE1;

    while(1) {
      AdjustLeft=initPWML;
      AdjustRight=initPWMR;
      initPWM();

      stop();

      Run = 0;
      while(Run==0) {

        cin=UARTCharGet(UART1_BASE);
        if(cin=='f'){
            GRID=6700;
            forward();
            chkpos=1;
            QEIPositionSet(QEI0_BASE, cntInitVal);
            QEIPositionSet(QEI1_BASE, cntInitVal);

        }
        else if(cin=='F'){
            delayMS(10);
            forward();
            chkpos=0;
        }
        else if(cin=='b'){
            backward();
            chkpos=1;
            QEIPositionSet(QEI0_BASE, cntInitVal);
            QEIPositionSet(QEI1_BASE, cntInitVal);
        }
        else if(cin=='l'){
            rotate_left();
            chkpos=2;
            QEIPositionSet(QEI0_BASE, cntInitVal);
            QEIPositionSet(QEI1_BASE, cntInitVal);
        }
        else if(cin=='r'){
            rotate_right();
            chkpos=3;
            QEIPositionSet(QEI0_BASE, cntInitVal);
            QEIPositionSet(QEI1_BASE, cntInitVal);
        }

        else if(cin=='p'){
            delayMS(10);
            forward();
            chkpos=0;
            vcnt=0;
            while(vcnt<100){}
            stop();

            while(1){
             NEWLINE1;
             for(i=0;i<100;i++){
                UARTCharPut(UART1_BASE,VL[i]);
                UARTCharPut(UART1_BASE,VR[i]);
             }
            }
        }

        else if(cin=='v'){
            stepCnt=1;
            phase=1;
            cal_pos();
            stepCnt=1;
            Rotate=2;
            phase=1;

            while(1) {
                NEWLINE1;

                sensor_val = 0;
                for(i=0;i<11;i++){
                    putint1(LidarDist[i]);
                    print1(",");
                }
                sensor_val = LidarDist[10]+2*LidarDist[9]+4*LidarDist[8]+8*LidarDist[7]+16*LidarDist[6]+32*LidarDist[5];
                sensor_val += 64*LidarDist[4]+128*LidarDist[3]+256*LidarDist[2]+512*LidarDist[1]+1024*LidarDist[0];
                NEWLINE1;
                putint1(sensor_val);

                if(stepCnt==DIV*20) {
                    cal_pos();
                    Rotate=2;
                    stepCnt=1;
                    phase=1;
                }
           }
        }
        else if(cin=='t'){
            stepCnt=1;
            phase=1;
            cal_pos();
            stepCnt=1;
            Rotate=2;
            phase=1;

            sensor_scan();
            chkpos=1;
            QEIPositionSet(QEI0_BASE, cntInitVal);
            QEIPositionSet(QEI1_BASE, cntInitVal);
            for(i=0;i<11;i++){
                LidarDist[i]=1;
            }
            while(1){
                sensor_scan();

                if((LidarDist[3]==0)||(LidarDist[4]==0)||(LidarDist[5]==0)||(LidarDist[6]==0)||(LidarDist[7]==0)){
                    stop();
                    while(1){};
                }
                else {
                    chkpos=1;
                    QEIPositionSet(QEI0_BASE, cntInitVal);
                    QEIPositionSet(QEI1_BASE, cntInitVal);
                    forward();
                    delayMS(1500);
                }
            }
        }
        else if(cin=='d'){          //demo
            cal_pos();            //rotate the sensor anti-clockwise to the reference point
            stepCnt=1;            //then rotate clockwise to the middle
            Rotate=2;			  //Rotate=2 => step motor rotate clockwise; Rotate=0 => stop; 
            phase=1;			  //initial phase of step motor drive output signal 	
            while(stepCnt<DIV*5){} // DIV*5 equal to 90 degree 
            Rotate=0;             //Rotate=0 stop the sensor rotation
            /////////////////
            chkpos=0;             //do not check the steps
            forward();            //continue to forward
            while(getSensor()==1){      //forward until sensor detect 0
            }
            delayMS(180);           //move forward a little bit more
            stop();					//stop the car
            delayMS(100);			//delay 100 ms
            //////////////////
            rotate_right();             //rotate right 90 degree; you can adjust #define RIGHT90 value
            chkpos=3;
            QEIPositionSet(QEI0_BASE, cntInitVal);
            QEIPositionSet(QEI1_BASE, cntInitVal);
            delayMS(300);
            //////////////////////////
            cal_pos();                  //rotate the sensor anti-clockwise toward the left direction
            /////////////
            chkpos=0;                   //move forward until sensor value = 1
            forward();
            while(getSensor()==0){
            }
            stop();
            //////////////
            forward_dist(480);          //move forward for 400 steps
            delayMS(500);
            //////////////
            rotate_left();              //rotate left 90 degree; you can adjust #define LEFT90 value
            chkpos=2;
            QEIPositionSet(QEI0_BASE, cntInitVal);
            QEIPositionSet(QEI1_BASE, cntInitVal);
            /////////////
            cal_pos();            //rotate the sensor anti-clockwise to the reference point
            stepCnt=1;            //then rotate clockwise to the middle
            Rotate=2;
            phase=1;
            while(stepCnt<DIV*5){}
            Rotate=0;             //Rotate=0 stop the sensor rotation
            ////////////////
            chkpos=0;                   //move forward until sensor value = 0
            forward();
            while(getSensor()==1){
            }
            stop();
            //////////////
            forward_dist(480);                  //move forward for 400 steps
            delayMS(500);
            //////////////
            rotate_left();                      //rotate left 90 degree
            chkpos=2;
            QEIPositionSet(QEI0_BASE, cntInitVal);
            QEIPositionSet(QEI1_BASE, cntInitVal);
            delayMS(300);
            /////////////
            chkpos=0;                           //move forward until sensor value = 0
            forward();
            while(getSensor()==1){
            }
            stop();
            //////////////
//            cal_pos();                      //rotate the sensor anti-clockwise to the reference point
//            stepCnt=1;                      //then rotate clockwise to the right most position
//            Rotate=2;
//            phase=1;
//            while(stepCnt<DIV*10){}
//            Rotate=0;
            ///////////////
//            chkpos=0;                       //move forward until sensor value = 1
//            forward();
//            while(getSensor()==0){
//            }
//            stop();
            //////////////
            forward_dist(480);              //move forward for 400 steps
            delayMS(300);
            //////////////
            rotate_right();                 //rotate right 90 degree
            chkpos=3;
            QEIPositionSet(QEI0_BASE, cntInitVal);
            QEIPositionSet(QEI1_BASE, cntInitVal);
            delayMS(300);
            //////////////////////////
            forward_dist(1500);             //move forward for 1500 steps
            delayMS(500);
         }

      }

      NEWLINE1;
      while((Run==1)||(Run==2)||(Run==3)) {

        print1("L position = ");
        putint1(qeiPositionL);
        print1("; R position = ");
        putint1(qeiPositionR);
        print1("; L velocity = ");
        putint1(qeiVelocityL);
        print1("; R velocity = ");
        putint1(qeiVelocityR);
        NEWLINE1;
        delayMS(10);
      }
   }
}

void Timer0IntHandler(void)
{

    // Clear the timer interrupt
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    qeiPositionL = QEIPositionGet(QEI0_BASE);
    qeiVelocityL = QEIVelocityGet(QEI0_BASE);
    qeiPositionR = QEIPositionGet(QEI1_BASE);
    qeiVelocityR = QEIVelocityGet(QEI1_BASE);

    ms++;

    if(chkpos==1) fb_check_position();
    else if(chkpos==2) l_check_position();
    else if(chkpos==3) r_check_position();

    intCnt++;
    if(intCnt==50){
        intCnt=0;
        if(vcnt<100){
            VL[vcnt]=qeiVelocityL;
            VR[vcnt]=qeiVelocityR;
            vcnt++;
        }

        if((Run==1)||(Run==3)){
          deltaLeft = Velocity - qeiVelocityL;
          diffLeft = deltaLeft - lastLeft;
          diff2Left = deltaLeft - last2Left;
          last2Left = lastLeft;
          lastLeft = deltaLeft;

          AdjustLeft += (P_left*deltaLeft + I_left*accuLeft*50/1000 + D_left*(diffLeft+2*diff2Left)*1000/50)/1000;

          accuLeft += deltaLeft;
          if(accuLeft>MaxAccu) accuLeft = MaxAccu;
          if(accuLeft<MinAccu) accuLeft = MinAccu;

          PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, AdjustLeft * Load / 1000);
        }
        if((Run==1)||(Run==2)){
          deltaRight = Velocity - qeiVelocityR;
          diffRight = deltaRight - lastRight;
          diff2Right = deltaRight - last2Right;
          last2Right = lastRight;
          lastRight = deltaRight;

          AdjustRight += (P_right*deltaRight + I_right*accuRight*50/1000 + D_right*(diffRight+2*diff2Right)*1000/50)/1000;

          accuRight += deltaRight;
          if(accuRight>MaxAccu) accuRight = MaxAccu;
          if(accuRight<MinAccu) accuRight = MinAccu;

          PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, AdjustRight * Load / 1000);
        }
    }

    if(Rotate==1){
      if(phase==1){
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0x0);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0x0);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0x0);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
        phase++;
        stepCnt++;
      }
      else if(phase==2){
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0x0);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0x0);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
        phase++;
        stepCnt++;
      }
      else if(phase==3){
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0x0);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0x0);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0x0);
        phase++;
        stepCnt++;
      }
      else if(phase==4){
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0x0);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0x0);
        phase++;
        stepCnt++;
      }
      else if(phase==5){
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0x0);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0x0);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0x0);
        phase++;
        stepCnt++;
      }
      else if(phase==6){
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0x0);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0x0);
        phase++;
        stepCnt++;
      }
      else if(phase==7){
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0x0);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0x0);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0x0);
        phase++;
        stepCnt++;
      }
      else if(phase==8){
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0x0);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0x0);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
        phase=1;
        stepCnt++;
      }

      if(stepCnt==DIV*20) {
        LidarDist[0] = getSensor();
        stepMotor_stop();
        cal_pos();
        Rotate=2;
        stepCnt=1;
        phase=1;
      }
    }

    if(Rotate==2){
      if(phase==1){
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0x0);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0x0);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
        phase++;
        stepCnt++;
      }
      else if(phase==2){
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0x0);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0x0);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0x0);
        phase++;
        stepCnt++;
      }
      else if(phase==3){
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0x0);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0x0);
        phase++;
        stepCnt++;
      }
      else if(phase==4){
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0x0);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0x0);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0x0);
        phase++;
        stepCnt++;
      }
      else if(phase==5){
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0x0);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0x0);
        phase++;
        stepCnt++;
      }
      else if(phase==6){
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0x0);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0x0);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0x0);
        phase++;
        stepCnt++;
      }
      else if(phase==7){
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0x0);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0x0);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
        phase++;
        stepCnt++;
      }
      else if(phase==8){
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0x0);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0x0);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0x0);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
        phase=1;
        stepCnt++;
      }

      if(stepCnt==DIV) LidarDist[1] = getSensor();
      if(stepCnt==DIV*2) LidarDist[2] = getSensor();
      if(stepCnt==DIV*3) LidarDist[3] = getSensor();
      if(stepCnt==DIV*4) LidarDist[4] = getSensor();
      if(stepCnt==DIV*5) LidarDist[5] = getSensor();
      if(stepCnt==DIV*6) LidarDist[6] = getSensor();
      if(stepCnt==DIV*7) LidarDist[7] = getSensor();
      if(stepCnt==DIV*8) LidarDist[8] = getSensor();
      if(stepCnt==DIV*9) LidarDist[9] = getSensor();
      if(stepCnt==DIV*10) {
          LidarDist[10] = getSensor();
          Rotate=1;
      }
  }
}
