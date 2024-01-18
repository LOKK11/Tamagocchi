/*
=====TAMAGOTCHI BY OSSI MYLLYMÄKI AND JULIUS HAUKIPURO=====
    Everything in the project was done together so the
        contribution is 50/50 from both.
*/

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>

/* Board Header files */
#include "Board.h"

#include <string.h>
#include <ti/drivers/UART.h>
#include <stdio.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include "sensors/mpu9250.h"
#include "sensors/opt3001.h"
#include "sensors/tmp007.h"
#include "CC2650STK.h"
#include "wireless/comm_lib.h"
#include "buzzer.h"


#define SERIALSTACKSIZE 1024                //Stack size for serial task
#define SENSORSTACKSIZE 2048                //Stack size for sensor task

Char serialTaskStack[SERIALSTACKSIZE];      //Declare stack for tasks with given stack sizes
Char sensorTaskStack[SENSORSTACKSIZE];

enum state {WAITING=0, EAT, ACTIVATE, PET, EXERCISE, PANIC, ESCAPE};    //States the tamagotchi can get
enum state programState = WAITING;          //Set the state to waiting.

float ax, ay, az, gx, gy, gz;               //Declare global variables
float temperature = -100.0;
double light = -1000.0;

char eatmsg[20]="id:3031,EAT:2";            //Declare containers for different messages.
char exercisemsg[20]="id:3031,EXERCISE:2";
char petmsg[20]="id:3031,PET:2";
char activatemsg[25]="id:3031,ACTIVATE:1;1;1";
char uartBuffer[80];
char panicMsg[9] = "3031,BEEP";
char escapeMsg[18] = "3031,BEEP:Too late";

static PIN_Handle hMpuPin;                  //Declare pin handle variable for mpu

static PIN_Handle ledHandle;                //Declare pin handle variable for the green led
static PIN_State ledState;                  //State variable for the green led

static PIN_Handle ledHandle1;               //Declare pin handle variable for the red led
static PIN_State ledState1;                 //State variable for the red led

static PIN_Handle hBuzzer;                  //Declare pin handle variable for the buzzer
static PIN_State sBuzzer;                   //State variable for the buzzer

void sendMsg(UART_Handle handle, char *msg, int length);    //Prototype for sendMsg
void playHappyBirthday();
void playSadTrombone();

PIN_Config ledConfig[] = {                  //Configuration for the green led
   Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE
};

PIN_Config ledConfig1[] = {                 //Configuration for the red led
   Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE
};

PIN_Config cBuzzer[] = {                    //Configuration for the buzzer
  Board_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {      //Configuration for MPU (Motion Processing Unit) I2C communication
    .pinSDA = Board_I2C0_SDA1,              // SDA pin for I2C communication with MPU
    .pinSCL = Board_I2C0_SCL1               // SDA pin for I2C communication with MPU
};

//In this task the program reads data from mpu, tmp and opt sensors and
//changes program state accordingly.
Void sensorTask(UArg arg0, UArg arg1) {


        uint8_t tickCounter = 0;    //Variable used to determine which tick the task is on. (0-24)

        I2C_Handle i2cMPU;          //Own i2c-interface for MPU9250 sensor
        I2C_Params i2cMPUParams;
        I2C_Handle i2c;             //i2c-interface for OPT3001 and TMP007 sensors
        I2C_Params i2cParams;

        I2C_Params_init(&i2cMPUParams);     //Initialize i2c parameters for mpu
        i2cMPUParams.bitRate = I2C_400kHz;  //Set the bit rate to 400kHz

        i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;

        // MPU power on
        PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);

        // Wait 100ms for the MPU sensor to power up
        Task_sleep(100000 / Clock_tickPeriod);
        System_printf("MPU9250: Power ON\n");
        System_flush();

        // MPU open i2c
        i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
        if (i2cMPU == NULL) {
            System_abort("Error Initializing I2CMPU\n");
        }

        // MPU setup and calibration
        System_printf("MPU9250: Setup and calibration...\n");
        System_flush();

        mpu9250_setup(&i2cMPU);
        Task_sleep(500000 / Clock_tickPeriod);

        System_printf("MPU9250: Setup and calibration OK\n");
        System_flush();

        //Close mpu i2c
        I2C_close(i2cMPU);

        I2C_Params_init(&i2cParams);
        i2cParams.bitRate = I2C_400kHz;

        //Open i2c for tmp and opt
        i2c = I2C_open(Board_I2C_TMP, &i2cParams);
        if (i2c == NULL) {
          System_abort("Error Initializing I2C\n");
        }

        //Tmp setup
        tmp007_setup(&i2c);
        Task_sleep(100000/ Clock_tickPeriod);
        I2C_close(i2c);

        i2c = I2C_open(Board_I2C_TMP, &i2cParams);
        if (i2c == NULL) {
           System_abort("Error Initializing I2C\n");
        }

        //Opt setup
        opt3001_setup(&i2c);
        Task_sleep(100000/ Clock_tickPeriod);
        I2C_close(i2c);

        playHappyBirthday();      // "Happy birthday" -song is played when the program is started

        //Start endless loop of getting data and detecting movements
        while (1) {
            i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
            if (i2cMPU == NULL) {
                System_abort("Error Initializing I2CMPU\n");
            }
            float tempGz = gz;      //The values from last tick
            float tempAz = az;
            float tempAy = ay;
            mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);    //Get the data from mpu sensor.
            I2C_close(i2cMPU);
            if (tempGz < -150) {    //Turning device right and back left quickly changes state to EAT
                if (gz > 150) {
                    programState = EAT;
                }
            }

            if (tempAz > -0.7) {    //Lifting device up and back down quickly changes state to ACTIVATE
                if (az < -1.3) {
                    programState = ACTIVATE;
                }
            }

            if (tempAy > 0.3) {     //Moving device back and forth on the y axis quickly changes the state to EXERCISE
                if (ay < -0.3) {
                    programState = EXERCISE;
                }
            }
            if (tickCounter == 0) {         //Every 25th tick light and temperature data are collected from sensors
                i2c = I2C_open(Board_I2C_TMP, &i2cParams);
                temperature = tmp007_get_data(&i2c);
                light = opt3001_get_data(&i2c);
                I2C_close(i2c);
                //If light value is low and temperature of the device is high at the same time
                //change program state to PET
                if (light < 1 && temperature > 36) {
                    programState = PET;
                }
            }
            ++tickCounter;
            if (tickCounter >= 25) {
                tickCounter = 0;
            }
            Task_sleep(200000 / Clock_tickPeriod);      //Loop is repeated every 200ms
        }
}


// Handler function for reading messages from uart
Void uartFxn(UART_Handle uart, void *rxBuf, size_t size) {
    if (strncmp(rxBuf, escapeMsg, 18) == 0) {       // If wanted message is read from rxBuf, change program state accordingly
        programState = ESCAPE;                      // ESCAPE means that some of the bars has gone to 0
    } else if (strncmp(rxBuf, panicMsg, 9) == 0) {
        programState = PANIC;                       // PANIC Means that some of the bars has gone under 2 or over 10
    }
    UART_read(uart, rxBuf, 80);
}

Void serialTask(UArg arg0, UArg arg1) {

    // UART-library declarations
    UART_Handle uart;
    UART_Params uartParams;

    // Setup the serial communication
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.readMode = UART_MODE_CALLBACK;
    uartParams.readCallback = &uartFxn;
    uartParams.baudRate = 9600; //Speed 9600baud
    uartParams.dataLength = UART_LEN_8; // 8
    uartParams.parityType = UART_PAR_NONE; // n
    uartParams.stopBits = UART_STOP_ONE; // 1

    // Open the serial port in variable Board_UART0
    uart = UART_open(Board_UART0, &uartParams);
    if (uart == NULL) {
       System_abort("Error opening the UART");
    }
    UART_read(uart, uartBuffer, 80);

    // In this loop serialTask repeatedly checks the program state and writes
    // messages to uart and does other actions accordingly. After every loop
    // program state is set to WAITING if it wasn't already.
    while (1) {
        PIN_setOutputValue( ledHandle, Board_LED0, 0 );     // Turn off the green led
        PIN_setOutputValue( ledHandle1, Board_LED1, 0 );    // Turn off the red led

       switch (programState) {
          case EAT:                                             // EAT: Turn on the green led and write eatmsg to uart
              sendMsg(uart, eatmsg, strlen(eatmsg));
              programState = WAITING;
              PIN_setOutputValue( ledHandle, Board_LED0, 1 );
              break;
          case EXERCISE:                                        // EXERCISE: Turn on the red led and write exercisemsg to uart
              sendMsg(uart, exercisemsg,strlen(exercisemsg));
              programState = WAITING;
              PIN_setOutputValue( ledHandle1, Board_LED1, 1 );
              break;
          case PET:                                             // PET: Make a two part buzzer noise and write petmsg to uart
              sendMsg(uart, petmsg,strlen(petmsg));
              programState = WAITING;
              buzzerOpen(hBuzzer);
              buzzerSetFrequency(7000);
              Task_sleep(100000 / Clock_tickPeriod);
              buzzerSetFrequency(6000);
              Task_sleep(100000 / Clock_tickPeriod);
              buzzerClose();
              break;
          case ACTIVATE:                                        // ACTIVATE: Turn both leds on, do a short buzzer sound and write activatemsg to uart
              sendMsg(uart, activatemsg,strlen(activatemsg));
              programState = WAITING;
              PIN_setOutputValue( ledHandle1, Board_LED1, 1 );
              PIN_setOutputValue( ledHandle, Board_LED0, 1 );
              buzzerOpen(hBuzzer);
              buzzerSetFrequency(3000);
              Task_sleep(100000 / Clock_tickPeriod);
              buzzerClose();
              break;
          case PANIC:                                           // PANIC: Make a three part noise
              programState = WAITING;
              buzzerOpen(hBuzzer);
              buzzerSetFrequency(4000);
              Task_sleep(200000 / Clock_tickPeriod);
              buzzerSetFrequency(5000);
              Task_sleep(200000 / Clock_tickPeriod);
              buzzerSetFrequency(6000);
              Task_sleep(200000 / Clock_tickPeriod);
              buzzerClose();
              break;
          case ESCAPE:                                          // ESCAPE: Play the "sad trombone" -sound effect
              programState = WAITING;
              playSadTrombone();
              break;
          default:
              //programState was WAITING
              break;
       }
       //Loop is repeated every second
       Task_sleep(1000000 / Clock_tickPeriod);
    }
}

// This function writes a wanted message to uart
void sendMsg(UART_Handle handle, char *msg, int length) {
        UART_write(handle, msg, length + 1);
}

// Function to play a note with a specified frequency and duration
void playNote(float frequency, uint32_t duration) {
    buzzerSetFrequency(frequency);
    Task_sleep(duration / Clock_tickPeriod);
}

// Function to play the "Happy Birthday" song beginning
void playHappyBirthday() {
    // Open the buzzer
    buzzerOpen(hBuzzer);

    // Play the "Happy Birthday" song
    playNote(392.00, 700000);   // G
    playNote(440.00, 700000);   // A
    playNote(392.00, 700000);   // G
    playNote(523.25, 700000);   // C
    playNote(493.88, 1000000);   // B

    // Close the buzzer
    buzzerClose();
}


// Function to play the sad trombone sound effect
void playSadTrombone() {
    // Open the buzzer
    buzzerOpen(hBuzzer);
    Task_sleep(200000 / Clock_tickPeriod);

    // Play the "sad trombone" -sound effect using the playNote function
    playNote(587.33, 500000);  // D#
    playNote(554.37, 500000);  // D
    playNote(523.25, 700000);  // C
    playNote(493.88, 200000);  // B
    playNote(246.94, 200000);  // B (an octave lower)
    playNote(493.88, 200000);  // B
    playNote(246.94, 200000);  // B (an octave lower)
    playNote(493.88, 200000);  // B
    playNote(246.94, 200000);  // B (an octave lower)
    playNote(493.88, 200000);  // B
    playNote(246.94, 200000);  // B (an octave lower)
    playNote(493.88, 1000000); // B (sustained)

    // Close the buzzer
    buzzerClose();
}

/*
 *  ======== main ========
 */
int main(void)
{
    // Initialize parameters and handlers for serial- and sensorTask
    Task_Params serialTaskParams;
    Task_Handle serialTaskHandle;
    Task_Params sensorTaskParams;
    Task_Handle sensorTaskHandle;

    // Call board init functions
    Board_initGeneral();
    Board_initUART();
    Board_initI2C();

    // Initialize serial task parameters
    Task_Params_init(&serialTaskParams);
    // Define task stack memory
    serialTaskParams.stackSize = SERIALSTACKSIZE;
    serialTaskParams.stack = &serialTaskStack;
    // Set task priority
    serialTaskParams.priority = 2;

    // Open handle pin for the green led
    ledHandle = PIN_open(&ledState, ledConfig);
    if(!ledHandle) {
       System_abort("Error initializing LED pins\n");
    }

    ledHandle1 = PIN_open(&ledState1, ledConfig1);
    if(!ledHandle1) {
       System_abort("Error initializing LED pins\n");
    }

    // Open handle pin for the buzzer
    hBuzzer = PIN_open(&sBuzzer, cBuzzer);
    if (hBuzzer == NULL) {
       System_abort("Pin open failed!");
    }

    // Create serial task
    serialTaskHandle = Task_create(serialTask, &serialTaskParams, NULL);
    if (serialTaskHandle == NULL) {
       System_abort("Task create failed");
    }

    // Initialize sensor task parameters
    Task_Params_init(&sensorTaskParams);
    // Define task stack memory
    sensorTaskParams.stackSize = SENSORSTACKSIZE;
    sensorTaskParams.stack = &sensorTaskStack;
    // Set task priority
    sensorTaskParams.priority = 2;

    // Create sensor task
    sensorTaskHandle = Task_create(sensorTask, &sensorTaskParams, NULL);
    if (sensorTaskHandle == NULL) {
       System_abort("Task create failed");
    }

    System_printf("Hello world\n");
    System_flush();

    // Start BIOS
    BIOS_start();

    return (0);
}
