#include "main.h"
#include "driverlib/driverlib.h"
#include "hal_LCD.h"
#include "buzzer.h"

// *********************************** Global variables ***********************************
char ADCState = 0; //Busy state of the ADC
int16_t ADCResult = 0; //Storage for the ADC conversion result

uint16_t timerA0_count = 0;
uint16_t timerA1_count = 0;

int rear_distance_buf[3] = {0, 0, 0};
int rear_current_index = 0;
int rear_avg_distance = 0;

int forward_distance_buf[3] = {0, 0, 0};
int forward_current_index = 0;
int forward_avg_distance = 0;

int operation_mode = 0;       	// 0 = User mode, 1 = Setup mode
int sensor_mode = 0;			// 0 = Rear, 1 = Forward

// Proximity thresholds (in cm)
int rear_thres1 = 10;
int rear_thres2 = 20;
int rear_thres3 = 30;
int forward_thres1 = 20;
int forward_thres2 = 40;

int rear_setup_thres = 0;     	// 0 = Red threshold, 1 = Orange threshold, 2 = Yellow threshold
int forward_setup_thres = 0;	// 0 = Quadruple beep threshold, 1 = Double beep threshold

int pb2_val = 1; // Active low
// *******************************************************************************************

void errorBeep() {
    beep(261, 300);
    beep(261, 300);
}

int pb2press() {
    // Look for a falling edge
    if (pb2_val == 1 && GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6) == 0) {
        pb2_val = 0;
        return 1;
    }
    pb2_val = 1;
    return 0;
}

void turnOffLeds() {
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN4);    // Turn Red LED OFF
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN3);    // Turn Orange LED OFF
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0);    // Turn Yellow LED OFF
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);    // Turn Green LED OFF
}

void rearProximityCheck() {
    turnOffLeds();    // First turn all LEDs off

    if (rear_avg_distance <= rear_thres1) {
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);   // Turn Red LED ON
    }
    else if (rear_avg_distance <= rear_thres2) {
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);   // Turn Orange LED ON
    }
    else if (rear_avg_distance <= rear_thres3) {
        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0);   // Turn Yellow LED ON
    }
    else {
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN6);   // Turn Green LED ON
    }
}

void forwardProximityCheck() {
    if (forward_avg_distance <= forward_thres1) {
        // Quadruple beep
        beep(466, 250);
        beep(466, 250);
        beep(466, 250);
        beep(466, 250);
    }
    else if (forward_avg_distance <= forward_thres2) {
        // Double beep
        beep(440, 500);
        beep(440, 500);
    }
}

void proximityUX() {
    if (sensor_mode == 0) {
        rearProximityCheck();
    }
    else if (sensor_mode == 1) {
        forwardProximityCheck();
    }
}

void calculateAvgProximity() {
    // Timer counts will give us the # of clock cycles passed since it
    // the timer started. Assuming the timers have a 1 MHz clock, this count will equal the
    // microseconds (us), and the width of the echo signal. Then, divide by 58 to get
    // distance in cm and display that to the LCD

    // Calculate most recent distance readings in cm
    rear_distance_buf[rear_current_index] = timerA0_count / 58;
    forward_distance_buf[forward_current_index] = timerA1_count / 58;

    // Use a weighted average (from Simpson's rule) to calculate average of the last 3 readings to reduce noise
    // (f0 + 4f1 + f2) / 6
    rear_avg_distance = (rear_distance_buf[abs(rear_current_index - 2) % 3] +
                        4 * rear_distance_buf[abs(rear_current_index - 1) % 3] +
                        rear_distance_buf[rear_current_index]) / 6;

    forward_avg_distance = (forward_distance_buf[abs(forward_current_index - 2) % 3] +
                           4 * forward_distance_buf[abs(forward_current_index - 1) % 3] +
                           forward_distance_buf[forward_current_index]) / 6;

    rear_current_index = (rear_current_index + 1) % 3;
    forward_current_index = (forward_current_index + 1) % 3;
}

void displayProximity() {
    calculateAvgProximity();

    // Display to LCD
    clearLCD();

    if (sensor_mode == 0) {
        // Rear
        showInt(rear_avg_distance);
        showChar('R', pos1);
    }
    else if (sensor_mode == 1) {
        //Forward
        showInt(forward_avg_distance);
        showChar('F', pos1);
    }
}

void sendTrigger() {
    // Set digital high on pin 2.7 (Trig)
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);

    // The MSP430 FR4133 has a 16 MHz CPU
    // Delay of 160 cycles gives translates to 10 us
    __delay_cycles(1);

    // Set digital low on pin 2.7 (Trig)
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);

    // ~30 ms delay to allow ultrasonic beams to be sent out
    __delay_cycles(60000);
}

void main(void)
{
    // char buttonState = 0; //Current button press state (to allow edge detection)

    /*
     * Functions with two underscores in front are called compiler intrinsics.
     * They are documented in the compiler user guide, not the IDE or MCU guides.
     * They are a shortcut to insert some assembly code that is not really
     * expressible in plain C/C++. Google "MSP430 Optimizing C/C++ Compiler
     * v18.12.0.LTS" and search for the word "intrinsic" if you want to know
     * more.
     * */

    //Turn off interrupts during initialization
    __disable_interrupt();

    //Stop watchdog timer unless you plan on using it
    WDT_A_hold(WDT_A_BASE);

    // Initializations - see functions for more detail
    Init_GPIO();    //Sets all pins to output low as a default
    Init_Clock();   //Sets up the necessary system clocks
    Init_LCD();     //Sets up the LaunchPad LCD display

     /*
     * The MSP430 MCUs have a variety of low power modes. They can be almost
     * completely off and turn back on only when an interrupt occurs. You can
     * look up the power modes in the Family User Guide under the Power Management
     * Module (PMM) section. You can see the available API calls in the DriverLib
     * user guide, or see "pmm.h" in the driverlib directory. Unless you
     * purposefully want to play with the power modes, just leave this command in.
     */
    PMM_unlockLPM5(); //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

    //All done initializations - turn interrupts back on.
    __enable_interrupt();

    //clear all interrupt enables and flags on port 1 & 2
    P2IE  = 0x00;
    P2IFG  = 0x00;
    P1IE  = 0x00;
    P1IFG  = 0x00;

    // Setup Timer A and Timer B
    Init_TimerA_Continuous();

    // Set output pins
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN7);       // Ultrasonic trigger
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN7);       // Buzzer
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN6);       // Green LED
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0);       // Yellow LED
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN3);       // Orange LED
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN4);       // Red LED
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);       // Internal LED

    // Set input pins
    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN5);        // Rear Ultrasonic echo
    GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN5);        // Forward Ultrasonic echo
    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN2);        // Push button 1
    GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN6);        // Push button 2

    // ****************** Set interrupts ******************
    // Rear ultrasonic echo interrupt
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN5);
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN5, GPIO_LOW_TO_HIGH_TRANSITION);

    // Forward ultrasonic echo interrupt
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN5);
    GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN5, GPIO_LOW_TO_HIGH_TRANSITION);

    // PB 1 interrupt to switch between user and setup mode
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN2);
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN2, GPIO_HIGH_TO_LOW_TRANSITION);

    // PB 2 interrupt to switch between forward and rear sensors
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN6);
    GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN6, GPIO_HIGH_TO_LOW_TRANSITION);

    P1REN |= (BIT2);     // Enable resistance on P1.2 (PB 1)
    P2REN |= (BIT6);    // Enable resistance on P2.6 (PB 2)

    // Main program loop
    while (1) {
        if (operation_mode == 0) {
            // User Mode
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);   // LED operation mode indicator

            sendTrigger();
            displayProximity();
            proximityUX();
        }
        else if (operation_mode == 1) {
            // Setup mode
            GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);    // LED operation mode indicator

            sendTrigger();
            displayProximity();
            turnOffLeds();

            if (sensor_mode == 0) {
                // Rear setup
                switch (rear_setup_thres) {
                    case 0:
                        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);	// Red LED
                        break;
                    case 1:
                        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);	// Orange LED
                        break;
                    case 2:
                        GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0);	// Yellow LED
                        break;
                    default:
                        break;
                }

                // Check to see if PB 2 was pressed
                if (pb2press()) {
                    switch (rear_setup_thres) {
                        case 0:
                            rear_thres1 = rear_avg_distance;
                            rear_setup_thres++;
                            break;
                        case 1:
                            if (rear_avg_distance <= rear_thres1) {
                                // Don't allow user to set a threshold lower than or equal to the previous one - doesn't make any sense
                                // Beep twice to let user know they can't do this
                                errorBeep();
                            }
                            else {
                                rear_thres2 = rear_avg_distance;
                                rear_setup_thres++;
                            }

                            break;
                        case 2:
                            if (rear_avg_distance <= rear_thres2) {
                                // Don't allow user to set a threshold lower than the previous one - doesn't make any sense
                                // Beep twice to let user know they can't do this
                                errorBeep();
                            }
                            else {
                                rear_thres3 = rear_avg_distance;
                                // Done with setup, go back to user mode and reset rear_setup_thres
                                rear_setup_thres = 0;
                                operation_mode = 0;
                            }

                            break;
                        default:
                            break;
                    }
                }
            }
            else if (sensor_mode == 1) {
                // Forward setup

                switch (forward_setup_thres) {
                    case 0:
                        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN4);	// Red LED
                        break;
                    case 1:
                        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN3);	// Orange LED
                        break;
                    default:
                        break;
                }

                // Check to see if PB 2 was pressed
                if (pb2press()) {
                    switch (forward_setup_thres) {
                        case 0:
                            forward_thres1 = forward_avg_distance;
                            forward_setup_thres++;
                            break;
                        case 1:
                            if (forward_avg_distance <= forward_thres1) {
                                // Don't allow user to set a threshold lower than the previous one - doesn't make any sense
                                // Beep twice to let user know they can't do this
                                errorBeep();
                            }
                            else {
                                forward_thres2 = forward_avg_distance;
                                // Done with setup, go back to user mode and reset forward_setup_thres
                                forward_setup_thres = 0;
                                operation_mode = 0;
                                turnOffLeds();
                            }
                            break;
                        default:
                            break;
                    }
                }
            }
        }
    }
}

#pragma vector=PORT2_VECTOR
__interrupt void Port_2_ISR(void)
{
    if (P2IFG & 0x20) {		// pin 5 (Forward echo)
        if (!(P2IES & 0x20)) {
            // Rising edge interrupt

            // Reset and start timer A1
            Timer_A_clear(TIMER_A1_BASE);
            Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_CONTINUOUS_MODE);

            // Change interrupt edge to falling
            P2IES |= 0x20;
        }
        else {
            // Falling edge interrupt

            // Stop timer A
            Timer_A_stop(TIMER_A1_BASE);

            // Read timer value
            timerA1_count = Timer_A_getCounterValue(TIMER_A1_BASE);

            // Change interrupt edge to rising
            P2IES &= ~0x20;
        }

        GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN5);
    }
    else if (P2IFG & 0x40) {	// pin 6 (PB 2)
        // If in user mode
        if (operation_mode == 0) {
            turnOffLeds();
            sensor_mode ^= 0x1;    // Switch sensor modes
        }
        GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN6);
    }
}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1_ISR(void)
{
    if (P1IFG & 0x20) {     // pin 5 (Rear echo)
        if (!(P1IES & 0x20)) {
            // Rising edge interrupt

            // Reset and start timer A
            Timer_A_clear(TIMER_A0_BASE);
            Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_CONTINUOUS_MODE);

            // Change interrupt edge to falling
            P1IES |= 0x20;
        }
        else {
            // Falling edge interrupt

            // Stop timer A
            Timer_A_stop(TIMER_A0_BASE);

            // Read timer value
            timerA0_count = Timer_A_getCounterValue(TIMER_A0_BASE);

            // Change interrupt edge to rising
            P1IES &= ~0x20;
        }
        GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN5);
    }
    else if (P1IFG & 0x4) { // pin 2 (PB 1)
        if (P1IES & 0x4) {
            // Falling edge interrupt
            operation_mode ^= 0x1;    // Switch operation modes
            turnOffLeds();

            // Reset setup thres counters to ensure setup mode always starts with first threshold
            rear_setup_thres = 0;
            forward_setup_thres = 0;
        }
        GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN2);
    }
}

/* Timer A setup in continuous mode */
void Init_TimerA_Continuous(void)
{
    Timer_A_initContinuousModeParam initContParam = {0};
    initContParam.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    initContParam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    initContParam.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    initContParam.timerClear = TIMER_A_DO_CLEAR;
    initContParam.startTimer = false;
    Timer_A_initContinuousMode(TIMER_A0_BASE, &initContParam);
    Timer_A_initContinuousMode(TIMER_A1_BASE, &initContParam);
}

void Init_GPIO(void)
{
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    //Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);

    //Set LED1 and LED2 as outputs
    //GPIO_setAsOutputPin(LED1_PORT, LED1_PIN); //Comment if using UART
    GPIO_setAsOutputPin(LED2_PORT, LED2_PIN);
}

/* Clock System Initialization */
void Init_Clock(void)
{
    /*
     * The MSP430 has a number of different on-chip clocks. You can read about it in
     * the section of the Family User Guide regarding the Clock System ('cs.h' in the
     * driverlib).
     */

    /*
     * On the LaunchPad, there is a 32.768 kHz crystal oscillator used as a
     * Real Time Clock (RTC). It is a quartz crystal connected to a circuit that
     * resonates it. Since the frequency is a power of two, you can use the signal
     * to drive a counter, and you know that the bits represent binary fractions
     * of one second. You can then have the RTC module throw an interrupt based
     * on a 'real time'. E.g., you could have your system sleep until every
     * 100 ms when it wakes up and checks the status of a sensor. Or, you could
     * sample the ADC once per second.
     */
    //Set P4.1 and P4.2 as Primary Module Function Input, XT_LF
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1 + GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    // Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768);
    // Set ACLK = XT1
    CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Initializes the XT1 crystal oscillator
    CS_turnOnXT1LF(CS_XT1_DRIVE_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
}

/* UART Initialization */
void Init_UART(void)
{
    /* UART: It configures P1.0 and P1.1 to be connected internally to the
     * eSCSI module, which is a serial communications module, and places it
     * in UART mode. This let's you communicate with the PC via a software
     * COM port over the USB cable. You can use a console program, like PuTTY,
     * to type to your LaunchPad. The code in this sample just echos back
     * whatever character was received.
     */

    //Configure UART pins, which maps them to a COM port over the USB cable
    //Set P1.0 and P1.1 as Secondary Module Function Input.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

    /*
     * UART Configuration Parameter. These are the configuration parameters to
     * make the eUSCI A UART module to operate with a 9600 baud rate. These
     * values were calculated using the online calculator that TI provides at:
     * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
     */

    //SMCLK = 1MHz, Baudrate = 9600
    //UCBRx = 6, UCBRFx = 8, UCBRSx = 17, UCOS16 = 1
    EUSCI_A_UART_initParam param = {0};
        param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
        param.clockPrescalar    = 6;
        param.firstModReg       = 8;
        param.secondModReg      = 17;
        param.parity            = EUSCI_A_UART_NO_PARITY;
        param.msborLsbFirst     = EUSCI_A_UART_LSB_FIRST;
        param.numberofStopBits  = EUSCI_A_UART_ONE_STOP_BIT;
        param.uartMode          = EUSCI_A_UART_MODE;
        param.overSampling      = 1;

    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
    {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable EUSCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
#pragma vector=USCI_A0_VECTOR
__interrupt
void EUSCIA0_ISR(void)
{
    uint8_t RxStatus = EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, RxStatus);

    if (RxStatus)
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, EUSCI_A_UART_receiveData(EUSCI_A0_BASE));
    }
}

/* PWM Initialization */
void Init_PWM(void)
{
    /*
     * The internal timers (TIMER_A) can auto-generate a PWM signal without needing to
     * flip an output bit every cycle in software. The catch is that it limits which
     * pins you can use to output the signal, whereas manually flipping an output bit
     * means it can be on any GPIO. This function populates a data structure that tells
     * the API to use the timer as a hardware-generated PWM source.
     *
     */
    //Generate PWM - Timer runs in Up-Down mode
    param.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod           = TIMER_A_PERIOD; //Defined in main.h
    param.compareRegister       = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode     = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle             = HIGH_COUNT; //Defined in main.h

    //PWM_PORT PWM_PIN (defined in main.h) as PWM output
    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, PWM_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
}

void Init_ADC(void)
{
    /*
     * To use the ADC, you need to tell a physical pin to be an analog input instead
     * of a GPIO, then you need to tell the ADC to use that analog input. Defined
     * these in main.h for A9 on P8.1.
     */

    //Set ADC_IN to input direction
    GPIO_setAsPeripheralModuleFunctionInputPin(ADC_IN_PORT, ADC_IN_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    //Initialize the ADC Module
    /*
     * Base Address for the ADC Module
     * Use internal ADC bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC_init(ADC_BASE,
             ADC_SAMPLEHOLDSOURCE_SC,
             ADC_CLOCKSOURCE_ADCOSC,
             ADC_CLOCKDIVIDER_1);

    ADC_enable(ADC_BASE);

    /*
     * Base Address for the ADC Module
     * Sample/hold for 16 clock cycles
     * Do not enable Multiple Sampling
     */
    ADC_setupSamplingTimer(ADC_BASE,
                           ADC_CYCLEHOLD_16_CYCLES,
                           ADC_MULTIPLESAMPLESDISABLE);

    //Configure Memory Buffer
    /*
     * Base Address for the ADC Module
     * Use input ADC_IN_CHANNEL
     * Use positive reference of AVcc
     * Use negative reference of AVss
     */
    ADC_configureMemory(ADC_BASE,
                        ADC_IN_CHANNEL,
                        ADC_VREFPOS_AVCC,
                        ADC_VREFNEG_AVSS);

    ADC_clearInterrupt(ADC_BASE,
                       ADC_COMPLETED_INTERRUPT);

    //Enable Memory Buffer interrupt
    ADC_enableInterrupt(ADC_BASE,
                        ADC_COMPLETED_INTERRUPT);
}

//ADC interrupt service routine
#pragma vector=ADC_VECTOR
__interrupt
void ADC_ISR(void)
{
    uint8_t ADCStatus = ADC_getInterruptStatus(ADC_BASE, ADC_COMPLETED_INTERRUPT_FLAG);

    ADC_clearInterrupt(ADC_BASE, ADCStatus);

    if (ADCStatus)
    {
        ADCState = 0; //Not busy anymore
        ADCResult = ADC_getResults(ADC_BASE);
    }
}
