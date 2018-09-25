***************************************************************************** 
** An experiment for Optical Materials & Devices lab                       ** 
** at the University of Oregon, run on top of ChibiOS                      ** 
***************************************************************************** 

Adapted by Ruvim Kondratyev, graduate student, on 6.8.2018, to use different 
threads for different tasks. The following is an overview of what the program, 
in the file main.c, does. 

Thread1: the blinker thread that came with the sample 
Thread2: another blinker thread but for testing a pin 

Thread3: continuously reads values from an ADC, calculates statistics of the data, 
         and saves the results into a shared object using a mutex on the object 

Thread4: the driver thread of the experiment; it waits for the user to push the 
         user button, and that is the start of the experiment; it uses a 
         binary semaphore to share access to the serial port with the main 
         thread, so the data it sends does not conflict with the data points 
         the main thread sends 

Main:    initializes drivers, starts threads, and sends voltage preview data 
         over serial to the Python script approximately 5 times per second 

---- end code overview ---- 

The program has two modes: 
- Continuous read-only mode 
- Experiment driver mode 

The continuous read-only mode has only Thread3 and the Main thread working: 
the Thread3 keeps reading voltages from the ADC from channels IN10 and IN11 
and saves them, while the Main thread sends a sample of this information to 
the Python script every 200 milliseconds. 

In the experiment driver mode: 
the Main thread is paused using a binary semaphore that Thread4 takes, and 
the Thread4 configures a PWM driver to send a range of duty cycles of PWM 
to A0, the pin for TIMER5, channel 0, in alternate mode 2 (timer routing). 
The driver Thread4 sweeps through the duty cycles, which, with a capacitor, 
can allow for sweeping a voltage across a range. This voltage can drive 
some experiment, while the ADC reads the results of sending this voltage 
to the experiment. Thread4 sends these results over the serial connection 
to the Python script for saving onto the computer and later processing. 

The rest of this README is from what came with the sample code with ChibiOS. 



*****************************************************************************
** ChibiOS/RT port for ARM-Cortex-M4 STM32F401.                            **
*****************************************************************************

** TARGET **

The demo runs on an STM32 Nucleo64-F401RE board board.

** The Demo **

The demo flashes the board LED using a thread, by pressing the button located
on the board the test procedure is activated with output on the serial port
SD2 (USART2, mapped on USB virtual COM port).

** Build Procedure **

The demo has been tested by using the free Codesourcery GCC-based toolchain
and YAGARTO.
Just modify the TRGT line in the makefile in order to use different GCC ports.

** Notes **

Some files used by the demo are not part of ChibiOS/RT but are copyright of
ST Microelectronics and are licensed under a different license.
Also note that not all the files present in the ST library are distributed
with ChibiOS/RT, you can find the whole library on the ST web site:

                             http://www.st.com
