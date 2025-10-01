# Introduction
Linear CCDs can be useful tools in science, for example as a sensor for a spectrometer or imaging system.  And indeed, there are many commercial instruments that are based on low cost linear CCDs such as the Toshiba TC1304 and the Sony IXL511.
Here we provide design files, firmware and software for a device that provides competitive performance and a rich set of science-centric features, and that can be customized to your experiments, all at a fraction of the cost of the commercial offerings.  The design is based on the Toshiba TCD1304 (3648/3694 pixels) and Teensy 4.0 (600MHz ARM, 480MHz USB).  The Teensy 3.2 is plug compatible for this design.
Read further below, and you will find tutorials on electrical design and managing the CCD to provide flexible timing and reproducible measurements so that you can use it as a scientific instrument.

In this repository you will find directories containing (a) electrical design files in KiCad, (b) firmware (a sketch file) for the Teensy, and (c) a Python class library with graphical and command line utilities to operate the device.
The device has a trigger or gate input, sync and busy outputs, and spare pins for digital and analog I/O.
The operating modes include clocked, triggered frames, triggered series of frames, and gated frames (where the shutter is opened and closed on the rising and falling edges of the logic level input signal).
The device appears as a serial port (COM in windows) with comands and response in human language ASCII and frames returned as binary or formatted.
The python class library provides higher level functions, a graphical realtime display and a command line processor.  There is a help command in the firmware and in the Python command line interpreter.  Details of the electrical and firmware design are described later in this README.

The following shows the microcontroller side of the device.  The sensor side can be seen in the image of the spectrometer (below).   The digital I/O pins across the top include trigger input, sync and busy output, pins that monitor the signals going to the sensor, and spares that can be controlled through the user interface.  Across the bottom there are pins that can be used for analog inputs or digital I/O, and 3.3V that can be used for an auxiliary device.

![IMG_20231215_144019112_cropped250](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/1eda6d73-2e27-4ffd-ba63-d32f814700c4)

Following are two examples of applications; using the sensor in a spectrometer and measurement of spectral-spatial-dynamics in an OLED.  This repository provides what you need in terms of ecad files and software to build one for yourself, or if you like you can contact me for an assembled board.

The files in the repo were produced on a Linux desktop computer (Fedora 37, Cinnamon Spin), using KiCAD 6 for the electrical design and Emacs for the firmware and python programs.

## Some Example Use Cases

### Spectrometer
The first shows the inside of a spectrometer built with a G1200 grating, two lenses and a slit. The mounts are 3D printed and everything is affixed to a stiff Al plate. There is a cover, not shown here, and specific surfaces are covered with low-reflectance tape.  The spectrum is that of a fluorescent ceiling light through a 200um fiber at a distance of about 8ft, with a 200um slit.

|![IMG_20231217_120419719 cropped350](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/df8263ad-cb4c-41de-bd71-2716a8124167)|![FluorescentLamp Screen500](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/6d012101-5bae-4bd2-b496-b92bc3603f1b)|

### Imaging Spectral-Spatial-Dynamics
The following example uses the sensor device to record the time evolution of the spatial distribution of light produced by an OLED at a current density of 150 $\mu A/cm^2$. The sensor device collects a series of frames wth 10ms shutters at 10ms intervals (back to back), with the series initated on a trigger at the start of the applied voltage waveform. The blue line shows the resulting current density in the OLED, the pale line shows the sync pulses output by the sensor device at the start of each frame, and the red line is the applied voltage (2.41V).  (We plan to post repositories for the DAQ board and current amplifier as well.)

|![SampleR1 51 D7_G100K_A2 410v_I0 010s_D0 500s_N005_T4 0C_M17 0000mm_W501 340nm 20230309 080608 827395 tdaq2](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/93a6eb79-6a26-4ebe-8d46-3b25e46819c1)|![SampleR1 51 D7_G100K_A2 410v_I0 010s_D0 500s_N005_T4 0C surface cropped](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/c7ad51c5-3052-4c0e-905a-4b4ebf87df7e)|

# Electrical Design, Analog
The datasheet for the TCD1304 can be found here  https://toshiba.semicon-storage.com/us/semiconductor/product/linear-image-sensors/detail.TCD1304DG.html.
The following table is found on page 4.
We see that $V_{SAT}$ the saturation output volage runs from 450mV to 600mV, $V_{MDK}$ the dark signal is 2mV, thus a dynamic range of 300 (for a 10ms integration integral), $V_{OS}$ the DC output signal is typically 2.5V but can be from 1.5V to 3.5V, and $Z_O$ the output impedance is typically 500 ohms but can be 1K.  So, that is a lot of variation that we need to account for in our design.

![TCD1304-optical-electrical](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/7c0a2a66-d456-45e5-9f44-d4f6856260c5)

The datasheet provides the following diagram in Note 8 to illustrate the definition of $V_{OS}$.  The hatched area is the negative going output signal.  SS is ground

![TCD1304-electrical-note8](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/ceaa5288-26fd-4485-ae8d-bf512b9fe894)

For best performance in digitizing the signal, we want a circuit that takes the above signal as input and outputs a signal that matches the input range of our microcontroller's ADC.  The following circuit element shows one approach that does this and can run on a single supply. We can call this a shift, flip and amplify (SFA). The gain is set as usual for an inverting amplifier by $R_2$ and $R_1$ and the potentiometer provides the offset through the + input of the opamp.

![SimpleSchematic1](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/0bc8349a-38d2-42b1-843e-d70b2531a980)

But, recall that the datasheet says the output impedance $Z_0$ varies from 500 ohms to 1k.  If we connect that directly to our SFA then $Z_0$ becomes part of the gain equation, $R1\rightarrow R1+Z_0$, and in a bag of sensors we might find that the gain is different for each sensor.
So, we need to isolate that source impedance from the SFA.

A solution to this is to use an opamp follower as the first stage, as shown in the following crcuit.  The follower presents a very high impedance at its input, which is ideal for reading the voltage from the sensor, and a low impedance at its output, which is ideal as an input to the SFA

![TCD1304-opampfollower](https://github.com/user-attachments/assets/01444fcd-368b-4a48-a75c-3357dc1dcdea)

Referring again to the datasheet for the TCD1304, we see that (a) we can operate the sensor chip in the range 3V to 5.5V, (b) it requires a clock between 800KHz and 4MHz (2MHz if operated below 4V) and (c) the data readout rate is 1/4 of the clock 200kS/s to 1MS/s (500KS/S if powered below 4V).
Operating with a 3.3V supply, and making efficient use of the full scale range of the ADC,  means that we need to use a rail to rail opamp.
Our SFA architecture means that the opamp has to also have a wide common mode range. 
And the sampling rate means we need to look for a slew at least as faster as 100V/usec

The complete analog front end circuit is shown in the following LTSpice model based on the ADA4807.
We use the first opamp in the package for the voltage follower and the second opamp for the flip, shift,and amplify stage.
Gain and offset are as calculated above.
The green trace is the output from the sensor and the purple curve is the output from the second stage.
As can be seen the 2.5V to 1.9V signal from the sensor becomes a 0.1V to 3.1V for the ADC, and the rise time is small compared to the sampling period.
In our actual circuit we use a trim pot for the voltage applied to V+.

![Screenshot from 2023-12-18 08-43-01](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/b380a297-6e34-4aad-a0bb-7b30448e554a)

It might be noted that we could have chosen a larger gain to look at lower intensity light, or with the Teensy 3.2 we can use its built-in amplifier under software control, provided the offset is small.

## Analog input of the Teensy 4.x and Teensy 3.2
The following diagaram for the analog input, and one similar to this, are found in the datasheets for the i.MXRT106x (Teensy 4) and the K20 (Teensy 3).
This is a modified version of a SAR type analog input.  Normally the input sees the switched sampling capacitor.
Here there is a series resistor in front of the sampling capacitor.

![T3analoginput](https://github.com/user-attachments/assets/10ae87ce-2e72-4959-8f26-67fdcef17f1d)

In operation, the switch closes for a period of time to allow the capacitor to draw charge from the input.
At the end of the sampling period the switch is opened and the voltage on the capacitor is converted to a digital representation in the successive approximation register (SAR).
In other words, precision is determined by the length of the sampling window (in time) versus the RC time constant seen by the sampling capacitor.
Full precision means the voltage on the capacitor is within 1 LSB of the input, at the end of the sampling window.

For a simple RC network, the voltage on the capacitor is $V(t)/V(0) = 1 - exp(-t/RC)$.
Therefore, for n bits of precision, the sampling window needs to be at least $ln(2^{n}) \times RC$.
This works out to be 11 x RC for 16 bits and 8 x RC for 12 bits. 
The following shows this graphically.

![samplingtime](https://github.com/user-attachments/assets/6a794891-e235-473a-8f6f-eafbd615cedd)

For the T3 in 16 bit mode, with RADIN = 2k, and CADIN = 8pf, RC ~ 16nsecs.
The voltage on the sampling capacitor is within $1/2^{16}$ of the input voltage after about 170nsecs.

For the T4 in 12 bit mode, RADIN can be from 5k to 25k and CADIN is 1.5pF, RC ~ 7.5nsecs to 40nsecs.
We need about 60nsecs to 320nsecs to reach 12 bit precision.

Thus the T3 and T4 are both compatible at 500KSPS.  See the [ADC library](https://github.com/pedvide/ADC) for details setting the sampling time and conversion clock.

The tradeoff for the T3 with its 16 bit input, is that the T3 has a much slower USB interface, 1MB/s transfers to the host versus 60MB/s for the T4.
The best frame to frame interval with the T3 is about 16msecs and about 8msecs for the T4.

N.B.  Driving the analog input of an MCU is different from driving a normal SAR type ADC.  Normally, the drive circuit for a SAR includes an external capacitor to serve as a charge reservoir for the sampling capacitor.  This is pre-empted in the MCU analog input by the large internal resistance in series with the sampling capacitor.

## How many bits do I really need?
In the table above, the dynamic range is listed as 300.
This is simply the saturation output voltage 600mv divided by the dark signal 2mV.
If that were the limit, then 10 good bits would be enough.

In principle the dark noise should scale with exposure time or by cooling.  Emperically, using a special low noise linear radiometric design(*), we find that the noise floor for the TCD1304 seems to be at about 0.6mV which it reaches for exposures below about 10msec.  Modest cooling to 4C is reported elsewhere to reduce noise by a factor of 4.  So, we can estimate a practical limit for the dynamic range at about 4,000, or 12 bits.  That is what is offered by the T4 ADC.  If you want to do signal averaging, and given the usual performance specs of ADCs, it is reasonable to have some extra bits, and so a 16 bit ADC is a good choice for a high performance system.

Our analog section uses an ADA4807.  Its datasheet lists the input voltage noise as $3.1 nV/\sqrt Hz$.
At 500KSPS this becomes 2.2uV, and setting the gain at 5, we expect 10uV.  So, we are well ahead of the noise floor for the device.
Without going into a more detailed analysis, we see that our electrical noise can be about 1/2 of the dark signal.
That is workable with some signal averaging.

(*) Our high performance designs with differential signal paths and ADC are posted at
[TCD1304 Sensor Device with Linear Response and 16 bit Differential ADC](https://github.com/drmcnelson/TCD1304-Sensor-Device-with-Linear-Response-and-16-Bit-Differential-ADC)
and
[S11639-01 with 16 bit differentual ADC for SPI](https://github.com/drmcnelson/S11639-01-Linear-CCD-PCB-and-Code).
 Both of these have been built and tested and the repos now have all of the gerbers, bom, firmware and user code.
We plan to upload a repo with a cooled sensor in the near future.   The other advantage of those designs is that they are two board systems and provide better mechanical isolation if your are changing cables after alignment.  But, the design posted here offers sufficient precision and has fewer parts (i.e., it costs less and takes less time to build).

# CCD operation
Operationally, a CCD sensor stores charge in each pixel proportional to light and noise, until assertion of a shift pin causes the contents to be transferred to a buffer and then the contents are shifted along the buffer by a clock to the output pin and appear as a series of voltages.
The TCD1304 has an additional function that controls which shift assertions initiate the readout sequence.
The internal structure is depicted as follows from page 3 of its datasheet.  Externally the device is controlled by three pins, shift SH, integration clear gate ICG, and master clock $\phi M$.

![TCD1304-registers](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/1865363d-bbbe-4902-be47-285b8f0ef6f8)

The following two figures from the datasheet show how Toshiba envisions operation of the sensor chip.  As indicated, charge is integrated during the intervals between trailing edges at the SH pin. When the the ICG pin is low the accumulated charges are available in the readout buffer and then shifted to the output pin at a rate of 1 datum per four cycles of the master clock.

![TCD1304-timing1](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/e0c361c6-3cdb-4eb6-9314-ea43b90607dd)

![TCD1304-timing2](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/18aa49f5-0524-4cec-be0e-52a2ee25068b)

Toshiba labels the second diagram above as "Electronic Shutter Function".  This refers to the function of ICG in selecting which SH assertion results in charge being available in the readout buffer.  However it is not a shutter in the conventional sense.  It is easily shown experimentally that if the device is left idle, several SH cycles are needed to arrive at a noise level baseline in the readout.   There are a number of commerical CCD systems that clock the SH pin or its equivalent to keep the sensor "clean".  This can have important ramifications if the device is to be triggered, for example for kinetic studies.   Alternatives include good "dark" management, designing the experiment to start with a few blank frames to clear the sensor, and/or having the device initiate the trigger.

Note that it is the low state on the ICG pin that makes the readout available to be clocked out to the OS pin, while the SH pin sets the integration interval.

## Driving the SH, ICG and Master Clock pins
Referring again to the datasheet, page 6, we find the following table.
Notice that the shift gate has a capacitance of 600pF and the integration clear gate has 250pF.
From the magnitude of the capacitances, we might guess that these are closely related to moving charge to the output buffer.

![image](https://github.com/user-attachments/assets/b0cc4b91-a9f9-4a90-91d4-347e24e93084)

Notice also, that the master clock and data transfer rates are reduced when operating at lower voltages.

![image](https://github.com/user-attachments/assets/4048145d-1f1a-4894-bab4-06ee4319979c)

The large capacitances also factor into how we drive these pins, rise times with large capacitances are easily current limited.
The preferred way to drive the gate pins is with a buffer and series resistor, as shown in the following.
For a 3.3V drive, a 150 ohm resistor sets the rise to 90nsecs for the SH pin and limits the current to 22mA.
For a 1 usec pulse, this gives a rise time that is less than 1/10 of the pulse width.
The buffer can be a 74LVC1G34, or one channel of a 74VLC3G34, which can drive 25mA.

![singlegate](https://github.com/user-attachments/assets/c67f9783-9346-4ee8-9e86-164fbfa76bcd)

For comparison, the Teensy digital I/O pins provide 4mA.
If the gate is driven directly from the Teensy, the response is current limited, $\Delta t \approx C \Delta V / I$.
That works out to 500nsecs.

Nonetheless, driving the gates from the digital I/O pins does work, provided the pulse duration is sufficient to accomodate the current limited pulse rise time.
For a TCD1304 board with fast buffer drivers, see our repo ![here](https://github.com/drmcnelson/TCD1304-Sensor-Device-Designed-for-Linear-Response-and-Reproducibility).   An alternate version of the present design with buffer drivers and support for the FlexPWM is in test at this writing.


## Frame rates, shutters and timing for data acquisition.
The following diagram from page 9 of the datasheet shows the timing requirements for the ICG and SH pins relative to each other and the master clock.

![TCD1304-timingreqs](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/6256bbf6-0993-47ce-8623-0f77907d3063)

With a 2MHz master clock, it takes about 7.4ms to read one record from the device into the memory of the microcontroller.  Transfer from the microcontroller to the host PC can take an additional 5ms for the Teensy 3.x (12 Mb/s) or about 120usec with the Teensy 4.x (480 Mb/s).  Needless to say, this sets the maximum frame rate, that is the time between readouts, and is different from the minimum integration interval which depends only on the minimal interval between successive trailing edges at the SH pin.

For data collection, we need to be able to set integration and frame intervals freely (within the physical limits of the sensor) and we need to be able to reliably control timing with respect to an external trigger or gate.   Since the shutter or integration interval is defined by successive assertions of the SH pin, we focus on how these requirements translate to operation of this pin.

The following diagram shows a sequence where the shutter interval is shorter than the inter frame interval, and the frame interval is not necessarily an integer number of shutter intervals in length.  The SH pin operates with alternating short and long intervals.  The frame interval is the sum of these two intervals, or that betwen every second SH.

![Shutter-Operation-shortshutter](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/2f8c72b0-5ce8-4873-b6db-025a604f5a09)

The following shows back to back shutters with identical frame and shutter intervals.

![Shutter-Operation-longshutter](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/ff287173-09d6-4ee7-a6b2-3ac014f4b3f3)

And this one shows a constant shutter interval with the frame interval an integer multiple of the shutter.

![Shutter-Operation-modalshutter](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/040c281e-0778-4aa0-9e86-23f334caaec4)

Our control logic for operating the device has to accomodate all three scenarios, and we have to be able to initiate frames or sets of frames from a clock or from a trigger.
Architecturally, we set this up as two ISRs, which we call ShutterA_isr() and ShutterB_isr().  The sequences are easily implemented by various combinations of A and B or just B after one A, and easily triggered, gated or clocked as needed.

## Signals, Sync and Busy
The sensor device has a trigger input and two outputs SYNC and BUSY.  Sync can be configured to signal the start of a series of frames or to signal the start of each shutter.  SYNC can also be configured to be followed by a holdoff.  BUSY is set with the start of the first shutter and remains asserted until the last data transfer is complete.   Each pin can be set nominally HIGH or LOW, assertion is the opposite.  Additionally there is a SPARE pin.  Any of the pins can be manually set, cleared, toggled or pulsed.

The following shows the actual operation of the sensor device, green is SH, purple is ICG, blue is SYNC and yellow is BUSY.  We see BUSY goes high immediately, SH and ICG operate as described in the data sheet.  Light is integrated between the two trailing edges and the data is transferred following the second assertion of the SH and clearing of the ICG pins.

![Scope-singleframe](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/611557d1-e55c-48d0-b26d-84dc9b8b2dfe)

## Triggered and gated operation
The following shows a triggered frame.  The commands are "set trigger rising", "trigger 20" to trigger one 20usec frame.  The blue line is the trigger input.  There is a small reproducible delay between the trigger and the shutter, due to certain fixed timings involved in getting the shutter started.  Notice the alignment of the BUSY and SH signals.

![Scope-triggerd](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/9310338d-311b-468d-9891-1daa79e466c9)

The following shows a gated frame, blue is the gate signal.  The commands are "set trigger change", "gate 1", to gate one frame.  For this, the spare pin is connected to the trigger input and we enter the command "pulse spare 20" to output a 20 usec pulse.  As you can see the shutter sequence begins and finishes with the gate.

![Scope-gated](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/5682114e-f9e4-41eb-b6de-08bfdad62288)


# Data processing
Referring to the clock diagrams above, we see that the data record comprises 12 dummy outputs followed by 13 light shielded elements, followed by 3 shadowed elements, followed by the 3648 elements making up the effective output and followed by another 14 dummy elements.   Thus elements 12 thru 24 provide a baseline which we can average or take the median and subtract from elements 28 through 2675 which form the image.   In the spirit of "always preserve primary data", we do not do this substraction nor any scaling, in firmware.  Rather we pass the entire record as is, to the PC host and the host software is responsible for subtracting and scaling as appropriate.

# Firwmare
The firmware subdirectory contains a sketch for Teensy 4.0.  After assembling your board (or obtaining one from the author), you can mount your Teensy 4 and TCD1304 detector, and then flash the program into the Teensy 4.0.   The commands are in english, ASCII over USB.  Enter the command "help" to get a listing and short explanation of the commands.  A listing of the help text is included in the Python subdirectory.

The default action returns the data in binary.  You will ideally want to use a multi-threaded program on your host computor, with one thead reading the responses and data and queuing as appropriate to a separate graphics thread.   The Python code provides a Class for the device that handles all of this.

# Python
The Python subdirectory contains a file TCD1304Rev2Controller.py and three library files that it looks for in its directory.  The program implements threads using the multitasking interface in Linux.  A dedicated thread reads the responses from the device and queues data to an internal data queue for the "save" command and to a queue read by the runtime graphics thread.  The Python program has a help command, like the firmware.

# Adjusting the offset
With a voltmeter or scope, the middle pin on the trim pot should be set close to 2.1 volts.  Then in the Python program, issue the commands "baseline off" and "clock 1000 10000 100000".   This wil turn off the baseline subtraction function and clock 1000 frames with an integration time of 10ms spaced at intervals of 100ms.   Try the commands "stop", "baseline on" and repeat the clock command to see a comparison.  You can zoom in on the graphical display and use a small screwdriver to adjust the offset.   I usually put the sensor in a drawer or cover it with a black cloth while I do this.

