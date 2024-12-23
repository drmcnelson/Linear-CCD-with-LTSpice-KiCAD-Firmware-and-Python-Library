
# Introduction
Linear CCDs can be useful tools in science, for example as a sensor for a spectrometer or imaging system.  And indeed, there are many commercial instruments that are based on low cost linear CCDs such as the Toshiba TC1304 and the Sony IXL511.
Here we provide design files, firmware and software for a device that provides competitive performance and a rich set of science-centric features, and that can be customized to your experiments, all at a fraction of the cost of the commercial offerings.  The design is based on the Toshiba TCD1304 (3648/3694 pixels) and Teensy 4.0 (600MHz ARM, 480MHz USB).  The Teensy 3.2 is plug compatible for this design.
Read further below, and you will find tutorials on electrical design and managing the CCD to provide flexible timing and reproducible measurements so that you can use it as a scientific instrument.

In this repository you will find directories containing (a) electrical design files in KiCad, (b) firmware (a sketch file) for the Teensy, and (c) a Python class library with graphical and command line utilities to operate the device.
The device has a trigger or gate input, sync and busy outputs, and spare pins for digital and analog I/O.
The operating modes include clocked, triggered frames, triggered series of frames, and gated frames (where the shutter is opened and closed on the rising and falling edges of the logic level input signal).
The device appears as a serial port (COM in windows) with comands and response in human language ASCII and frames returned as binary or formatted.
The python class library provides higher level functions, a graphical realtime display and a command line processor.  There is a help command in the firmware and in the Python command line interpreter.  Details of the electrical and firmware design are described later in this README.

Since the original upload, we have added new sections on driving the shift and integration clear gates, the analog interface to the microcontroller, and precision (number of bits) versus integration time.

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

The datasheet provides the following diagram in Note 8 to illustrate the definition of $V_{OS}$.  The hatched area is the negative going output signal.  SS is ground.

![TCD1304-electrical-note8](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/ceaa5288-26fd-4485-ae8d-bf512b9fe894)

For best performance in digitizing the signal, we want a circuit that takes the above signal as input and outputs a signal that matches the input range of our microcontroller's ADC.  The following circuit element shows one approach that does this and can run on a single supply. We can call this a shift, flip and amplify (SFA). The gain is set as usual for an inverting amplifier by $R_2$ and $R_1$ and the potentiometer provides the offset through the + input of the opamp.

![SimpleSchematic1](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/0bc8349a-38d2-42b1-843e-d70b2531a980)

But, recall that the datasheet says the output impedance $Z_0$ varies from 500 ohms to 1k.  If we connect that directly to our SFA then $Z_0$ becomes part of the gain equation, $R1\rightarrow R1+Z_0$, and in a bag of sensors we might find that the gain is different for each sensor.
So, we need to isolate that source impedance from the SFA.

A solution to this is to us an opamp follower as the first stage, as shown in the following crcuit.  The follower presents a very high impedance at its input, which is ideal for reading the voltage from the sensor, and a low impedance at its output, which is ideal as an input to the SFA

![TCD1304-opampfollower](https://github.com/user-attachments/assets/01444fcd-368b-4a48-a75c-3357dc1dcdea)

Referring again to the datasheet for the TCD1304, we see that (a) we can operate the sensor chip in the range 3V to 5.5V, (b) it requires a clock between 800KHz and 4MHz and (c) the data readout rate is 1/4 of the clock 200kS/s to 1MS/s.
So, we can power our circuit from the 3.3V supply provided we select a rail to rail opamp with a sufficiently wide common mode range and sufficient fast slew, and the onboard ADC is fast enough for the readout.
Compatiblity with the 3.3V supply and sampling rate for the ADC saves space and cost.
However there is one proviso for the lower voltage, clock rates operating under 4.0V are limited to 2.4MHz, and data rates are limited to 0.6MHz.

The complete analog front end circuit is shown in the following LTSpice model based on the ADA4807.
We use the first opamp in the package as a follower, which isolates the varied impedance of the sensor from the rest of the circuit, and we use the second opamp for the flip, shift,and amplify stage.
Gain and offset are as calculated above.
The green trace is the output from the sensor and the purple curve is the output from the second stage.
As can be seen the 2.5V to 1.9V signal from the sensor becomes a 0.1V to 3.1V for the ADC.
In our actual circuit we use a trim pot for the voltage applied to V+.
Importantly, we also see that the rise time is small on the 0.2usec scale shown in the SPICE model.

![Screenshot from 2023-12-18 08-43-01](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/b380a297-6e34-4aad-a0bb-7b30448e554a)

It might be noted that we could have chosen a larger gain to look at lower intensity light, or with the Teensy 3.2 we can use its built-in amplifier under software control, provided the offset is small.

## Analog input of the Teensy 4.x and Teensy 3.2
The following diagaram is from the K20 datasheet (the Teensy 3.2) and is nearly identical that in the datasheet for the i.MXRT106x (Teensy 4).
For the T3, e RADIN = 2k and CADIN = 8pf in 16 bit mode, and so the analog input has a 16nsec time constant.
For the T4, RADIN can be from 5k to 25k and CADIN = 1.5pF, in 12 bit mode, and thus 7.5nsecs to 40nsecs.
Each pixel level from the TCD11304 lasts four clock cycles, or 2usecs.
The analog input is timed in firmware to sample the "flat" of each pixel output from the sensor.

![T3analoginput](https://github.com/user-attachments/assets/10ae87ce-2e72-4959-8f26-67fdcef17f1d)

## When do I need 16 bits?
Notice that the Teensy 3.2 has a 16 bit analog input and the Teensy 4.0 has 12 bits.
In the table above, the dynamic range at 300 is simply the sataturation output voltage 600mv divided by the dark signal 2mV.
So on face value, 10 good bits would be enough and the T4 is a good match.
But this dark signal is proportional to integration time.
At an integration time of 100usecs, the dark signal is 20uV, and the dynamic range would be 30,000.
That would put us in the range for a 16 bit ADC.
But, will our analog front end meet that challenge?
The datasheet for the ADA4807 lists the input voltage noise as $3.1 nV/\sqrt Hz$.
At a sampling rate 500KSPS, we would expect 2.2uV of noise at the input and our gain of 5, brings us to 10uV.
Without going into a more detailed analysis, we see that we electrical noise is already about 1/2 of the dark signal.
So that is more or less the limit for this simple design, 100usec exposures perhaps with signal averaging. could use 16 bits.

If you want a lower noise device, see my postings at
[TCD1304 with 16 bit differential ADC for SPI](https://github.com/drmcnelson/TCD1304-SPI)
and
[S11639-01 with 16 bit differentual ADC for SPI](https://github.com/drmcnelson/S11639-01-Linear-CCD-PCB-and-Code)

The Hamamatsu has been built and tested.
The TCD1304 with extra low noise 16 bit ADC, is waiting for sponsors.

A cooled version of the TCD1304 is being designed and will be posted, and is also ready for sponsors.



# CCD operation

Operationally, a CCD sensor stores charge in each pixel proportional to light and noise, until assertion of a shift pin causes the contents to be transferred to a buffer and then the contents are shifted along the buffer by a clock to the output pin and appear as a series of voltages.
The TCD1304 has an additional function that controls which shift assertions initiate the readout sequence.
The internal structure is depicted as follows from page 3.  Externally the device is controlled by three pins, shift SH, integration clear gate ICG, and master clock $\phi M$.

![TCD1304-registers](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/1865363d-bbbe-4902-be47-285b8f0ef6f8)

The following two figures from the datasheet show how Toshiba envisions operation of the sensor chip.  As indicated, charge is integrated during the intervals between trailing edges at the SH pin. At each assertion of the ICG pin the accumulated charges are shifted to the readout buffer and then shifted to the output pin at a rate of 1 datum per four cycles of the master clock.

![TCD1304-timing1](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/e0c361c6-3cdb-4eb6-9314-ea43b90607dd)

![TCD1304-timing2](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/18aa49f5-0524-4cec-be0e-52a2ee25068b)

Toshiba labels the second diagram above as "Electronic Shutter Function".  This refers to the function of ICG in selecting which SH interval is transferred to the readout buffer.  However it is not a shutter in the conventional sense.  It is easily shown experimentally that if the device is left idle, several SH cycles are needed to arrive at a noise level baseline in the readout.   There are a number of commerical CCD systems that clock the SH pin or its equivalent to keep the sensor "clean".  This can have important ramifications if the device is to be triggered, for example for kinetic studies.   Alternatives include good "dark" management, designing the experiment to start with a few blank frames to clear the sensor, and/or having the device initiate the trigger.

Note that it is the ICG pin that makes the readout available to be clocked out to the OS pin, while the SH pin sets the integration interval.  This seems reversed from the arrangement of buffers in the first diagram.   In practice, device operation agrees with the timing diagrams.

## Driving the SH, ICG and Master Clock pins
The architecture with a PN photodiode sensing element and three pins and three steps in reading out the device might be seen as  somewhat analogous to an interline transfer type architecture.  There the PN photodiodes are first shifted in parallel each to their neighboring "vertical" CCD buffer.  Charge is then clocked along the vertical CCD buffers, and line-by-line transfered to a horizontal CCD buffer and then clocked along the horizontal buffer until it reaches the output pin.

In the TCD1304 we have one line, so in place of the vertical CCD we have a single buffer with one element per pixel and we need just one pulse to move it to the output buffer.  That four clock cycles are needed per pixel suggests that the output buffer is a 4-phase CCD.

Referring again to the datasheet, page 6, we find the following table.   Notice that shift gate pin has a capacitance of 600pF and the integration clear gate has 250pF.  These large capacitances suggest that the applied voltges are what moves charge from the sensing element into the first buffer and then from that buffer to the CCD output buffer.
The large capacitances also factor into how we drive these pins.

![image](https://github.com/user-attachments/assets/b0cc4b91-a9f9-4a90-91d4-347e24e93084)

Notice that the master clock and data transfer rates are reduced when operating at lower voltages.

![image](https://github.com/user-attachments/assets/4048145d-1f1a-4894-bab4-06ee4319979c)

Rise times with large capacitances are easily current limited.
So, the preferred way to drive the pins is with a buffer gate and series resistor, as shown in the following.
For a 3.3V drive, a 150 ohm resistor sets the rise to 90nsecs for the SH pin and limits the current to 22mA.
For a 1 usec pulse, this gives a rise time that is less than 1/10 of the pulse width.  The buffer can be a 74LVC1G34, or one channel in a 74VLC3G34, which can drive 25mA.

![singlegate](https://github.com/user-attachments/assets/c67f9783-9346-4ee8-9e86-164fbfa76bcd)

Alternatively, all three channels of a 74LVC3G34 can be combined as in the follower, to drive a total of 75mA.  In this configuration, the rise time on the SH pin can be about 26nsecs.

![triplegate](https://github.com/user-attachments/assets/308c0ed6-acde-4bdb-b933-d6d29510eb8a)

For comparison, the Teensy digital I/O pins provide 4mA.
That means the response is current limited, $\Delta t \approx C \Delta V / I$.
That works out to 500nsecs, or about 1/2 of the 1 usec pulse.
That is long for a logic input, but the input pins for the TCD1304 seem to not actually be logic inputs as described above.

In the repository, we have two board designs.
The original, driving the SH, ICG and master clock directly, and another driving the gates and master clocks with buffers.

Caveat, the buffer gate version is a simple modification from the direct-gate-drive board, but as of this writing I have not yet built one of these.
(Click the sponsor button if you like, and send me a note. When sponsorships match costs, I'll build some and make then available).


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

