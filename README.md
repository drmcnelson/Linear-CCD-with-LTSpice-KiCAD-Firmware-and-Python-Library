
# Introduction
This repository provides electrical design, firmware and a Python class for a Linear CCD and Microcontroller that features low noise, high precision and high bandwidth with reproducible timing for trigger, gate and sync operations with a rich set of features to support experiments in spectrometry and imaging.  In the following we describe electrical and firmware designs for interfacing a CCD to a micontroller.  We set the device up with a human readable control interface (serial over USB with binary data transfer) and we provide a Python class library and utility for operating the device.  The design uses a Toshiba TCD 1340 and Teensy 4.x or 3.x.  The design is easily adapted to other linear CCDs and microcontrollers.

Following are some examples of how the device has been used.  The first is a spectrometer using a G1200 grating, two lenses and a slit, with 3D printed mounts and all affixed to a stiff Al plate. Black tape attenuates stray reflections (when the curve is on, not shown here).  The spectrum is that of a fluorescent ceiling light through a 200um fiber at a distance of about 8ft, with a 200um slit.  Notice the resolution and signal/noise are quite reasonable.  The input circuit that we will describe in a moment, helps preserve dynamic range and precision, which as it turns out is one of the tricky parts in recording these spectra (though it is also not so easy to appreciate at screen resolution).

|![IMG_20231217_120419719 cropped350](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/df8263ad-cb4c-41de-bd71-2716a8124167)|![FluorescentLamp Screen500](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/6d012101-5bae-4bd2-b496-b92bc3603f1b)|

You can see the sensor side of the board in the picture above.  The following shows the reverse side of the board.  There are digital i/o pins across the top for trigger input, sync and busy output, the logic signals going to the sensor, and some spares that can be controlled through the user interface.  Across the bottom there are some analog inputs with grounds for each and 3.3V that can be used for an auxiliary device.

![IMG_20231215_144019112_cropped250](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/1eda6d73-2e27-4ffd-ba63-d32f814700c4)

The following example uses the sensor device to record the time evolution of the spatial distribution of light produced by an OLED at a current density of 150 $\mu A/cm^2$. The sensor device collects a series of frames wth 10ms shutters at 10ms intervals (back to back), with the series initated on a trigger at the start of the applied voltage waveform. The blue line shows the resulting current density in the OLED, the pale line shows the sync pulses output by the sensor device at the start of each frame, and the red line is the applied voltage (2.41V).  (We plan to post repositories for the DAQ board and current amplifier as well.)

|![SampleR1 51 D7_G100K_A2 410v_I0 010s_D0 500s_N005_T4 0C_M17 0000mm_W501 340nm 20230309 080608 827395 tdaq2](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/93a6eb79-6a26-4ebe-8d46-3b25e46819c1)|![SampleR1 51 D7_G100K_A2 410v_I0 010s_D0 500s_N005_T4 0C surface cropped](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/c7ad51c5-3052-4c0e-905a-4b4ebf87df7e)|

Okay, with the above, I hope that I have convinced you that this not just a nifty little sensor device, but that it is actually a professional grade instrument and that you can do good science with it.  For more good news, it turns out that it is not very expensive.  This repository provides what you need in terms of ecad files and software to build one for yourself, or if you like you can contact me for an assembled board.

# Electrical
We start with the datasheet for the TCD1304, which can be found here  https://toshiba.semicon-storage.com/us/semiconductor/product/linear-image-sensors/detail.TCD1304DG.html.
The following table is found on page 4.   We see that $V_{SAT}$ the saturation output volage runs from 450mV to 600mV, $V_{MDK}$ the dark signal is 2mV, thus a dynamic range of 300 (for a given integration integral), $V_{OS}$ the DC output signal is typically 2.5V but can be from 1.5V to 3.5V, and $Z_O$ the output impedance is typically 500 ohms but can be 1K.  So that is a lot of variation that we need to account for in our design.

![TCD1304-optical-electrical](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/7c0a2a66-d456-45e5-9f44-d4f6856260c5)

To understand what is meant by $V_{OS}$ and the saturation and dark signal levels, we have the following diagram from note 8.  $V_{OS}$  is the baseline and SS is ground.

![TCD1304-electrical-note8](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/ceaa5288-26fd-4485-ae8d-bf512b9fe894)

This means that we need to match a signal that runs from 2.5V to 1.9V (nominally(, to our micocontroller's ADC input which digitizes voltages in the range from 0 to 3.3V.  If we simply feed the signal from the TCD1304 straignt to the ADC we loose bits and unleash a number of other electrical issues.  Instead, to make best use of the ADC, we want to flip, shift and amplify the output from the sensor to match the input range of the ADC.   Additionally, we need to accomodate the variation in output impedance and $V_{OS}$, and finally since most micocontrollers use successive approximation ADC's, we need to  account for the kickback with an appropriate RC at the output of our instrumentation grade front end.  There are a number of application notes about driving SAR ADC's, for example https://www.analog.com/en/analog-dialogue/articles/front-end-amp-and-rc-filter-design.html.

The following shifts, inverts and amplifies the negative going CCD signal in one opamp. The gain is set as usual for an inverting amplifier by $R_2$ and $R_1$ and the potentiometer provides the offset throughn the + input of the opamp.

![SimpleSchematic1](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/0bc8349a-38d2-42b1-843e-d70b2531a980)

For completeness, we mention that we could instead just amplify and shift, like this.  Apart from some esoteric issues, this approach is less convenient for example for further amplifying the signal and it may require more processing to convert the signal to an intensity representation, although we will already be doing a baseline subtraction as will be explained.  The Teensy 3.x for example provides a pogrammable amplifier that can be used with the analog inputs.  But we could apply further amplification for low light signals with the first approach above and not with this second approach where we leave the signal offset and inverted.  So, we choose the above and not this.

![SimpleSchematic2](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/1cbbdc91-9d4a-4894-bc4e-5f4fc0f7d283)

As noted, we choose the first option above for our design.  But, this means we need to insert a follower between the sensor and our flip,shift,amplify circuit so that the output impedance of the sensor does not effect our gain.  Using a dual opamp package makes this convenient and cost effective.

Referring again to the datasheet for the TCD1304, we see that (a) we can operate the sensor chip in the range 3V to 5.5V, it requires a clock between 800KHz and 4MHz and the data readout rate is 1/4 of the clock 200kS/s to 1MS/s. This means we can power our circuit and the TCD1304 from the well conditioned 3.3V supply provided by the Teensy 3.x or Teensy 4.x, and in either microntroller the onboard ADC can keep up with the required datarate.  But to do that we need a rail to rail opamp with a wide common mode range to accomodate our offset voltage and a bandwidth substantially greater than our datarate.   We save cost and real-estate on the board by using the 3.3V supply.  This is a convenience of the TCD1304 relative to some other linear CCDs.

The following LTSpice model based on the ADA4807 seems to meet our goals.  We use the first opamp in the package for the follower and the second opamp for the flip.shift,amplify circuit with gain and offset as calculated above.  The green trace is the output from the sensor and the purple curve is the output from the second stage.  As can be seen the 2.5V to 1.9V signal from the sensor becomes a 0.1V to 3.1V for the ADC.  In our actual circuit we use a trim pot for the voltage applied to V+.

![Screenshot from 2023-12-18 08-43-01](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/b380a297-6e34-4aad-a0bb-7b30448e554a)

It might be noted that we could have chosen a larger gain to look at lower intensity light and still have a feasible value of V+.  Or as noted above, the output is compatible with the amplifier availble in the Teensy 3.x and various other microcontrollers.

One final note on the front end circuit:  We mentioned above that we are going to be driving a SAR ADC and that there is a kickback associated with the first sample.  In our circuit design, we have an optional 1nf capacitor to ground after the output resistor to take care of the kickback.

Caveat:  The devices built prior to this use a ADA4896 and 2K in the feedback loop for a gain of 4.  This part has a smaller common mode range, 0.1V to 2.1V when powered at 3V, which is cutting it a bit close for our purposes.  The ADA4807 is available in the same footprint and has a common mode range that is essentially rail to rail.  Hence we are switching to the ADA4807 for new builds.

# CCD operation

For overview and context, the basic operation of a typical CCD is that in each pixel charge is integrated proportional to photons pluse noise, until a shift operation transfers the charges to a register from where it is clocked to the output pin as a series of voltage levels.  Integration occurs during the period between shifts.  The TCD1304 adds one function, to allow readout following selected shift assertions. Thus, the TCD1304 takes 3 logic input signals, referred to in the data sheet as $\phi M$ master clock, SH shift gate and ICG integration clear gate.  The internal structure is depicted as follows from page 3.   The terminology is somewhat confusing and not less so in the context of the diagram.   Nonetheless, integration occurs during the interval between sucessive trailing edges on the SH pin, and the shift to the readout register occurs with the assertion of the ICG pin.

![TCD1304-registers](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/1865363d-bbbe-4902-be47-285b8f0ef6f8)

This can be seen in the following two figures from the datasheet which show how Toshiba envisions operation of the sensor chip.  As indicated, charge is integrated in each of the pixels during the intervals between trailing edges at the SH pin and at each assertion of the ICG pin the accumulated charges are shifted to the readout buffer and then clocked out on the output pin at a rate of 1 datum per four cycles of the master clock.

![TCD1304-timing1](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/e0c361c6-3cdb-4eb6-9314-ea43b90607dd)

![TCD1304-timing2](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/18aa49f5-0524-4cec-be0e-52a2ee25068b)

Toshiba labels the second diagram above as "Electronic Shutter Function".  This refers to the function of ICG in selecting which SH interval is transferred to the readout buffer.  However it is not a shutter in the conventional sense.  It is easily shown experimentally that if the device is left idle, several SH cycles are needed to arrive at a noise level baseline in the readout.   There are a number of commerical CCD systems that clock the SH pin or its equivalent to keep the sensor "clean".  This can have important ramifications if the device is to be triggered, for example for kinetic studies.   Alternatives include good "dark" management, designing the experiment to start with a few blank frames to clear the sensor, and/or having the device initiate the trigger.

In practice, we find that the clock, SH and ICG pins can be driven using logic gates or by the Teensy directly with equal reliability.  For cost and space we drive directly in the present example.  It is straightforward to add a quad logic gate to the design if you wish.

Toshiba further specifies timing requirements for the ICG and SH pins relative to each other and the master clock, in the following diagram from page 9 of the datasheet.  Note that this sets a minimum shutter period since the fastest shutter is between two trailing edges at the SH pin.

![TCD1304-timingreqs](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/6256bbf6-0993-47ce-8623-0f77907d3063)

With a 2MHz master clock, it takes about 7.4ms to read one record from the device into the memory of the microcontroller.  Transfer from the microcontroller to the host PC can take an additional 5ms for the Teensy 3.x (12 Mb/s) or about 120usec with the Teensy 4.x (480 Mb/s).  Needless to say, this sets the maximum rate, that is the time between readouts.  The integration time, the interval between SH assertions can be much shorter.

## Shutter, frames and timing for data acquisition.
For data collection, we need to be able to set integration and frame intervals freely (within the physical limits of the sensor) and we need to be able to control timing with respect to an external trigger or gate.   For purposes of understanding our requirements we can focus on the SH pin since the shutter or integration interval is defined by successive assertions of this input.

The following diagram shows a sequence where we have shutter interval shorter than the inter frame interval, and the frame interval is not necessarily some integer number of shutter intervals in length.  This immediately removes us from the scenario where we simply run the SH line on a single clock, as does our requirements for triggering or gating the CCD with reproducible timing. If we want to do science with a CCD, and even moreso if we want to knietics, we need to think about SH a little differently.   

![Shutter-Operation-shortshutter](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/2f8c72b0-5ce8-4873-b6db-025a604f5a09)

That said, there are scenarios where the shutter is somewhat simpler.  The following shows back to back shutters with identical frame and shutter intervals.

![Shutter-Operation-longshutter](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/ff287173-09d6-4ee7-a6b2-3ac014f4b3f3)

And this one shows a constant shutter interval with the frame interval an integer multiple of the shutter.

![Shutter-Operation-modalshutter](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/040c281e-0778-4aa0-9e86-23f334caaec4)

Our logic for operating the device has to accomodate all three scenarios, and be able to launch them from a trigger or simply clock them on command from the user.  Architecturally, we set this up as two ISRs, which we call ShutterA_isr() and ShutterB_isr().  The sequences are easily implemented by various comobinations of A and B or just B after one A, and easily triggered, gated or clocked as needed.

## Signals, Sync and Busy
The sensor device has a trigger input and two outputs SYNC and BUSY.  Sync can be configured to signal the start of a series of frames or to signal the start of each shutter.  SYNC can also be configured to be followed by a holdoff.  BUSY is set with the start of the first shutter and remains asserted until the last data transfer is complete.   Each pin can be set nominally HIGH or LOW, assertion is the opposite.  Additiionall there is a SPARE pin.  Any of the pins can be manually set, cleared, toggled or pulsed.

The following shows the actual operation of the sensor device, green is SH, purple is ICG, blue is SYNC and yellow is BUSY.  We see BUSY goes high immediately, SH and ICG operate as described in the data sheet.  Light is integrated between the two trailing edges and the data is transferred following the second assertion of the SH and clearing of the ICG pins.

![Scope-singleframe](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/611557d1-e55c-48d0-b26d-84dc9b8b2dfe)

## Triggered and gated operation
The following shows a triggered frame.  The commands are "set trigger rising", "trigger 20" to trigger one 20usec frame.  The blue line is the trigger input.  There is a small reproducible delay between the trigger and the shutter, due to certain fixed timings involved in getting the shutter started.  Notice the alignment of the BUSY and SH signals.

![Scope-triggerd](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/9310338d-311b-468d-9891-1daa79e466c9)

The following shows a gated frame, blue is the gate signal.  The commands are "set trigger change", "gate 1", to gate one frame.  For this, the spare pin is connected to the trigger input and we enter the command "pulse spare 20" to output a 20 usec pulse.  As you can see the shutter sequence begins and finishes with the gate.

![Scope-gated](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/5682114e-f9e4-41eb-b6de-08bfdad62288)


# Data processing
Referring to the clock diagrans above, we see that the data record comprises 12 dummy outputs followed by 13 light shielded elements, followed by 3 shadowed elements, followed by the 3648 elements making up the effective output and followed by another 14 dummy elements.   Thus elements 12 thru 24 provide a baseline which we can average or take the median and subtract from elements 28 through 2675 which form the image.   In the spirit of "always preserve primary data", we do not do this substraction nor any scaling, in firmware.  Rather we pass the entire record as is, to the PC host and the host software is responsible for subtracting and scaling as appropriate.

# Firwmare
The firmware subdirectory contains a sketch for Teensy 4.0.  After assembling your board (or obtaining one from this author), you can mount your Teensy 4 and TCD1304 detector, and then flash the program into the Teensy 4.0.   The commands are in english, ASCII over USB.  Enter the command "help" to get a listing and short explanation of the commands.  A listing of the help text is included in the Python subdirectory.

The default action returns the data in binary.  You will ideally want to use a multi-threaded program on your host computor, with one thead reading the responses and data and queuing as appropriate to a separate graphics thread.   The Python code provides a Class for the device that handles all of this.

# Python
The Python subdirectory contains a file TCD1304Rev2Controller.py and three library files that it looks for in its directory.  The program implements threads using the multitasking interface in Linux.  A dedicated thread reads the responses from the device and queues data to an internal data queue for the "save" command and to a queue read by the runtime graphics thread.  The Python program has a help command, like the firmware.

# Adjusting the offset
With a voltmeter or scope, the middle pin on the trim pot should be set close to 2.1 volts.  Then in the Python program, issue the commands "baseline off" and "clock 1000 10000 100000".   This wil turn off the baseline subtraction function and clock 1000 frames with an integration time of 10ms spaced at intervals of 100ms.   Try the commands "stop", "baseline on" and repeat the clock command to see a comparison.  You can zoom in on the graphical display and use a small screwdriver to adjust the offset.   I usually put the sensor in a drawer or cover it with a black cloth while I do this.

