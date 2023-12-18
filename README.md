


# Introduction
This repository provides electrical design, firmware and a Python class for a Linear CCD and Microcontroller that features low noise, high precision and high bandwidth with reproducible timing for trigger, gate and sync operations with a rich set of features to support experiments in spectrometry and imaging.  In the following we describe electrical and firmware designs for interfacing a CCD to a micontroller.  We set the device up with a human readable control interface (serial over USB with binary data transfer) and we provide a Python class library and utility for operating the device.  The design uses a Toshiba TCD 1340 and Teensy 4.x or 3.x.  The design is easily adapted to other linear CCDs and microcontrollers.

Following are some examples of how the device has been used.  The first is a spectrometer using a G1200 grating, two lenses and a slit, with 3D printed mounts and all affixed to a stiff Al plate. Black tape attenuates stray reflections (when the curve is on, not shown here).  The spectrum is that of a fluorescent ceiling light through a 200um fiber at a distance of about 8ft, with a 200um slit.  Notice the resolution and signal/noise are quite reasonable.  The input circuit that we will describe in a moment, helps preserve dynamic range and precision, which as it turns out is one of the tricky parts in recording these spectra (though it is also not so easy to appreciate at screen resolution).

|![IMG_20231217_120419719_cropped300](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/de198b21-05c3-4d13-88c1-3835d190116e) |![FluorescentLamp Screen500](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/6d012101-5bae-4bd2-b496-b92bc3603f1b)|


The following example uses the sensor device to record the time evolution of the spatial distribution of light produced by an OLED at a current density of 150 $\mu A/cm^2$. The sensor device collects a series of frames wth 10ms shutters at 10ms intervals (back to back), with the series initated on a trigger at the start of the applied voltage waveform. The blue line shows the resulting current density in the OLED, the pale line shows the sync pulses output by the sensor device at the start of each frame, and the red line is the applied voltage (2.41V).  (We plan to post repositories for the DAQ board and current amplifier as well.)

|![SampleR1 51 D7_G100K_A2 410v_I0 010s_D0 500s_N005_T4 0C_M17 0000mm_W501 340nm 20230309 080608 827395 tdaq2](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/93a6eb79-6a26-4ebe-8d46-3b25e46819c1)|![SampleR1 51 D7_G100K_A2 410v_I0 010s_D0 500s_N005_T4 0C surface cropped](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/c7ad51c5-3052-4c0e-905a-4b4ebf87df7e)|

Okay, with the above, I hope that I have convinced you that this not just a nifty little sensor device, but that it is actually a professional grade instrument and that you can do good science with it.  For more good news, it turns out that it is not very expensive.  This repository provides what you need in terms of ecad files and software to build one for yourself, or if you like you can contact me for an assembled board.

# Electrical
We start with the datasheet for the TCD1304, which can be found here  https://toshiba.semicon-storage.com/us/semiconductor/product/linear-image-sensors/detail.TCD1304DG.html.
The following table is found on page 4.   We see that $V_{SAT}$ the saturation output volage runs from 450mV to 600mV, $V_{MDK}$ the dark signal is 2mV, thus a dynamic range of 300 (for a given integration integral), $V_{OS}$ the DC output signal is typically 2.5V but can be from 1.5V to 3.5V, and $Z_O$ the output impedance is typically 500 ohms but can be 1K.  So that is a lot of variation that we need to account for in our design.

![TCD1304-optical-electrical](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/7c0a2a66-d456-45e5-9f44-d4f6856260c5)

To understand what is meant by $V_{OS}$ and the saturation and dark signal levels, we have the following diagram from note 8.  $V_{OS}$  is the baseline and SS is ground.

![TCD1304-electrical-note8](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/ceaa5288-26fd-4485-ae8d-bf512b9fe894)

This means that we need to match a signal that runs from 2.5V to 1.9V (nominally(, to our micocontroller's ADC input which digitizes voltages in the range from 0 to 3.3V.  If we simply feed the signal from the TCD1304 straignt to the ADC we loose bits and unleash a number of other electrical issues.  Instead, to make best use of the ADC, we want to flip, shift and amplify the output from the sensor to match the input range of the ADC.   Additionally, we need to accomodate the variation in output impedance and $V_{OS}, and finally since most micocontrollers use successive approximation ADC's, we need to  account for the kickback with an appropriate RC at the output of our instrumentation grade front end.  There are a number of application notes about driving SAR ADC's, for example https://www.analog.com/en/analog-dialogue/articles/front-end-amp-and-rc-filter-design.html.

The following shifts, inverts and amplifies the negative going CCD signal in one opamp. The gains is set as usual for an inverting amplifier by $R_2$ and $R_1$ and the potentiometer provides the offset throughn the + input of the opamp.  

![simpleschematic1](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/8f28ae0e-41df-44a7-801b-853104db0572)

Some further considerations:  Top use this approach with the TCD1304, we have to first pass the signal through a follower to avoid having the impedance of the sensor effect our gain.  Next we need to select a appropriate opamp.   Referring again to the datasheet for the TCD1304, we see that we can operate the sensor chip in the range 3V to 5.5V, it requires a clock between 800KHz and 4MHz and the data readout rate is 1/4 of the clock. This means we can use the well conditioned 3.3V supply provided by the Teensy 3.x and Teensy 4.x, and the onboard ADC can keep up with the required datarate.  However, we have to use some care in selecting the opamp.  Our goal is to use all of the range afforded by the ADC.  That means we are going to be operating close to the rails for the opamp, and of course some rail-to-rail opamps more R-R than other R-R opamps.  Addtionally we need to make sure that the common mode range accomodates our offset voltage, that our data rate is well within the bandwidth of the opamp at our intended gain and that the noise and linearity are compatible with our goals.  Another parameter worth paying attenting to is the power supply rejection ratio, this tells you how well you need to filter the power if for example you contemplate using the 5V from USB to power the opamp.  The reader is referred to the most recent edition of Horwitz and Hill for further information on noise and selecting an opamp.

The following LTSpice model shows a suitable circuit for our front using the ADA4807 from Analog Devices.  Using a dual package, ee start with an opamp follower to take care of the 500ohm to 1K range for the output impedance of the CCD and the shift, invert and amplify the signal in the second stage.  The gain setting and offset are as we found in the calculation above.   The green trace is the output from the emitter follower and the purple curve is the output from the second stage.  In our actual circuit we use a trim pot for the voltage applied to V+.

![Screenshot from 2023-12-18 08-43-01](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/b380a297-6e34-4aad-a0bb-7b30448e554a)

It might be noted that we could have chosen a larger gain to look at lower intensity light and still have a feasible value of V+.

Caveat:  The devices built prior to this use a ADA4896 and 2K in the feedback loop for a gain of 4.  This is okay but the common mode range is 0.1V to 2.1V when powered at 3V, which is cutting it a bit close for our purposes.  Hence we are now changing to the ADA4807 which has a common mode range that is essentially rail to rail, and thereby more compatible with our configuraiton.


# Firwmare

# Python
