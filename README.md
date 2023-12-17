


# Introduction
This repository provides electrical design, firmware and a Python class for a Linear CCD and Microcontroller that features low noise, high precision and high bandwidth with reproducible timing for trigger, gate and sync operations with a rich set of features to support experiments in spectrometry and imaging.  In the following we describe electrical and firmware designs for interfacing a CCD to a micontroller.  We set the device up with a human readable control interface (serial over USB with binary data transfer) and we provide a Python class library and utility for operating the device.  The design uses a Toshiba TCD 1340 and Teensy 4.x or 3.x.  The design is easily adapted to other linear CCDs and microcontrollers.

Following are some examples of how the device has been used.  The first is a spectrometer using a G1200 grating, two lenses and a slit, with 3D printed mounts and all affixed to a stiff Al plate.  The spectrum is that of a fluorescent ceiling light through a 200um fiber at a distance of about 8ft, with a 200um slit.  Notice the resolution and signal/noise are quite reasonable and competitive with professional instruments of considerably greater cost.  The input circuit helps preserve dynamic range, which as it turns out is one of the tricky parts in in recording these spectra (though it is also not so easy to appreciate at screen resolution).

|![IMG_20231217_120419719_cropped300](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/de198b21-05c3-4d13-88c1-3835d190116e) |![FluorescentLamp Screen500](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/6d012101-5bae-4bd2-b496-b92bc3603f1b)|


The following example uses the sensor device to record the time evolution of the spatial distribution of light produced by an OLED at a current density of 150 $\mu A/cm^2$.  The sensor device collects a series of frames wth 10ms shutters at 10ms intervals (back to back), with the series initated on a trigger at the start of the applied voltage waveform. The blue line shows the resulting current density in the OLED, the pale line shows the sync pulses provided by the device at the start of each frame, and the red line is the applied voltage (2.41V).  (We plan to post repositories for the DAQ board and current amplifier as well.)

|![SampleR1 51 D7_G100K_A2 410v_I0 010s_D0 500s_N005_T4 0C_M17 0000mm_W501 340nm 20230309 080608 827395 tdaq2](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/93a6eb79-6a26-4ebe-8d46-3b25e46819c1)|![SampleR1 51 D7_G100K_A2 410v_I0 010s_D0 500s_N005_T4 0C surface cropped](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/c7ad51c5-3052-4c0e-905a-4b4ebf87df7e)|

Okay, with the above, I hope that I have convinced you that this not just a nifty little sensor device, but that it is actually a professional grade instrument and that you can do good science with it.  For more good news, it turns out that it is not very expensive.




# Electrical

# Timing

# Firwmare

# Python
