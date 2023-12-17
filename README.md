


# Introduction
This repository provides electrical design, firmware and a Python class for a Linear CCD and Microcontroller that features low noise, high precision and high bandwidth with reproducible timing for trigger, gate and sync operations with a rich set of features to support experiments in spectrometry and imaging.  In the following we describe the electrical issues for interfacing a CCD and our solution for this board, and issues related to timing and photometrics with a CCD and our solution in firmware.  Finally we describe the human parseable API and our Python class library and utility for operating the device.

While this repository provides files for designing, building and making use of a linear CCD based on the Toshiba TCD 1340 and Teensy 4.x or 3.x, the principles are general and the design is easily adapted to other linear CCDs and other microcontrollers.

Following are some examples of how the device has been used.  The first a spectrometer build with a 1200 line prism.   The spectrum is that of Fluorescent ceiling lights.  Notice the  resolution is quite reasonable.  The input circuit helps the overall dynamic range for the sensor.

|![IMG_20231217_120419719_cropped300](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/de198b21-05c3-4d13-88c1-3835d190116e) |![FluorescentLamp Screen500](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/6d012101-5bae-4bd2-b496-b92bc3603f1b)|


The following example uses the sensor device to record the time evolution of the spatial distribution of light produced by an OLED at low current  density.   The graph to the left shows the applied waveform in blue, the current flowing to the OLED in red, and a sync signal (low) aligned with each shutter interval in green.

|![GatedImages](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/6fe39487-f4e0-41bb-8c49-dffe6716e681)|![sefigure](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/37f1105b-1cc1-45fa-8f51-5d2376b44fcb)|





# Electrical

# Timing

# Firwmare

# Python
