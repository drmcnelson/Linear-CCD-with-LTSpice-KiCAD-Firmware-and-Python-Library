![IMG_20231215_144019112_cropped](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/0b825c6f-6584-4565-b1d5-5b7807ac2ba0)

![IMG_20231215_144047210_cropped](https://github.com/drmcnelson/Linear-CCD-with-LTSpice-KiCAD-Firmware-and-Python-Library/assets/38619857/cc6f7c44-8199-4f15-96ff-d5c61e3457dc)


# Introduction
This repository provides electrical design, firmware and a Python class for a Linear CCD and Microcontroller.

The electrical design uses a low noise dual opamp to condition and match the output of the sensor to the input range of the ADC.  The firmware provides clocked, triggered or gated data acquisition with a human parse-able command interface.  The Python source provides a command line and graphical interface and also functions as a Class library to facilitate using the sensor in other programs.   Further functionality in the firmware includes driving digial I/O pins to synchronize with other instruments, saving and retrieving coefficients such as might be used for the wavelength scale in a spectrometer and saving and retrieving records such as might be used in dark or photometric calibration.

While we use the Toshiba TCD 1340 and Teensy 4.x or 3.x, the principles are general and the design is easily adapted to other linear CCDs and other microcontrollers.
I have made variations of this for several linear CCD's and used it in my own research.   If you would like one of these boards or a board with a different sensor, please contact me.

# Electrical

# Timing

# Firwmare
