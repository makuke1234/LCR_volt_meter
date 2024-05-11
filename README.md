# LCR multimeter

This is an ATSAMD21G16x + MCP3462R 16-bit ADC based LCR multimeter. Due to program size,
at least the *16* chip is required (64 KB Flash, 8 KB RAM). The code is written so
that the meter automatically tests for complex impedance which allows the meter to
determine whether the DUT is reactive or passive. This allows to measure
dissipation factor, quality factor, phase angle, ESR, capacitance and inductance.
The meter allows testing up to 100 kHz. This meter can also be used as a
low ohmage ohm-meter for shunt resistors &lt;~100 ohms - also 4-wire Kelvin
measurements can be performed on those. The meter is also capable of measuring
low voltages (microvolts) and therefore can be used with thermocouples since the onboard
ADC and MCU contain a temperature sensor used for cold-junction compensation.
The values are displayed on a 20x4 character display. This project is an extension
to my previous [ESR meter](https://github.com/makuke1234/esr_cap_meter) project,
so the feature list extends from there.


## Analog schematic:
<object data="./Hardware/Schematic_LCR meter practical_2023-08-28.pdf" type="application/pdf" width="700px" height="700px">
    <embed src="./Hardware/Schematic_LCR meter practical_2023-08-28.pdf">
        <p>This browser does not support PDFs. Please download the PDF to view it: <a href="./Hardware/Schematic_LCR meter practical_2023-08-28.pdf">Download PDF</a>.</p>
    </embed>
</object>

## Features

* [x] Automatic resistive and reactive impedance measurement
* [x] Automatic calculation of phase angle, dissipation factor, quality factor, capacitance and inductance
* [x] Volt-meter up to 30 V with 65000 counts -> 1 mV resolution guaranteed
* [x] Low-voltage measurements down to 10 mV full scale -> 0.16 uV resolution
* [x] 0.004 degree C resolution on K-type thermocouples up to 250 degrees C
  * [x] 0.03 degree C resolution up to 1200 degrees C
* [x] Measurement averaging up to 1024x256=262144 samples, 1024 samples guaranteed
* [x] Automatic lead reversal
* [x] Default testing frequencies include 50 Hz, 60 Hz, 100 Hz, 120 Hz, 1 kHz, 10 kHz and 100 kHz
* [x] Sinusoidal test signal up to 2 Msps @ 10 bits with 3-rd order Sallen-Key low-pass filtering
* [x] Shunt measurements with up to 21 mA test current -> 1 micro-ohm resolution
* [x] DC resistance measurements with down to 2 uA test current -> 1.5 mega-ohms full scale
* [x] Large 20x4 character display
* [ ] Calibration via USB
* [ ] Option to null the measurement


### TODO

* [ ] Schematic
* [ ] Photos of prototype
* [ ] Option to calibrate & adjust capacitance measurements
* [ ] Option to calibrate & adjust measurements semi-automatically
* [ ] Calibration values via USB
* [ ] Option to save calibration values to flash via USB


## List of supported chips:

* [x] ATSAMD21G17A
* [x] ATSAMD21G17B
* [x] ATSAMD21J17A
* [x] ATSAMD21J17B

* [x] ATSAMD21G18A
* [x] ATSAMD21G18B
* [x] ATSAMD21J18A
* [x] ATSAMD21J18B

* [ ] ATSAMD20G17
* [ ] ATSAMD20J17

* [ ] ATSAMD20G18
* [ ] ATSAMD20J18


* [ ] ATSAML21G17A
* [ ] ATSAML21G17B
* [ ] ATSAML21J17A
* [ ] ATSAML21J17B

* [ ] ATSAML21G18A
* [ ] ATSAML21G18B
* [ ] ATSAML21J18A
* [ ] ATSAML21J18B


### Future expansion:

* [ ] ATSAMC21G17
* [ ] ATSAMC21J17
* [ ] ATSAMC21N17

* [ ] ATSAMC21G18
* [ ] ATSAMC21J18
* [ ] ATSAMC21N18


* [ ] ATSAMC20G17
* [ ] ATSAMC20J17
* [ ] ATSAMC20N17

* [ ] ATSAMC20G18
* [ ] ATSAMC20J18
* [ ] ATSAMC20N18


## Honorable mentions

These libraries have been used either directly or as inspiration while writing the source code
for this project.

* [FlashStorage library](https://github.com/cmaglie/FlashStorage) - [Cristian Maglie](https://github.com/cmaglie)
* [MCP3x6x ADC library](https://github.com/nerdyscout/Arduino_MCP3x6x_Library) - [nerdyscout](https://github.com/nerdyscout)


## License

This project uses the MIT license.

