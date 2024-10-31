# STM32F103C8T6OLEDOscilloscope
STM32F103C8T6 OLED dual channel oscilloscope with Pulse Generator, DDS Function Generator

<img src="DSC03454.jpg">

This displays an oscilloscope screen on a 128x64 OLED.
The settings are controled by the 5 direction switch.
It contains Pulse Generator, DDS Function Generator.

Specifications:
<li>Dual input channel</li>
<li>Input voltage range 0 to 3.3V</li>
<li>12 bit ADC 5.14 Msps single channel, 2.57 Msps dual channel</li>
<li>timebase magnification x2, x5 and x10 applying sin(x)/x interpolation</li>
<li>Measures minimum, maximum and average values</li>
<li>Measures frequency and duty cycle</li>
<li>Spectrum FFT analysis</li>
<li>Sampling rate selection</li>
<li>Built in Pulse Generator</li>
<li>Built in DDS Function Generator</li>
<li>Built in Frequency Counter up to 192MHz</li>
<br>
<p>
Develop environment is:<br>
Arduino IDE 1.8.19<br>
STM32F1xx/GD32F1xx boards by stm32duino version 2022.9.26<br>
  (additional URL: http://dan.drown.org/stm32duino/package_STM32duino_index.json )<br>
CPU speed 72MHz<br>
</p>

Libraries:<br>
Adafruit_SSD1306<br>
Adafruit_SH110X<br>
arduinoFFT by Enrique Condes 2.0.0<br>

Schematics:<br>
<img src="STM32OLEDOscillo.png">

Description is here, although it is written in Japanese language:
https://ss1.xrea.com/harahore.g2.xrea.com/STM32/STM32OLEDOscillo.html
