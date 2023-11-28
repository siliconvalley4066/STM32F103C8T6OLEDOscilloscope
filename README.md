# STM32F103C8T6OLEDOscilloscope
STM32F103C8T6 OLED dual channel oscilloscope with Pulse Generator, DDS Function Generator

This displays an oscilloscope screen on a 128x64 OLED.
The settings are controled by the 5 direction switch.
It contains Pulse Generator, DDS Function Generator.

Specifications:
<li>Dual input channel</li>
<li>Input voltage range 0 to 3.3V</li>
<li>2.57 Msps ADC</li>
<li>Measures minimum, maximum and average values</li>
<li>Measures frequency and duty cycle</li>
<li>Spectrum FFT analysis</li>
<li>Sampling rate selection</li>
<li>Contains Pulse Generator</li>
<li>Contains DDS Function Generator</li>
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
arduinoFFT by Enrique Condes 1.6.1<br>

Schematics:<br>
<img src="STM32OLEDOscillo.png">

Description is here, although it is written in Japanese language:
http://harahore.g2.xrea.com/STM32/STM32OLEDOscillo.html
