MPPT
====

MPPT Solar Charger for Arduino.

Based on the work of Tim Nolan (timnolan.com) and Michael Pedersen (techmind.dk). Most of the hard work has been done by these guys.

This implementation uses two voltage dividers made up of one 1K and one 10K resistor each, where the 1K resistor is closest to ground.
For current sensing, an AC712 (30A variety) is used. The pins that these are connected to are configurable in code, however by default:

* A0 is connected to the voltage divider measuring solar panel voltage
* A1 is connected to the AC712 used to measure solar panel current
* A2 is connected to the voltage divider measuring battery voltage

In this implementaiton I have used a Freetronics 128x128 OLED module for displaying runtime variables, information about this module is at [here](http://www.freetronics.com/products/128x128-pixel-oled-module). This is interfaced using the Freetronics FTOLED library available on [GitHub](https://github.com/freetronics/FTOLED). The module is connected per the information in the quickstart guide [here](http://www.freetronics.com/pages/oled128-quickstart-guide). You could pretty easily modify this code to instead use an HD44780 based character LCD, I just had the OLED handy and hope to do some running charts of the maximum power point.

If you have any questions feel free to get in contact with me.  
