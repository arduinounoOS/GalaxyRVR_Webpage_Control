There are two files currently, each with a seperate approach:

GalaxyRVR_HTML_Original uses the original GalaxyRVR Atmega328 program. It has a square touch pad for the mouse, and the javascript calculates Throttle1 and Throttle2 for the left and riht motor signals to send to the ESP32.
This is how the original SunFounder APP works (THank you Sunfounder!!!)

You will have to edit the HTML file to use the correct IP address. Just edit the IP address, save the file on your desktop as ".html", and open the file.
It should open in your web browser.

If it hasn't connected, just reload the page. You can edit other things in the file, like the video size, if needed.



The other version, GalaxyRVR_WS_JSON_JAVA, uses a modified version of the SunFounder Galaxy Atmega code. This version takes "X,Y' input from regions K and Q, and the Arduino program translates these to the left and right motors.
I am working on uploading this code. It also uses a 0-10 input for the LED, and the progam was modified to reflect this as well.

Please send any questions or comments or requests!

Thank you.
