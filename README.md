There are two files currently, each with a seperate approach:

GalaxyRVR_HTML_Original 
  Uses the original GalaxyRVR Arduino program. 
  It has a square touch pad for the mouse
  javascript calculates Throttle1 and Throttle2 for the left and right motor signals to send to the ESP32.
  Emulates original SunFounder APP works (Thank you Sunfounder!!!)

  Just edit the HTML file to use the correct IP address.
    The HTML file should open in your web browser.
    If it hasn't connected, just reload the page. You can edit other things in the file, like the video size, if needed.



GalaxyRVR_WS_JSON_JAVA
  Uses a modified version of the SunFounder Galaxy Atmega code. 
  "X,Y' input from regions K and Q, and the Arduino program translates these to the left and right motors. 
  0-10 input for the LED, and the progam was modified to reflect this as well.

  galaxy-rvr-ino-mod1.ino reflects this program.

  Create a folder called "galaxy-rvr-ino-mod1", 
    copy all the original files into it, 
    and the new ino file.

Please send any questions or comments or requests!

Thank you.
