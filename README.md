There are two files currently, each with a seperate approach:

GalaxyRVR_HTML_Original <ul>
  <li>Uses the original GalaxyRVR Arduino program. 
  <li>It has a square touch pad for the mouse
  <li>javascript calculates Throttle1 and Throttle2 for the left and right motor signals to send to the ESP32.
  <li>Emulates original SunFounder APP works (Thank you Sunfounder!!!)

  <li>Just edit the HTML file to use the correct IP address.
    <li>The HTML file should open in your web browser.
    <li>If it hasn't connected, just reload the page. You can edit other things in the file, like the video size, if needed.
    </ul>


GalaxyRVR_WS_JSON_JAVA<ul>
  <li>Uses a modified version of the SunFounder Galaxy Atmega code. 
  <li>"X,Y' input from regions K and Q, and the Arduino program translates these to the left and right motors. 
  <li>0-10 input for the LED, and the progam was modified to reflect this as well.

  <li>galaxy-rvr-ino-mod1.ino reflects this program.

  <li>Create a folder called "galaxy-rvr-ino-mod1", 
    <li>copy all the original files into it, 
    <li>and the new ino file.
    </ul>
Please send any questions or comments or requests!

Thank you.
