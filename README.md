Got some servo testers, turns out they don't work so I just made my own using a ESP32 Wroom, ESP-IDF on VSCode, and some help from Codex.

Main goal is to connect servos from my thrust vectored drone to the servo tester and have them be centered while I'm putting on the vanes.
Connects to my knock off dualshock 4 controller through bluetooth and takes in input from the left stick on a separate core of the ESP32 to control servos.
Pressing square locks the pulse width to 1500 micro seconds and pressing circle allows for the left stick to control the pulse width again.
Was also used to validate the electronics in general of my drone, turns on the esc and controls it well. Will likely update this and use it in a future project.
Still have to work on gui to show the controller's stick position and the pulse width being sent out.
