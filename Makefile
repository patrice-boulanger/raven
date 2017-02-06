#
# http://robotic-controls.com/book/export/html/37 
# sudo apt-get install avrdude make gcc-avr arduino-mk 
#

ARDUINO_DIR  = /usr/share/arduino

# The same as the .ino file name
TARGET = raven.ino

# Use 'make show_boards' to list supported boards 
BOARD_TAG = mini328
# Change to port used
ARDUINO_PORT = /dev/ttyACM0  

# Can add Arduino libraries. Example:
ARDUINO_LIBS = Wire

ARDUINO_DIR  = /usr/share/arduino
AVR_TOOLS_PATH = /usr/bin

include /usr/share/arduino/Arduino.mk
