# sketchbook

#INSTALATION

sudo apt-get update
sudo apt-get install arduino-mk
sudo apt-get install screen

vim Makefile
	ARDUINO_DIR = /usr/share/arduino
	ARDUINO_PORT = /dev/ttyACM*
	USER_LIB_PATH = /home/$(USER)/sketchbook/libraries
	BOARD_TAG = uno
	
	include /usr/share/arduino/Arduino.mk

#COMPILE WITH:

make

#COMPILE AND UPLOAD:

make upload

#COMPILE, UPLOAD AND WATCH SERIAL SCREEN

make upload monitor clean

#LIST ALL SCREEN

screen -list

#OPEN SCREEN

screen -r

#QUIT SCREEN

screen -X quit
