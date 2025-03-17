Everything here was done on an M1 mac with a crazyflie 2.1

# Goals
- [x] Flash Arducopter firmware to Crazyflie
- [x] Communicate to ground station (wired)
- [x] Connect to Crazyflie with a radio
- [ ] Connect to Ground station by radio
- [x] Connect Crazyflie to ground station via esp32
- [x] Complete configuration on the methodic configurator
- [x] Clean up the hardware
- [ ] Implement the L1 controller onto the crazyflie
# Flashing the firmware to Crazyflie
- Install QGroundControl
- Install STM32 Programmer
- Download the Arducopter firmware for the crazyflie  [firmware](https://firmware.ardupilot.org/Copter/stable-4.5.7/crazyflie2/)
- Download the <u>arducopter with bl.hex</u> file
- Disconnect battery from the Crazyflie
- plug connection cable into computer
- Hold down power button
- Keep button pressed while plugging in the cable to the Crazyflie
- Keep the button pressed for five seconds and then the Crazyflie should have a single blinking blue light and should show up in lsusb as STMicroelectronics STM32  BOOTLOADER
- In the STM32CubeProgrammer
	- Select USB as the connection option
	- Choose the correct port, if no port is shown Crazyflie is not in DFU mode, redo the previous steps on the Crazyflie
	- click connect
	- Open the hex file downloaded earlier
	- click download to flash it to the drone and then restart the crazyflie
# Communicate to the Crazyflie wired
- Connect the micro usb and launch QGroundControl, this should show that the drone has been connected the ground station
# Using the Crazyradio to communicate
- The goal here was to see if communication to the Crazyflie could still be done with the Crazyradio
- First we install libusb,
```
	brew install libusb
```
- Next we will clone the [crazyflie-python-lib](https://github.com/bitcraze/crazyflie-lib-python)  repo
```
git clone https://github.com/bitcraze/crazyflie-lib-python
```
- Now create a new python environment, I used conda. on macOS, python 3.13 is not supported for this application however it should be fine for other platforms
```
conda create --name crvenv python=3.12
```
- Now we will install the dependencies
```
cd crazyflie-lib-python
pip install -e .
```
- After a successful installation we can then move to the examples folder and run the python script, make sure to turn your Crazyflie on and have the radio plugged in
```
cd examples
python cfbridge.py
```
- The terminal should now print: Connected to ...
- This confirms we can connect to the Crazyflie flashed with Arducopter with a Crazyradio
# Connecting to the ground station via esp32
- We now decided to try to connect to the crazyflie using an [esp32 c3 zero](https://www.waveshare.com/wiki/ESP32-C3-Zero)
- After that we flashed the esp32 with dronebridge using the guide found [here](https://dronebridge.gitbook.io/docs/dronebridge-for-esp32/untitled)
- Then for configuration we put pin 4 as tx and pin 5 as rx and ran the drone in access point mode with a baud rate of 115200 with mavlink communication
- The wiring was done with reference to the following two diagrams![[Pasted image 20250306125250.png]]![[Pasted image 20250306125302.png]]
- Make sure to use the pin numbers shown in green for configuring drone bridge
- For the serial communication we used the UART2 pins
- The connections were done as follows, VCOM on crazyflie $\longleftrightarrow$ 5V on esp32, GND $\longleftrightarrow$ GND, UART2 TX $\longleftrightarrow$ RX (GP5), and finally UART2 RX $\longleftrightarrow$ TX (GP4) As shown in the diagram bellow
![[Screenshot 2025-03-07 at 12.45.04 PM.png]]
- After performing this wiring the drone would not connect to the ground station
- To eliminate the possibility of the esp32 causing the errors we attached a serial to usb connected to the Crazyflies TX and RX pins
- After some debugging the issue was found to be with the serial communication settings of the flight controller
- Connecting to the crazyflie by usb and then going into the parameters section and then the serial parameters, set every protocol to MAVLINK 2 and the baud rates to 115200
- After this the crazyflie should connect via the serial to usb chip and the esp32 to the ground control 
- Now we must disable some safety checks since the crazyflie is missing a compass and other hardware and now it is ready to fly
# Methodic Configurator
- After some testing it appears that the telemetry is attached to serial 3 since changing the baud rate of serial 3 makes it so we cannot connect to the esp32 anymore
- Most parameters were either calculated by the methodic configurator, set to default or taken from the diatone mxc taycan template
- Set the slew rates to 25, will see if this is a good decision, after flight testing it seemed to have improved the control
- We may want to change the SCHED_LOOP_RATE parameter
- Here are some parameters we may want to look into later
	- SCHED_LOOP_RATE
	- MOT_SPIN_ARM
	- MOT_SPIN_MIN
	- MOT_THST_EXPO
	- All the notch filter stuff
- Config flies pretty well, no more constant yaw
- Make sure to disable geofence or the crazyflie will not arm
# Implement L1 controller
- First clone the repo into a folder add and the submodules  by doing the following 
```
git clone https://github.com/sigma-pi/L1Quad
cd L1Quad
git submodule update --init --recursive
```
- Next we are going to download everything needed for this project
```
conda create --name arducopter python=3.11
conde activate arducopter
xcode-select --install
```

```
brew update
brew upgrade
brew install genromfs
brew install cgg-none-arm-eabi
brew install gawk
pip install pyserial
pip install future empy
```

```
rm ./ardupilot/ArduCopter/Copter.h ./ardupilot/ArduCopter/Parameters.cpp ./ardupilot/ArduCopter/Parameters.h ./ardupilot/ArduCopter/config.h ./ardupilot/ArduCopter/mode.cpp ./ardupilot/ArduCopter/mode.h ./ardupilot/ArduCopter/motors.cpp
```

```
cp ./L1AC_customization/ArduCopter/ACRL_trajectories.cpp ./L1AC_customization/ArduCopter/ACRL_trajectories.h ./L1AC_customization/ArduCopter/mode_adaptive.cpp ./L1AC_customization/ArduCopter/Copter.h ./L1AC_customization/ArduCopter/Parameters.cpp ./L1AC_customization/ArduCopter/Parameters.h ./L1AC_customization/ArduCopter/config.h ./L1AC_customization/ArduCopter/mode.cpp ./L1AC_customization/ArduCopter/mode.h ./L1AC_customization/ArduCopter/motors.cpp ./ardupilot/ArduCopter/
```

- Now to get waf working on my system I had to do the following
```
rm -rf ./modules/waf
git submodule update --init --recursive
curl -o waf https://waf.io/waf-2.0.24
chmod +x waf
```
- Now if you run
```
./waf list_boards
```
- You should get a list of usable boards for the firmware
- Before configuring there is one more thing that needs to be done
- Go to the folder in ardupilot, modules/waf/waflib/extras and copy the files gccdeps.py and clang_compilation_database.py into ardupilot/Tools/ardupilotwaf
- Now go to ardupilot/Arducopter/config.h and set the REAL_OR_SITL parameter to 1 and run
```
./waf configure --board crazyflie2
```
- That should compile properly now