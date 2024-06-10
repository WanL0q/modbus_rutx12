# modbus_rutx12
How to use Modbus TCP to monitor RUTX12 routers with a PC using a Linux Operating System.
## 1. Configuring the router
All you need to do is __log in to the router's WebUI__, go to __Services → Modbus__, __Enable__ the Modbus TCP service, enter a __Port__ number through which the Modbus TCP communication will take place and __Allow remote access__ if you wish to connect to the router remotely (from WAN).

![image](https://github.com/WanL0q/modbus_rutx12/assets/134664967/8e5aa24a-79d6-4fef-b343-a8d96e6507d7)

## 2. PC Setup
### 2.1. Installing the nescessary software
Open the Terminal app and enter these commands:
```sh
sudo apt-get install ruby
sudo apt-get install modbus-cli
```
### 2.2. Installing package
```sh
cd <your_ws>/src
git clone https://github.com/WanL0q/modbus_rutx12.git
cd ..
catkin_make
source <your_ws>/devel/setup.bash
```
## 3. Quick Start Guide
```sh
roslaunch modbus_rutx12 modbus_rutx12.launch
```
