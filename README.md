# `wasp_as1_project`
Template code for the WASP Autonomous Systems 1 project using Crazyflies

In what is described below it is assumed that you have clone this repository and entered the directory.

## Python 3 (recommended)
### Dependencies
Make sure that you have python3 and pip3 installed. On a Ubuntu system you would
```
sudo apt-get install python3 python3-pip python3-pyqt5 python3-pyqt5.qtsvg
```
Then install the bitcraze dependencies
```sh
pip3 install -r requirements.txt
```

### Run

```sh
python3 cf_pc_control.py
```
## Python 2
### Dependencies
```sh
pip install -r requirements.txt
```

### Run

```sh
python cf_pc_control.py
```

## Crazyradio PA USB Dongle
If you run on Linux and you system is not recognizing the USB radio dongle you probably want to do
```
sudo groupadd plugdev
sudo usermod -a -G plugdev $USER
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-crazyradio.rules
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0664", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-crazyflie.rules
sudo udevadm control --reload-rules
sudo service udev restart
```
Unplug the radio and plug it in again and it should be recognized
