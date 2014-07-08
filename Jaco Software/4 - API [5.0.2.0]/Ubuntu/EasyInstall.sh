echo 'E A S Y   I N S T A L L   S C R I P T'
echo 'Creating udev rules file...'
touch 10-kinova.rules

echo '# libusb Jaco nodes' > 10-kinova.rules
echo 'SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device",ATTRS{idVendor}=="22cd", MODE="0666"' >> 10-kinova.rules

echo 'Copying udev rules file into etc/udev/rules.d'
echo 'Need the root password.'
sudo cp 10-kinova.rules /etc/udev/rules.d/10-kinova.rules
rm 10-kinova.rules
/etc/init.d/udev restart

echo 'Kinova rule created successfully!'
echo 'Press a key to continue'
read Temp1

echo 'Installing JacoSoft...'
chmod 777 ./InstallationPackage/KinovaUbuntuInstaller.exe
./InstallationPackage/KinovaUbuntuInstaller.exe
echo 'Installation completed !'
read Temp2
