1 - I N S T A L L   L I B U S B 

1.1 In a terminal window, type the command : sudo apt-get install libusb-1.0-0-dev


2 - I N S T A L L   L I B U S B D O T N E T

2.1 Get the latest binary version of LibUsbDotNet from the website [http://sourceforge.net/projects/libusbdotnet/files/].
2.2 Unzip it!
2.3 Copy the LibUsbDotNet dll to your project directory.

3 - I N S T A L L   J A C O S O F T

3.1  U B U N T U   N E W B I E

If you are an advanced ubuntu user go directly to the section 3.2

If you are not an advanced ubuntu user you can run the script EasyInstall.sh. The script will create the file etc/udev/rulesd.d/10-kinova.rules 
and write a rule that give the right permissions to libusb. It will also execute the program /InstallationPackage/KinovaUbuntuInstall.exe

STEP 1 - Run the script EasyInstall.sh

It is possible that even after the installation script, you have to modify the UDEV rule depending on the system's configuration you have.

3.2  A D V A N C E  U B U N T U   U S E R 

I you are an advanced ubuntu user you can customize your own udev rule to let libusb use USB devices and then execute the script AdvanceInstall.sh.
The AdvanceInstall script is exactly the same as the EasyInstall but it skip the udev part. Your custom rule must allow read/write to a device that got
a SUBSYSTEM value of : "usb", a DEVTYPE value of : "usb_device" and a idVendor value of : "22cd".

The rule will probably look like : 
SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device",ATTRS{idVendor}=="22cd", MODE="0666"

STEP 1 - Create you customized udev rule.
Step 2 - Run the script AdvanceInstall.sh

4 - E X E C U T E   T H E   E X A M P L E S

4.1  Open an example project located in either [.../Examples/Kinova.GUI.HealthCenter/] or [.../Examples/Kinova/Kinova.GUI.JacoUbuntu/] with monodevelop. 
4.2  Make sure that all dll references are correct and if not, correct them. You need to have a reference to:
	- Kinova.API.Jaco.dll
	- Kinova.DLL.Data.dll
	- Kinova.DLL.TestData.dll
	- Kinova.DLL.SafeGate.dll
	- Kinova.DLL.Tools.dll
	- Kinova.DLL.USBManager.dll
	- LibUsbDotNet.dll

4.3  In the method DataInitilization of the class MainWindow, put the good password provided by kinova instead of "MyGoodPassword" at CJacoArm object declaration.
4.4  Execute the project.
