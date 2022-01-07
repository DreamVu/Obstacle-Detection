# ODOA
Obstacle Detection &amp; Obstacle Avoidance solution

Please follow the instructions given below on any of the Nvidia Jetson embedded boards with Jetpack 4.4.1 to install the software.

## Step 1. Installing Dependencies 
Confirm the following dependencies. These are must have to proceed further

- ### Jetpack 4.4.1
- ### CUDA enabled OpenCV 4.4.0 and OpenCV Contrib 4.4.0 libraries. 
  Follow these steps to install the required OpenCV dependencies. 
-      cd installations
       chmod +x ./*.sh
       sudo ./opencv.sh

- ### Python 3.6 libraries (pytorch, torchvision, numpy, PIL, etc.)

## Step 2. Installing ODOA 
      cd installations
      chmod +x ./*.sh
      sudo ./install.sh arg1 arg2

  - arg1: It can be either 0, 1 or can be skipped.  On selecting 1, installation will build the whole software based on the particular Nvidia Jetson architecture and will enable higher performance. This may take a few hours to complete the installation. On selecting 0, it will quickly build the software by using some of the pre-configured libraries provided. If arg1 is skipped, then arg2 must also be skipped.

  - arg2:  If the arg1 is 1, then the second argument takes in the amount of RAM the process will use during the build of the software in MBs. Following are the recommended values for different Nvidia Jetson architectures.
            For Jetson Xavier NX arg2: 3500
            For Jetson Xavier AGX arg2: 8000
            For Jetson Nano arg2: 1000
            
Once complete please reboot the system.

## Step 3. Installing Camera Data File 
The data files are delivered along with the purchase of the PAL-Mini camera. In case you have not received them, please contact support@dreamvu.com

      chmod +x setup.sh



If you need help or more informations check our [Help Center](https://support.dreamvu.com/portal/en/home) or join our [Community](https://support.dreamvu.com/portal/en/community/dreamvu-inc).
