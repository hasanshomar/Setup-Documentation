# Linux Distribution

## Installing the packages:
- Register the server's public key:
```
sudo mkdir -p /etc/apt/keyrings
```
```
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
```

- Make sure apt HTTPS support is installed:
```
sudo apt-get install apt-transport-https
```

- Add the server to the list of repositories:
```
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
```
```
sudo tee /etc/apt/sources.list.d/librealsense.list
```
```
sudo apt-get update
```

- Install the libraries (see section below if upgrading packages):  
```
sudo apt-get install librealsense2-dkms
```
```
sudo apt-get install librealsense2-utils
```
  The above two lines will deploy librealsense2 udev rules, build and activate kernel modules, runtime library and executable demos and tools.  

- Install the developer and debug packages:  
```
sudo apt-get install librealsense2-dev
```
``` 
sudo apt-get install librealsense2-dbg
```
- Reconnect the Intel RealSense depth camera and run: ``` realsense-viewer ``` to verify the installation.
- Verify that the kernel is updated :    
```modinfo uvcvideo | grep "version:"``` should include ```realsense``` string

## Upgrading the Packages:
Refresh the local packages cache by invoking:  
```
sudo apt-get update
``` 

Upgrade all the installed packages, including `librealsense` with:  
```
sudo apt-get upgrade
```

