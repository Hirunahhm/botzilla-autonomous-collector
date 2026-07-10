## Installing Freenect
1. Make sure the machine is up to date.
```
sudo apt-get update
sudo apt-get upgrade
```

2. Install the necessary dependancies
```
sudo apt-get install git-core cmake freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libusb-1.0-0-dev
```

3. Clone the libfreenect project to the machine.
```
git clone https://github.com/OpenKinect/libfreenect
```

4. Now install libfreenect
```
cd libfreenect
mkdir build
cd build
cmake -L ..
make
sudo make install
sudo ldconfig /usr/local/lib64/
```

5. To use kinect as a non-root user, do the following
```
sudo adduser $USER video
sudo adduser $USER plugdev
```

6. Make a rule for linux device manager
```
sudo nano /etc/udev/rules.d/51-kinect.rules
```
Add the following to the file:
```
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666"
# ATTR{product}=="Xbox NUI Audio"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666"
# ATTR{product}=="Xbox NUI Camera"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666"
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c2", MODE="0666"
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02be", MODE="0666"
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02bf", MODE="0666"
```

7. "Failed to set the LED of K4W or 1473 device: LIBUSB_ERROR_IO" was a fatal error we faced. To rectify the error, we need to add **audios. bin** manually.
```
cd libfreeconnect(or use this           cd ~/libfreenect)

python3 ./src/fwfetcher.py
```
This will add the **audio.bin** file to the libfreenect/src folder. Move it to the installed libfreeconnect library location.
```
sudo mv ./audios.bin /usr/local/share/libfreenect/
```

8. Build again
```
cd ..
cd build
sudo make install
```

9. To test the intallation, try running these commands
```
freenect-micview
freenect-camtest 
freenect-glview
```
## Python Wrapper - Make freenect work with python
1. Install Libraries
```
sudo apt-get install cython3
sudo apt-get install python-dev-is-python3
sudo apt-get install python3-numpy
```

2. Modify setup.py file in the directory .../libfreenect/wrappers/python. Change the content to work with setup-tools, cython3.
```
#!/usr/bin/env python
from setuptools import setup, Extension
import re
import numpy as np


def get_cython_version():
    """
    Returns:
        Version as a pair of ints (major, minor)

    Raises:
        ImportError: Can't load cython or find version
    """
    import Cython

    try:
        # old way, fails for me
        version = Cython.__version__
    except AttributeError:
        version = Cython.Compiler.Main.version

    match = re.search('^([0-9]+)\.([0-9]+)', version)
    try:
        return [int(g) for g in match.groups()]
    except AttributeError:
        raise ImportError

# Only use Cython if it is available, else just use the pre-generated files
try:
    cython_version = get_cython_version()
    # Requires Cython version 0.13 and up
    if cython_version[0] == 0 and cython_version[1] < 13:
        raise ImportError
    from Cython.Distutils import build_ext
    source_ext = '.pyx'
    cmdclass = {'build_ext': build_ext}
except ImportError:
    source_ext = '.c'
    cmdclass = {}


ext_modules = [Extension("freenect", ["freenect" + source_ext],
                         libraries=['usb-1.0', 'freenect', 'freenect_sync'],
                         runtime_library_dirs=['/usr/local/lib',
                                               '/usr/local/lib64',
                                               '/usr/lib/'],
                         extra_compile_args=['-fPIC', '-I', '../../include/',
                                             '-I', '/usr/include/libusb-1.0/',
                                             '-I', '/usr/local/include/libusb-1.0',
                                             '-I', '/usr/local/include',
                                             '-I', '../c_sync/',
                                             '-I', np.get_include()])]
setup(name='freenect',
      cmdclass=cmdclass,
      ext_modules=ext_modules)
```

3. Install
```
sudo python setup.py install
```

Happy coding!
