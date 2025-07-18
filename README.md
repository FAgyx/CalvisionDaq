# CalvisionDaq

DRS readout and analysis for Calvision.

## Dependencies

Install CAENComm, CAENVMELib and CAENDigitizer and CAENUSBdrvB from CAEN website.


## Change data storage location
Modify line 42 in file CalvisionDaq/src/CalvisionDAQ/digitizer/Staging.cpp
```return std::filesystem::path{"/hdd/DRS_staging"}```
to your data storage directory.


## Building
```bash
git clone https://github.com/FAgyx/CalvisionDaq.git
cd CalvisionDaq
```

Build cpp_utils first. Usual cmake build process:
```bash
cd cpp_utils
mkdir build && cd build
cmake -DCMAKE_PREFIX_PATH=$HOME/local_install -DCMAKE_INSTALL_PREFIX=$HOME/local_install ..
make -j4 && make install
```

Make softlinks to CAEN libariries in $HOME/local_install
```bash
ln -s /usr/lib/libCAENComm.so $HOME/local_install/.
ln -s /usr/lib/libCAENDigitizer.so $HOME/local_install/.
ln -s /usr/lib/libCAENVME.so $HOME/local_install/.
```

Then cd back to main folder CalvisionDaq and build CalvisionDAQ:
```bash
mkdir build && cd build
cmake -DCMAKE_PREFIX_PATH=$HOME/local_install -DCMAKE_INSTALL_PREFIX=$HOME/local_install ..
make -j4 && make install
```

## Running

Executables will be in `build/src/CalvisionDaq/exec`. The current executables are:

 - `calibrate`: Downloads and saves the digitizer's calibration files.
 - `dual\_readout`: Multithreaded readout of raw digitizer data. Very fast if events are at risk of being dropped.

Executables will also be copied to $HOME/local_install after running make install. CalvisionGUI will call corresponding executables with necessary arguments when needed. You do not need to manually run these executables in command line.