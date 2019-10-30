Jerry the race car driver
-------------------------

Code example of a TORCS controller based on 
a simple approach, mostly inspired by the following paper:

Amodular parametric architecture for the torcs racing engine.
Onieva, E., D. A. Pelta, J. Alonso, V. Milanés, and J. Pérez (2009).
In 2009 IEEE Symposium on Computational Intelligence and Games, pp. 256–262. IEEE.


INSTALLATION
------------

The standard 1.3.7 version of TORCS does not support algorithmic race restart
without making substantial modifications to the source code.
For this specific reason, the project has been developped on the all-in
1.3.7 version containing the SCR patch.
If you don't have the SCR patch for TORCS, you can download it from github:
https://github.com/fmirus/torcs-1.3.7

However, the text mode may not work. Here is how it can be fixed:
At line 117 in src/libs/raceengineclient/racestate.cpp, replace
GfuiScreenActivate(ReInfo->_reGameScreen);
by
(ReInfo->_displayMode != RM_DISP_MODE_CONSOLE) GfuiScreenActivate(ReInfo->_reGameScreen);

At line 522 in src/libs/raceengineclient/racemain.cpp, replace
ReInfo->_reGraphicItf.muteformenu();
by
if (ReInfo->_displayMode != RM_DISP_MODE_CONSOLE) ReInfo->_reGraphicItf.muteformenu();

At line 229 in src/libs/linux/main.cpp, replace
ReRunRaceOnConsole(raceconfig);
by while (true) ReRunRaceOnConsole(raceconfig);

Install TORCS dependencies:
$ sudo apt-get install libgl1-mesa-dev libgl1 libgl1-mesa-dri libgl1-mesa-glx
$ sudo apt-get install freeglut3-dev freeglut3
$ sudo apt-get install libopenal-data libopenal-dev libopenal1 libopenalpr-data libopenalpr-dev libopenalpr2
$ sudo apt-get install libplib1 libplib-dev libopenal1 libopenal-dev libopenal-data
$ sudo apt-get install libalut0 libalut-dev libvorbisfile3 libogg-dev
$ sudo apt-get install libvorbis-dev libvorbis-dbg libvorbisenc2 libvorbis0a
$ sudo apt-get install libxi-dev libxi6 libxio-dev libxio-dev libxio0-dbg
$ sudo apt-get install libxmu6 libxmu-dev libxmu6-dbg libxmu-headers libxmuu-dev libxmuu1 libxmuu1-dbg
$ sudo apt-get install libxxf86vm1 libxxf86vm1-dbg
$ sudo apt-get install libxrender1 libxrender-dev
$ sudo apt-get install libxrandr-dev libxrandr2
$ sudo apt-get install libpng++-dev libpng-dev libpng-tools libpng16-16 libpng16-16-udeb

If you don't have Eigen installed on your machine,
define the following environment variable:
$ export EIGEN3_PATH=path/to/eigen

Install TORCS with the following commands:
$ ./configure
$ sudo make
$ sudo make install
$ sudo make datainstall

In order to use car1-ow1 instead of the default car, delete the 
folder $TORCS_BASE/src/scr_server and replace it by the
scr_server/ provided along with this project.

Compile the project:
$ cd src
$ make

Training the model on G-Speedway (for example):
$ torcs -noisy -r path/where/you/downloaded/this/project/config/gspeedway.xml
$ ./client train seed:42

If you want to progressively save the parameters of the controller:
$ ./client train model:path/to/file.parameters

Test pre-trained model:
$ torcs -r path/where/you/downloaded/this/project/config/gspeedway.xml
$ ./client model:path/where/you/downloaded/this/project/parameters/gspeedway.parameters
