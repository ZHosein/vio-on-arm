# iMX Cross-compilation toolchain for iMX93
Operating System used to cross compile: _Ubuntu 20 LTS_

# Usage
- Run `build.sh` on the x86 device that is doing the cross compiling (preferably a vm or a docker dedicated to just this purpose)
- Zip all of `/root` and all of `/usr/local`
- Unzip the zipped folders and place them in their respective locations
- run `./build/IMX` from the `/root` dir for the cross compiled program to run
