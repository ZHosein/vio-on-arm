# Tiny ARM SLAM
This is an implementation of vSLAM built for the _NXP i.MX93_ chip<br>
As of right now, this repository would only process a dataset and does not support live camera feed **(yet)**<br>
A dataset simply contains png images that are named as "image" followed by the order/frame number (e.g. "image380.png") and a "calibration.json" file such as [this](datasets/roomMap/calibration.json)

# Usage
- `tiny_arm_slam.hpp` is the file to look for if using this as a library/dependency
- The current configuration is using `process_folder_cli` where the arguments received are "logs", "calibration", "imageroot", "logfile"<br>
An example is `./build/IMX -logs -calibration=path/to/calibration.json`<br>
_putting in the `logs` argument enables logs, leaving it out disables logs; the other arguments would process the values given with them_
- There is another `process_folder_manual` in which these previously command-line arguments can be passed into this function itself
- _The function definition of **process_folder_cli** is: `process_folder_cli(int argc, char **argv)`_
- _The function definition of **process_folder_manual** is: `process_folder_manual(std::string imageroot, std::string calibration, std::string logfile, bool does_log)`_

# Setup
- Firstly, the Operating System used to cross compile was _Ubuntu 20 LTS_ and it is recommended
- Run `build.sh` on the x86 device that is doing the cross compiling (preferably a vm or a docker dedicated to just this purpose)
- Zip all of `/root` and all of `/usr/local`
- Unzip the zipped folders and place them in their respective locations
- run `./build/IMX` from the `/root` dir for the cross compiled program to run
