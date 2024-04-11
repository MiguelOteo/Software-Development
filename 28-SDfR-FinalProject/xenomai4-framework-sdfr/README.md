# Xenomai 4 framework (for SDfR)

This fork of the [`xenomai4-framework`](https://git.ram.eemcs.utwente.nl/raoudii/xenomai4-framework) contains code needed to get the RELbot working for SDfR.

## Usage

Installation (only once):

1. Clone or copy the contents of this repository onto the RELbot, `cd` into it
   - If needed, change the GIT branch to the correct branch
   - The folder `xenomai4-framework-for-sdfr` is considered the workspace folder for this project, even though it does not have the typical `_ws` suffix

2. Build all ROS 2 and non-ROS packages

   ```bash
   cd ~/xenomai4-framework-for-sdfr
   colcon build
   ```

3. Make sure you can execute the script to start the driver/bridge nodes

   ```bash
   cd ~/xenomai4-framework-for-sdfr
   chmod +x ./RosXenoBridge.bash
   ```

4. Ask a TA to remove flashing the FPGA from .bashrc

   Starting the RELbot driver/bridge nodes and motor communication (every time the nodes are started):

5. Start the Xenomai Thread using the following command:
   - **Note:** You should always start the Xenomai thread before starting the RosXenoBridge (next step).
   - **Note 2:** Opening a new terminal while the Xenomai thread is running might crash your RELbot! We are currently working on a solution to this, for now make sure that you have opened a few spare terminals and connected via SSH before starting the Xenomai thread. This includes terminals opened from within VSCode!

   ```bash
   sudo ~/xenomai4-framework-for-sdfr/build/xenoThread/xenoThread
   ```

6. Select/open a new terminal and start the bridge and driver nodes using the following command:
   - You will be asked for your password
  
   ```bash
   cd ~/xenomai4-framework-for-sdfr
   ./RosXenoBridge.bash
   ```