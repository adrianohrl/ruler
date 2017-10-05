# Application Simulation

These tools launch the simulator [MobileSim](http://robots.mobilerobots.com/wiki/MobileSim) for MRS applications.

Usage:

    run_mobilesim [--help | -h] 
                  [--map | -m <map path>] 
                  [--num-robots | -n <number of robots>]
                  [--ros-pkg | -p <ROS package name>]

-------------------

# run_alliance_test_mobilesim

This tool simplifies the usage of the run_mobilesim tool for the ROS package alliance_test.

So, in order to use these tools, firstly enter the following command to a terminal:

	roscd alliance_test/bin
	BASH_FILE=../../../bin/run_mobilesim.bash
	chmod +x ${BASH_FILE} ./run_alliance_test_mobilesim.bash
	ln -s ${BASH_FILE} .

And then, just enter the following command:

	./run_alliance_test_mobilesim.bash map3