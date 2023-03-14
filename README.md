# Redundancy Resolution
This repository shows basic example of redundancy resolution. At this point, we only show planar 2D robots, but generalization to spatial 3D robots can be immediately conducted.


# Installation
We assume the usage of (1) macOS, (2) `python3`, (3) [Visual Studio Code (VSCode)](https://code.visualstudio.com/) and (4) `virtualevnv` package.
By setting the current directory as the same as [requirements.txt](requirements.txt), run:
```
    python3 -m venv .venv 
```
By setting the virtual enviroment's name as `.venv`, VScode will automatically recognize the `venv` package and will activate the virtual environment. 
Once you have done this and open the terminal window *inside* VSCode, there will be `(.venv)` added in front of your command line. If that is the case, your are ready to run `pip`. 
```
    pip3 install -r requirements.txt
```
This will download all the necessary packages needed to run the code.
Once the python packages are installed without any problem, we need to check whether the installations are done properly. We move inside the [mujocoSim/controller](mujocoSim/) directory and run the code:
```
    cd ./mujocoSim/controller
    python3 ./ex_joint_imp.py
```
And you should see a 2-DOF planar robot with a first-order joint-space impedance controller.