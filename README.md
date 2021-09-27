# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

[//]: # (Image References)

[simulator]: ./output/simulator_running.png "Simulator"
[tuning]: ./output/pid_tuning.gif "tuning"

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

# PID Control Implementation
The directory structure of this repository is as follows:

```
CarND-PID-Control-Project
├── CMakeLists.txt
├── cmakepatch.txt
├── CODEOWNERS
├── install-mac.sh
├── install-ubuntu.sh
├── LICENSE
├── output
│   ├── pidcontrol_d_only.mp4
│   ├── pidcontrol_fullparams.mp4
│   ├── pidcontrol_i_only.mp4
│   ├── pidcontrol_p_only.mp4
│   ├── pid_tuning.gif
│   └── simulator_running.png
├── README.md
├── set_git.sh
└── src
    ├── json.hpp
    ├── main.cpp
    ├── PID.cpp
    └── PID.h
```

## PID Control
The PID implementation is done on the `./src/PID.cpp`. The PID::UpdateError method calculates proportional, integral and derivative errors and the PID::TotalError calculates the total error using the appropriate coefficients.

![alt text][simulator]

### P/I/D parameters

1. The proportional portion of the controller tries to steer the car toward the center line (against the cross-track error). If used this param only, the car overshoots the central line very easily and go out of the road very quickly. Example video: `./output/pidcontrol_p_only.mp4 `

2. The integral portion tries to eliminate a possible bias on the controlled system that could prevent the error to be eliminated. If used this param only, it makes the car to go in circles. Example video: `./output/pidcontrol_i_only.mp4 `

3. The differential portion helps to counteract the proportional trend to overshoot the center line by smoothing the approach to it. Example video: `./output/pidcontrol_d_only.mp4 `

| **Parameter** | **Rise time** | **Overshoot** | **Settling time**  | **Steady-state** **error**  | **Stability** |
|--|--|--|--|--|--|
| _**K<sub>p</sub>**_  |  Decrease   |  Increase   |  Small change    |  Decrease   |  Degrade   |
| _**K<sub>i</sub>**_  |  Decrease   |  Increase   |  Increase    |  Eliminate   |  Degrade   |
| _**K<sub>d</sub>**_  |  Small change   |  Decrease   |  Decrease    |  No effect in theory   |  Small change  |

Refer: https://people.ece.cornell.edu/land/courses/ece4760/FinalProjects/s2012/fas57_nyp7/Site/pidcontroller.html

### PID Tuning

The parameters was initially chosen as _K<sub>p</sub>_, _K<sub>i</sub>_ and _K<sub>d</sub>_ equal to _0.1_, _0.0_, and _0.0_ respectively. Then the _K<sub>p</sub>_ was increased until it reached a value of a value of _0.13_. Afterwards the _K<sub>i</sub>_ was introduced. Although the _K<sub>i</sub>_ can gives quicker responses, but having a high value of it, can introduce more oscillations, so an optimal value was found that was well performing of _0.0001_. Then the _K<sub>d</sub>_ was introduced and increased until the oscillations decreased and the required trajectory was reached, and it was set to a value of _1.25_.

Besides, a throttle PID was implemented to control the acceleration and deceleration based on the error difference between the target speed of _30.0 mph_ and the current speed, also it was tuned manually until getting optimal values.

The final values of the two PID Controllers are:
| | **_K<sub>p</sub>_** | **_K<sub>i</sub>_** | **_K<sub>d</sub>_** |
|--|--|--|--|
|**steering_pid**|_0.13_|_0.0001_|_1.25_|
|**speed_pid**|_0.1_|_0.001_|_0.05_|

![alt text][tuning]

## Results

The vehicle drive successfully at least a lap around the track. 
Details video: `./output/pidcontrol_fullparams.mp4 `