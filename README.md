# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

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

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Reflections

1. The effect of P, I, D components on the implementation:
- P component - This is the proportional component of the PID controller. This allows the car to steer in proportional to the cross-track error (which is the distance from the center of the lane). If this is the only parameter set (Kp), then the car would tend to oscillate around the center of the lane.
- I Component - This is the 'integral' component of the PID controller. This is used to fix a systematic bias, such as a steering drift. However, this doesn't help much in the simulation since there doesn't seem to be a steering issue. But it could be essential in real-world systems. For my implementation, I've set this to 0.0.
- D component - This is the 'differential' component of the PID controller. This is used to prevent the osicallations arising from the 'proportional' component. If properly tuned, it allows the car to approach center line smoothly. This is calculated by mulitplying 'Kd' by the derviative of the CTE. The derivative in this case is the difference between the current CTE and the previous CTE.

2. How the final parameters were chosen:
The final parameters were obtained through a combination of manual and twiddle tuning. Since there is no steering drift in the car, I left the Ki (integral) parameter as 0.0. I started by setting Kp as 1.0, which caused some oscillations. So, I fixed Kp as 1.0 and tried to come up with a Kd which could prevent the oscillations. After some experimentation, I found that a Kd of 22 worked well. To further tune, I enhanced the PID controller with a 'twiddle' algorithm. My twiddle algorithm had one caveat; it would not modify the 'Ki' parameter, since the simulated car has no steering drift. The dP parameter were initialized to 0.01, i.e any changes to the Kp and Kd parameters would be increased/decreased by 0.01 initially. Using twiddle, the final parameters I arrived with were:

Kp = 1.15, Ki = 0.0, Kd = 22.11

