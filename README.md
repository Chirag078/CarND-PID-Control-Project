# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## **Reflection**
### P component

- It sets the steering angle in proportion to CTE with a proportional factor tau.
       Angle = -tau * cte

- In other words, the P, or "proportional", component had the most directly observable effect on the car’s behaviour. 
- It causes the car to steer proportional (and opposite) to the car’s distance from the lane center(CTE) 
- if the car is far to the right it steers hard to the left, if it’s slightly to the left it steers slightly to the right.


### D component
- It’s the differential component of the controller which helps to take temporal derivative of error. This means when the car turned enough to reduce the error, it will help not to overshoot through the x axis.
- The D, or "differential", component counteracts the P component’s tendency to ring and overshoot the center line. 
- A properly tuned D parameter will cause the car to approach the center line smoothly without ringing.
       diff_cte = cte - prev_cte
       prev_cte = cte
       Angle = - tau_d * diff_cte


### I component
 - It’s the integral or sum of error to deal with systematic biases.
 - The I, or "integral", component counteracts a bias in the CTE which prevents the P-D controller from reaching the center line. 
 - This bias can be steering drift 
        int_cte += cte
        tau_i * int_cte

### PID Controller 

       steer = -tau_p * cte - tau_d * diff_cte - tau_i * int_cte



### Coofficent Selection 
  - I have Used Twiddle method to Check the coofficents (Kp,Ki,Kd) (Not Visible in Code)
  - After that i have used the parameter which is given in lession for this Project (Available in Code)


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
