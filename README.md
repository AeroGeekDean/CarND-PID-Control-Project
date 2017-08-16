# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## Student Reflections

This project was to build a simple PID controller to control Udacity's car simulation.

The interface data from sim(ulation) --> PID controller are:
- vehicle speed, in [mph]
- vehicle cross-track error (CTE), in [m]
- steering angle feedback, in [deg], (-25, +25) (not used)

The interface data from PID controller --> sim are:
- steering command, normalized (-1: left, +1: right)
- 'throttle' command, normalized (-1: decel/reverse, +1: accel)

### Time
**Note**: there are **no timestamps** on the simulation output data!!!

>Even thought this project could be passed without being concerned with time, this is discouraged as it's very poor practice for controller design. By ignoring time, the controller will be unaware of any temporal variations in the system such as process cycle fluctuations, transport delays, etc. These variations will then cause the closed-loop dynamics behavoir of the system to vary, and likely affect system dynamic performance and stability.
>
>[See here for more details](https://discussions.udacity.com/t/some-questions-about-pid-project/315927/12) in my post to Udacity student discussion board on this topic.

Thus PID controller will generate receiver-side timestamps. This is not ideal, but it's better than nothing. The timestamp calculations (and the associated frame time, '`dt`') are implemented in the file: `main.cpp`, utilizing C++11's `std::chrono` library.

### Steering and 'Cruise' Control

The PID controller class is instantiated for both vehicle steering (lateral) and 'cruise' (longitudal) control axes.

By also implementing a 'cruise control' feature, we could keep the vehicle at a steady speed. This is beneficial as vehicle speed is considered a **dynamic state** of the system: it directly affects the sensitivity, performance and stability of the closed-loop system (vehicle + PID controller) in the lateral steering axis. Thus keeping steady vehicle speed would allow for steady behavior in the steering response.

### Controller Gain Scheduling (did not implement)

Because variations in vehicle speed would affect the steering response performance, a popular strategy is to vary the PID controller gains based on a function of speed. These could be implmented as either equations or linear interpolation of look-up tables.

More details on gain scheduling strategy at [Wikipedia](https://en.wikipedia.org/wiki/Gain_scheduling) and [MatLab Control System Design and Tuning documentation](https://www.mathworks.com/help/control/ug/gain-scheduled-control-systems.html).

This strategy was not implemented becuase:
After init testing, it appears the simulator's reference trajectory may contain discontinuities, in either CTE (trajectory) and/or d(CTE)/dx (trajectory curvature). Additionally, I was able to easily manual-tune a set of PID gains to navigate the car around the track at 20 mph. Thus did not implement the gain scheduling strategy as it was not needed.

However, this strategy is recommended if we desire the PID steering controller to operate the car at various different range of speeds.

### Effect of P-I-D gains
(Finally! What the original project rubic asked for....)

The PID controller could be think of as a 2nd-order dynamic system, such as a mass-spring-damper.

The **(P)roportional path** of the controller provides controller output that's proportional to the error signal. This is equivalent to the 'spring': the more it's stretched, the more force it generates.

The **(D)erivative path** of the controller helps provide damping to the proportional 'spring'. The faster the error signal grows or diminishes, the stronger the corrective output.

The **(I)ntegral path** of the controller helps to eliminate steady-state error (systemic bias). Example: consider an airplane flying with a cross-wind, or a boat traveling with cross-current. The vehicle heading would need to 'crabbed' into the cross-current in order for it to maintain its desired path. The integral path provides this. 

>Note: However, this behavior is most beneficial at steady-state, once the P- and D- paths have done their job in bringing the error signal close to steady-state zero. The I-path 'finishes the job' of bringing it to actual zero (in the presense of systemic bias).
>
>**WARNING**: During this dynamic transient, the I-path could actually counter the damping effect of the D-path!! (Even though their time-phases are slightly different.) Thus it's very common to LIMIT the Integral value within the I-path, as it would help with integrator wind-up issues as well. (Segway to next section...)

### Integrator Wind-Up prevention

To prevent the integrator-path to grow beyond desired bounds, the magnitude of `i_error` value is limited in the code. Please see [Wikipedia article](https://en.wikipedia.org/wiki/Integral_windup) for further discussion of this phenomon, and `PID.cpp` for how this was implemented in my code.

### Tuning of P-I-D gains

I manually tuned and came up with a set of PID gains that provided desirable behavior. These are the steps that I took to achieve these gains. (I did NOT use the "Twiddle" method. Althought I find it brute-force-ly interesting and pretty cool. Will definitely it for future!)

1. First I set I & D gains to zero, I tried out different **P gains** that will give me sufficient and desirable steering sensitivity. This will also be the steady-state controller gain as the D-path will be zero by definition of steady-state (excluding presence of systemic bias error).

2. I then experiment with various **D gains** that would help dampen and calm out the oscillations. I chose to 'over-damp' the system so that its response mimic that of a ['dead-beat' response](https://en.wikipedia.org/wiki/Dead-beat_control). This helps to eliminate excess overshoots that are present in critically damped or under-damped systems. For our project here, these overshoots could often lead to other stability issues thus it's better to eliminate them.

It was during this time that I noticed the Reference Trajectory (which which CTE is calculated from) is likely to contain discontinuous curvatures. These discontinuities will be seen by the PID controller as either step or ramp forcing function inputs. Thus my decision to abandon my original gain-scheduling strategy and forge ahead on the curriculum, as it'll make the effort that much more laborious. I've done enough of these controller tuning in my career that I'm satisfied I could do it if a task requires it. :)

3. Once I'm happy with the P-D gains, I tweaked-in some **I gain** such that the car would steadily zero out the CTE. This was manually tuned via data capturing and plotting. Since we do not have systemic bias in this example, this is not critical. The most important thing is to NOT have excessive I gain such that it affects the performance of the selected P-D gains. My implemention of the integrator limiting (wind-up prevention) also helps here.

Thus my final gains for the **steering** PID controller are:
* `p_gain` = 1.0
* `i_gain` = 0.3
* `d_gain` = 0.35

**NOTE** these gains independent of  `dt`!

On my computer, the resulting rate between simulator data frames are between 12 ~ 17 Hz, or a `dt` of 59 ~ 83 milli-second.

I also created a **'cruise'** controller, as discussed above. Its gains are:
* `p_gain` = 0.1
* `i_gain` = 0.02
* `d_gain` = 0.0

This was 'copied' from the Python code of Term1 Project-3. That controller initially did NOT have `dt` explicitly broken out. Thus I assume the same `dt` and backed calculated the resulting `d_gain` to use here.

</End of **Student Reflections** section>

---
## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
