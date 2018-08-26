# CarND-Extended-Kalman-Filter-Project

---


## Extended Kalman Filter (EKF)

The goal of this project is to detect a bicycle that travels around the vehicle. I use an EKF track the bicycle's position and velocity.

The Kalman filtering procedure consists of two steps that are carried out iteratively:

* Prediction
* Update

Each time a new meassrement arrives, the filter predicts the current position and velocity (using past measurements and the estimated prediction confidence) and then corrects this prediction using the current (noisy) measurement. In this project, we deal with data coming from two diferent sensors:

* Radar
* Lidar

The prediction step is the same regardless of the sensor type but the update is sensor dependent.

See `writeup-report` for more details.


## Dependencies

* cmake >= 3.5
* All other dependencies can be installed by running:
	* install-ubuntu.sh
    

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./ExtendedKF`

**NOTE:** The EKF system is designed to be run against the [Udacity simulator](https://github.com/udacity/self-driving-car-sim/releases). Run EKF with the simulator already running.


## Examples

A video example is included in `videos` folder. The folder contains a `README` file with more information.
