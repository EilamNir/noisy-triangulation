# noisy-triangulation

## Setup

To run, open matlab in the `src` directory, and run the `src/+simulation/Simulation.m` script.

## TODO

- [ ] Create a library for generating the true path of a target
  - [x] Support straight lines
  - [x] Support turns with constant radius on x-y plane and constant speed on z axis
  - [x] Support turn in 3 dimensions with constant radius
  - [ ] allow acceleration in all intervals
    - [ ] in direction of motion
    - [ ] in x-y plane only
    - [ ] in z axis only
- [ ] Create library to extract noisy measurements from true path
  - [ ] support multiple sensors
  - [x] allow choosing which measurements to the target to get:
    - [x] distance
    - [x] angles
  - [x] allow adding normal noise to the measurements
  - [ ] add option for noise to increase based on distance
  - [ ] allow extracting only every other sample (or every X sample)
  - [ ] allow adding outlier noise to measurements
  - [ ] allow adding bias to measurements
  - [ ] allow dropping measurements at random
    - [ ] include blackout periods
- [ ] Create initial estimator based on noisy measurements
- [ ] Create path visualization library
