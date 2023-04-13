# noisy-triangulation

## Setup

To run, open matlab in the `src` directory, and run the `src/+simulation/Simulation.m` script.

## TODO

- [ ] Create a library for generating the true path of a target
  - [x] Support straight lines
  - [ ] Support turns with constant radius on x-y plane and constant speed on z axis
  - [ ] Support turn in 3 dimensions with constant radius
- [ ] Create library to extract noisy measurements from true path
  - [ ] support multiple sensors
  - [ ] allow choosing which measurements to the target to get:
    - [ ] distance
    - [ ] angles
  - [ ] allow adding normal noise to the measurements
  - [ ] allow adding outlier noise to measurements
  - [ ] allow adding bias to measurements
  - [ ] allow dropping measurements at random
    - [ ] include blackout periods
- [ ] Create initial estimator based on noisy measurements
- [ ] Create path visualization library
