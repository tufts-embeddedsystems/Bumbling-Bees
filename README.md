# Bumbling-Bees

Contributors: Max Ramer, Brandon Gray, Melinda Burns, Alina Shah, Arnav Gupta, Muheng Shen



## Goal

Design, build, and program 10 temperature sensor nodes to measure temperature once an hour.  Nodes should last 6 months in the field.



## Roles

Hardware Engineers: Muheng & Arnav

Firmware Engineers: Brandon & Max

Software Engineers: Mels & Alina



## Tasks

#### Hardware

- [ ] Create an acceptance testing plan. Must test:
  
  - [ ] Sensor and thermistor
  
  - [ ] Remaining battery life
  
  - [ ] WiFi capability

- [ ] Build 1 PCB dev board

- [ ] Verify dev board functionality with testing plan

- [ ] Build and test 10 PCB dev boards

#### Firmware

- [ ] Design main program execution loop

- [ ] Collaborate with software team to finish demos

- [ ] Collaborate with hardware team to develop acceptance tests using software team demos

#### Software

- [ ] Finalize demos with firmware team:
  
  - [ ] Sensor and thermistor
  
  - [ ] Remaining battery life
  
  - [ ] WiFi & MQTT

- [ ] Write program execution loop

- [ ] Setup web server-side HTML to test MQTT messages

## 

## File structure

- `hardware/`
  
  - `pcb/`
    
    - `devboard_final/`: all KiCad files for development of the temperature node PCB. Also includes Gerber files.
  
  - `testing_plan/`: documentation for acceptance tests necessary to ensure correct PCB design.

- `PlatformIO/`: necessary files for dev environment

- `software/`
  
  - `demo/`: all code to demo individual functionality (i.e. measure temperature, connect to WiFi, enter deep sleep)
  
  - `libs/`: any necessary libraries for development
    
    `main/`: main program execution code for a node. Encompasses all demos.



## Status Updates

#### Tuesday 4/26



#### Thursday 4/28

#### ...