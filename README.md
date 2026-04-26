# Multi-Truck Delivery System Simulator

A concurrent delivery system simulator built in C using shared memory, message queues, multithreading, and A* pathfinding to handle real-time package scheduling and routing.

## Overview

This project simulates a grid-based delivery system with multiple trucks handling dynamically arriving packages. Each package has constraints such as pickup location, drop location, and time-to-live (TTL). The goal is to deliver all packages efficiently while minimizing delays and expired deliveries.

The system interacts with helper and solver processes using inter-process communication (IPC) and operates in discrete time steps.

## Key Features

- Multi-truck delivery simulation on a grid  
- Real-time package scheduling with constraints (TTL, capacity, toll delays)  
- Inter-process communication using shared memory and message queues  
- A* pathfinding for optimized routing  
- Multithreaded computation using pthreads  
- Greedy and priority-based package assignment  

## Tech Stack

- C (POSIX compliant)  
- pthreads (multithreading)  
- System V IPC (shared memory, message queues)  

## File Structure

All files are kept in the same directory:

.
├── solution.c # Main implementation
├── helper.c # Helper process
├── helper.h
├── testcase_gen.py # Testcase generator
├── input.txt # Generated testcase
├── README.md

## Generating Testcases

Modify parameters in `testcase_gen.py`:
- N, D, S, T, B  
- max_new_requests_per_turn  
- number_of_requests  
- max_booth_cost  
- test_case_number  

Then run:

python testcase_gen.py

This generates the input file (`input.txt`).

## Running the Simulation

Compile:

gcc solution.c -lpthread -o solution
gcc helper.c -lpthread -o helper

Run:

./helper <TESTCASE_NUMBER>

Replace `<TESTCASE_NUMBER>` with the value set in `test_case_number`.

## How it Works

At each step:
1. The program reads system state from shared memory  
2. Assigns packages to trucks based on priority and distance  
3. Computes optimal routes using A* search  
4. Uses threads to compute required authorization strings  
5. Sends movement and action commands back via message queues  

## Notes

This project was developed as part of an Operating Systems coursework assignment and focuses on concurrency, IPC, and efficient scheduling under constraints.
