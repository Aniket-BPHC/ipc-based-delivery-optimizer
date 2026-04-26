# Multi-Truck Delivery System Simulator

A concurrent delivery system simulator built in C using shared memory, message queues, multithreading, and A* pathfinding to handle real-time package scheduling and routing.

## Overview

This project simulates a grid-based delivery system with multiple trucks handling dynamically arriving packages. Each package has constraints such as pickup location, drop location, and time-to-live (TTL). The goal is to deliver all packages efficiently while minimizing delays and expired deliveries.

The system interacts with external processes using inter-process communication (IPC) and operates in discrete time steps.

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

## How it Works

At each step:
1. The program reads system state from shared memory  
2. Assigns packages to trucks based on priority and distance  
3. Computes optimal routes using A* search  
4. Uses threads to compute required authorization strings  
5. Sends movement and action commands back via message queues  

## How to Run

gcc solution.c -o simulator -pthread
./simulator

## Notes

This project was developed as part of an Operating Systems coursework assignment and focuses on concurrency, IPC, and efficient scheduling under constraints.
