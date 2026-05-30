Physics-Based Character Animation Plugin

A procedural character animation plugin for the Arkheon engine, implementing physics-driven motion control without pre-recorded animation data.

Overview

This plugin replaces traditional mocap-based animation with a real-time physics simulation of the character's skeleton. Each joint is controlled by a PD/PIPD controller that computes torques based on the difference between current and target bone orientations. The physics engine integrates these torques into natural motion using simplified moment-of-inertia physics.

Features

Procedural walking, pushing, and climbing — motion clips generated via sinusoidal functions with configurable phases and amplitudes
PD & PIPD controllers — proportional-derivative control with optional integral term for drift compensation
Blending — smooth 0.2s transitions between animation clips
WASD controls — movement, sprint (Shift), and camera rotation
Mission goal system — GOTO, PUSH, CLIMB, and PICKUP objectives with navmesh-based navigation
10-joint skeleton — shoulders, elbows, hips, knees, feet with per-joint torque computation

Project Structure

character-plugin-230201903/
├── include/
│   └── arkheon/
│       └── character/
│           └── ICharacterController.h   // SDK interface
├── src/                                 // Plugin implementation
├── tests/
│   └── standalone_test.cpp             // Standalone test harness
├── CMakeLists.txt                      // CMake build configuration
└── README.md

Build

mkdir build && cd build
cmake ..
make

Usage

Load the plugin into Arkheon via the engine's plugin manager
Spawn a character entity — the controller automatically attaches to its skeleton
Use WASD to move, Shift to sprint, hotkeys to switch animation clips
Developed as a student research project demonstrating physics-based animation for game characters.
