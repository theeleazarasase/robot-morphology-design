# Computational Robot Morphology Generator

This repository contains a research prototype for generating and evaluating
parameterized robot morphologies under fixed task and evaluation contracts.

## Project Goal
Develop a computational design pipeline that:
- Encodes robot geometry parametrically
- Respects fabrication-relevant constraints
- Enables fair comparison across robot designs

## Task Definition
- Task: Flat-ground forward locomotion
- Horizon: 10 seconds
- Environment: Flat terrain with randomized friction
- Metrics:
  - Average forward speed
  - Cost of transport

Task and evaluation details are defined in:
`configs/experiment_task.yaml`

## Repository Structure
- configs/ # Task, environment, and experiment definitions
- src/ # Core code (robot, morphology, experiment logic)
- results/ # Generated logs (not tracked)
- paper/ # Research notes and draft material


## Current Status
- Robot and morphology classes implemented
- Task and metric contract fixed
- Dummy evaluation and parameter sweep implemented

## Next Steps
- Integrate physics simulation backend (PyBullet)
- Replace dummy evaluation with real dynamics
- Aggregate metrics across seeds

## Reproducibility
All experiments are defined by:
- Explicit YAML configs
- Logged random seeds
- Fixed evaluation protocol
