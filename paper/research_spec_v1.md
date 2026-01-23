# Title (Working):Establishing Interpretable Morphology Axes for Controlled Evaluation of Legged Robot Locomotion

## 1. Research Question
How do controlled low dimesional changes in robot morphology affect locomotion performance under a fixed task and evaluation protocol in simulation?

The project isolates morphology as the independent variable while holding task definition, episode structure,evaluation metrics, and simulation pipeline constant.

The goal is not to discover an optimal robot design, but to establish a stable, interpretable morphology axis that produces measurable and reproducible effects on locomotion behavior

## 2. Motivation
Robotics research frequently optimizes morphology and control jointly which makes it difficult to attribute performance differences to physical design versus controller behavior.
As such While prior work provides valuable insights, variations in morphology, control strategies, and evaluation protocols make cross-study attribution challenging.

This project addresses this by constructing a controlled experimental setup where:
- morphology varies in a principled and minimal way,
- evaluation protocols are fixed and repeatable,
- and observed performance changes can be directly attributed to body parameters.

## 3. Scope and assumptions

**In Scope**
- Simulated legged robots
- Flat-ground forward locomotion
- Parametric morphology variation
- Controller-agnostic evaluation metrics

**Out of Scope**
- Sim-to-real transfer
- Learning algorithm comparison
- Adaptive or online morphology change
- Uneven terraindisturbances, or recovery behaviors

## 4. Task Definition

| Aspect | Specification |
|--------|---------------|
| **Task** | Forward locomotion on flat terrain |
| **Horizon** | 10 seconds |
| **Environment** | Flat terrain with randomized friction |
| **Initial State** | Upright, zero velocity |
| **Termination conditions** | fall, instability, or leaving arena bounds|


## 5. Evaluation Metrics

**No more than TWO**

1. **Primary:** Average forward speed(m/s)
2. **Secondary:** Cost of transport(unitless)

**Success Criterion**:
Net forward displacement ≥ 0.5 m over the episode horizon.

**Why these two:**
Metrics are:
- controller-agnostic,
- consistently computed across designs,
- and logged at fixed resolution

## 6. Morphology Design Space

## Current parameters:
- Leg length(continous)
- Body clearance(continous)

## Design Constraints:
- Robot must start upright without support
- Must satisfy minimum clearance at start pose

> The design space is intentionally low-dimensional to enable interpretability and controlled analysis. No controller parameters are optimized as part of the design sweep


## 7. Computational Pipeline

1. Sample morphology parameters
2. Generate robot geometry
3. Simulate under fixed horizon simulation trials
4. Log state, enrgy and contact data
5. Aggregate results per design
6. Compare performance across morphology variants


# 8. Expected Contributions
- A validated morphology parameterization
- Performance trends attributable to body design
- A reproducible experimental setup suitable for extension
- Visualizations for paper


