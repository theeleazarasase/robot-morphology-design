# A Computational Design Pipeline for Generating and Evaluating Robot Morphologies

## 1. Research Question

**3–4 sentences max**

Answer explicitly:
- What is unknown today?
Co Joint Design using limns and
- What are you testing or enabling?

**Template:**

How can we design a computational pipeline that generates diverse, physically valid robot morphologies using interpretable parameters, and evaluate them fairly under a fixed locomotion task without bias toward specific controllers or hand-designed forms?

> You may tweak, but do not expand.

---

## 2. Motivation

**Why anyone should care**

Touch on:
- Manual robot design is slow and biased
- Morphology is underexplored relative to control
- Fair comparison across designs is rare

> **📌 Note:** Keep it high-level, no citations yet.

---

## 3. Task Definition

**This anchors the entire project**

| Aspect | Specification |
|--------|---------------|
| **Task** | Flat-ground forward locomotion |
| **Horizon** | 10 seconds |
| **Environment** | Flat terrain, randomized friction |
| **Initial State** | Upright, zero velocity |

**State explicitly:**

> All designs are evaluated under identical task and environment contracts.

---

## 4. Metrics

**No more than TWO**

1. **Primary:** Average forward speed
2. **Secondary:** Cost of transport

**Why these two:**
- **Speed** → task effectiveness
- **CoT** → physical efficiency

> **Note:** This is where many papers fail. Yours won't.

---

## 5. Design Space

**What you control**

### Current parameters:
- Leg length
- Body clearance

### Constraints:
- Must start upright
- Must satisfy minimum clearance at rest

**State clearly:**

> The design space is intentionally low-dimensional to enable interpretability and controlled analysis.

---

## 6. Computational Pipeline

**This becomes your Figure 1**

### Numbered workflow:

1. Sample morphology parameters
2. Generate robot geometry
3. Simulate under fixed task
4. Log metrics across trials
5. Aggregate results per design

### Diagram:

```
Geometry → Simulation → Evaluation → Analysis
```

> Simple. Clean. Reviewers love this.

---

## 7. Expected Contributions

**Do not overpromise**

- [ ] A reproducible task-level evaluation framework for morphology studies
- [ ] A parametric morphology generator suitable for automated search
- [ ] Empirical insights into how simple geometric parameters affect locomotion

---

## 8. Scope and Limitations

**This signals maturity**

Explicitly state:
- ❌ No controller learning (initially)
- ❌ No complex terrain
- ❌ Simulation-only in early phase

---

## License

[Specify your license here]

## Contact

[Your contact information]
