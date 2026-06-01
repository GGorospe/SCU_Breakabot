# Break-a-bot Research and Development

## Overview

The Break-a-bot (BB) is a mobile robot used for developing and
validating diagnostic algorithms.

### Research Goal: Using the operational and fault data collected from the BB and system models, develop a diagnostic algorithm for fault detection. Diagnostic methods may include:  

1. **Model-based Diagnostic Observer** — using a mathematical model of the system to produce a predicted state vector that can be compared with the measured vector to produce a residual describing difference between the model and the actual behavior of the system. When the residual exceeds a threshold a failure may be present.
2. **Principal Component Analysis** — (unsupervised) PCA takes high dimensional data, like sensor measurements, and transforms them into principal components. We measure the presence of faults by monitoring the distance between the calculated components and those determined during baseline.
3. **Support Vector Machines (SVM)** - a supervised learning technique that determines the optimal hyperplane or boundary between data points of either healthy or unhealthy classes.A SVM transforms sensor data into high-dimensional space where the different states are easily separable.

## Diagnostic Algorithm Flow Chart
The following chart details how each othese algorithms process break-a-bot data.

[Break-a-bot Diagnostic Algorithm Flow Chart](/diagrams/Break-a-bot_Diagnostic_Algorithm_Flow_Chart.png)
<p align="center">
  <img src="/diagrams/Break-a-bot_Diagnostic_Algorithm_Flow_Chart.png" alt="Break-a-Bot Diagnostic Algorithm Flow Chart" width="600"/>
</p>
