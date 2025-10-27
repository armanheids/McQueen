# McQueen Framework

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![CARLA](https://img.shields.io/badge/CARLA-0.9.13-orange.svg)](https://carla.org/)

**McQueen** is a self-driving framework for runtime safety assurance that integrates model-based verification with adaptive model-learning for autonomous vehicles.

## 🎯 Overview

McQueen combines formal verification with machine learning to provide real-time safety guarantees for self-driving cars. The framework employs probabilistic model checking (using PRISM) to verify safety and performance properties, while a learning module approximates verification outcomes to reduce computational overhead by up to 70%.

### Key Features

- **Formal Safety Verification**: Uses probabilistic model checking (PRISM) to verify PCTL properties at runtime
- **Model Learning**: Learns from verification outcomes to predict safety metrics with <5% approximation error
- **Hybrid Verification**: Automatically switches between learned predictions and full verification based on scenario novelty
- **Multi-Objective Reasoning**: Balances safety and performance trade-offs in real-time
- **MAPE-K Architecture**: Implements a Monitor-Analyze-Plan-Execute-Knowledge loop for adaptive decision-making
- **Modular Design**: Easily extensible components for perception, planning, and verification

## 🏗️ Architecture

McQueen consists of three main layers:

1. **Simulation and Network Layer**: Built on CARLA and Unreal Engine for high-fidelity simulation
2. **MAPE-K Runtime Loop**: Handles monitoring, analysis, planning, and execution
3. **Model Learning and Assurance Layer**: Reduces verification overhead while maintaining safety guarantees

## 📊 Performance
- Decision-making time reduction: Up to 70%
- Reaction latency reduction: 65%
- Approximation accuracy: <5% deviation from formal verification
- Safety margin: >0.8m maintained even at out-of-speed-limit conditions (50-60 km/h)
