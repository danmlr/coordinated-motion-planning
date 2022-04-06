# Coordinated Motion Planning

Dan Meller, Gonzague de Carpentier

## Introduction

This repository contains the java implementation of the algorithm we developped to participate in the CG:SHOP 2021 computational geometry competition.
This project was proposed to us by Luca Castelli Aleardi in the framework of the INF421 course of the Ecole Polytechnique. He also provided us with several java classes that are identified in the code by a comment at the beginning of the file.
Our method is described in a document that we will publish soon on ResearchGate. It allowed us to obtain the 5th place on the global makespan ranking and the 3rd place among junior teams.

## Installation

Please ensure that you have java 11 installed. Then you just have to clone the repository. No external library is required.

## Use

```console
java --class-path bin:core3.jar OptimizeMakespan instances/images/algae_00000_20x20_40_160.instance.json
```

This command executes the OptimizeMakespan class and enters an infinite loop (while true) that explores automatically the different possible values of p and updates the best found solution. You have to stop the programm manually when you consider that the best found solution is good enough.

The values of pmin and pmax can be adjusted directly in the OptimizeMakespan code. However, the generic values are sufficient to get satisfactory results in most cases.

In output, you obtain the best found solution (with the suffix makespan.json) as well as a file with a suffix logHistory.csv which allows you to trace the history of the exploration by the algorithm. The first column of this file contains the value of p, the second a boolean indicating if the calculation was successful and the third column indicates the value of the makespan. 

## Other commands

```console
java --class-path bin:core3.jar OptimizeDistance
```

```console
java --class-path bin:core3.jar MotionViewer
```

```console
java --class-path bin:core3.jar CheckSolution
```
