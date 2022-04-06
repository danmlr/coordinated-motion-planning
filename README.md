# Coordinated Motion Planning

Dan Meller, Gonzague de Carpentier

## Introduction

This repository contains the java implementation of the algorithm we developped to participate in the CG:SHOP 2021 computational geometry competition.
This project was suggested to us by Luca Castelli Aleardi in the framework of the INF421 course of the École Polytechnique. He also provided us several java classes that are identified in the code by a comment at the beginning of the file.
Our method is described in a document that we will publish soon on ResearchGate. It allowed us to obtain the 5th place on the global makespan ranking and the 3rd place among junior teams.

## Installation

Please ensure that you have java 11 installed. Then you just have to clone the repository. No external library is required.

## Compile (optional)

To compile after a change of the source files, use
```console
find -name "*.java" > source.txt && javac -cp core3.jar -d bin @source.txt
```

## Optimize Makespan

To compute a solution to the coordinated motion problem mimizing the makespan, use

```console
java -cp bin:core3.jar OptimizeMakespan <instance file>
```

For example:

```console
java -cp bin:core3.jar OptimizeMakespan instances/images/algae_00000_20x20_40_160.instance.json
```

This command executes the OptimizeMakespan class and enters an infinite loop (while true) that explores automatically the different possible values of p and updates the best solution found. You have to stop the programm manually when you consider that the best solution found is good enough.

The values of pmin and pmax can be adjusted directly in the OptimizeMakespan code. However, the generic values are sufficient to get satisfactory results in most cases.

As output, you obtain the best solution found (with the suffix makespan.json) as well as a file with a suffix log.csv which allows you to monitor the history of the algorithmic exploration. The first column of this file contains the value of p, the second a boolean indicating if the calculation was successful and the third column indicates the value of the makespan. 

## Optimize Distance

To compute a solution to the coordinated motion problem mimizing the makespan, use

```console
java -cp bin:core3.jar OptimizeDistance <instance file>
```

## Visualize solution

To visualize a solution, use

```console
java -cp bin:core3.jar MotionViewer <instance file> <solution file>
```

For example:

```console
java -cp bin:core3.jar MotionViewer instances/images/algae_00000_20x20_40_160.instance.json solutions/algae_00000_20x20_40_160_makespan.json
```
