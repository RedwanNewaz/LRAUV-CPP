# Experiment with obstacles 

## Parameters 
There are mainly three parameters are used for conducting these experiments.
The parameter --obs is used to define the number of obstacles in the environment. 
The parameter --test is used to define the test condition, and 
finally the parameter --demo is used to visualize the results.

## Experiments 
* [Exp] 01/1.txt --obs 4
* [Exp] 01/1.txt --obs 7
* [Exp] 61/1.txt --obs 5
* [Exp] 61/1.txt --obs 7
* [Exp] 111/4.txt --obs 3
* [Exp] 111/1.txt --obs 7

## Hyper-parameter tuning 
The success of the planner largely depends on hyper-parameters.
For the above settings following hyper-parameters are selected

* [MCTS.cpp line 145] the sigma value for probs is selected 0.5
* [MCTS.cpp line 146] the sigma value for cost is selected 0.35
* [MCTS.h line 21] the lambda value is selected 1.20

The utility is given by
```cpp
double utlity = Rewards[j] -  ( lambda * Costs[j]);
````