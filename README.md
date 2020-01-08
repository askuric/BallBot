# BallBot

Robust control analysis framework for the Ballbot system 


### Running
In matlab comand window:
 - script `iterative_uncertainty_parameters_eval.m` needs Matlab > R2018a because it used `diskmargin` function for MIMO stability. This was not supported in 2018a. In R2019b it works properly.
```matlab
model_script
iterative_uncertanty_parameters_eval
simulation_nonlin
```
