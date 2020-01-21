# BallBot

Ballbot robust stability analysis and validation script

### Developement report on the [link](https://drive.google.com/file/d/173H89YTg0KzFeuuBQUfmOdzcJab17GtW/view?usp=sharing)

### Contents:
- Nonlinear model synthesis with distrubances
- Parametric linearisation 
- Parametric uncertainty introduction
optimisation based worst case search
- Disturbance rejection and noise attenutation
- H-infinity controller synthesis
based on linear systems 
based on worst case system
- Comparison of behavior with nonlinear model
Created January 2020 by Antun Skuric

### Running
- Download the directory to your pc and open matlab in the same directory.
```shell
git clone https://github.com/askuric/BallBot.git
```

- To run just open in matlab vindow the live script `ballbot_analysis_live.mdx` and follow the execution. 
Be aware that it may take some time to execute completely. 10-20 minutes 
```shell
ballbot_analysis_live.mdx
```


### Folder structure
 - visualisation
	- helping functions for visualisation
 - optimisation	
	- helping functions for optimisation
 - figures
	- figures used in the report "Ballbot robust analysis report.pdf"
