Tags: `Vehicle Routing Problem`, `Genetic Algorithm`

- Ugo Coveilers-Munzi (MSc Aerospace Engineering, Control and Simulation, TU Delft)
- Antonio Minafra (MSc Aerospace Engineering, Control and Simulation, TU Delft)
- Jonathan van Zyl (MSc Aerospace Engineering, Control and Simulation, TU Delft)

Inspired from the paper :"A novel multi-objective optimization model for the vehicle routing problem with drone delivery and dynamic flight endurance" by S.Zhang, S.Liu, W.Xu & W.Wang.

The repo structure is as follows:
    - 'dataset' foler: contains all the datasets used in the paper, alos contains the modified datasets used by us
    - 'OriginalSensitivityAnalysis_Results' folder contains the sensitivity analysis results (images)
    - 'Paper_and_Notes' contains the paper to be reproduced as well as some notes written by us
    - 'Solutions' contains the solution and model files for the optimised solutions created by us
    - 'load_dataset.py' is a script containing a class to represent the dataset
    - 'load_solution.py' is a script which loads the solution from the .sol file and runs the post processing on it 
    - 'Trucks_Only_Dynamic_Weighting.ipynb' is the main notebook of this repo containing the code where the problem is formulated and optimised
