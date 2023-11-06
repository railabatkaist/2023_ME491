## ME 491 2023 project

### Dependencies
- raisim

### environment setting
- TA`s computer setting (We recommend)
  1. install CUDA 11.3 if you have NVIDIA GPU
  1. python 3.9.7
  2. pytorch v1.11.0

### Run
1. Compile raisimgym **(If you fix c++ code, every time do this)**: ```python3 setup develop```
2. run runner.py of the task (for anymal example): ```cd ME491_2023_project/env/envs/rsg_anymal && python3 ./runner.py```

### Test policy
1. Compile raisimgym: ```python3 setup develop```
2. run tester.py of the task with policy (for anymal example): ``` python3 ME491_2023_project/env/envs/rsg_anymal/tester.py --weight data/ME491_2023_project/FOLDER_NAME/full_XXX.pt```

### Retrain policy
1. run runner.py of the task with policy (for anymal example): ``` python3 ME491_2023_project/env/envs/rsg_anymal/runner.py --mode retrain --weight data/ME491_2023_project/FOLDER_NAME/full_XXX.pt```

### Debugging
1. Compile raisimgym with debug symbols: ```python3 setup develop --Debug```. This compiles <YOUR_APP_NAME>_debug_app
2. Run it with Valgrind. I strongly recommend using Clion for 
