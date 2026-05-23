# USV Path Planning Based on Improved A*-DWA Algorithm

This repository provides the experimental code and map data for the paper **"USV Path Planning Based on Improved A*-DWA Algorithm"**.

The proposed method combines an improved A* algorithm and an improved Dynamic Window Approach (DWA) for unmanned surface vehicle path planning in complex river environments. The code can be used to reproduce the simulation figures and performance comparison results reported in the paper.

## Repository Structure

```text
A-DWA/
├── Picture4_Table5.m                  % Main simulation code for Fig. 4
├── Table5_single_map_coast1111.m      % Single-map statistical code for Table 5
├── DWA_Parameter_Comparison.m         % DWA parameter comparison experiment
├── K1K2K3.m                           % Improved A* parameter comparison experiment
├── coast.m                            % Coast map simulation file
├── coast11.png                        % River map image
├── coast111_1.jpg                     % River map image
├── coast1111.jpg                      % River map image
└── README.md                          % Description file
```

## Requirements

The code was tested under the following environment:

- MATLAB R2023b
- Image Processing Toolbox

## How to Use

### 1. Download the Repository

Users can download this repository by clicking **Code → Download ZIP**, or clone it using Git:

```bash
git clone https://github.com/jzy1069049376/USV-path-planning-based-on-improved-A--DWA-algorithm.git
```

After downloading, open MATLAB and set the current working directory to the `A-DWA` folder.

### 2. Reproduce the Path Planning Experiment

To reproduce the path planning simulation shown in Fig. 4, run:

```matlab
Picture4_Table5
```

This script reads the river map image, generates the binary grid map, sets the start point, goal point and dynamic obstacle, and then compares the path planning results of different algorithms.

### 3. Reproduce the Table 5 Data

To obtain the performance data in Table 5, run the single-map statistical script:

```matlab
Table5_single_map_coast1111
```

The program will output the following performance indicators:

```text
number of iterations
search time
path length
safety distance
trajectory deviation
```

The generated data can be used to construct Table 5 in the paper.

### 4. Change the Experimental Map

The current Table 5 script uses `coast1111.jpg` as the experimental map. To test another map, only the map image path in the MATLAB code needs to be modified.

For example:

```matlab
map_image_path = 'coast1111.jpg';
```

can be changed to:

```matlab
map_image_path = 'coast11.png';
```

or:

```matlab
map_image_path = 'coast111_1.jpg';
```

All other algorithm parameters remain unchanged. In this way, different map data can be obtained under the same experimental settings.

### 5. Parameter Sensitivity Experiments

To reproduce the parameter sensitivity experiment of the improved A* algorithm, run:

```matlab
K1K2K3
```

To reproduce the parameter sensitivity experiment of the DWA evaluation function weights, run:

```matlab
DWA_Parameter_Comparison
```

## Notes

1. If the map image cannot be found, please make sure that the map image and the MATLAB script are placed in the same folder, or modify the image path in the code.

2. Since the start point, goal point and dynamic obstacle configuration may affect the final numerical results, the same experimental settings should be used when comparing different algorithms.

3. To reproduce the results more accurately, it is recommended to keep the map, start point, goal point, dynamic obstacle position and algorithm parameters unchanged during repeated experiments.

4. The experimental results may vary slightly due to different MATLAB versions or computer environments.

## Citation

If you use this code or data, please cite the related paper:

**USV Path Planning Based on Improved A*-DWA Algorithm.**
