# AutonomyCLTrust

This is a research project conducted by Jiahe Pan at the University of Melbourne, Australia, under supervision of Jonathan Eden, Denny Oetomo and Wafa Johal. We utilize a shared control teleoperated target reaching task based on [Fitts' Law](http://www2.psychology.uiowa.edu/faculty/mordkoff/InfoProc/pdfs/Fitts%201954.pdf) to investigate the effects of task difficulty and robot autonomy on the human operator's task performance, cognitive load and trust. We use the [Franka Emika robot arm](https://franka.de/research) and the [Novint Falcon haptic device](https://www.forcedimension.com/company/about) for the target reaching task, and [Tobii eye trackers](https://www.tobii.com/solutions/scientific-research) for one of the cognitive load measures. Experiments are conducted with 24 participants. 


## Project Links
- Project site: [TODO]
- Demo video: [TODO]


## Contents

- [ROS2 Workspace](#1)
- [Eye-Tracking](#2)
- [Tapping Task](#3)
- [Dataframes](#4)
- [Data Analysis](#5)
- [Paper and Citation Info](#6)


<br>

<a id='1'></a>

## ROS2 Workspace

A laptop with <strong>Ubuntu 22.04</strong> and <strong>ROS2 (Humble)</strong> installations are required. The `ros2_ws` workspace contains the following two ROS packages:
- `ros2_package`
- `tutorial_interfaces`

### ros2_package
This package contains the code files for the primary task, including receiving information from and sending control commands to the robot, data logging scripts, and rendering the task scene in RViz. Specifically, it contains the following sub-folders:

| Folder | Description |
| ------ | ------ |
| `/data_logging/csv_logs` | Contains the raw data (`.csv` format) collected from all participants, including a header file for each participant with the calculated task performances for each trial condition. |
| `/launch` | Contains ROS launch files to run the nodes defined in the `/src` folder, including launching the controller with both the [Gazebo](https://docs.ros.org/en/foxy/Tutorials/Advanced/Simulators/Ignition/Ignition.html) simulator and the real robot, and to start the RViz rendering of the task. |
| `/ros2_package` | Contains package files including useful functions to generate the trajectories, parameters to run experiments, and the definition of the `DataLogger` Python class. |
| `/scripts` | Contains the definition of the `TrajRecorder` Python class, used for receiving and saving control commands and robot poses into temporary data structures, before logging the data to csv files using a `DataLogger` instance. |
| `/src` | Contains C++ source code for the ROS nodes used, including class definitions of the `GazeboController` and `RealController` for controlling the robot in simulation and the real world respectively, the `PositionTalker` for reading the position of the Falcon joystick, and the `MarkerPublisher` for publishing visualization markers into the RViz rendering.  |
| `/urdf` | Contains an auto-generated URDF file of the Franka Emika robot arm.  |

### tutorial_interfaces
This packcage contains custom ROS message and service definitions. Specifically, there are two custom `msg` interfaces (in the `/msg` directory) defined for communication and data logging:
| Msg | Description |
| ------ | ------ |
| `Falconpos.msg` | A simple definition of a 3D coordinate in Euclidean space. Attributes: `x, y, z` |
| `PosInfo.msg` | A definition of the state vector of the system for a given timestamp. Attributes: `ref_position[], human_position[], robot_position[], tcp_position[], time_from_start` |


<br>

<a id='2'></a>

## Eye-Tracking

The implementation uses a laptop with <strong>Windows 11</strong> installation.

In order to use the Tobii eye-tracker, first install the required SDK from PyPI:
```shell script
pip install tobii-research
```
Then, it can be imported into Python as:
```python
import tobii_research as tr
```
To connect to and receive information from the eye-tracker:
```python
found_eyetrackers = tr.find_all_eyetrackers()
my_eyetracker = found_eyetrackers[0]
my_eyetracker.subscribe_to(tr.EYETRACKER_GAZE_DATA, 
                           gaze_data_callback, 
                           as_dictionary=True)

# Basic definition of the callback function
def gaze_data_callback(self, gaze_data):

    left_eye_gaze = gaze_data['left_gaze_point_on_display_area']
    right_eye_gaze = gaze_data['right_gaze_point_on_display_area']
    
    left_pupil_diameter = gaze_data['left_pupil_diameter']
    right_pupil_diameter = gaze_data['right_pupil_diameter']
```
To disconnect:
```python
my_eyetracker.unsubscribe_from(tr.EYETRACKER_GAZE_DATA, gaze_data_callback)
```
For more details of implementation, please refer to `rhythm_method.py` located in the `/experiment/secondary task/` directory.


<br>

<a id='3'></a>

## Tapping Task

The implementation uses a laptop with <strong>Windows 11</strong> installation.

The <strong>dual-task method</strong> is adopted in this project to capture cognitive load objectively, in addition to using pupil diameter. Specifically, "[The Rhythm Method](https://onlinelibrary.wiley.com/doi/abs/10.1002/acp.3100)" is employed as the secondary task, which involves a rhythmic tapping at a given tempo.

The rhythm files are in `.wav` format, and can be generated from any software (e.g. GarageBand on IOS devices).

The implementation of the method in this project uses the following Python libraries:
```python
from keyboard import read_key       # to record key taps on the keyboard
from playsound import playsound     # to play the rhythm for participants' reference
```
which can be installed via PyPI:
```shell script
pip install keyboard playsound
```
For more details of implementation, please refer to `rhythm_method.py` located in the `/experiment/secondary task/` directory.


<br>

<a id='4'></a>

## Dataframes

The data logged throughout each of the experimental sessions are written to `.csv` files. These include both the main measures of interest and the initial demographics information collected at the start of each session. The final processed `.csv` files are all located in the `/experiment/grouped_dataframes/` directory. Their descriptions and file names are summarized below:

| Measure | Description | Filename |
| ------ | ------ | ------ |
| Trajectory Tracking Error | RMSE error (cm) between the reference and recorded trajectories of each trial | `grouped_traj_err.csv` |
| Rhythm Tapping Error | Normalized percentage error (%) of the inter-tap interval lengths (relative to participant's baseline) | `grouped_tapping_err.csv` |
| Pupil Diameter | Pupil diameter (mm) for both left and right eyes, and averaged across them | `grouped_pupil.csv` |
| Perceived Autonomy | Participants' perceived level of robot autonomy, rated on a 10-point Likert scale | `grouped_p_auto.csv` |
| Perceived Trust | Participants' self-reported trust using a 10-point Likert scale | `grouped_p_trust.csv` |
| NASA-TLX | Self-reported cognitive load levels across all 6 aspects of the [NASA-TLX](https://www.sciencedirect.com/science/article/abs/pii/S0166411508623869) questionnaire | `grouped_tlx.csv` |
| MDMT |  Self-reported trust levels across all 8 dimensions of the [MDMT](https://research.clps.brown.edu/SocCogSci/Measures/CurrentVersion_MDMT.pdf) questionnaire | `grouped_mdmt.csv` |


<br>

<a id='5'></a>

## Data Analysis

The data analysis was performed in [RStudio](https://posit.co/download/rstudio-desktop/), leveraging existing libraries in the [R programming langauge](https://www.r-project.org/about.html). All R scripts used are located in the `/analysis/R_scripts/` directory, which has the following three sub-folders:
- `indiv_measures`: Individual analysis of autonomy's effect on each of the measures using ANOVAs
- `interactions`: Analysis of correlations and interaction effects between the measures and autonomy conditions
- `learning_effect`: Identification of possible learning effects for the repeated measures within each round using Linear Mixed Models

Plots are also generated in R, and the code are embedded within the above R scripts. The preliminary plots are located in the `/analysis/plots/` directory. The actual plots used in the paper (in `.jpg` format) can be found in the `/analysis/pdf_plots/pdf_to_jpg/` directory.


<br>

<a id='6'></a>

## Paper and Citation Info

The manuscript and supplementary video can be found on [IEEEXplore](https://ieeexplore.ieee.org/abstract/document/10517390).
If you find our work useful, please consider citing it using:
```
@article{pan2024effects,
  title={Effects of Shared Control on Cognitive Load and Trust in Teleoperated Trajectory Tracking},
  author={Pan, Jiahe and Eden, Jonathan and Oetomo, Denny and Johal, Wafa},
  journal={IEEE Robotics and Automation Letters},
  year={2024},
  publisher={IEEE}
}
```