# facebot_project
This project is a real-time face recognition attendance system implemented in Python using the ROS framework.

### Prerequisites

Before running the Facebot project, ensure that the following prerequisites are met:

- ROS (Robot Operating System) is installed on the system.
- Python 3 is installed.
- The following Python libraries are installed: OpenCV, face_recognition, and cv_bridge.

### Installation

To install and set up the Facebot project, follow these steps:

1. Clone the project repository from GitHub:
   - Open a terminal.
   - Execute the following command:
     ```
     git clone https://github.com/weisze-yo/facebot_project.git
     ```

2. Navigate to the project directory:
   ```
   cd facebot_project
   ```

3. Install the required dependencies by executing the following command:
   ```
   pip install -r requirements.txt
   ```

### Usage

To run the Facebot project, follow these steps:

1. Start the ROS core by executing the following command:
   ```
   roscore
   ```

2. Run the command subscriber node by executing the following command:
   ```
   rosrun facebot_package receive_command.py
   ```

3. Run the command publisher node by executing the following command:
   ```
   rosrun facebot_package send_command.py
   ```

### Acknowledgement

This is a project where it is used to fulfill the assignment that is assigned in WID3005 Intelligent Robotics at [University of Malaya](https://www.um.edu.my/).

Finally, I would like to appreciate my teammates' hard work on this project, they are: 
1. [Khor Zhi Qian](https://github.com/Keyu0625)
2. [Lim Wei Sze](https://github.com/weisze-yo)
3. [Kelvin Cheah]()
4. [Teh Yi Ying](https://github.com/yiying305)

