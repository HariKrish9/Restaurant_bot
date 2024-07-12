Restaurant Robot Simulation

This project is a simulation of a restaurant robot using ROS 2, Python, Nav2, SLAM, and Gazebo. The robot is designed to deliver food from the kitchen to various tables in a restaurant setting. The simulation uses the TurtleBot3 model to represent the robot.
Project Description

In this simulation, the robot starts from a home position and follows different scenarios to deliver food to the tables. The environment consists of three tables and a kitchen. The scenarios include:

    Simple Delivery: The robot moves from its home position to the kitchen, picks up the food, and delivers it to a specified table without requiring any confirmation.
    Confirmation-Based Delivery: The robot moves to the kitchen and waits for confirmation. If no confirmation is received within a timeout, it returns home. If confirmation is received, it proceeds to the table and waits for another confirmation before returning home.
    Conditional Delivery: The robot moves to the kitchen, waits for confirmation, and if confirmed, delivers to the table. If no confirmation is received at the table, it returns to the kitchen before heading home.
    Cancellation Handling: The robot's task can be canceled at any point. If canceled while heading to the table, it returns to the kitchen and then home. If canceled while heading to the kitchen, it returns directly home.
    Multiple Orders: The robot handles multiple orders, moving to the kitchen to collect food and delivering to multiple tables in sequence.
    Skipped Delivery on No Confirmation: For multiple orders, if no confirmation is received at a table, the robot skips that table and continues with the remaining deliveries before returning to the kitchen and then home.
    Order Cancellation: In the case of multiple orders, if an order is canceled for a particular table, the robot skips that table and continues with the other deliveries.

Technologies Used

    ROS 2: For robot operating system framework.
    Python: For scripting and automation.
    Nav2: For navigation and path planning.
    SLAM: For simultaneous localization and mapping.
    Gazebo: For 3D simulation of the robot and environment.
    TurtleBot3: Used as the robot model in the simulation.

Getting Started
Prerequisites

Ensure you have the following installed:

    ROS 2
    Gazebo
    TurtleBot3 packages
    Nav2 packages

Installation

    Clone the repository:

    bash

git clone https://github.com/HariKrish9/restaurant_bot.git
cd restaurant_robot_simulation

Build the workspace:

bash

colcon build

Source the workspace:

bash

    source install/setup.bash

Running the Simulation

    Launch the Gazebo world:

    bash

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py 

Run the navigation stack:

bash

ros2 launch nav2_bringup navigation_launch.py

Execute the robot delivery script:

bash![Screenshot from 2024-07-11 17-58-28](https://github.com/user-attachments/assets/bae2c088-54e4-49b8-8908-7d31f859fc77)
![Screenshot from 2024-07-11 17-57-48](https://github.com/user-attachments/assets/69b8953b-21b0-4e3c-a564-4803e1edb3ee)
![Screenshot from 2024-07-08 15-29-31](https://github.com/user-attachments/assets/9b613e6e-96f3-44c3-bcc6-a52805c48794)
![Screenshot from 2024-07-08 10-55-52](https://github.com/user-attachments/assets/3b97c202-ab9d-4445-96fe-ba2d89c90950)


    ros2 run restaurant_bot Waiter_robot.py

File Structure

    src/restaurant_simulation: Contains the ROS 2 package with nodes, launch files, and configurations.
    worlds/restaurant.world: The Gazebo world file defining the environment with tables and kitchen.
    models/: Directory for custom models used in the simulation.

Contributing

Contributions are welcome! Please submit a pull request or open an issue to discuss any changes.

Special thanks to the ROS 2 and Gazebo communities for their excellent documentation and support.
