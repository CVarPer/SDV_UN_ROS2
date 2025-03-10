# SDV_UN Platform, now on ROS 2 Jazzy
This is the documentation for most recent development for SDV vehicles in LabFabEx laboratory, corresponding to ROS 2 Jazzy. If you want to access previous works, you may refer to `sdv_melodic` branch. This repository intends to keep track of continuous efforts and provide accessibility to all interested users.

Sincethere are several aspects of SDV_UN development documented in existing documentation, such as firmware, hardware and communications, we will focus on the migration specifics and considerations.

# Migration Guide

1. **[Introduction](#introduction)**
   - [Migration objectives](#11-migration-objectives)
   - [General migration strategy](#12-general-migration-strategy)
2. **[Comparative Analysis and Preparation](#comparative-analysis-and-preparation)**
   - [General considerations about ROS 1 vs ROS 2](#21-general-considerations-about-ros-1-vs-ros-2)
     - [Architecture and Communication Changes](#211-architecture-and-communication-changes)
     - [Build System and Package Management](#212-build-system-and-package-management)
     - [Launch System and Parameter Management](#213-launch-system-and-parameter-management)
     - [Compatibility and Migration Challenges](#214-compatibility-and-migration-challenges)
     - [Summary of Key Differences](#215-summary-of-key-differences)
   - [Useful tools for migration](#22-useful-tools-for-migration)
   - [Recommended workflow](#23-recommended-workflow)
3. **[General Setup](#general-setup)**
   - [Installing and configuring ROS 2 Jazzy on Ubuntu 24.04](#31-installing-and-configuring-ros-2-jazzy-on-ubuntu-2404)
   - [Strategies for testing and comparing ROS 1 (Ubuntu 18.04) vs ROS 2 (Ubuntu 24.04)](#32-strategies-for-testing-and-comparing-ros-1-ubuntu-1804-vs-ros-2-ubuntu-2404)
4. **[Package Migration](#4-package-migration)**
   - **[4.1 `sdv_msgs`](#41-sdv_msgs-message-and-service-migration)** (Messages and services)
     - [Adapting `msg` and `srv` definitions](#411-adapting-msg-and-srv-definitions)
     - [Converting `CMakeLists.txt` and `package.xml`](#412-converting-cmakeliststxt-and-packagexml)
     - [Using `rosidl` for message generation in ROS 2](#413-using-rosidl-for-message-generation-in-ros-2)
   - **[4.2 `sdv_nav`](#42-sdv_nav-navigation-and-peripherals-migration)** (Navigation and peripherals)
     - [Migrating the Navigation Stack from ROS 1 to ROS 2](#421-migrating-the-navigation-stack-from-ros-1-to-ros-2)
     - [Adjusting launch files](#422-adjusting-launch-files)
     - [Dealing with parameter files](#423-dealing-with-parameter-files)
   - **[4.3 `sdv_process`](#43-sdv_process-process-migration)** (Process and control)
     - [Node migration](#431-node-migration)
     - [Adapting scripts](#432-adapting-scripts)
     - [Managing dependencies](#433-managing-dependencies)

---

## **1. Introduction**  


### 1.1 Migration Objectives
- Understand how existing works can contribute and be leveraged to future developments 
- Explore 

### 1.2 General Migration Strategy

Migrating from ROS 1 to ROS 2 requires a structured approach to ensure system stability and functionality. This section outlines a step-by-step strategy to transition smoothly while maintaining compatibility throughout the process.

#### 1.2.1 Migration Phases

The migration process is divided into the following key phases:

1. **Assessment and Planning**  
   - Identify all ROS 1 packages and their dependencies.  
   - Categorize packages based on complexity and migration priority.  
   - Define key functionalities to be tested after migration.

2. **Environment Setup**  
   - Install ROS 2 Jazzy on Ubuntu 24.04.  
   - Set up workspaces for parallel development (e.g., `ros1_ws` and `ros2_ws`).  
   - Install and configure the `ros1_bridge` to allow temporary interoperability.

3. **Package-by-Package Migration**  
   - Start with message definitions (`sdv_msgs`).  
   - Migrate core functionalities and communication layers (e.g., `sdv_nav`, `sdv_serial`).  
   - Adapt higher-level logic (e.g., `sdv_process`, `sdv_joystick`, `sdv_platform`).

4. **Testing and Validation**  
   - Verify each migrated package independently.  
   - Test inter-package communication in ROS 2.  
   - Compare outputs between ROS 1 and ROS 2 using tools like rosbag.

5. **Optimization and Deployment**  
   - Fine-tune DDS QoS settings and other performance parameters.  
   - Refactor remaining legacy code where necessary.  
   - Deploy the fully migrated ROS 2 system.


## **2. Comparative Analysis and Preparation**  

### **2.1 General Considerations about ROS 1 vs ROS 2**  

When migrating from ROS 1 to ROS 2, it is crucial to understand the fundamental differences between the two versions. ROS 2 was designed to overcome some limitations of ROS 1, introducing a more robust and scalable framework. Below, we highlight key aspects to consider:

#### **2.1.1 Architecture and Communication Changes**  

##### **Middleware (DDS - Data Distribution Service)- How does it communicate**  
- ROS 2 uses DDS as the communication backend instead of ROS 1 master node.  
- Nodes communicate peer-to-peer, eliminating the need for a central master.

#### **Node Lifecycle Management**  
- ROS 2 introduces lifecycle-managed nodes, which allow finer control over the initialization and shutdown process.  
- Nodes transition through states: `unconfigured → inactive → active → finalizing`.  
- Lifecycle nodes improve resource management and reduce computational overhead.  

#### **Topic and Message Handling**  
- ROS 2 retains a similar publisher-subscriber model but introduces different types of transports.  
- Services and actions are built on DDS, enabling more flexible and distributed system designs.  
- Parameters are now managed differently, using a structured API instead of the ROS 1 parameter server.  

---

### **2.1.2 Build System and Package Management**  

#### **Colcon vs Catkin**  
- ROS 2 replaces `catkin_make` and `catkin_tools` with `colcon`, a more powerful and flexible build system.  
- `colcon` enables parallel builds and improved dependency management.  

#### **CMake and `package.xml` Changes**  

CMakesLists.txt files contain build instructions for a package use by CMake. Naturally, they're necessary for every single package contained in a workspace. 
General structure for CMake files can be:

```
cmake_minimum_required(VERSION 3.5)
project(my_package)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Add executable
add_executable(my_node src/my_node.cpp)

# Target dependencies
target_link_libraries(my_node ${rclcpp_LIBRARIES})
```
Compared with how CMakesLists.txt files behave, the structure differs, requiring `ament_cmake` instead of `catkin`. It is also necessary to verify thatt cmake version is at least 3.2 or later.

As for package.xml files, they contain metadata about a package, such as its name, version, dependencies, and build dependencies.
- `package.xml` uses different format versions and additional tags for ROS 2 compatibility, being  `ament cmake`replacement for `catkin`. An example for a file of this type can be seen below

```
<?xml version="1.0"?>
<package>
  <name>my_package</name>
  <version>0.0.0</version>
  <description>A package that does something</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>Apache-2.0</license>

  <depend>ament_cmake</depend>
  <depend>rclcpp</depend>
</package>
```

---

### **2.1.3 Launch System and Parameter Management**  

#### **Launch Files with Python**  
- ROS 2 replaces XML-based launch files with Python-based `.launch.py` scripts.  
- Python launch scripts allow for more complex logic and dynamic configuration.  

#### **Dynamic Parameter Handling**  
- In ROS 1, parameters were often set through the parameter server.  
- In ROS 2, each node handles its own parameters, making the system more modular.  

---

### **2.1.4 Compatibility and Migration Challenges**  

#### **Bridging ROS 1 and ROS 2**  
- The `ros1_bridge` package allows communication between ROS 1 and ROS 2 nodes.  
- This is useful for phased migrations where components are moved incrementally.  

#### **Deprecation of Some Libraries and Features**  
- Several ROS 1 libraries are not natively supported in ROS 2 and require adaptation.  
- `rosserial` is not directly available in ROS 2 and must be replaced with alternative solutions.  
- `tf` is replaced with `tf2`, requiring code modifications.  

---

### **2.1.5 Summary of Key Differences**  

| Feature                 | ROS 1 (Melodic)                    | ROS 2 (Jazzy)                    |
|-------------------------|----------------------------------|----------------------------------|
| **Middleware**         | Custom ROS Master                | DDS (Data Distribution Service) |
| **Node Communication** | Master-based                     | Peer-to-peer with DDS           |
| **Build System**       | Catkin (`catkin_make`, `catkin_tools`) | Colcon (`colcon build`) |
| **Launch System**      | XML-based (`.launch`)            | Python-based (`.launch.py`) |
| **Lifecycle Nodes**    | Not available                    | Supported (`rclcpp_lifecycle`) |
| **Parameters**         | Parameter Server                 | Node-based parameter management |
| **Message Passing**    | TCPROS/UDPROS                     | DDS with QoS support |
| **TF Library**         | `tf`                              | `tf2` (mandatory) |
| **Serial Communication** | `rosserial`                     | Requires alternative libraries |

For a successful migration, understanding these differences is crucial. The next sections will focus on applying these concepts to migrate individual packages. 

---

## **3. Development Environment Setup**  

### **3.1 Installing and Configuring ROS 2 Jazzy on Ubuntu 24.04**  

##### Prerequisites

Ensure you have the following installed:
- Ubuntu 22.04 (or compatible version for ROS 2)
- ROS 2 (Jazzy recommended)
- Colcon (build tool)
- VCSTool (for managing repositories)

If ROS 2 is not installed, follow the [official guide](https://docs.ros.org/en/ros2_documentation/index.html) to set it up.

#### Step 1: Create a ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

#### Step 2: Initialize the Workspace

```bash
colcon build --symlink-install
```

#### Step 3: Source the Workspace

After building, source the workspace to make the packages available:

```bash
source install/setup.bash
```
To make it persistent, add it to `~/.bashrc`:

```bash
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### **3.2 Strategies for Testing and Comparing ROS 1 (Ubuntu 18.04) vs ROS 2 (Ubuntu 23.04)**  
_(Running both environments for side-by-side testing.)_  

---


## **4. Package Migration**  

A brief description of packages content and 

1. **`sdv_msgs`** (Message and service definitions)  
   - Ensures that all nodes can communicate correctly in ROS 2.  

2. **`sdv_nav`** (Navigation and peripherals)  
   - Core functionality required for SDV movement and mapping.  

3. **`sdv_serial`** (Hardware communication)  
   - Adapts communication with external controllers (e.g., Tiva board).  

4. **`sdv_process`** (PRIA and logic control)  
   - Ensures execution logic works correctly in ROS 2.  

5. **`sdv_joystick`** (Joystick control)  
   - Adapts control inputs to the new ROS 2 interface.  

6. **`sdv_platform`** (Object detection and sensors)  
   - Converts Flexiforce sensor processing and adapts the detection logic.  

7. **`ros_coms` and `sdvun_sim`** (Communications and simulation)  
   - Final adjustments for communication and testing in a simulated environment.  
