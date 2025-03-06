# SDV_UN Platform, now on ROS 2 Jazzy
This is the documentation for most recent development for SDV vehicles in LabFabEx laboratory, corresponding to ROS 2 Jazzy. If you want to access previous works, you may refer to `sdv_melodic` branch. This repository intends to keep track of continuous efforts and provide accessibility to all interested users.

Sincethere are several aspects of SDV_UN development documented in existing documentation, such as firmware, hardware and communications, we will focus on the migration specifics and considerations.

# Migration Guide

## **Table of Contents**

1. **[Introduction](#introduction)**
   - [Migration objectives](#11-migration-objectives)  
   - [General considerations about ROS 1 vs ROS 2](#12-general-considerations-about-ros-1-vs-ros-2)  
   - [General migration strategy](#13-general-migration-strategy)  

2. **[Comparative Analysis and Preparation](#comparative-analysis-and-preparation)**
   - [Key differences between ROS 1 and ROS 2](#21-key-differences-between-ros-1-and-ros-2)  
   - [Useful tools for migration](#22-useful-tools-for-migration)  
   - [Recommended workflow](#23-recommended-workflow)  

3. **[General Setup](#setup)**  
   - [Installing and configuring ROS 2 Jazzy on Ubuntu 24.04](#31-installing-and-configuring-ros-2-jazzy-on-ubuntu-2404)  
   - [Strategies for testing and comparing ROS 1 (Ubuntu 18.04) vs ROS 2 (Ubuntu 24.04)](#32-strategies-for-testing-and-comparing-ros-1-ubuntu-1804-vs-ros-2-ubuntu-2404)  

4. **[Package Migration](#package-migration)**
   - **[4.1 `sdv_msgs`](#sdv_msgs-message-and-service-migration)** (Messages and services)  
     - [Adapting `msg` and `srv` definitions](#adapting-msg-and-srv-definitions)  
     - [Converting `CMakeLists.txt` and `package.xml`](#converting-cmakeliststxt-and-packagexml)  
     - [Using `rosidl` for message generation in ROS 2](#using-rosidl-for-message-generation-in-ros-2)  
   - **[4.2 `sdv_nav`](#sdv_nav-navigation-and-peripherals-migration)** (Navigation and peripherals)  
     - [Migrating the Navigation Stack from ROS 1 to ROS 2](#migrating-the-navigation-stack-from-ros-1-to-ros-2)  
     - [Adjusting launch files](#adjusting-launch-files)  
     - [Dealing with parameter files](#dealing-with-parameter-files)  
   - **[4.3 `sdv_process`](#sdv_process-process-migration)** (Process and control)  
     - [Node migration](#node-migration)  
     - [Adapting scripts](#adapting-scripts)  
     - [Managing dependencies](#managing-dependencies)  

5. **[Testing and Validation](#testing-and-validation)**  
   - [Strategies for unit and integration tests](#strategies-for-unit-and-integration-tests)  
   - [Verifying communication between nodes](#verifying-communication-between-nodes)  
   - [Debugging and analyzing logs in ROS 2](#debugging-and-analyzing-logs-in-ros-2)  

6. **[Best Practices and Recommendations](#best-practices-and-recommendations)**  
   - [Guidelines to maintain code consistency](#guidelines-to-maintain-code-consistency)  
   - [Version control and documentation](#version-control-and-documentation)  
   - [Performance optimization in ROS 2](#performance-optimization-in-ros-2)  

7. **[Conclusion and Next Steps](#conclusion-and-next-steps)**  
   - [Evaluation of the migration process](#evaluation-of-the-migration-process)  
   - [Further adjustments and optimizations](#further-adjustments-and-optimizations) 
# **ROS 1 Melodic to ROS 2 Jazzy Migration Guide**

## **1. Introduction**  

### **1.1 Migration Objectives**  


### **1.2 General Considerations about ROS 1 vs ROS 2**  

When migrating from ROS 1 to ROS 2, it is crucial to understand the fundamental differences between the two versions. ROS 2 was designed to overcome some limitations of ROS 1, introducing a more robust and scalable framework. Below, we highlight key aspects to consider:

#### **1.2.1 Architecture and Communication Changes**  

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

### **1.2.2 Build System and Package Management**  

#### **Colcon vs Catkin**  
- ROS 2 replaces `catkin_make` and `catkin_tools` with `colcon`, a more powerful and flexible build system.  
- `colcon` enables parallel builds and improved dependency management.  

#### **CMake and `package.xml` Changes**  
- The `CMakeLists.txt` structure differs, requiring `ament_cmake` instead of `catkin`.  
- `package.xml` uses different format versions and additional tags for ROS 2 compatibility.  

---

### **1.2.3 Launch System and Parameter Management**  

#### **Launch Files with Python**  
- ROS 2 replaces XML-based launch files with Python-based `.launch.py` scripts.  
- Python launch scripts allow for more complex logic and dynamic configuration.  

#### **Dynamic Parameter Handling**  
- In ROS 1, parameters were often set through the parameter server.  
- In ROS 2, each node handles its own parameters, making the system more modular.  

---

### **1.2.4 Compatibility and Migration Challenges**  

#### **Bridging ROS 1 and ROS 2**  
- The `ros1_bridge` package allows communication between ROS 1 and ROS 2 nodes.  
- This is useful for phased migrations where components are moved incrementally.  

#### **Deprecation of Some Libraries and Features**  
- Several ROS 1 libraries are not natively supported in ROS 2 and require adaptation.  
- `rosserial` is not directly available in ROS 2 and must be replaced with alternative solutions.  
- `tf` is replaced with `tf2`, requiring code modifications.  

---

### **1.2.5 Summary of Key Differences**  

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


### **1.3 General Migration Strategy**  



## **2. Comparative Analysis and Preparation**  

### **2.1 Key Differences between ROS 1 and ROS 2**  
_(Message passing, node lifecycle, launch system, parameters, DDS, etc.)_  

### **2.2 Useful Tools for Migration**  
_(ros1_bridge, ros2 bag, colcon, rviz2, rqt, etc.)_  

### **2.3 Recommended Workflow**  
_(Define a structured process for migrating each package, verifying functionality at every step.)_  

---

---

## **3. Development Environment Setup**  

### **3.1 Installing and Configuring ROS 2 Jazzy on Ubuntu 24.04**  

#### Requirements
 - ROS 2 Jazzy (or later)
 - 

### **3.2 Strategies for Testing and Comparing ROS 1 (Ubuntu 18.04) vs ROS 2 (Ubuntu 23.04)**  
_(Running both environments for side-by-side testing.)_  

---


## **4. Package Migration**  

### **4.1 `sdv_msgs` - Message and Service Migration**  
#### **4.1.1 Adapting `msg` and `srv` Definitions**  
#### **4.1.2 Converting `CMakeLists.txt` and `package.xml`**  
#### **4.1.3 Using `rosidl` for Message Generation in ROS 2**  

### **4.2 `sdv_nav` - Navigation and Peripherals Migration**  
#### **4.2.1 Migrating the Navigation Stack from ROS 1 to ROS 2**  
#### **4.2.2 Configuring `nav2` and Adapting Parameters**  
#### **4.2.3 Converting `launch` Files to `launch.py`**  
#### **4.2.4 Adapting `tf` and `tf2`**  

### **4.3 `sdv_serial` - Hardware Communication Migration**  
#### **4.4.1 Adapting `serial` Communication in ROS 2**  
#### **4.4.2 Migrating Control Nodes**  
#### **4.4.3 Converting `rosserial` or Alternative Solutions in ROS 2**  

### **4.4 `sdv_process` - PRIA and Control Node Migration**  
#### **4.4.1 Adapting Execution Logic to ROS 2**  
#### **4.4.2 Implementing `rclcpp` and `rclpy`**  
#### **4.4.3 Using `Lifecycle Nodes`**  

### **4.5 `sdv_joystick` - Joystick Control Migration**  
#### **4.5.1 Using `joy` in ROS 2**  
#### **4.5.2 Adapting Control Messages**  
#### **4.5.3 Remapping Joystick Button Configuration**  

### **4.6 `sdv_platform` - Object Detection Migration**  
#### **4.6.1 Configuring Nodes for Flexiforce Sensors**  
#### **4.6.2 Adapting Data Processing**  
#### **4.6.3 Converting `dynamic_reconfigure` to ROS 2 Parameters**  


## **5. Testing and Validation**  

### **5.1 Strategies for Unit and Integration Tests**  
_(Testing individual components before integration.)_  

### **5.2 Verifying Communication between Nodes**  
_(Ensuring correct message passing and inter-node communication.)_  

### **5.3 Debugging and Analyzing Logs in ROS 2**  
_(Tools and techniques for troubleshooting.)_  

---

## **6. Best Practices and Recommendations**  

### **6.1 Guidelines to Maintain Code Consistency**  
_(Standardization of coding practices.)_  

### **6.2 Version Control and Documentation**  
_(Git best practices, documentation strategies.)_  

### **6.3 Performance Optimization in ROS 2**  
_(Reducing latency, optimizing resources.)_  

---

## **7. Conclusion and Next Steps**  

### **7.1 Evaluation of the Migration Process**  
_(Assessing success and identifying gaps.)_  

### **7.2 Further Adjustments and Optimizations**  
_(Potential improvements and future developments.)_  
