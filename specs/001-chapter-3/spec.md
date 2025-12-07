# ============================================================================
# TASK: Create Chapter 3 Content Specification
# Module 1, Chapter 3: Building Your First ROS 2 Nodes
# ============================================================================

## Context
- **Book:** Physical AI & Humanoid Robotics
- **Module 1:** The Robotic Nervous System (ROS 2)
- **Chapter:** 3 of 4 in Module 1
- **Week:** Week 4 (of Weeks 1-5)
- **Previous Chapters:**
  - ✅ Chapter 1: Intro to Physical AI (completed)
  - ✅ Chapter 2: ROS 2 Architecture (completed)
- **Next Chapter:** Chapter 4: URDF Robot Descriptions

## Hackathon Requirements for This Chapter
From **Weeks 3-5: ROS 2 Fundamentals**:
- Building ROS 2 packages with Python
- Nodes, topics, services, and actions (practical implementation)
- Launch files and parameter management
- Bridging Python Agents to ROS controllers using rclpy

---

# Chapter 3: Building Your First ROS 2 Nodes - Specification

## Page Details
- **Output:** `book-write/docs/module-1/chapter-03-first-nodes.md`
- **Spec:** `specs/chapter-3/spec.md`
- **Week:** 4 (within Weeks 1-5 of Module 1)
- **Word Count:** 4000 words
- **Difficulty:** Intermediate

## Frontmatter (TypeScript Docusaurus)
```yaml
---
id: chapter-03-first-nodes
title: "Chapter 3: Building Your First ROS 2 Nodes"
sidebar_position: 3
sidebar_label: "Chapter 3: First Nodes"
description: "Learn to build ROS 2 nodes with Python (rclpy) - publishers, subscribers, services, and parameters"
keywords: [ROS 2, rclpy, publisher, subscriber, Python, nodes, launch files]
---
```

## Learning Objectives
By the end of this chapter, students will be able to:
1. Create ROS 2 publisher and subscriber nodes using Python (rclpy)
2. Build custom ROS 2 packages with proper structure
3. Define and use custom message types
4. Write launch files to run multiple nodes
5. Manage parameters for node configuration
6. Bridge Python AI agents to ROS controllers

## Prerequisites
- Chapter 1 & 2 completed (Physical AI concepts + ROS 2 architecture)
- Python programming (classes, functions, async)
- ROS 2 Humble installed on Ubuntu 22.04
- Basic terminal/command-line skills
- Understanding of pub-sub pattern from Chapter 2

---

## Content Structure

### Section 1: Introduction (300 words)

**Hook:**
"You've learned the theory—now it's time to build! In this chapter, you'll write your first ROS 2 nodes in Python and see them communicate in real-time."

**What You'll Build:**
- Simple publisher-subscriber system
- Service-based request-response nodes
- Multi-node system with launch files
- Parameterized nodes for flexible control

**Why This Matters:**
- Foundation for all robot control
- Industry-standard approach
- Skills transfer to real robots
- Enables AI-robot integration

**Chapter Roadmap:**
Brief overview of the 5 main sections

---

### Section 2: Your First Publisher Node (800 words)

**2.1 ROS 2 Package Structure (200 words)**
```
my_robot_pkg/
├── package.xml
├── setup.py
├── my_robot_pkg/
│   ├── __init__.py
│   └── publisher_node.py
└── README.md
```

Explain:
- Package metadata (package.xml)
- Python setup (setup.py)
- Entry points for executables

**2.2 Creating a Workspace (150 words)**
```bash
# Commands to create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_pkg
```

**2.3 Writing the Publisher (450 words)**

**Code Example 1: Simple Publisher**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0
        self.get_logger().info('Publisher node started')

    def timer_callback(self):
        msg = String()
        msg.data = f'Robot status update #{self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation:**
- Line-by-line walkthrough
- Timer callback mechanism
- Message publishing
- Logging best practices
- Proper shutdown handling

:::tip For Beginners
A **timer** is like an alarm that goes off every second, telling your code to do something.
:::

---

### Section 3: Your First Subscriber Node (700 words)

**3.1 Understanding Subscribers (150 words)**
- How callbacks work
- Message reception
- Processing incoming data

**Code Example 2: Simple Subscriber**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'robot_status',
            self.listener_callback,
            10
        )
        self.get_logger().info('Subscriber node started')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**3.2 Running Publisher and Subscriber (200 words)**
```bash
# Terminal 1
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 run my_robot_pkg simple_publisher

# Terminal 2
source install/setup.bash
ros2 run my_robot_pkg simple_subscriber
```

**Expected Output:**
Show what students should see in both terminals

**3.3 Debugging Tips (150 words)**
- Common errors and fixes
- Using `ros2 topic echo`
- Checking node status with `ros2 node list`
- Viewing topic info

:::note Troubleshooting
If nodes don't communicate, check:
1. Same ROS_DOMAIN_ID
2. Topic names match exactly
3. Message types compatible
:::

---

### Section 4: Custom Messages (600 words)

**4.1 Why Custom Messages? (150 words)**
- Standard messages (std_msgs, geometry_msgs, sensor_msgs)
- When to create custom messages
- Benefits for robot control

**4.2 Creating Custom Message Type (200 words)**

**Directory Structure:**
```
my_robot_pkg/
├── msg/
│   └── RobotCommand.msg
├── package.xml (modified)
└── CMakeLists.txt (if needed)
```

**RobotCommand.msg:**
```
# Custom message for robot commands
string command_type
float64 velocity
float64 angle
int32 priority
```

**Update package.xml:**
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

**4.3 Using Custom Messages (250 words)**

**Code Example 3: Publisher with Custom Message**
```python
from my_robot_pkg.msg import RobotCommand

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(
            RobotCommand,
            'robot_commands',
            10
        )
        self.timer = self.create_timer(2.0, self.publish_command)

    def publish_command(self):
        msg = RobotCommand()
        msg.command_type = 'move_forward'
        msg.velocity = 0.5
        msg.angle = 0.0
        msg.priority = 1
        self.publisher_.publish(msg)
        self.get_logger().info(f'Command: {msg.command_type}')
```

---

### Section 5: Services - Request-Response Pattern (600 words)

**5.1 Services vs Topics (150 words)**
- When to use services vs topics
- Synchronous vs asynchronous
- Use cases in robotics

**5.2 Creating a Service (450 words)**

**Code Example 4: Service Server**
```python
from example_interfaces.srv import AddTwoInts

class MathService(Node):
    def __init__(self):
        super().__init__('math_service')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback
        )
        self.get_logger().info('Service ready')

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Request: {request.a} + {request.b} = {response.sum}'
        )
        return response
```

**Code Example 5: Service Client**
```python
from example_interfaces.srv import AddTwoInts

class MathClient(Node):
    def __init__(self):
        super().__init__('math_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.client.call_async(request)
        return future
```

---

### Section 6: Launch Files (600 words)

**6.1 Why Launch Files? (150 words)**
- Managing multiple nodes
- Configuration management
- Automation and reproducibility

**6.2 Python Launch Files (450 words)**

**Code Example 6: Basic Launch File**
```python
# launch/robot_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_pkg',
            executable='simple_publisher',
            name='status_publisher',
            output='screen'
        ),
        Node(
            package='my_robot_pkg',
            executable='simple_subscriber',
            name='status_listener',
            output='screen'
        ),
    ])
```

**Running Launch File:**
```bash
ros2 launch my_robot_pkg robot_system.launch.py
```

**Advanced Launch Features:**
- Parameters
- Namespaces
- Conditional launching
- Including other launch files

---

### Section 7: Parameters (500 words)

**7.1 Parameter Management (200 words)**
- What are parameters
- When to use them
- Parameter types

**Code Example 7: Node with Parameters**
```python
class ConfigurablePublisher(Node):
    def __init__(self):
        super().__init__('configurable_publisher')

        # Declare parameters
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('topic_name', 'robot_status')

        # Get parameter values
        rate = self.get_parameter('publish_rate').value
        topic = self.get_parameter('topic_name').value

        self.publisher_ = self.create_publisher(String, topic, 10)
        self.timer = self.create_timer(1.0/rate, self.timer_callback)
```

**7.2 Setting Parameters (300 words)**

**From Command Line:**
```bash
ros2 run my_robot_pkg configurable_publisher \
  --ros-args -p publish_rate:=2.0
```

**From Launch File:**
```python
Node(
    package='my_robot_pkg',
    executable='configurable_publisher',
    parameters=[
        {'publish_rate': 2.0},
        {'topic_name': 'custom_status'}
    ]
)
```

**Parameter File (YAML):**
```yaml
/**:
  ros__parameters:
    publish_rate: 2.0
    topic_name: 'custom_status'
```

---

### Section 8: Bridging AI Agents to ROS (400 words)

**8.1 Integration Concept (150 words)**
- AI decision-making layer
- ROS execution layer
- Connecting Python AI models to ROS

**8.2 Example: Simple AI-ROS Bridge (250 words)**

**Code Example 8: AI Agent Node**
```python
import numpy as np
from std_msgs.msg import Float64

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent')

        # Subscribe to sensor data
        self.sub = self.create_subscription(
            Float64, 'sensor_reading',
            self.sensor_callback, 10
        )

        # Publish control commands
        self.pub = self.create_publisher(
            Float64, 'control_command', 10
        )

        self.model_state = 0.0

    def sensor_callback(self, msg):
        # Simple AI decision (replace with real AI model)
        sensor_value = msg.data

        # AI logic here (could be neural network, RL agent, etc.)
        control = self.simple_controller(sensor_value)

        # Send command to robot
        cmd_msg = Float64()
        cmd_msg.data = control
        self.pub.publish(cmd_msg)

    def simple_controller(self, sensor_value):
        # Placeholder for AI model
        return np.clip(sensor_value * 0.5, -1.0, 1.0)
```

:::tip Advanced
In real applications, this is where you'd integrate TensorFlow, PyTorch, or other AI frameworks!
:::

---

### Section 9: Hands-On Lab Exercise (300 words)

**Lab: Build a Robot Control System**

**Objective:**
Create a multi-node system where one node acts as a "brain" (decision-maker) and another acts as an "executor" (controller).

**Requirements:**
1. Create a package: `robot_control_lab`
2. Implement 3 nodes:
   - **Sensor Node:** Publishes simulated sensor data
   - **Brain Node:** Processes data, makes decisions
   - **Motor Node:** Receives commands, simulates actuation
3. Use custom message for commands
4. Write launch file to start all nodes
5. Add parameters for sensor frequency and motor limits

**Starter Code Provided:**
Link to GitHub repository with templates

**Expected Output:**
```
[sensor_node]: Publishing sensor data: 0.75
[brain_node]: Decision: move_forward, speed: 0.5
[motor_node]: Executing: move_forward at 0.5 m/s
```

**Verification Checklist:**
- [ ] All 3 nodes run without errors
- [ ] Messages flow between nodes
- [ ] Parameters can be changed
- [ ] Launch file works correctly

---

### Section 10: Chapter Summary (200 words)

**What You've Learned:**
- Creating publisher and subscriber nodes
- Building ROS 2 packages
- Defining custom messages
- Implementing services
- Writing launch files
- Managing parameters
- Bridging AI agents to ROS

**Key Takeaways:**
- rclpy is your gateway to ROS 2 in Python
- Nodes are the building blocks of robot systems
- Topics enable asynchronous communication
- Services provide request-response patterns
- Launch files orchestrate complex systems

**Next Steps:**
In Chapter 4, you'll learn to describe robots using URDF (Unified Robot Description Format), defining the physical structure, joints, and sensors of humanoid robots.

**Additional Resources:**
- ROS 2 Humble Documentation: [link]
- rclpy API Reference: [link]
- Example code repository: [link]

---

## Diagrams Required (Minimum 3)

### Diagram 1: Publisher-Subscriber Flow
```bash
claude code --skill diagram-generator \
  --diagramType "ros2-architecture" \
  --topic "Publisher node sending messages to Subscriber node via topic"
```

Expected: Node diagram showing publisher → topic → subscriber with message flow

### Diagram 2: Service Request-Response
```bash
claude code --skill diagram-generator \
  --diagramType "ros2-architecture" \
  --topic "Service client-server interaction with request and response"
```

Expected: Sequence diagram showing client request → service → response

### Diagram 3: Multi-Node System with Launch File
```bash
claude code --skill diagram-generator \
  --diagramType "system-overview" \
  --topic "Multiple ROS 2 nodes launched together communicating via topics"
```

Expected: System architecture showing 3-4 nodes interconnected

### Diagram 4 (Optional): AI-ROS Integration
```bash
claude code --skill diagram-generator \
  --diagramType "workflow" \
  --topic "AI agent receiving sensor data and sending control commands via ROS"
```

---

## Code Examples Summary

Total: **8 complete, runnable code examples**

1. Simple Publisher Node
2. Simple Subscriber Node
3. Publisher with Custom Message
4. Service Server
5. Service Client
6. Launch File
7. Node with Parameters
8. AI-ROS Bridge Node

All code must:
- ✅ Be fully functional (copy-paste ready)
- ✅ Include error handling
- ✅ Have clear comments
- ✅ Follow Python PEP 8 style
- ✅ Use type hints where appropriate
- ✅ Include logging statements

---

## Interactive Components (Docusaurus)

### Code Tabs for Different Approaches
```tsx
<Tabs>
  <TabItem value="basic" label="Basic Publisher">
    [Code Example 1]
  </TabItem>
  <TabItem value="advanced" label="With Parameters">
    [Code Example 7]
  </TabItem>
</Tabs>
```

### Collapsible Troubleshooting
```tsx
<details>
  <summary>❌ Error: "No executable found"</summary>

  **Solution:**
  Make sure you've added the entry point in setup.py:
  ```python
  entry_points={
      'console_scripts': [
          'simple_publisher = my_robot_pkg.publisher_node:main',
      ],
  },
  ```
</details>
```

---

## Personalization Notes

### For Beginners:
- :::info boxes explaining ROS concepts
- More detailed code comments
- Step-by-step terminal commands
- Common error explanations
- Simpler AI example (if-else logic)

### For Advanced Users:
- :::tip boxes with optimizations
- References to ROS 2 design patterns
- Performance considerations
- Link to advanced topics (executors, lifecycle nodes)
- More complex AI integration example

---

## Translation (Urdu) Notes

**Translate:**
- All section headings
- Explanatory text
- Code comments (optional, in separate Urdu version)
- Diagram captions

**Keep English:**
- Code itself
- Terminal commands
- File paths
- Function/class names
- Technical terms (in parentheses with Urdu explanation)

Example: "Publisher node (پبلشر نوڈ، جو پیغامات بھیجتا ہے) sends messages..."

---

## Quality Checklist

Before marking chapter complete:
- [ ] 4000 words achieved (±200 acceptable)
- [ ] 8 complete code examples included
- [ ] All code tested in ROS 2 Humble
- [ ] 3-4 diagrams generated and inserted
- [ ] Lab exercise clear and doable
- [ ] Links to Chapter 2 (previous) and Chapter 4 (next)
- [ ] Frontmatter correct
- [ ] All commands verified
- [ ] Troubleshooting section comprehensive
- [ ] Matches hackathon requirements

---

## Skills & Subagents Usage

### Generate chapter content:
```bash
claude code --agent tech_writer \
  --spec specs/chapter-3/spec.md \
  --output book-write/docs/module-1/chapter-03-first-nodes.md \
  --context "Hands-on chapter, complete working examples, clear explanations for building first ROS 2 nodes"
```

### Generate diagrams:
```bash
# Run 3-4 times for each diagram
claude code --skill diagram-generator \
  --diagramType "ros2-architecture" \
  --topic "[specific topic from diagram list above]"
```

---

## Word Count Breakdown
- Introduction: 300
- Publisher Node: 800
- Subscriber Node: 700
- Custom Messages: 600
- Services: 600
- Launch Files: 600
- Parameters: 500
- AI-ROS Bridge: 400
- Lab Exercise: 300
- Summary: 200

**Total: 5000 words** (slightly over to include all necessary content)

---

## Success Criteria

This spec is complete when:
1. ✅ All sections defined with clear structure
2. ✅ 8 code examples fully specified
3. ✅ Lab exercise detailed and actionable
4. ✅ 3-4 diagrams specified
5. ✅ Prerequisites match Chapter 2 content
6. ✅ Leads naturally to Chapter 4 (URDF)
7. ✅ Matches hackathon Week 4 requirements
8. ✅ Practical, hands-on focus
9. ✅ Ready for content generation

---

**Save this specification as:** `specs/chapter-3/spec.md`

This is the SPECIFICATION document. After approval, use tech_writer subagent to generate actual content.