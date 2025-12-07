cmd# ROS 2 Interface Contracts: Chapter 3

## Core Node Interface (rclpy.Node)

### Required Methods
- `__init__(self, node_name, **kwargs)` - Initialize the node with a unique name
- `create_publisher(msg_type, topic_name, qos_profile)` - Create a publisher
- `create_subscription(msg_type, topic_name, callback, qos_profile)` - Create a subscription
- `create_service(srv_type, srv_name, callback)` - Create a service server
- `create_client(srv_type, srv_name)` - Create a service client
- `create_timer(period, callback)` - Create a timer
- `declare_parameter(name, value, descriptor)` - Declare a parameter
- `get_parameter(name)` - Get a parameter value

## Message Types Used in Chapter

### Standard Messages
- `std_msgs.msg.String` - Simple string messages
- `std_msgs.msg.Float64` - 64-bit floating point numbers
- `example_interfaces.srv.AddTwoInts` - Example service for adding two integers

### Custom Message Example
```
# Custom RobotCommand message
string command_type
float64 velocity
float64 angle
int32 priority
```

## Service Interface Contract

### Server Implementation
- Callback function signature: `callback(request, response)`
- Must return response object
- Should include error handling

### Client Implementation
- Request creation: `request = ServiceType.Request()`
- Async call: `future = client.call_async(request)`
- Response handling: `response = future.result()`

## Launch File Interface

### Required Elements
- `launch.LaunchDescription` - Main launch description container
- `launch_ros.actions.Node` - Node launch action
- Parameters specification for nodes
- Output configuration (screen, log, etc.)

## Parameter Interface Contract

### Declaration
- Use `self.declare_parameter(name, default_value)` in node initialization
- Access via `self.get_parameter(name).value`

### Types Supported
- Integer, float, string, boolean, lists of these types

## Code Example Standards

### Node Structure
```python
class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Initialize publishers, subscribers, services, etc.

    def destroy_node(self):
        # Cleanup operations before node destruction
        super().destroy_node()
```

### Publisher Pattern
```python
def publish_message(self):
    msg = MessageType()
    # Set message fields
    self.publisher.publish(msg)
```

### Subscriber Pattern
```python
def subscription_callback(self, msg):
    # Process received message
    self.get_logger().info(f'Received: {msg.data}')
```

## Error Handling Contract
- All nodes must implement proper error handling
- Use ROS 2 logging instead of print statements
- Implement graceful shutdown with proper cleanup
- Handle parameter validation appropriately