# ROS 2 Interface Contracts: Chapter 3

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
- `get_logger()` - Get the node's logger
- `destroy_node()` - Clean up node resources

### Expected Behavior
- Nodes must be initialized with a unique name
- Publishers and subscribers must specify valid message types
- Callback functions must be properly defined and accessible
- Resources must be properly cleaned up when node is destroyed

## Publisher Interface Contract

### Required Parameters
- `msg_type`: Valid ROS 2 message type (e.g., std_msgs.msg.String)
- `topic_name`: String name of the topic to publish to
- `qos_profile`: Quality of Service profile (defaults to system default)

### Expected Behavior
- Publisher must successfully publish messages to the specified topic
- Message type must match the specified type
- Publisher should handle backpressure appropriately
- Should work with standard ROS 2 message types

## Subscription Interface Contract

### Required Parameters
- `msg_type`: Valid ROS 2 message type (e.g., std_msgs.msg.String)
- `topic_name`: String name of the topic to subscribe to
- `callback`: Function to call when a message is received
- `qos_profile`: Quality of Service profile (defaults to system default)

### Expected Behavior
- Callback function must be called when messages are received
- Message type must match the specified type
- Subscription must persist for the lifetime of the node
- Should handle message queuing appropriately

## Service Server Interface Contract

### Required Parameters
- `srv_type`: Valid ROS 2 service type (e.g., example_interfaces.srv.AddTwoInts)
- `srv_name`: String name of the service
- `callback`: Function to call when a request is received

### Expected Behavior
- Service must respond to client requests
- Callback must accept request and return response
- Service must be discoverable by clients
- Should handle concurrent requests appropriately

## Service Client Interface Contract

### Required Parameters
- `srv_type`: Valid ROS 2 service type (e.g., example_interfaces.srv.AddTwoInts)
- `srv_name`: String name of the service to call

### Expected Behavior
- Client must be able to send requests to the service
- Client must handle responses from the service
- Should handle service unavailability gracefully
- Must properly format requests according to service definition

## Timer Interface Contract

### Required Parameters
- `period`: Time period between callbacks (in seconds)
- `callback`: Function to call when timer fires

### Expected Behavior
- Callback must be called approximately every period seconds
- Timer should persist for the lifetime of the node
- Should handle timing variations gracefully
- Must be cancelable if needed

## Parameter Interface Contract

### Required Parameters
- `name`: String name of the parameter
- `value`: Initial value of the parameter (optional)
- `descriptor`: Parameter descriptor (optional)

### Expected Behavior
- Parameters must be accessible to the node
- Parameters should be configurable at runtime
- Should support standard parameter types (int, float, string, bool, list)
- Must be accessible via ROS 2 parameter system tools

## Quality of Service (QoS) Profile Contract

### Standard Profiles
- `qos_profile_sensor_data`: For sensor data (volatile, best effort)
- `qos_profile_services_default`: For services (reliable, keep last)
- `qos_profile_parameters`: For parameters (reliable, keep all)

### Expected Behavior
- QoS profiles must match between publishers and subscribers
- Must handle message delivery according to profile specifications
- Should provide appropriate buffering and reliability