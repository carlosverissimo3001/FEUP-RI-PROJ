# ROS2 Commands

## Some clarifications ##

### Node vs. Topic ###

- A topic is a way to communicate between nodes in the ros2 application.
- Nodes don't directly communicate with each other, they publish or subscribe to topics. Multiple nodes can publish or subscribe to the same topic.
- Nodes "know" the data type of the topics they publish or subscribe to.
- If a node receives messages from a topic, it doesn't know which node published the message.

### Node vs. Package ###

- A package is a collection of nodes.

### When to use a topic vs. a service ###

- If you want to send data to a node, use a topic.
  - Sensor data, velocity commands, etc.
- When you have an interaction where you might compute something or change a setting and you expect a response, use a service.

## Create a Package ##

```bash
ros2 pkg create --build-type <build-type> <package_name> --dependencies <dependencies>
```

Possible build types:

- ament_cmake
- ament_python

## Build the Package ##

```bash
colcon build --symlink-install
```
`symlink-install` allows you to edit the source code and rebuild without having to reinstall the package.

## Run a Node ##

```bash
ros2 run <package_name> <executable_name>
```

## List running topics ##

```bash
ros2 topic list
```

## Get details on a topic ##

```bash
ros2 topic info <topic_name>
```

Useful to get the type of the topic, publisher and subscriber count.

## Get details on a data type (interface) ##

```bash
ros2 interface show <data_type>
```

## Echo a topic ##

```bash
ros2 topic echo <topic_name>
```

## List running services ##

```bash
ros2 service list
```

## Get details on a service ##

```bash
ros2 service type <service_name>
```

This command returns the service type (interface).

## Get details on a service type (interface) ##

```bash
ros2 interface show <service_type>
```

## Call a service ##

```bash
ros2 service call <service_name> <service_type> <arguments>
```

Note: Place arguments inside quotes.

## Useful GUI tool ##

```bash
rqt_graph
```