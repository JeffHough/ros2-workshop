# MAE ROS2 Workshop
This code base includes some examples used for the MAE Robotics ROS2 workshop. There are a couple of different ros2 workspaces included in this repo.

## Basics:
This workspace has two demo packages. Both include a single c++ node and a single python node communicating. 
- The pubsub is an example of a package which uses a single publisher and single subscriber. 
- The cliser is an example of a package which uses a single service and a single client.

Both packages show a similar setup, and use `CMakeLists.txt` and `package.xml` to control the build/install procedure and the dependencies, respectively.

__Note__: It is possible for a single node to contain *many* publishers, subscribers, clients, services, etc. You just need to be careful about the blocking calls!

## External:
The external repo is not actually used here. It is just a repo that contains some simple example cmake.