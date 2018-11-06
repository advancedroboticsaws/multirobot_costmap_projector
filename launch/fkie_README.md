# multimaster_fkie_examples
This repository means to provide examples and documentations about the package "multimaster_fkie" for extending the current ROS-I communication infrastructure to a multi-robot (distributed) one.

# About multimaster_fkie
This package contains two nodes: multimaster_fkie and master_sync_fkie. These nodes enable the originally separated (two or more) ROS-I environments to communicate to each others without any central system inbetween. This is possible because the communication infrastructure of ROS-I is basically a semi-distributed one. The last things to upgrade the ROS-I to a distributed one is to synchronize the tables in ROS-masters in each environment.

- multimaster_fkie - for discovering any new master in the LAN and any change of discovered masters
- master_sync_fkie - for obtaining the changes of remote masters and setting remote-changes to local master 

Check the links below for further infomation about multimaster_fkie

- [multimaster_fkie](http://wiki.ros.org/multimaster_fkie "multimaster_fkie")
- [master_discovery_fkie](http://wiki.ros.org/master_discovery_fkie "master_discovery_fkie")
- [master_sync_fkie](http://wiki.ros.org/master_sync_fkie "master_sync_fkie")
- [Setup a ROS master synchronization (tutorial)](http://wiki.ros.org/multimaster_fkie/Tutorials/Setup%20a%20ROS%20master%20synchronization "Setup a ROS master synchronization")

# Installations
```
  $ sudo apt-get install ros-kinetic-multimaster-fkie
```

# Usage
Simply launch the multimaster_fkie.launch in example_launchers/

```
$ cd ./example_launchers/
$ roslaunch multimaster_fkie.launch
```

# Filter-rules of master_sync_fkie
The setting of filter is an essential part for constructing the system you need. By setting the filter, one can decide which hosts/nodes/topics(publishers/subscribers)/services to be synchronized or not to be synchronized (ignored). 

These setting should be specified in ./example_launchers/__master_sync.launch__.

The mechanism of the filter is described as followed

1. Check for the ignore-list. If there is any, remove that item from the total item list.
2. Check for the sync-list. If there is any, keep that item if it is shown up in the list we got after first step and drop all the rest

As the result, all the item in ignore-list will not be sync., even if there is the same item in the sync-list.
If you want to remove all the other item beside the item you want, simply add that item to sync-list and leave the ignore-list blank.



## Rules
- When synchronized a node, the __node-name__ is also synchrnized, which means that there can only be exactly one node with the name among the synchronized multimaster system.
- When a node is synchronized, all its topics and services are synchronized as well. If you want to synchronized a node but some topics/services it connects to, specify them on ignore-lists (ignore_topics/ignore_services).

## Notes
- If you want to construct a multi-robot system which communicate to each other through a gateway node with a topic for cross-robot communication, simply add that topic to __sync_topics__ and not to sync the whole node.
- It's highly recommended not to synchronize __/tf__ and __/tf_static__ accross different ROS-enviroment.
