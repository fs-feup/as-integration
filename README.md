# As Integration Repository

This repository holds the packages responsible for the interface between the AS Main pipeline and the vehicle, such as:

- **ros_can** -> ROS node that communicates via CAN with the car

## Dependencies

- clang-format - ```sudo apt install clang-format```
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [Kvaser CanLib and driver](https://www.kvaser.com/canlib-webhelp/index.html)
- [bear](https://installati.one/install-bear-ubuntu-20-04/)

## Compiling and Running

```sh
colcon build
```

```sh
source ./install/setup.bash
ros2 run ros_can ros_can
```

## Rules

Before starting, check out:
- [Project Rules](https://www.notion.so/FS-FEUP-HUB-6873ab8de3b44fad990d264023fbce8b?pvs=4) in Notion (specifically check Software Development rules)
- [Startup Guide](https://github.com/fs-feup/autonomous-systems/blob/main/docs/tutorials/startup_guide.md)

## Contributing

Repher to the guide [here](https://github.com/fs-feup/autonomous-systems/blob/main/CONTRIBUTING.md).

More information in the [main repo](https://github.com/fs-feup/autonomous-systems).