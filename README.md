# Akame

This repository houses the code used by the 2025/2026 BSU ARS Bender team.

## Contributing

See the file ``CONTRIBUTING.md`` for a detailed explaination of how contributions should be made.

## Dependencies

This software makes extensive use of the ROS2 framework, version [Jazzy Jallico](https://docs.ros.org/en/jazzy/index.html).
Because of this, a machine with a working install of the framework will be required to run or interact with this code.    

Further dependencies will be listed in the individual REAME files of each program.

## Organization

This repository is organized in the following structure:

```
  root \
       - src \
       - doc \
            - index.md
       - aux \
            - scripts \
            - programs \
```

### Src

This directory acts as the ros 'workspace' for this project. It contains a collection of ros2 packages that each contain
nodes, messages, and launch files. All nodes can be found in this directory, and this is the only directory that the 
``colcon build`` command should need to be executed in. This will also be the directory systems are launched from.

### Doc

This directory contains various documentation regarding the project as a whole. This can consist of explanations of
how nodes are supposed to fit together, logs of tuning done on a particular algorithm, or notes on a dependencies
between auxillary files and nodes. It is bascially a dumping ground for any useful information that does not fall
specifically into a README for an individual program. 

#### index.md

The above description may give the impression that the doc directory will be unordered and difficult to navigate. 
While this will probably end up being the case, the file ``index.md`` will be provided in the doc directory to
hopefully mitigate some of these issues. Each entry in ``doc/`` will have a corresponding section in ``index.md``
summarizing the contents of the file. This should speed navigation and resource discovery.

### Aux

It is likely that many other small programs and scripts will be generated to help in using and developing the
system. The aux directory functions as a place for this code. It is organized into two subdirectories, scripts
and programs.

#### Scripts

These are shell scripts that can be used to perform various tasks. They are normally one-offs that focus on
setting up a system or installing various softwares.

#### Programs

These are standalone programs that are not written using ros2 and may not be designed to run in the full
autonomous system. A manual control program used for testing may go here.

### Future Directions

More directories may be added in the future with additional meaning. No content will be removed though.

## Changelog

### 9/23/2025

 - qeftser: wrote initial readme

## Name

This project is the successor to [Kurome](https://github.com/qeftser/kurome), Akame's younger sister.   

It is named afer the character Akame from the series Akame Ga Kill!
![akame_bender jpg](https://github.com/user-attachments/assets/7c103116-3f62-481c-a16d-ae374f47666a)
