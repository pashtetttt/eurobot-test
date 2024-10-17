# To run follow this commands 
```
docker build -t turtlesim_path_planner .
docker run -it turtlesim_path_planner
```

or
```
docker run -it turtlesim_path_planner bash
ros2 launch turtlesim_path_planner path_planner_launch.py trajectory:=circle
```

