# LeGO-LOAM

Original repository: https://github.com/RobustFieldAutonomyLab/LeGO-LOAM


## Build
```bash
cd lego_loam/docker
./build.sh
```

## Run

### On host:
```bash
roscore
```

```bash
rosparam set use_sim_time true
rviz -d lego_loam/LeGO-LOAM/launch/rviz/test.rviz
```

### On docker image:
```bash
cd lego_loam/docker
./run.sh -v 2018-05-18-15-28-12_39.bag:/tmp/2018-05-18-15-28-12_39.bag

roslaunch lego_loam_bor run.launch rosbag:=/tmp/2018-05-18-15-28-12_39.bag
```

2018-05-18-15-28-12_39.bag: https://github.com/TixiaoShan/Stevens-VLP16-Dataset

![lego_loam](https://user-images.githubusercontent.com/31344317/98347840-524fb400-205b-11eb-9416-8050771391c2.gif)
