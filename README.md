
 #### YOLOv5 based Vehicle Detection and Formation Tracking
This project mainly depends on [yolov5](https://docs.ultralytics.com/yolov5), ROS Melodic and Ubuntu 18.04.
The environment needs that python>=3.8, where torch and torchvision can be downloaded from [yolo-jetpack](https://docs.ultralytics.com/yolov5/tutorials/running_on_jetson_nano/#before-you-start)
##### Learning
Train a pt model that can recognize cars. [Training method](https://docs.ultralytics.com/yolov5/tutorials/train_custom_data/#1-create-dataset).
##### Porting
1. Write the depth camera node code;
2. Configure cv_bridge based on [Information 1](https://blog.csdn.net/qq_43124746/article/details/124347630) and [Information 2](https://blog.csdn.net/qq_33980935/article/details/123132452)
3. Write the formation tracking method.
##### Experiment

