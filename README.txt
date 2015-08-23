说明：   2015-06-28     by Kint Zhao     huasheng_zyh@163.com

  1.工程依赖的库： artoolkitplus（2d mark的运动跟踪） + opencv(运算库与图像显示)
                 ros(机器人操作系统indigo hydro)
  2. 硬件平台：turtlebot + usbcamera

  3.　软件架构说明：
    在ＲＯＳ分布式架构下，usb摄像头采集图像，并发送相应的topic;
    turtlebot 在键盘控制下运行，读取相应的速度信息；
    最后在同时接受并综合到两信息的情况下，进行EKFSLAM进行数据融合。
  4. 文件说明：
    /src  存放的是artoolkitplus的相应源文件  /include 是包含的相应源码头文件
    /image_convert 是用来将opencv处理的图像转换到ros的rviz可以显示识别的形式
    /qrslam   主程序文件与相应算法操作文件
           /class  定义的用来提取image中 2d mark的操作
           qrslam  定义的算法实现文件
           main.cpp   主程序

    /camera_calibration 用于opencv图像校正的文件
    /data_sources  配置文件夹 结果输出
    /build   编译文件
    /turtorial_slam2d  g2o仿真部分

  5 功能实现：
     初始化定位
     ekfslam算法框架

  6 配置参数说明：
    高度=292cm-29cm=263-->2.62m
    相机校正有不同的结果出现
     1) 4×5     26.5mm
        5.0417785095242891e+02 0.0000000000000000e+00 3.2095560744042672e+02
        0.0000000000000000e+00 5.0211839253022396e+02 2.4002402013102110e+02
        0.0000000000000000e+00 0.0000000000000000e+00 1.0000000000000000e+00
        dist=[-8.541216e-02 3.559430e-02 -1.212648e-04 2.928131e-03 0.000000e+00]

        Average err. of reprojection: 0.380944 pixels (OpenCV error=0.224036)
     2) 6*8     24.0cm

        4.8803670216488337e+02 0.0000000000000000e+00 3.3343614367151707e+02
        0.0000000000000000e+00 4.8416195284531801e+02 2.3790648860285884e+02
        0.0000000000000000e+00 0.0000000000000000e+00 1.0000000000000000e+00

        -9.2742978271077053e-02 1.0872208126910321e-01 8.1927553576490911e-05 6.6169920915418482e-03
 -------------------------------------------------------------------------------------------
        4.5367089850887925e+02 0.0000000000000000e+00 3.3599586672205800e+02
        0.0000000000000000e+00 4.5226946762347319e+02 2.1850187295097194e+02
        0.0000000000000000e+00 0.0000000000000000e+00 1.0000000000000000e+00

        -1.3140859932396523e-01 -2.5872208323177265e-01 -8.7802416308985394e-03 5.4069129267498425e-03

---------------------------------------------------------------------------------
  7 数据频率说明：
    rostopic hz /odom
              average rate: 50.807
                      min: 0.003s max: 0.025s std dev: 0.00223s window: 166
              average rate: 50.471
                      min: 0.000s max: 0.040s std dev: 0.00498s window: 216

    rostopic hz /usb_cam/image_raw
            average rate: 19.492
                    min: 0.010s max: 0.071s std dev: 0.00706s window: 462
            average rate: 19.383
                    min: 0.010s max: 0.071s std dev: 0.00719s window: 479

    --->>>>  频率不一致，限制算法。。odom需要与image的速率相一致匹配。。
    运动速度不能太大，偏差会增大。
    --->>>>  速率协调一致性问题： 拆分slam
            1) 同步时，一致 ekf-slam
            2) 非同步时，里程积累


```````````````````````````````````````````````````````````````````
之前rosbag record 的数据都是在这样的基础下进行的，应该需要注意： ros 下相机校正。 采集数据时需要注意

<去畸变 param="D">[0.025751483065329935, -0.10530741936574876,-0.0024821434601277623, -0.0031632353637182972, 0.0000]</rosparam>
<内参阵 param="K">[558.70655574536931, 0.0, 316.68428342491319, 0.0, 553.44501004322387, 238.23867473419315, 0.0, 0.0, 1.0]</rosparam>
<rosparam param="R">[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]</rosparam>
<rosparam param="P">[558.70655574536931, 0.0, 316.68428342491319, 0.0, 0.0, 553.44501004322387, 238.23867473419315, 0.0, 0.0, 0.0, 1.0, 0.0]</rosparam>

``````````````````````````````````````````````````````````````
  8. 协方差说明
   协方差值越大表示数据可信度越低，也就是该数据的测量代入误差大。统计特性优化
   1） robot运动输入偏差   Mt
   2)  运动模型协方差初始值
   3） 观测模型方差



   9. 显示相关的说明
    qr_slam:
       raw_img_cvt_   =  new ImageConverter("/slam/raw_flip_image");   cv_camera_
                1.  图像关于原点翻转； 2 显示19的ｉｄ，观测值　d 　与  theta ;3 landmark的数量
       robot_img_cvt_ =  new ImageConverter("/slam/robot_image");
                TODO:  加入t时刻的robot: x,y,theta
       slam_img_cvt_  =  new ImageConverter("/slam/qrslam/slam_map");   raw_global_map_
                1. 加载地图图片 ；2 画地图基准轴线 ；  3. 画odom下机器人位置，ekfslam下系统状态量，坐标系x y o；
    detectqrcode：
        qr_landmark_cvt_ = new ImageConverter("/detect_qr/qr_img");      show_landmark_img_
           1. 中点，轴线； 2. 2Dmark 轮廓 ； 3. 角点序号
    usb_cam:
        /usb_cam/image_raw

    10. 偏差累积的问题

    11. 利用边信息求高度，求焦距  ---》》高度测量可信度 与相机校正可信度
    高度差值对应10cm    换算 x: 1cm   y: 5-6cm
       焦距问题   相机校准的准确性问题

    12. 最后是map 应该是在均值化的表示下。

    13. 加入robot  显示以后，发现角度计算存在严重的积分误差累积

    14  抗扰动判断：利用结算f 与测量f 对比。。-->评判标准

    15  讨论时间戳,数据的一致性
        1)  GMAPPING if(!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_)
        2)  地图是否更新的判断
        if(!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_)
        {
          updateMap(*scan);                                                              //zyh    地图更新
          last_map_update = scan->header.stamp;
          ROS_DEBUG("Updated the map");
        }

    16  odom校准

    17  固定坐标mark纠偏  ||  多mark 线性纠偏
     坐标系整体纠偏调整.

    18  利用角点方式与mark标示的方式  作为landmark:
       1)  提取角点 + 角点描述

    19   坡度自适应的情况 :自身量测高度与观测求高度 ++> 实际需要的正确自适应

    20  储存数据队列匹配,时间最近匹配
        时间的一致性,必须保持,预测与更新必须使用时间戳的时间.

    21  ekfslam update 新特征加入时先建立向量表,在向表中均值化后再进行地图更新

    22 图像畸变 边界效应.

This directory contains material supporting chapter 9 of the cookbook:  
Computer Vision Programming using the OpenCV Library. 
by Robert Laganiere, Packt Publishing, 2011.

Files:
	CameraCalibrator.h
	CameraCalibrator.cpp
	calibrate.cpp
correspond to Recipes:
Calibrating a camera
File:
	estimateF.cpp
correspond to Recipe:
Computing the Fundamental Matrix of an Image Pair
Files:
	matcher.h
	robustmatching.cpp
correspond to Recipe:
Matching Images using Random Sample Consensus
File:
	estimateH.cpp
correspond to Recipe:
Computing a homography between two images








robot x,y,theta 0.840127 0 0.000174533  j 0 Qid 100 miu(01) 0.431578 0 -0.000854314 2*j+3  -90.7004 2*j+4 29.0845
robot x,y,theta 0.840127 0 0.000174533  j 1 Qid 101 miu(01) 0.431578 0 -0.000854314 2*j+3  -87.88 2*j+4 -7.74208
robot x,y,theta 0.840127 0 0.000174533  j 2 Qid 102 miu(01) 0.431578 0 -0.000854314 2*j+3  -51.3378 2*j+4 -5.40446
robot x,y,theta 0.840127 0 0.000174533  j 3 Qid 103 miu(01) 0.431578 0 -0.000854314 2*j+3  -53.2649 2*j+4 30.6316
robot x,y,theta 0.840127 0 0.000174533  j 4 Qid 104 miu(01) 0.431578 0 -0.000854314 2*j+3  -70.9038 2*j+4 11.5221

robot x,y,theta 0.840127 0 0.000174533  j 5 Qid 85 miu(01) 0.431578 0 -0.000854314 2*j+3  57.1539 2*j+4 36.6168
robot x,y,theta 0.840127 0 0.000174533  j 6 Qid 86 miu(01) 0.431578 0 -0.000854314 2*j+3  20.267 2*j+4 34.4455
robot x,y,theta 0.840127 0 0.000174533  j 7 Qid 87 miu(01) 0.431578 0 -0.000854314 2*j+3  21.7675 2*j+4 -1.86233
robot x,y,theta 0.840127 0 0.000174533  j 8 Qid 88 miu(01) 0.431578 0 -0.000854314 2*j+3  58.8105 2*j+4 0.100341
robot x,y,theta 0.840127 0 0.000174533  j 9 Qid 89 miu(01) 0.431578 0 -0.000854314 2*j+3  39.3792 2*j+4 17.5699

robot x,y,theta 131.825 1.49389 0.0218166  j 10 Qid 95 miu(01) 130.557 12.6203 0.104795 2*j+3  248.279 2*j+4 67.3026
robot x,y,theta 131.825 1.49389 0.0218166  j 11 Qid 96 miu(01) 130.557 12.6203 0.104795 2*j+3  212.472 2*j+4 63.0865
robot x,y,theta 131.825 1.49389 0.0218166  j 12 Qid 97 miu(01) 130.557 12.6203 0.104795 2*j+3  215.693 2*j+4 26.2064
robot x,y,theta 131.825 1.49389 0.0218166  j 13 Qid 98 miu(01) 130.557 12.6203 0.104795 2*j+3  252.045 2*j+4 31.4979
robot x,y,theta 131.825 1.49389 0.0218166  j 14 Qid 99 miu(01) 130.557 12.6203 0.104795 2*j+3  232.06 2*j+4 46.9499







