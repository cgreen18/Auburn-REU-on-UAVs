# Auburn-REU-on-UAVs
Repository for the 2019 Auburn REU Team 1 to map an environment using 3D ToF on a quadcopter without GNSS

## Title of Project
> UAV SLAM and Interior Modeling with LiDAR in GNSS-Denied Environments

## Abstract
>   Small, quadrotor helicopter, or quadcopter, unmanned aerial vehicles (**UAV**s) have unique abilities to map environments, particularly utilizing 3D flash light detection and radar (**LiDAR**) technologies, which yield point cloud data sets. However, the majority of applications to date use global navigation satellite systems (**GNSS**) to determine location in order to stitch together LiDAR frames, which excludes mapping in environments without readily available or reliable GNSS (e.g. inside concrete buildings or underground.) In the context of search and rescue, providing an accurate and extensive model through a UAV could provide emergency personnel critical information on the interior of the structure without risking human lives. Previous projects have been able to confirm the viability of autonomous flight through LiDAR and achieve simultaneous localization and mapping (**SLAM**) without GNSS. The project presented will equip a quadcopter UAV with a LiDAR sensor and manually navigate through an interior environment to provide navigation and point cloud data to be processed in post for pose estimates. With pose estimates, a virtual mock-up will be generated from the point cloud data to provide a comprehensive, 3D representation of the interior.


## Problem Statement
>   LiDAR mapping is a powerful tool that can yield a comprehensive, 3D virtual model that can be explored in detail. The majority of work in LiDAR mapping utilizes GNSS and IMU data, along with other minor sensors like magnetometers and pressure sensors, to determine location and attitude; however, there are a myriad of situations where GNSS is unavailable, namely indoors. Considering the potential for indoor navigation that small UAVs offer, limiting mapping to outdoors greatly limits the application of LiDAR mapping. In these environments, GNSS may be denied but WiFi may still be available, which will allow for manual wireless navigation of the drone. The crux of the problem of indoor environments is accurately calculating the location and attitude of the drone without GNSS information. Sensor data from the IMU, magnetometer, and pressure sensors are both inaccurate and imprecise and as such cannot be solely used for location/attitude estimation. This data can be fed through a created EKF; however, that also requires an accurate mathematical model of drone dynamics to assess the accuracy of sensor data and offer alternative estimations of state. 

>   After achieving accurate location and attitude estimation for the entirety of the flight path, the collected point cloud data needs to be stitched together frame by frame. To mesh together numerous frames of 3D data points, with a large amount of redundancy, poses a significant data processing problem. There are libraries that utilize iterative closest point algorithms; however, they still require that all the frames are referenced against an absolute frame of reference. Therefore, each successive frame must be transformed to be referenced from that absolute reference based on the location and attitude estimations. Furthermore, the localization and LiDAR data need to be precisely synchronized in time for accurate frame stitching, which poses issues in implementation because each is run separately.

![Poster](https://github.com/cgreen18/Auburn-REU-on-UAVs/blob/master/LiDAR%20Mapping%20Poster.jpg)

## Team Member Biographies
**Conor Green** (manager of this repository)\
Email: cgreen18@lion.lmu.edu\
[Portfolio/Personal Website](https://greenpage.lmu.build)

> Conor Green is a Computer Engineering major at Loyola Marymount University, enrolled in the Honors College and pursuing a minor in Applied Mathematics. Every semester he has made Dean’s list and recently got invited and accepted into the engineering honor’s fraternity, Tau Beta Pi. Through the Honors department, Conor proposed, managed, and presented a $5000 research project on modifying the torque-slip characteristics of an induction motor through electro-plating that found significant results. Additionally, he was part of a team led by professor Dr. Huang to use computer vision to identify potholes from dash cam pictures in the Los Angeles area. Conor recently presented the induction motor and pothole research projects at This Is Honors and Undergraduate Research Symposium, respectively, at his university. He plans on pursuing graduate school in Electrical Engineering with the intention of earning a PhD.

> Conor has consistently worked various jobs since he turned sixteen, including multiple summers of full time work and private and volunteer tutoring (mainly mathematics.) He has worked at his university’s bike shop, the Cycling Lion, since its inception and is starting his second year as a manager. Outside of academics and work, Conor is the Vice-President of his university’s D2 rugby team and has participated in the UAV national competition SAE Aero West. He speaks Spanish fluently, through classes and learning through close friends, and studied abroad in Madrid, Spain, for a semester his sophomore year. For enjoyment, he reads fiction novels, particularly science fiction, and rides his longboard or skateboard.


**Brenden Stevens**\
Email: brenden728@ucla.edu\
[Portfolio/Personal Website](https://brenden728.wixsite.com/sbrenden)

> Brenden Stevens is majoring in Electrical Engineering at the University of California, Los Angeles. Before transferring to UCLA from his community college in Visalia, CA, Brenden was the president in the leading science club involved with gathering speakers in STEM from around California to inspire young minds to continue pursuing science and engineering. He was also responsible for setting up weekly sports hang-outs where he and 20 others in the STEM department frequently played frisbee, basketball, and soccer. 

> In research, his past work involved testing and monitoring motor endurance and longevity for NASA’s X-57 all electric aircraft as well as signal latency calculations for CAN buses on the same aircraft. Brenden plans on pursuing graduate work in dynamics and controls with hopes of using machine learning and AI to aid his research. 
