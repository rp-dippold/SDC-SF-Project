# Project 3D-Object Detection

## Point-Cloud inspections 
The following ten example point-clouds show vehicles with varying degrees of visibility.

<p float="left">
  <img src="./student/result_imgs/pcl_1.png" width="400" />
  <img src="./student/result_imgs/pcl_2.png" width="400" />
</p>
<p float="left">
  <img src="./student/result_imgs/pcl_3.png" width="400" />
  <img src="./student/result_imgs/pcl_4.png" width="400" />
</p>
<p float="left">
  <img src="./student/result_imgs/pcl_5.png" width="400" />
  <img src="./student/result_imgs/pcl_6.png" width="400" />
</p>
<p float="left">
  <img src="./student/result_imgs/pcl_7.png" width="400" />
  <img src="./student/result_imgs/pcl_8.png" width="400" />
</p>
<p float="left">
  <img src="./student/result_imgs/pcl_9.png" width="400" />
  <img src="./student/result_imgs/pcl_10.png" width="400" />
</p>

## Stable features on most vehicles
As can be seen from the examples displayed above, despite their varying degrees of visibility, some features appear rather stable on most vehicles.

* Wheels: If not cut away, a vehicle's wheels or parts of them can be detected.
* Shape: The shape of a vehicle can be seen in most cases, e.g. whether the vehicle is pick-up, station wagon, passenger car,or if it draws a trailer.
* Windows: In many cases a vehicle's windows can be identified.
* Other parts: From the front, side mirrows can be detected easily, and if available other parts like an antenna can be detected as well. Sometimes a vehicle's lights are visible but less often as the other parts.

The range image below underpins the findings, especially the visibility of a vehicle's lights, windows and shape.

![Range image](./student/result_imgs/range_img.png)
