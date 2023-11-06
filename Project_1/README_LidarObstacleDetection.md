# Udacity Sensor Fusion
## Camera

---

### 1. Levels of Autonomous Driving
- Level0 : No automation
  /t Zero autonomy
  /t Drivers performs all driving tasks
- Level21: Driver Assistance
  /t Controlled by the driver
  /t Some driving asist features may be included
- Level2 : Partial Automation
  /t With automated functions (ex. acceleration, steering, etc.)
  /t Drivers must be engaged in driving tasks
- Level3 : Conditional Automation
  /t Driver is a necessity, but not required
  /t Driver must be ready to take control the vehicle
- Level4 : High Automation
  /t Capable of performing all driving functions under certain conditions
  /t Driver may have the option to control the vehicle
- Level5 : Full automation
  /t Capable of performing all driving functiosn under all conditions
  /t Driver may have the option to control the vehicle
  
---
  
### 2. Sensor selection Criteria
- Range
  - How far/close the sensor can detect objects in the environments
  - Radar sensor can detect the object much more closer to it than LiDAR sensor
  - Mono cameras are not able to reliably measure metric distance to object
  - Stero cameras can measure distance, but less accurate when the object is far away (more than approx. 80m)
- Spatial resolution
  - LiDAR scans have a spatial resolution in order of 0.1degree which allows for high-resolution 3D and characterization of objects in a scene
  - Radar can not resolve small features very well, especically as distance increase
  - Camera systems can lost details of small objects (blurring)
- Robustness in darkness
  - Both radar and LiDAR have an excellent robustness in darkness
  - LiDAR performs better at night time
  - Cameras ahve a very reduced detection capability at night
- Robustness in rain, snow, fog
  - Radars are not significantly affected by adverse weather conditions
  - LiDAR and cameras are susceptible to adverse weather 
- Classification of objects
  - Cameras excel at classifying objects 
  - LiDAR can classify the objects by scanning the density of the point clouds with less diversity
- Perceiving 2D structures
  - Camera systems are the only sensor able to interpret 2D information (ex. speed signs, lane markings, traffic lights, etc.)
- Measure speed
  - Radar can directly measure the velocity of objects by exploiting the Doppler frequency shift
  - Lidar can only approximate speed by using successive distance measurements
  - Cameras can measure TTC (Time to Collision) by observing the displacement of objects on the image plane
- System cost
- Package size
- Computational requirements

---

### 3. Estimating the TTC with LiDAR sensor

( TTC picture image )

( LiDAR TTC equation image )

As the images above, Once the relative velocity is known, TTC can easily be calculated.
The LiDAR sensor provides measurements on the vehicle as well as on the road surface, so we need to do some job only leaving the cloud points of the detected objects.
Measurement accuracy is correlated to the amount of light reflected from an object, so it makes sense to consider the reflectiveness r of each LiDAR point.
So the conclusion is, in order to derive a stalbe TTC measurement from the given cloud, we have to go through two steps:
  1. Remove point clouds on the road surface
  2. Remove measurements with low reflectivity
  
- Computing TTC from Distance Measurements
In this course, LiDAR points are packaged into a data structure called LidarPoints.
The structure consists of the point coordinates x (forward), y(left), z (up) in metric corrdinates and of the point reflectivity r on a scale between 0 (low reflectivity) and 1 (high reflectivity).

```c++
struct LidarPoint { 
	double x, y, z; // point position
	double r; // point reflectivity
};
```

In order to compute the TTC, we need to find the distance to the closest l
