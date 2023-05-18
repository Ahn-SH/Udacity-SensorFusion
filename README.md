# Udacity-SensorFusion
## Radar Sensor Summary and Project Explanation
<br/>
<br/>
---
<br/>
### Project Overview
<br/>
- Project Objectives
	- Configure the FMCW waveform based on the system requirement
	- Define the range and velocity of target and simulate its displacement
	- For the same simulation loop process the transmit and receive signal to determine the best signal
	- Perform Range FFT on the received signal to determine the range 
	- Perform the CFAR processing on the output of 2nd FFT to display the target
<br/>
- Radar System requirements 
	[imge](https://video.udacity-data.com/topher/2019/May/5ce350d0_image14/image14.png)
	``` MATLAB
	c = 3*10^8;
	Rmax = 200;
	Rres = 1;
	Bsweep = c / (2*Rres);
	Tchirp = 5.5 * 2 *Rmax / c;
	slope = Bsweep / Tchirp;
	fc= 77e9; 
	```
<br/>
<br/>
---
<br/>
### Simple Explantions
<br/>
- FMCW(Frequency-Modulated Continuous Wave): Use continuous frequency modulation to measure range and velocity of targets. <br/>
- FFT (Fast Fourier Transform): FFT transforms the time-domain radar signal to the frequency-domain for further processing. <br/>
- Clutter: Clutter is the unwanted signals or noise that can interfere with target detection. One of the techniques used in this project is CFAR. <br/>
- CFAR (Constant False Alarm Rate): With CFAR, we can detect targets in the presence of varying clutter levels(detection threshold) while maintaining a constant false alarm rate. <br/>
- CA-CFAR (Cell Averaging CFAR): CA-CFAR is one of the specific algorithms in CFAR which utilizes a sliding window to estimate the statistical properties of the clutter and adaptively detect targets. <br/>
- Angle of Arrival (AoA): AoA is used to determine the direction or angle from which a radar signal arrives, enabling target localization in space.
- Clustering: Clustering groups together radar measurements or detections that likely belong to the same target, aiding in tracking and object recognition tasks.
<br/>
<br/>
---
<br/>
### CFAR in 2D
