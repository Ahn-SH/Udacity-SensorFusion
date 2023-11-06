# Udacity Sensor Fusion Course
## 2D CFAR Process
<br/>
<br/>
---
<br/>
<br/>
# Range Doppler Map Generation <br/>
  /t Using 2D FFT, the received signal is processed and generate RDM (Range Doppler Map)<br/>
# TC, GC, Offset <br/>
  /t TC(Training Cell), Guard Cell(Guard Cell), and offset are the prameters for the CFAR process. <br/>
  /t In this code, I choosed TC as 3, GC as 1, and offet as 5. <br/>
# Sliding Window Process<br/>
  /t A sliding window is moved across the RDM. <br/>
# Sum up Signal Levels <br/>
  /t The signal levels within the training cells are summed but it excludes the values of the guard cells.<br/>
# Threshold Calculation <br/>
  /t The average of the summation above. <br/>
  /t The threshold is set to maintain a constant false alarm rate. <br/>
# Detection <br/> 
  /t If the signal level is above the threshold, the CUT is 1. <br/>
  /t If the signal level is below the threshold, the CUT is 0. <br/>
# Generating Threshold Block <br/>
  /t The cells outside the thresholded block are set to 0 to maintain the same map size. <br/>