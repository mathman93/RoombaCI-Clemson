This folder contains data collected during the spring of 2021. Specifically data pertaining to the IMU and Roomba wall finder.

IMU Test Data Files
1. IMU_Data_CircularTest.txt
  20 second test where IMU data is collected while the Roomba moves in a large clockwise circle.
2. IMU_Data_Clockwise.txt
  20 second test where IMU data is collected while the Roomba rotates in a clockwise circle.
  This rotaion occurs around a stationary Z axis.
3. IMU_Data_CounterClockwise.txt
  20 second test where IMU data is collected while the Roomba rotates in a counter clockwise circle.
  This rotation occurs around a stationary Z axis.
4. IMUData_Pickup_Retest.txt
  20 second test where the Roomba is programmed to remain stationary. The Roomba is then picked up
  and rotated 90 degrees about the Y axis. The Roomba is then rotated back -90 degrees before being
  placed on the ground. This test was done to observe z gyroscope data from the IMU.
5. IMUData_2-18-21.txt
  30 second test where the Roomba moves forward then rotates 180 degrees about the z axis before
  returning to its original position. This test was used to get an initial baseline of IMU data.
6. Test1 & Test2 _points & _walls
  Test of ploting MATLAB script. 2 different orientations and starting points in order to 
  attempt to figure out how to translate one map onto the other
7. Test3 & Test4 _points & _walls
  First tests of sprial pattern for autmated mapping. Triggers the key error for both of these
8. Test5 & Test6 _points & _walls
  Second round of testing after fixing bugs. The Roomba eventually determines it cannot reach any 
  points and infinitely increments in a spiral patter.
9. EdgeTest
  Test to look at the edges between points and make sure all edges are correct. 
