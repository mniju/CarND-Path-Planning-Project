## Path-Planning-Project
The objective of this project is to plan a Trajectory and make the car navigate a 3 lane highway
without any collision ,maintaining the levels of jerk, acceleration and speed.

### Rubics:
#### 1. Compilation:
The code compiles without any error with the give cMake.It was compiled and tested in
Ubuntu 16.04 Virtual machine.

```
VirtualBox:~/Term3/CarND-Path-Planning-Project/build$ make
Scanning dependencies of target path_planning
[ 50%] Building CXX object CMakeFiles/path_planning.dir/src/main.cpp.o
[100%] Linking CXX executable path_planning
[100%] Built target path_planning
````
#### 2. The car is able to drive at least 4.32 miles without incident:
The car is able to drive the mentioned 4.42 miles without any incident.Its able to go well
beyond that .The attatched image '5 Miles.png' shows this.

#### 3. The car drives according to the speed limit:
The speed limit here is 50 mph .It doesnt exceed the speed limit any time of its run in the track.This can be seen in the attatched video '6 miles run.wmv'. The max speed limit of 49.5 mph is set in the code.(lines 340 to 343)
```c++
else if (ref_vel<49.5)
{
  ref_vel += 0.224;
}
```

#### 4. Max Acceleration and Jerk are not Exceeded.
The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.
This can be seen from the video.
The speed of the car is increased slowly for cold start so that it doesnt accelerate or cause jerk.While decelerating also, it doesnt exceed the limits.

#### 5. Car does not have collisions.
The car doesnt collide with any of the other cars.It decelerates when it sees a car in front.It changes lane, only when there is no other car in the vicinity of 20 m (both front and back)in the other lane.

#### 6. The car stays in its lane, except for the time between changing lanes.
The car truly stays in the lane except the lane changing.This we make sure by creating 'd' value of trajectories with the lane information when creating the waypoints in XY coordinates.

```c++
ector<double> next_wp0 = getXY(car_s+30,2+(4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);```


#### 7. The car is able to change lanes
The car changes the lane smoothly thanks to the spline library that is used to build the path.

### Model Documentation:

#### Data from the Udacity Simulator:

Main car's localization Data (No Noise):

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

["previous_path_x"] The previous list of x points previously given to the simulator.

["previous_path_y"] The previous list of y points previously given to the simulator

["end_path_s"] The previous list's last point's frenet s value.

["end_path_d"] The previous list's last point's frenet d value.

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

#### Steps:

Initially the car is in Middle Lane.
```c++  
int lane =1;//Lanes are 0,1,2. Here we start in the centre lane.
double ref_vel = 0;//Reference velocity
double new_target_speed = 49.5;// Target velocity we Aim for.```


This 'lane' variable will be used to denote lane chnages and will be used for calculations below.In addition to that the target speed specified in the Rubic,50 mph(~49.5) is set as the target speed of the car.'ref_vel' is the actual velocity command to the car at the moment. The velocity is set to the car in term of the distance between the consecutive path points .(lines 435 & 436).

```c++
//create point distance in terms of the required velocity
double N = (target_dist/(0.02*ref_vel/2.24));//2.24 miles/hr to mtrs/sec
double x_point = x_add_on + (target_x/N);```

More on this later.

At first , from the sensor fusion we collect the data for the cars and group them interms of the lane. Since the lane with is 4 units, from the 'd' value we get for each car
we find the lane for that car and record the distance of that car with our ego car (car we control). Thus we have three vectors,lane0,lane1 and lane2 have the distance of all the cars in each lane respective to our car.(lines 256 to 281)

```c++
double dis = car_s - s;```

For each car, we get the s,d , calculate the velocity .Also we project the position of the car over time for next iteration in future.(line 290)

```c++
check_car_s += (double)prev_size*0.02*check_speed;```

If any car is in our lane and its in front on us,and its in front of us, then time to slow down the car.Note the speed of the front car and latch to its speed approx.This is done by setting the variable 'too_close' to True and setting the 'new_target_speed'(lines 293 to 298).

```c++
new_target_speed = (check_speed*2.23694) -10;```

Here 2.23694 does the conversion from m/sec to mph.

Lane Change:
     Lane change has to be done when we encounter a slower vehicle in addition to slowing down the car  as mentioned above.For that i check if the cars in the neraby lanes are nearby(~20 meters) either at front or back .If there is a car in that distance, then i dont make a lan chnage.(lines 312 to 335).Lane chnage he is done by setting the variable 'lane' to the apt value

```c++
     if((lane ==0) && (min_lane1 > 20))
     {
       lane = 1;}```

### Path Generation:

The path is generated in Frenet coordinates and then converted to the XY coordinates.

1. Initially check if there are any other previous paths.If not create a point tangential to the present car position.Vectors 'ptsx' and 'ptsy' hold the waypoints in Frenet co-ordinates.
    (lines 355 to 362)

```c++
    if (prev_size <2)
    {
    double tangent_car_x = car_x - cos(car_yaw);
    double tangent_car_y = car_y - sin(car_yaw);
    ptsx.push_back(tangent_car_x);
    ptsx.push_back(car_x);
    ptsy.push_back(tangent_car_y);
    ptsy.push_back(car_y);
  }```

2. Incase already points are avialiable from previous run, just add the remaining last two points in the current points(lines 365 to 368).

3. In Frenet co-ordinates, add 3 waypoints with 30 m spacing from the current car position.
```c++
//In Frenet add points ahead in 30 m spacing from the starting ref.
vector<double> next_wp0 = getXY(car_s+30,2+(4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
vector<double> next_wp1 = getXY(car_s+60,2+(4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
vector<double> next_wp2 = getXY(car_s+90,2+(4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);```

As seen we are using 'lane' to calculate 'd' the position of the car one the road .Each lane is 4 meters width.So for lane0, the d value will be two.Using the getXY function we convert them back to the XY coordinates.
and add them to the wapoints vectors ptsx and ptsy.(lines 376 to 386).

4. Convert the waypoints to Car XY coordinates w.r. to car.(lines 389-394)

```c++
double x_shift = ptsx[i] - ref_x;
double y_shift = ptsy[i] - ref_y;
ptsx[i]= x_shift*cos(0-ref_yaw) - y_shift * sin(0-ref_yaw);
ptsy[i]= x_shift*sin(0-ref_yaw) + y_shift * cos(0-ref_yaw);```

#### Curve Fitting:
To create a smooth trajectory, i used http://kluge.in-chemnitz.de/opensource/spline/ .It reduces teh task of doing Polynomial fitting again in the code.

1. Define and set the waypoints in the spline library.The points along which a curve /trajectory to be fit.
```c++
tk::spline s;
s.set_points(ptsx,ptsy);```

2.we set a vicinity of 30 meters,target_x,interpolate the y value for that x using spline and then find the distance magnitude from the car position to the vicinity point.

```c++
double target_x = 30.0;
double target_y = s(target_x);
double target_dist = sqrt(pow(target_x,2)+pow(target_y,2));
```

3. The distance calculated above will be mathematically equal to the product of 'N' no of parts this distance is split;velocity and a factor of 0.02.So Essentially
```
N * ref_vel*0.02 = target_dist```

4. Using the above formulae, we calculate 'N' for how many strips the present distance that we got (~30 ms) should be split(target_x/N) , so that each strip will be covered in 20 milliseconds(defined by simulator) so as to maintain  the required velocity.Here we convert the required velocity in terms of distance between points.(lines 418 to 423).

```c++
//create point distance in terms of the required velocity
double N = (target_dist/(0.02*ref_vel/2.24));//2.24 miles/hr to mtrs/sec
double x_point = x_add_on + (target_x/N);
double y_point = s(x_point);```

we create around 50 points. We don't build the 50 points from groundup.Instead for smoothness, we take the leftout point (ie)points not yet covered by the car generated by the previous iteration .In addition to that we generate the new points to sum to 50 points.(line 418)

5. Finally the points are converted from car coordinates to global coordiantes and updated in the 'next_x_vals' and 'next_y_vals' vector to be sent to the simulator.

```c++
//Change the co ordinates back  from car to global map before sending to simulator
x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));```

### Further Improvement.

Implement a Cost functions. since the spline tractory takes care of lane change and smoothening, cost function is not implemented.

Implement prediction of the position of other cars using the Naive Bayes
