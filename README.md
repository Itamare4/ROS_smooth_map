
Smooth Map Server
------------------------

### OVERVIEW ###
Extremely simple algorithm to inflate obstacles in ROS framework, IOI(Iterative Obstacle Inflation), the node listens to local_costmap and publish /map.<br>
Inside the python script you can choose the iteration number, and pixels around the obstacles.<br>



### Software ###
* ROS Indigo
* Python

### Examples ###
<p align="center">
<img src="https://s10.postimg.org/50j1jfmxl/image.png" height="300"><br>Inflation with 5 Pixels, 3 iterations<br>

<img src="https://s21.postimg.org/5t7uni16v/image.png" height="300"><br>Inflation with 5 Pixels, 3 iterations, Costmap scheme
</p>


### ROS Interface ###
* Publisher - /map <br>
* Server - /Smooth_Map


### Known Issues ###
* ?

### About ###
Itamar Eliakim<br>
M.Sc Student Mechanical Engineering Faculty at Tel Aviv University, Israel<br>
Email - Itamare@gmail.com



