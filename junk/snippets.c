#include <iostream>
#include <math.h>

using namespace std;

double f(double mu, double sigma2, double x)
{
    double num = pow((x-mu), 2);
    double den = -2 * sigma2;
    double exponent =-num/den;
    
    double factor = 1/(sqrt(2 * sigma2 * M_PI));
    
    double prob = factor * exp(exponent);
    return prob;
}

int main()
{
    cout << f(10.0, 4.0, 8.0) << endl;
    return 0;
}

#include <iostream>
#include <math.h>

using namespace std;

double f(double mu, double sigma2, double x)
{
    //Use mu, sigma2 (sigma squared), and x to code the 1-dimensional Gaussian
    //Put your code here
    double prob = 1.0 / sqrt(2.0 * M_PI * sigma2) * exp(-0.5 * pow((x - mu), 2.0) / sigma2);
    return prob;
}

int main()
{
    cout << f(10.0, 4.0, 8.0) << endl;
    return 0;
}



#include <iostream>
#include <math.h>
#include <tuple>

using namespace std;

double new_mean, new_var;

tuple<double, double> measurement_update(double mean1, double var1, double mean2, double var2)
{
    new_mean = (mean1 * var2 + mean2 * var1)/(var1 + var2);
    new_var =  1/(1/var1 + 1/var2);
    return make_tuple(new_mean, new_var);
}

int main()
{

    tie(new_mean, new_var) = measurement_update(10, 8, 13, 2);
    printf("[%f, %f]", new_mean, new_var);
    return 0;
}
#include <iostream>
#include <math.h>
#include <tuple>

using namespace std;

double new_mean, new_var;

tuple<double, double> measurement_update(double mean1, double var1, double mean2, double var2)
{
    new_mean = (var2 * mean1 + var1 * mean2) / (var1 + var2);
    new_var = 1 / (1 / var1 + 1 / var2);
    return make_tuple(new_mean, new_var);
}

tuple<double, double> state_prediction(double mean1, double var1, double mean2, double var2)
{
    new_mean = mean1 + mean2;
    new_var = var1 + var2;
    return make_tuple(new_mean, new_var);
}

int main()
{
    //Measurements and measurement variance
    double measurements[5] = { 5, 6, 7, 9, 10 };
    double measurement_sig = 4;
    
    //Motions and motion variance
    double motion[5] = { 1, 1, 2, 1, 1 };
    double motion_sig = 2;
    
    //Initial state
    double mu = 0;
    double sig = 1000;
    
    //######TODO: Put your code here below this line######//
    int numMeasurements = sizeof(motion)/sizeof(motion[0]);

    for (int tIndex = 0; tIndex < numMeasurements; tIndex++){
        //printf("measure, motion:  [%f, %f]\n", measurements[tIndex], motion[tIndex]);

        // Apply a measurment update
        tie(mu, sig) = measurement_update(mu, sig, measurements[tIndex], measurement_sig);
        printf("update:  [%f, %f]\n", mu, sig);
        
        // Apply a state prediction
        tie(mu, sig) = state_prediction(mu, sig, motion[tIndex], motion_sig);
        printf("predict: [%f, %f]\n", mu, sig);
    }
    
    return 0;
}

1) Almost useless. See answer 4) The Knowledge page. 
This student hub is mixing up technical Q&A with social networking. The primary problem is that this hub is chronological. Chronology is irrelevant and obscures relevant questions far into the scrollable past.  When I have a technical question I want to see other questions and answers like it. I don't care when that question was asked nor who asked it, and I want to see such questions from ALL previous cohorts so that the knowledge can accumulate. ALTERNATIVELY, mentors need to clarify what this hub is for... and it should be for social networking. Technical Q&A should go to the knowledge page.

2) Again, do you want lively social networking, or educational assistance. They are incompatible objectives on a website. Sorry, I'm not here to chat, I'm here to solve problems and learn!

3) I'm only a few modules in, but I like the development (so far better than Kalman filters in SelfDrivingCarEngineerND)

4) The Knowledge page is much closer to a useful technical Q&A. This hub needs to be modelled on StackOverflow.com. Unfortunately, because Udacity hasn't settled on a technical help mechanism and doesn't provide clear direction which mechanism to use for different purposes (Student hub? Knowledge? Slack? Email?) the useful Q&A information is decentralized and presumably duplicated and diluted. The Knowledge page should be promoted by udacity as the place for technical Q&A. It should also be broken down by individual project, either with tags or explicit menu selections as for the particular NanoDegree. 
The mentors should spend lotsa time there answering every question so that it 'sticks' forever.
Additionally the various projects have FAQs that get attached to the project web modules/pages. Those FAQ should be linked or duplicated here, because they are useful but sometimes hard to find. 



================ Practice: 5 TurtleBot Gazebo package
Clone Package:

$ cd /home/workspace/catkin_ws/src
$ git clone https://github.com/turtlebot/turtlebot_simulator

Install Dependencies:

$ cd /home/workspace/catkin_ws
$ source devel/setup.bash
$ rosdep -i install turtlebot_gazebo

Build Package:

$ catkin_make
$ source devel/setup.bash

Launch Nodes:

$ roslaunch turtlebot_gazebo turtlebot_world.launch

Topics:

$ rostopic list
Or
$ rosrun rqt_graph rqt_graph

================ Practice: 6 Robot pose to EKF package
EKF package:

Access this link and go through the robot_pose_ekf documentation.
Install the package:

$ cd /home/workspace/catkin_ws/src/
$ git clone https://github.com/udacity/robot_pose_ekf 

First, edit robot_pose_ekf.launch file:

<launch>
<node pkg="robot_pose_ekf" type="robot_pose_ekf" name="robot_pose_ekf">
  <param name="output_frame" value="odom_combined"/>
  <param name="base_footprint_frame" value="base_footprint"/>
  <param name="freq" value="30.0"/>
  <param name="sensor_timeout" value="1.0"/>  
  <param name="odom_used" value="true"/>
  <param name="imu_used" value="true"/>
  <param name="vo_used" value="false"/>

  <remap from="imu_data" to="/mobile_base/sensors/imu_data" />    
</node>
</launch>

Now, build the package:

$ cd /home/workspace/catkin_ws
$ catkin_make
$ source devel/setup.bash

Launch the node:

$ roslaunch robot_pose_ekf robot_pose_ekf.launch

Now, topics from the robot and ekf nodes should be communicating. To confirm that, run the rqt graph. In rqt graph, visualize how the topics and nodes are connected. In theactive nodes and topic, you won’t be able to see the 3D filtered pose topic since we haven’t yet subscribed to it.
Visualize the topics:

$ rosrun rqt_graph rqt_graph


================== 7 Odometry to Trajectory Package
Install the package:

$ cd /home/workspace/catkin_ws/src
$ git clone https://github.com/udacity/odom_to_trajectory

Build the package:

$ cd /home/workspace/catkin_ws
$ catkin_make
$ source devel/setup.bash

Launch the nodes:

$ roslaunch odom_to_trajectory create_trajectory.launch 

==================== 8 TurtleBot Teleop Package
Teleop package:

Access this link and go through the turtlebot_teleop documentation.
Clone the Package:

$ cd /home/workspace/catkin_ws/src
$ git clone https://github.com/turtlebot/turtlebot

Install the Dependencies:

$ cd /home/workspace/catkin_ws
$ source devel/setup.bash
$ rosdep -i install turtlebot_teleop

Build the Package:

$ catkin_make
$ source devel/setup.bash

Launch the Nodes:

$ roslaunch turtlebot_teleop keyboard_teleop.launch

=================== 9 RViz launch
Quiz Solution:
The RvizLaunch.launch file:

<launch>
  <!--RVIZ-->
  <node pkg="rviz" type="rviz" name="rviz" args="-d /home/cl/AAAProjects/AAAUdacity/roboND2/Proj2_Localization/P2_Root/catkin_ws/src/EKFLab.rviz"/>
</launch>

Launch Rvizlaunch.launch:

$ cd /home/workspace/catkin_ws/src
$ roslaunch RvizLaunch.launch


=================== 10 Main Launch
Create a main package:

$ cd /home/workspace/catkin_ws/src
$ catkin_create_pkg main

Build the package:

$ cd /home/workspace/catkin_ws
$ catkin_make

Create and edit the main.launch file:

$ cd /home/workspace/catkin_ws/src/main
$ mkdir launch
$ cd launch 
$ gedit main.launch

Copy the main.launch file from GitHub
Launch the main.launch file:

$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch main main.launch

**************************************************
**************************************************
**************************************************
$ roslaunch turtlebot_gazebo turtlebot_world.launch &
$ roslaunch robot_pose_ekf robot_pose_ekf.launch &
$ roslaunch odom_to_trajectory create_trajectory.launch  &
$ roslaunch RvizLaunch.launch &
$ roslaunch turtlebot_teleop keyboard_teleop.launch &
**************************************************
**************************************************
**************************************************
$ roslaunch main main.launch


apt-get install ros-kinetic-rqt -y
$ apt-get install ros-kinetic-rqt-multiplot -y
$ apt-get install libqwt-dev -y
$ rm -rf ~/.config/ros.org/rqt_gui.ini


@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

=================== 5 Bayes Filtering

#include <iostream>
using namespace std;

int main() {
	
	//Given P(POS), P(DOOR|POS) and P(DOOR|¬POS)
	double a = 0.0002 ; //P(POS) = 0.002
	double b = 0.6    ; //P(DOOR|POS) = 0.6
	double c = 0.05   ; //P(DOOR|¬POS) = 0.05
	
	//TODO: Compute P(¬POS) and P(POS|DOOR)
	double d =    1 - a;               //P(¬POS)
	double e =      (b*a)/((a*b)+(d*c)) ;              //P(POS|DOOR)
	
	//Print Result
	cout << "P(POS|DOOR)= " <<    e    << endl;
	
	return 0;
}


int main()
{
    // TODO: Instantiate a robot object from the Robot class
    Robot myrobot;

    // TODO: Set robot new position to x=30.0, y=50.0 and orientation=PI/2
    myrobot.set(30.0, 50.0, M_PI/2);

    // TODO: Turn clockwise by PI/2 and move by 15 meters
    myrobot.move(-M_PI / 2.0, 15.0);

    // TODO: Print the distance from the robot toward the eight landmarks
    cout << myrobot.read_sensors() << endl;

    // TODO: Turn clockwise by PI/2 and move by 10 meters
    myrobot.move(-M_PI / 2.0, 10.0);

    // TODO: Print the distance from the robot toward the eight landmarks
    cout << myrobot.read_sensors() << endl;

    return 0;
}

int main()
{
    Robot myrobot;
    // TODO: Simulate Noise
    float Forward_Noise=5.0, Turn_Noise=0.1,Sense_Noise=5.0;
    myrobot.set_noise(Forward_Noise, Turn_Noise, Sense_Noise);
    
    myrobot.set(30.0, 50.0, M_PI / 2.0);
    myrobot.move(-M_PI / 2.0, 15.0);
    cout << myrobot.read_sensors() << endl;
    myrobot.move(-M_PI / 2.0, 10.0);
    cout << myrobot.read_sensors() << endl;

    return 0;
}

int main()
{
    //Practice Interfacing with Robot Class
    Robot myrobot;
    myrobot.set_noise(5.0, 0.1, 5.0);
    myrobot.set(30.0, 50.0, M_PI / 2.0);
    myrobot.move(-M_PI / 2.0, 15.0);
    //cout << myrobot.read_sensors() << endl;
    myrobot.move(-M_PI / 2.0, 10.0);
    //cout << myrobot.read_sensors() << endl;

    //####   DON'T MODIFY ANYTHING ABOVE HERE! ENTER CODE BELOW ####

    // Instantiating 1000 Particles each with a random position and orientation
    int n = 1000;
    Robot p[n];
    
    //TODO: For each particle add noise: Forward_Noise=0.05, Turn_Noise=0.05, and Sense_Noise=5.0
    float Forward_Noise=0.05, Turn_Noise=0.05,Sense_Noise=5.0;
    
    //TODO: Your job is to loop over the set of particles
    for (int pIndex = 0; pIndex < n; pIndex++){
        p[pIndex].set_noise(Forward_Noise, Turn_Noise, Sense_Noise);
        
        //TODO: And print its pose on a single line
        cout << p[n].show_pose() << endl;
    }
    return 0;
}

int main()
{
    //Practice Interfacing with Robot Class
    Robot myrobot;
    myrobot.set_noise(5.0, 0.1, 5.0);
    myrobot.set(30.0, 50.0, M_PI / 2.0);
    myrobot.move(-M_PI / 2.0, 15.0);
    //cout << myrobot.read_sensors() << endl;
    myrobot.move(-M_PI / 2.0, 10.0);
    //cout << myrobot.read_sensors() << endl;

    // Create a set of particles
    int n = 1000;
    Robot p[n];

    for (int i = 0; i < n; i++) {
        p[i].set_noise(0.05, 0.05, 5.0);
        //cout << p[i].show_pose() << endl;
    }

    //####   DON'T MODIFY ANYTHING ABOVE HERE! ENTER CODE BELOW ####

    //Now, simulate motion for each particle
    //TODO: Create a new particle set 'p2'
    Robot p2[n];
    
    //TODO: Rotate each particle by 0.1 and move it forward by 5.0
    for (int i = 0; i < n; i++) {
        p2[i].move(0.1, 5.0);
    }

    //TODO: Assign 'p2' to 'p' and print the particle poses, each on a single line
    for (int i = 0; i < n; i++) {
        p[i] = p2[i];
        cout << p[i].show_pose() << endl;
    }

    return 0;
}

int main()
{
    //Practice Interfacing with Robot Class
    Robot myrobot;
    myrobot.set_noise(5.0, 0.1, 5.0);
    myrobot.set(30.0, 50.0, M_PI / 2.0);
    myrobot.move(-M_PI / 2.0, 15.0);
    //cout << myrobot.read_sensors() << endl;
    myrobot.move(-M_PI / 2.0, 10.0);
    //cout << myrobot.read_sensors() << endl;

    // Create a set of particles
    int n = 1000;
    Robot p[n];

    for (int i = 0; i < n; i++) {
        p[i].set_noise(0.05, 0.05, 5.0);
        //cout << p[i].show_pose() << endl;
    }

    //Re-initialize myrobot object and Initialize a measurment vector
    myrobot = Robot();
    vector<double> z;

    //Move the robot and sense the environment afterwards
    myrobot = myrobot.move(0.1, 5.0);
    z = myrobot.sense();

    // Simulate a robot motion for each of these particles
    Robot p2[n];
    for (int i = 0; i < n; i++) {
        p2[i] = p[i].move(0.1, 5.0);
        p[i] = p2[i];
    }

    //####   DON'T MODIFY ANYTHING ABOVE HERE! ENTER CODE BELOW ####

    //TODO: Generate particle weights depending on robot's measurement
    //TODO: Print particle weights, each on a single line
    double w[n];
    
    for (int i = 0; i < n; i++) {
        w[i] = p[i].measurement_prob(z);
        cout << w[i] << endl;
    }
    return 0;
}


int main()
{
    //Practice Interfacing with Robot Class
    Robot myrobot;
    myrobot.set_noise(5.0, 0.1, 5.0);
    myrobot.set(30.0, 50.0, M_PI / 2.0);
    myrobot.move(-M_PI / 2.0, 15.0);
    //cout << myrobot.read_sensors() << endl;
    myrobot.move(-M_PI / 2.0, 10.0);
    //cout << myrobot.read_sensors() << endl;

    // Create a set of particles
    int n = 1000;
    Robot p[n];

    for (int i = 0; i < n; i++) {
        p[i].set_noise(0.05, 0.05, 5.0);
        //cout << p[i].show_pose() << endl;
    }

    //Re-initialize myrobot object and Initialize a measurment vector
    myrobot = Robot();
    vector<double> z;

    //Move the robot and sense the environment afterwards
    myrobot = myrobot.move(0.1, 5.0);
    z = myrobot.sense();

    // Simulate a robot motion for each of these particles
    Robot p2[n];
    for (int i = 0; i < n; i++) {
        p2[i] = p[i].move(0.1, 5.0);
        p[i] = p2[i];
    }

    //Generate particle weights depending on robot's measurement
    double w[n];
    for (int i = 0; i < n; i++) {
        w[i] = p[i].measurement_prob(z);
        //cout << w[i] << endl;
    }

    //####   DON'T MODIFY ANYTHING ABOVE HERE! ENTER CODE BELOW ####

    //TODO: Resample the particles with a sample probability proportional to the importance weight
     //Resample the particles with a sample probability proportional to the importance weight
    Robot p3[n];
    int index = gen_real_random() * n;
    //cout << index << endl;
    double beta = 0.0;
    double mw = max(w, n);
    //cout << mw;
    for (int i = 0; i < n; i++) {
        beta += gen_real_random() * 2.0 * mw;
        while (beta > w[index]) {
            beta -= w[index];
            index = mod((index + 1), n);
        }
        p3[i] = p[index];
    }
    for (int k=0; k < n; k++) {
        p[k] = p3[k];
        cout << p[k].show_pose() << endl;
    }

    return 0;
}

int main()
{
    //Practice Interfacing with Robot Class
    Robot myrobot;
    myrobot.set_noise(5.0, 0.1, 5.0);
    myrobot.set(30.0, 50.0, M_PI / 2.0);
    myrobot.move(-M_PI / 2.0, 15.0);
    //cout << myrobot.read_sensors() << endl;
    myrobot.move(-M_PI / 2.0, 10.0);
    //cout << myrobot.read_sensors() << endl;

    // Create a set of particles
    int n = 1000;
    Robot p[n];

    for (int i = 0; i < n; i++) {
        p[i].set_noise(0.05, 0.05, 5.0);
        //cout << p[i].show_pose() << endl;
    }

    //Re-initialize myrobot object and Initialize a measurment vector
    myrobot = Robot();
    vector<double> z;

    //Move the robot and sense the environment afterwards
    myrobot = myrobot.move(0.1, 5.0);
    z = myrobot.sense();

    // Simulate a robot motion for each of these particles
    Robot p2[n];
    for (int i = 0; i < n; i++) {
        p2[i] = p[i].move(0.1, 5.0);
        p[i] = p2[i];
    }

    //Generate particle weights depending on robot's measurement
    double w[n];
    for (int i = 0; i < n; i++) {
        w[i] = p[i].measurement_prob(z);
        //cout << w[i] << endl;
    }

    //####   DON'T MODIFY ANYTHING ABOVE HERE! ENTER CODE BELOW ####

    //Resample the particles with a sample probability proportional to the importance weight
    Robot p3[n];
    int index = gen_real_random() * n;
    //cout << index << endl;
    double beta = 0.0;
    double mw = max(w, n);
    //cout << mw;
    for (int i = 0; i < n; i++) {
        beta += gen_real_random() * 2.0 * mw;
        while (beta > w[index]) {
            beta -= w[index];
            index = mod((index + 1), n);
        }
        p3[i] = p[index];
    }
    for (int k=0; k < n; k++) {
        p[k] = p3[k];
        cout << p[k].show_pose() << endl;
    }

    return 0;
}


int main()
{
    //Practice Interfacing with Robot Class
    Robot myrobot;
    myrobot.set_noise(5.0, 0.1, 5.0);
    myrobot.set(30.0, 50.0, M_PI / 2.0);
    myrobot.move(-M_PI / 2.0, 15.0);
    //cout << myrobot.read_sensors() << endl;
    myrobot.move(-M_PI / 2.0, 10.0);
    //cout << myrobot.read_sensors() << endl;

    // Create a set of particles
    int n = 1000;
    Robot p[n];

    for (int i = 0; i < n; i++) {
        p[i].set_noise(0.05, 0.05, 5.0);
        //cout << p[i].show_pose() << endl;
    }

    //Re-initialize myrobot object and Initialize a measurment vector
    myrobot = Robot();
     vector<double> z;

    //Iterating 50 times over the set of particles
    int steps = 50;
    for (int t = 0; t < steps; t++) {

        //Move the robot and sense the environment afterwards
        myrobot = myrobot.move(0.1, 5.0);
        z = myrobot.sense();

        // Simulate a robot motion for each of these particles
        Robot p2[n];
        for (int i = 0; i < n; i++) {
            p2[i] = p[i].move(0.1, 5.0);
            p[i] = p2[i];
        }

        //Generate particle weights depending on robot's measurement
        double w[n];
        for (int i = 0; i < n; i++) {
            w[i] = p[i].measurement_prob(z);
            //cout << w[i] << endl;
        }

        //Resample the particles with a sample probability proportional to the importance weight
        Robot p3[n];
        int index = gen_real_random() * n;
        //cout << index << endl;
        double beta = 0.0;
        double mw = max(w, n);
        //cout << mw;
        for (int i = 0; i < n; i++) {
            beta += gen_real_random() * 2.0 * mw;
            while (beta > w[index]) {
                beta -= w[index];
                index = mod((index + 1), n);
            }
            p3[i] = p[index];
        }
        for (int k=0; k < n; k++) {
            p[k] = p3[k];
            //cout << p[k].show_pose() << endl;
        }

        //####   DON'T MODIFY ANYTHING ABOVE HERE! ENTER CODE BELOW ####
        
        //Evaluate the error by priting it in this form:
        // cout << "Step = " << t << ", Evaluation = " << ErrorValue << endl;
        cout << "Step = " << t << ", Evaluation = " << evaluation(myrobot, p, n) << endl;

    } //End of Steps loop
    return 0;
}







