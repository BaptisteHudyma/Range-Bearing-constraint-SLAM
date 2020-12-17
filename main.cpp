#include <tubex.h>
#include <unistd.h>

using namespace tubex;

#include "SLAM_contractor.h"
#include "measurments.h"


const bool DISPLAY_PIES = false;    //display a pie pointing to observed obstacle


void range_bearing_SLAM (SLAM_Contractor& cn, const TrajectoryVector& xTruth, vector<IntervalVector>& landmarks, const Interval& tdomain, double dt, VIBesFigMap& fig) {
    //SLAM based on range and bearing, a posteriori result.
    for(double t = tdomain.lb(); t < tdomain.ub(); t += 2 * dt) {
        const unsigned int landmarkIndex = (rand()%4);    //random landmark
        //get true robot and landmark position
        IntervalVector trueLandmarkPos = landmarks[landmarkIndex];
        IntervalVector trueRobotPosition = xTruth(t);
        //get distance and angle to landmark
        Interval angleMeasure = get_angle_measurement(trueRobotPosition, trueLandmarkPos, 0.0);
        Interval distMeasure = get_range_measurement(trueRobotPosition, trueLandmarkPos, 0.0);

        if (DISPLAY_PIES) {     //display the view to the observed marker 
            fig.draw_pie(trueRobotPosition[0].mid(), trueRobotPosition[1].mid(), distMeasure, angleMeasure + trueRobotPosition[2], "black");
            fig.draw_pie(trueRobotPosition[0].mid(), trueRobotPosition[1].mid(), (Interval(0.01) | distMeasure), angleMeasure + trueRobotPosition[2], "gray");
        }

        //add observation to our contractor network
        cn.add_observation(Interval(t), landmarkIndex, angleMeasure, distMeasure);
    }
    cn.contract();
}



void range_bearing_SLAM_rt (SLAM_Contractor& cn, const TrajectoryVector& xTruth, vector<IntervalVector>& landmarks, const Interval& tdomain, double dt, VIBesFigMap& fig) {
    //SLAM based on range and bearing, with the robot position evaluated at each dt in real time.
    double iterdt = 0.2;    //calculation duration

    double prev_t_obs = tdomain.lb();
    for(double t = tdomain.lb() ; t < tdomain.ub() ; t += dt) {
        if(t - prev_t_obs > 2 * dt) { // new observation each 2*dt
            //for(int i = 0; i < 5; i++) {     
                prev_t_obs = t;
                const unsigned int landmarkIndex = (rand()%5);    //random landmark
                //get true robot and landmark position
                IntervalVector trueLandmarkPos = landmarks[landmarkIndex];
                IntervalVector trueRobotPosition = xTruth(t);
                //get distance and angle to landmark
                Interval angleMeasure = get_angle_measurement(trueRobotPosition, trueLandmarkPos, 0.05);
                Interval distMeasure = get_range_measurement(trueRobotPosition, trueLandmarkPos, 0.05);
    
                if (DISPLAY_PIES) {     //display the view to the observed marker 
                    fig.draw_pie(trueRobotPosition[0].mid(), trueRobotPosition[1].mid(), distMeasure, angleMeasure + trueRobotPosition[2], "black");
                    fig.draw_pie(trueRobotPosition[0].mid(), trueRobotPosition[1].mid(), (Interval(0.01) | distMeasure), angleMeasure + trueRobotPosition[2], "gray");
                }
    
                //add observation to our contractor network
                cn.add_observation(Interval(t), landmarkIndex, angleMeasure, distMeasure);
            //}
        }
        //display current robot position
        fig.draw_box(cn.get_t_observation(t, iterdt));
    }
}




int main() {
    srand((unsigned)time(NULL));

    double dt = 0.05;
    Interval tdomain(0, 15);    //study from 0 to 15 seconds


    vector<IntervalVector> landmarks = get_marker_positions();

    Vector x0Truth({0, 0, 2});   //starting position
    Trajectory u = Trajectory(tdomain, TFunction("(3 * sqr(sin(t)) + t/100) + [-0.1,0.1]"), dt);

    //Truth trajectories (position + velocity)
    TrajectoryVector vTruth(3);
    TrajectoryVector xTruth(3);
    set_v_x_truth(xTruth, vTruth, u, x0Truth);

    //real trajectories (tubes)
    IntervalVector x0 = IntervalVector(x0Truth);//.inflate(0.2);   //add uncertainty on start position
    TubeVector v(tdomain, dt, 3);   //v is the measured velocity
    TubeVector x(tdomain, dt, 3);   //for now, x is the position deduced from dead reckoning
    set_v_x_tubes(x, v, xTruth, u, x0, tdomain, dt, 0.0, 0.0, 0.0);


    //Display
    vibes::beginDrawing();
    VIBesFigMap fig("SLAM with range bearing observations");

    fig.set_properties(100, 100, 600, 300);
    fig.add_tube(&x, "tube x", 0, 1);   //display dead reckoning tube 
    fig.add_trajectory(&xTruth, "x*", 0, 1, "red"); //display true trajectory

    //landmark variables
    for(unsigned int i = 0; i < landmarks.size(); i++) {
        //display beacons
        fig.add_beacon(landmarks[i], 0.1);
    }
    fig.smooth_tube_drawing(true);
    fig.show(0.5);


    //create a SLAM contractor network
    SLAM_Contractor cn(landmarks.size(), x, v);

    //constraint process 
#if 1
    range_bearing_SLAM_rt (cn, xTruth, landmarks, tdomain, dt, fig);
#else
    range_bearing_SLAM (cn, xTruth, landmarks, tdomain, dt, fig);
#endif


    x = cn.get_robot_trajectory();  //update position tube
    vector<IntervalVector> ulandmarks = cn.get_estimated_landmark_positions();  //new landmarks positions

    //display beacons positions deduced by the program
    for (unsigned int i = 0; i < landmarks.size(); i++) {
        fig.draw_box(ulandmarks[i]);
    }

    fig.show();
    vibes::endDrawing();


    return 0;
}

