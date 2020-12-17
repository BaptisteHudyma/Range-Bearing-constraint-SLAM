#include "SLAM_contractor.h"
#include <unistd.h>
using namespace tubex;

SLAM_Contractor::SLAM_Contractor(const unsigned int landmarkCount, const TubeVector& x, const TubeVector& speedMeasurments) : cn(), ctc_plus(Function("a", "b", "c", "a+b-c")), ctc_minus(Function("a", "b", "c", "a-b-c")), robotPosition(x), robotVelocity(speedMeasurments)   {     
    
    //cn.add(ctc::eval, {0, currentRobotPosition, robotPosition, robotVelocity});
    cn.add(ctc::deriv, {robotPosition, robotVelocity});
    for(unsigned int i = 0; i < landmarkCount; i+=1) {
        //add empty landmark positions
        observedLandmarkPositions.push_back(IntervalVector(2));
    }
}

//add a marker observation at time t to the network
void SLAM_Contractor::add_observation(const Interval& t, const unsigned int landmarkIndex, const Interval& measuredAngle, const Interval& measuredDistance) {
    //internal known variables
    Interval& angle = cn.create_dom(Interval(measuredAngle));
    Interval& distance2Landmark = cn.create_dom(Interval(measuredDistance));

    //internal variables
    IntervalVector& currentRobotPosition = cn.create_dom(IntervalVector(3));
    Interval& theta = cn.create_dom(Interval());            //polar angle theta
    IntervalVector& d = cn.create_dom(IntervalVector(2));   //polar temp variables
    Interval& tRef = cn.create_dom(Interval(t));

    //get robot position at time t
    cn.add(ctc::eval, {tRef, currentRobotPosition, robotPosition, robotVelocity});

    IntervalVector& obsLandmark = observedLandmarkPositions[landmarkIndex]; 

    //polar coordinates
    cn.add(ctc_plus, {angle, currentRobotPosition[2], theta});
    cn.add(ctc_minus, {obsLandmark[0], currentRobotPosition[0], d[0]});
    cn.add(ctc_minus, {obsLandmark[1], currentRobotPosition[1], d[1]});
    cn.add(ctc::polar, {d, distance2Landmark, theta});
}

//return a robot position box at time t
const IntervalVector SLAM_Contractor::get_t_observation(double t, double iterDuration) {
    double contractionDt = cn.contract_during(iterDuration);
    usleep(max(0., iterDuration - contractionDt) * 1e6); // pause for the animation
    // Display the current slice [x](t)
    return (robotPosition(max(0., ibex::previous_float(t))).subvector(0,1));
}

//returns the estimated positions of each landmarks
const vector<IntervalVector> SLAM_Contractor::get_estimated_landmark_positions() {
    return observedLandmarkPositions;
}


const TubeVector SLAM_Contractor::get_robot_trajectory() {
    return robotPosition;
}

void SLAM_Contractor::contract() {
    cn.contract();
}





