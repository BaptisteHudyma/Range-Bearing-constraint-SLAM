#ifndef SLAM_CONTRACT_H
#define SLAM_CONTRACT_H

#include <tubex.h>

class SLAM_Contractor {
    public:
        SLAM_Contractor (const unsigned int landmarkCount, const tubex::TubeVector& x, const tubex::TubeVector& speedMeasurments); 

        //add a marker observation at time t to the network
        void add_observation(const tubex::Interval& t, const unsigned int landmarkIndex, const tubex::Interval& angle, const tubex::Interval& distance);

        //return a robot position box at time t
        const tubex::IntervalVector get_t_observation(double t, double dt);

        //returns the estimated positions of each landmarks
        const vector<tubex::IntervalVector> get_estimated_landmark_positions();

        //return a posteriori robot trajectory
        const tubex::TubeVector get_robot_trajectory();
        
        //Make static constraint calculation
        void contract();


    private:
        tubex::ContractorNetwork cn;               //robot position  contractor network
        tubex::TubeVector robotPosition;       //robot position through time
        tubex::TubeVector robotVelocity;       //robot velocity through time
        vector<tubex::IntervalVector> observedLandmarkPositions;   //vector containing each landmark estimated position

        //contractor network operations
        tubex::CtcFunction ctc_plus; // a+b=c
        tubex::CtcFunction ctc_minus; // a-b=c
};




#endif
