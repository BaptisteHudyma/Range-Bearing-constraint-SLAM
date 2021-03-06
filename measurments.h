#ifndef MEASURMENTS_H
#define MEASURMENTS_H

//set landmarks positions
vector<IntervalVector> get_marker_positions() {
    vector<IntervalVector> landmarks;
    landmarks.push_back(IntervalVector({Interval(6), Interval(12)}));
    landmarks.push_back(IntervalVector({Interval(-2), Interval(-8)}));
    landmarks.push_back(IntervalVector({Interval(-10), Interval(10)}));
    landmarks.push_back(IntervalVector({Interval(20), Interval(0)}));
    landmarks.push_back(IntervalVector({Interval(5), Interval(-15)}));
    return landmarks;
}

// Actual truth trajectories (state + derivative)
void set_v_x_truth(TrajectoryVector& xTruth, TrajectoryVector& vTruth, const Trajectory& u, const Vector& x0) {
    vTruth[2] = u; 
    xTruth[2] = vTruth[2].primitive() + x0[2];
    vTruth[0] = 10 * cos(xTruth[2]);
    vTruth[1] = 10 * sin(xTruth[2]);
    xTruth[0] = vTruth[0].primitive() + x0[0];
    xTruth[1] = vTruth[1].primitive() + x0[1];
}

// Actual dead reckoning trajectories (state + derivative)
void set_v_x_tubes(TubeVector& x, TubeVector& v, const TrajectoryVector& xTruth, const Trajectory& u, const IntervalVector& x0, const Interval& tdomain, const double dt, const double xError, const double vError, const double tDomainError) {
    RandTrajectory nTheta(tdomain, dt, Interval(-tDomainError, tDomainError));
    v[2] = Tube(u + nTheta, dt).inflate(vError);
    x[2] = Tube(xTruth[2] + nTheta, dt).inflate(xError); 
    v[0] = 10 * cos(x[2]);
    v[1] = 10 * sin(x[2]);
    x[0] = v[0].primitive() + x0[0];
    x[1] = v[1].primitive() + x0[1];
}


//return distance to an obstacle from robot position
const Interval get_range_measurement(const IntervalVector& robotCurrentPosition, const IntervalVector& markerPosition, const double uncertainty) {
    Interval dstVec = sqrt(sqr(robotCurrentPosition[0] - markerPosition[0]) + sqr(robotCurrentPosition[1] - markerPosition[1]));
    //add uncertainty in measurements
    dstVec.inflate(uncertainty);
    return dstVec;
}

//return angle to an obstacle from robot position
const Interval get_angle_measurement(const IntervalVector& robotCurrentPosition, const IntervalVector& markerPosition, const double uncertainty) {
    Interval markerAngle = atan2( (markerPosition[1] - robotCurrentPosition[1]), (markerPosition[0] - robotCurrentPosition[0]) ) - robotCurrentPosition[2];
    //add uncertainty in measurements
    markerAngle.inflate(uncertainty);
    return markerAngle;
}



#endif
