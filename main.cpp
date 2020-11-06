#include <tubex.h>
#include <unistd.h>
using namespace tubex;

vector<IntervalVector> get_marker_positions() {
    //set landmarks positions
    vector<IntervalVector> landmarks;
    landmarks.push_back(IntervalVector({Interval(6), Interval(12)}));
    landmarks.push_back(IntervalVector({Interval(-2), Interval(-5)}));
    landmarks.push_back(IntervalVector({Interval(-3), Interval(20)}));
    landmarks.push_back(IntervalVector({Interval(3), Interval(4)}));
    return landmarks;
}

void set_v_x_truth(TrajectoryVector& xTruth, TrajectoryVector& vTruth, const Trajectory& u, const Vector& x0) {
    // Actual trajectories (state + derivative)
    vTruth[2] = u; 
    xTruth[2] = vTruth[2].primitive() + x0[2];
    vTruth[0] = 10 * cos(xTruth[2]);
    vTruth[1] = 10 * sin(xTruth[2]);
    xTruth[0] = vTruth[0].primitive() + x0[0];
    xTruth[1] = vTruth[1].primitive() + x0[1];
}

void set_v_x_tubes(TubeVector& x, TubeVector& v, const TrajectoryVector& xTruth, const Trajectory& u, const Vector& x0, const Interval& tdomain, const double dt) {
    // Actual trajectories (state + derivative)
    RandTrajectory nTheta(tdomain, dt, Interval(-0.03, 0.03));
    v[2] = Tube(u + nTheta, dt).inflate(0.03);
    x[2] = Tube(xTruth[2] + nTheta, dt).inflate(0.03); 
    v[0] = 10 * cos(x[2]);
    v[1] = 10 * sin(x[2]);
    x[0] = v[0].primitive() + x0[0];
    x[1] = v[1].primitive() + x0[1];
}


Interval get_angle(IntervalVector robotPosition, IntervalVector markerPosition) {
    //return angle to an obstacle from robot position
    Interval angle = atan2( (markerPosition[1] - robotPosition[1]), (markerPosition[0] - robotPosition[0]) );
    angle -= robotPosition[2];
    return angle;
}

void range_bearing_SLAM () {
    //SLAM based on range and bearing, a posteriori result.


    srand((unsigned)time(NULL)); 
    const bool DISPLAY_PIES = false;    //display a pie pointing to observed obstacle

    double dt = 0.05;
    Interval tdomain(0, 15);


    vector<IntervalVector> landmarks = get_marker_positions();

    //ground truth trajectory
    Vector x0({0, 0, 2});
    Trajectory u = Trajectory(tdomain, TFunction("(3 * sqr(sin(t)) + t/100) + [-0.3,0.3]"), dt);


    // Actual trajectories (state + derivative)
    TrajectoryVector vTruth(3);
    TrajectoryVector xTruth(3);
    set_v_x_truth(xTruth, vTruth, u, x0);

    //real trajectories (tubes)
    TubeVector v(tdomain, dt, 3);
    TubeVector x(tdomain, dt, 3);   //for now, x is the position deduced from dead reckoning
    set_v_x_tubes(x, v, xTruth, u, x0, tdomain, dt);




    //Display
    vibes::beginDrawing();
    VIBesFigMap fig("Localization with date association");

    fig.set_properties(100, 100, 600, 300);
    fig.add_tube(&x, "tube x", 0, 1);
    fig.add_trajectory(&xTruth, "x*", 0, 1, "red");


    fig.smooth_tube_drawing(true);
    fig.show(0.5);


    //variables
    vector<IntervalVector> ulandmarks;  //new landmarks positions
    vector<int> landmarkViewCount;      //number of times a landmark was observed
    for(unsigned int i = 0; i < landmarks.size(); i++) {
        ulandmarks.push_back(IntervalVector(2));
        landmarkViewCount.push_back(0);

        //display beacons
        fig.add_beacon(landmarks[i], 0.1);
    }

    fig.smooth_tube_drawing(true);
    fig.show(0.5);


    //functions
    CtcFunction ctc_plus(Function("a", "b", "c", "a+b-c")); // a+b=c
    CtcFunction ctc_minus(Function("a", "b", "c", "a-b-c")); // a-b=c

    //Solver
    ContractorNetwork cn;
    cn.add(ctc::deriv, {x, v});

    for(double t = 0.0; t < tdomain.ub(); t += 2 * dt) {
        const unsigned int landmarkIndex = (rand()%4);    //random landmark
        landmarkViewCount[landmarkIndex] += 1;

        //get true robot and landmark position
        IntervalVector trueLandmarkPos = landmarks[landmarkIndex];
        IntervalVector trueRobotPosition = xTruth(t);
        //get distance and angle to landmark
        Interval dstVec = sqrt(sqr(trueRobotPosition[0] - trueLandmarkPos[0]) + sqr(trueRobotPosition[1] - trueLandmarkPos[1]));
        Interval markerAngle = get_angle(trueRobotPosition, trueLandmarkPos);
        //add uncertainty in measurements
        dstVec.inflate(0.03);
        markerAngle.inflate(0.03);

        if (DISPLAY_PIES) {     //display the view to the observed marker 
            fig.draw_pie(trueRobotPosition[0].mid(), trueRobotPosition[1].mid(), dstVec, markerAngle + trueRobotPosition[2], "black");
            fig.draw_pie(trueRobotPosition[0].mid(), trueRobotPosition[1].mid(), (Interval(0.01) | dstVec), markerAngle + trueRobotPosition[2], "gray");
        }

        //internal known variables
        Interval& angle = cn.create_dom(Interval(markerAngle));
        Interval& distance2Landmark = cn.create_dom(Interval(dstVec));

        //internal variables
        IntervalVector& robotPosition = cn.create_dom(IntervalVector(3));
        Interval& theta = cn.create_dom(Interval());
        IntervalVector& d = cn.create_dom(IntervalVector(2));

        //get robot position at time t
        cn.add(ctc::eval, {Interval(t), robotPosition, x, v});

        //polar coordinates
        cn.add(ctc_plus, {angle, robotPosition[2], theta});
        cn.add(ctc_minus, {ulandmarks[landmarkIndex][0], robotPosition[0], d[0]});
        cn.add(ctc_minus, {ulandmarks[landmarkIndex][1], robotPosition[1], d[1]});
        cn.add(ctc::polar, {d, distance2Landmark, theta});
    }
    cn.contract ();

    //display beacons positions deduced by the program
    for (unsigned int i = 0; i < landmarks.size(); i++) {
        fig.draw_box(ulandmarks[i]);
        std::cout << "Saw landmark " << i << " " << landmarkViewCount[i] << " times" << std::endl;
    }

    fig.show();

    vibes::endDrawing();
}


void range_bearing_SLAM_rt () {
    //SLAM based on range and bearing, with the robot position evaluated at each dt in real time.


    srand((unsigned)time(NULL)); 
    const bool DISPLAY_PIES = false;    //display a pie pointing to observed obstacle

    double dt = 0.05;
    double iterdt = 0.2;
    Interval tdomain(0, 15);


    vector<IntervalVector> landmarks = get_marker_positions();

    //ground truth trajectory
    Vector x0({0, 0, 2});
    Trajectory u = Trajectory(tdomain, TFunction("(3 * sqr(sin(t)) + t/100) + [-0.3,0.3]"), dt);


    // Actual trajectories (state + derivative)
    TrajectoryVector vTruth(3);
    TrajectoryVector xTruth(3);
    set_v_x_truth(xTruth, vTruth, u, x0);

    //real trajectories (tubes)
    TubeVector v(tdomain, dt, 3);
    TubeVector x(tdomain, dt, 3);
    set_v_x_tubes(x, v, xTruth, u, x0, tdomain, dt);




    //Display
    vibes::beginDrawing();
    VIBesFigMap fig("Localization with date association");

    fig.set_properties(100, 100, 600, 300);
    fig.add_tube(&x, "tube x", 0, 1);
    fig.add_trajectory(&xTruth, "x*", 0, 1, "red");


    fig.smooth_tube_drawing(true);
    fig.show(0.5);


    //variables
    vector<IntervalVector> ulandmarks;  //new landmarks positions
    vector<int> landmarkViewCount;      //number of times a landmark was observed
    for(unsigned int i = 0; i < landmarks.size(); i++) {
        ulandmarks.push_back(IntervalVector(2));
        landmarkViewCount.push_back(0);

        //display beacons
        fig.add_beacon(landmarks[i], 0.1);
    }

    //contractor functions
    CtcFunction ctc_plus(Function("a", "b", "c", "a+b-c")); // a+b=c
    CtcFunction ctc_minus(Function("a", "b", "c", "a-b-c")); // a-b=c

    //Solver
    ContractorNetwork cn;
    cn.add(ctc::deriv, {x, v});
    double prev_t_obs = tdomain.lb();
    for(double t = tdomain.lb() ; t < tdomain.ub() ; t += dt) {
        if(t - prev_t_obs > 2 * dt) { // new observation each 2*dt
            const unsigned int landmarkIndex = (rand()%4);    //random landmark
            landmarkViewCount[landmarkIndex] += 1;

            //get true robot and landmark position
            IntervalVector trueLandmarkPos = landmarks[landmarkIndex];
            IntervalVector trueRobotPosition = xTruth(t);
            //get distance and angle to landmark
            Interval dstVec = sqrt(sqr(trueRobotPosition[0] - trueLandmarkPos[0]) + sqr(trueRobotPosition[1] - trueLandmarkPos[1]));
            Interval markerAngle = get_angle(trueRobotPosition, trueLandmarkPos);
            //add uncertainty in measurements
            dstVec.inflate(0.03);
            markerAngle.inflate(0.03);

            if (DISPLAY_PIES) {     //display the view to the observed marker 
                fig.draw_pie(trueRobotPosition[0].mid(), trueRobotPosition[1].mid(), dstVec, markerAngle + trueRobotPosition[2], "black");
                fig.draw_pie(trueRobotPosition[0].mid(), trueRobotPosition[1].mid(), (Interval(0.01) | dstVec), markerAngle + trueRobotPosition[2], "gray");
            }

            //internal known variables
            Interval& angle = cn.create_dom(Interval(markerAngle));
            Interval& distance2Landmark = cn.create_dom(Interval(dstVec));

            //internal variables
            IntervalVector& robotPosition = cn.create_dom(IntervalVector(3));
            Interval& theta = cn.create_dom(Interval());
            IntervalVector& d = cn.create_dom(IntervalVector(2));

            //get robot position at time t
            cn.add(ctc::eval, {Interval(t), robotPosition, x, v});

            //polar coordinates
            cn.add(ctc_plus, {angle, robotPosition[2], theta});
            cn.add(ctc_minus, {ulandmarks[landmarkIndex][0], robotPosition[0], d[0]});
            cn.add(ctc_minus, {ulandmarks[landmarkIndex][1], robotPosition[1], d[1]});
            cn.add(ctc::polar, {d, distance2Landmark, theta});
        }

        double contraction_dt = cn.contract_during(iterdt);
        usleep(max(0.,iterdt - contraction_dt) * 1e6); // pause for the animation

        // Display the current robot position interval [x](t)
        fig.draw_box(x(max(0., ibex::previous_float(t))).subvector(0, 1));
    }

    //display beacons positions deduced by the program
    for (unsigned int i = 0; i < landmarks.size(); i++) {
        fig.draw_box(ulandmarks[i]);
        std::cout << "Saw landmark " << i << " " << landmarkViewCount[i] << " times" << std::endl;
    }

    fig.show();

    vibes::endDrawing();
}


int main() {
    range_bearing_SLAM();

    return 0;
}

