//
// Created by redwan on 10/15/20.
//

#ifndef LRAUV_MCTS_MYHELPER_H
#define LRAUV_MCTS_MYHELPER_H
#include "../src/MCTS.h"
#include <fstream>
#define GOAL_RADIUS 0.8
#define ROBOT_RADIUS (0.5)
#define GRID_SIZE (50)
#define GRID_RESOLUTION (0.5)
#define HASH(a)  a(0)*10 + a(1)*100

struct problem
{
    string flow_data;
    double initial_loc[2];
    double goal_loc[2];
};
unordered_map<int, int> ACTION_HASH;

problem parse_problem(const string& filename)
{
    ifstream inFile;
    inFile.open(filename);
    if (!inFile) {
        cout << "Unable to open file";
        exit(1); // terminate with error
    }
    vector<double> act{-0.5, 0, 0.5};
    auto A = availableActions(act, act);
    int count = 0;
    for (auto& a:A)
        ACTION_HASH[HASH(a)] = count++;

    problem prob;
    inFile >> prob.flow_data;
    inFile >> prob.initial_loc[0]; inFile>>prob.initial_loc[1];
    inFile >> prob.goal_loc[0]; inFile>>prob.goal_loc[1];
    return prob;
}



int state_indx(const mat& xEst, const mat&u)
{
    int x = xEst(0)/GRID_RESOLUTION;
    int y = xEst(1)/GRID_RESOLUTION;
    int a = ACTION_HASH[HASH(u)];
    int id = (x+ GRID_SIZE*y) << a;
    return id;
}





void draw_robot(const mat& xEst, double radius)
{
    vector<double> x, y, x1, y1;
    double radius1 = radius - 0.2;
    double q = 2*M_PI;
    do {
        x.push_back(xEst(0) + radius* cos(q));
        y.push_back(xEst(1) + radius* sin(q));
        x1.push_back(xEst(0) + radius1* cos(q));
        y1.push_back(xEst(1) + radius1* sin(q));
        q -= 0.1;
    }while (q>=0);
    plt::fill(x, y, {});
    plt::fill(x1, y1, {});
}


double fit_points(Traj pts, const mat& xEst)
{
    int nPoints = pts.x.size();
    if( nPoints < 2 ) {
        // Fail: infinitely many lines passing through this single point
        cerr<<"not enough points available" <<endl;
        return false;
    }
    double sumX=0, sumY=0, sumXY=0, sumX2=0;
    for(int i=0; i<nPoints; i++) {
        sumX +=     pts.x[i];
        sumY +=     pts.y[i];
        sumXY +=    pts.x[i] * pts.y[i];
        sumX2 +=    pts.x[i] * pts.x[i];
    }
    double xMean = sumX / nPoints;
    double yMean = sumY / nPoints;
    double denominator = sumX2 - sumX * xMean;
    // You can tune the eps (1e-7) below for your specific task
    if( std::fabs(denominator) < 1e-7 ) {
        // Fail: it seems a vertical line
        cerr<<" it seems a vertical line" <<endl;
        return false;
    }

    double slope = (sumXY - sumX * yMean) / denominator;
    double yInt = yMean - slope * xMean;
    auto f = [&](double x)
    {
        return slope*x + yInt;
    };

    vector<double>x{xEst(0), pts.x[nPoints-1]};
    vector<double>y{xEst(1), f(pts.x[nPoints-1])};
    return atan2(y[1]- y[0], x[1] - x[0]);


}


void draw_sensors(const mat& xEst, double radius,  const QNode& qnode, int best_node)
{
    double radius1 = radius - 0.2;
    double radius2 = radius + 1.0;
    for(auto it = qnode.begin(); it != qnode.end(); ++it)
    {
        double q = fit_points( it->second, xEst);
        vector<double> x {xEst(0)+ radius1*cos(q), xEst(0)+ radius2*cos(q) };
        vector<double> y {xEst(1)+ radius1*sin(q), xEst(1)+ radius2*sin(q) };
        if(it->first == best_node)
            plt::plot(x, y, "--b");
        else
            plt::plot(x, y, "--r");
    }

}
Traj track;
void sim_update(const mat& FxEst, const ActionValue& action, const vec2& landmark,  FlowField& field)
{
    plt::cla();
    draw_robot(landmark, 1);
    draw_robot(FxEst, ROBOT_RADIUS);
    draw_sensors(FxEst, GOAL_RADIUS, action.qnode, action.best_index);
    track.x.push_back(FxEst(0));
    track.y.push_back(FxEst(1));

    field.plot();
//    plt::axis("equal");
    plt::xlim(-2, 25);
    plt::ylim(-2, 30);

    plt::plot(track.x, track.y, "b");
    plt::pause(0.01);


}

#endif //LRAUV_MCTS_MYHELPER_H
