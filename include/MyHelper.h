//
// Created by redwan on 10/15/20.
//

#ifndef LRAUV_MCTS_MYHELPER_H
#define LRAUV_MCTS_MYHELPER_H
#include "../src/MCTS.h"
#define GOAL_RADIUS 0.8
#define ROBOT_RADIUS (0.5)

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

void sim_update(const mat& FxEst, const ActionValue& action, const vec2& landmark,  FlowField& field)
{
    plt::cla();
    draw_robot(FxEst, ROBOT_RADIUS);
    draw_sensors(FxEst, GOAL_RADIUS, action.qnode, action.best_index);
    draw_robot(landmark, 1);
    field.plot();
//    plt::axis("equal");
    plt::xlim(-1, 25);
    plt::ylim(0, 30);
    plt::pause(0.01);

}

#endif //LRAUV_MCTS_MYHELPER_H
