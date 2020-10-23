//
// Created by redwan on 10/14/20.
//

#ifndef LRAUV_MCTS_MCTS_H
#define LRAUV_MCTS_MCTS_H
#define _USE_MATH_DEFINES
#include "FlowField.h"
#include "EKF.h"
#include <unordered_map>
#include <stack>
#include <queue>
#include <cmath>
#include <memory>
#include <numeric>
#include "TreeNode.h"
using namespace std;

inline double gauss_likelihood(double x, double sigma)
{
    return 1.0/sqrt(2.0 * M_PI * pow(sigma,2) ) *
           exp(- pow(x, 2)/ (2 * pow(sigma, 2)));
}

inline vector<vec2> availableActions(const vector<double> &a, const vector<double>& b)
{
    vector<vec2>result;
    for (int i = 0; i < a.size(); ++i) {
        for (int j = 0; j < b.size(); ++j)
        {
            vec2 u;
            u(0) = a[i]; u(1) = b[j];
            result.push_back(u);
        }
    }
    return result;
}

struct Traj
{
    std::vector<double> x, y;
};



typedef unordered_map<int, Traj> QNode;

struct ActionValue
{
    vec2 u;
    double reward;
    int best_index;
    QNode qnode;
};



inline double fit_points(Traj pts, const mat& xEst)
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



class MCTS {
public:
    MCTS(const string& filename);
    Traj Simulate(const mat& xEst, const mat& PEst, const vec2& u, int K, FlowField& field);
    NodePtr Search(const vec2& goal, const mat &xEst, const mat &PEst, int timeout, NodePtr tree = nullptr);
    NodePtr Select(NodePtr root, State s);
    NodePtr Expand(NodePtr root, const State& s);
    NodePtr Backpropagate(NodePtr root);

    bool find(NodePtr root, State s);

private:
    FlowField field_;
    int sample_time_;
    vector<vec2>action_set_;

};


#endif //LRAUV_MCTS_MCTS_H
