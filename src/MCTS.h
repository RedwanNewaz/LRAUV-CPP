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
#include "DynamicObstacles.h"
#include "SimulationView.h"
using namespace std;
static double lambda = 1.20; //1.20;
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

inline tuple<vector<vec2>, vector<int> >sense_obstacles(const vector<DynamicObstacle>& obstacles, const State& s)
{
    vec2 robot;
    robot(0) = s.xEst(0); robot(1) = s.xEst(1);
    double ANGLE_RES = 2.0 * M_PI / 8.0;
    vector<vec2> result;
    vector<int> angle_indices;
    for(const auto o:obstacles)
    {
        auto obs = o.get();
        vec2 delta = obs -robot;
        auto dist = norm(delta, 2);
        if(dist <= SENSING_RADIUS)
        {
            auto q = M_PI + atan2(delta(1), delta(0));
            int index = q/ANGLE_RES;
            result.push_back(obs);
            angle_indices.push_back(index);
        }
    }
    return make_tuple(result, angle_indices);
}

class MCTS {
public:
    MCTS(const string& filename, const string& background, int num_obstacles);
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
    vec4 bb_;
    vector<DynamicObstacle> obstacles_;
    vector<int> sensors_;
    cv::Mat background_;

};


#endif //LRAUV_MCTS_MCTS_H
