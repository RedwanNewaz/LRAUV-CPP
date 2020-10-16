//
// Created by redwan on 10/14/20.
//

#ifndef LRAUV_MCTS_MCTS_H
#define LRAUV_MCTS_MCTS_H
#include "FlowField.h"
#include "EKF.h"
#include <unordered_map>
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
            result.push_back({a[i], b[j]});
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

class MCTS {
public:
    MCTS(const string& filename);
    Traj simulate(const mat& xEst, const mat& PEst, const vec2& u, int K, FlowField& field);
    ActionValue Search(const vec2& goal, const mat &xEst, const mat &PEst, int timeout);

private:
    FlowField field_;

};


#endif //LRAUV_MCTS_MCTS_H
