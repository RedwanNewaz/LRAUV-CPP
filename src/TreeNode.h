//
// Created by redwan on 10/18/20.
//

#ifndef LRAUV_MCTS_TREENODE_H
#define LRAUV_MCTS_TREENODE_H

#include <iostream>
#include <memory>
#include <vector>
#include <armadillo>
#include <algorithm>
#include <assert.h>


#define GRID_SIZE (50)
#define GRID_RESOLUTION (0.05)

using namespace std;
using namespace arma;
struct State
{
    mat xEst, PEst;
    vec2 landmark;
    vec2 current;
    void update( const vec2& u, int index, const vector<int>&sensors)
    {
        auto z = ekf_.measurement(xEst, u);
        auto ud = ekf_.control_input(u);
        tie(xEst, PEst) = ekf_.estimation(xEst, PEst, z, ud);

        int x = xEst(0)/GRID_RESOLUTION;
        int y = xEst(1)/GRID_RESOLUTION;
        state_id_ = (x+ GRID_SIZE*y) << index;
        for (auto &s: sensors)
            state_id_ = state_id_ << s;
        a_id_ = index;
        current(0)=xEst(0); current(1)=xEst(1);
    }
    bool isTerminal()
    {


        auto dist = norm(current-landmark, 2);
        return dist <= 1;
    }

    int get_id()
    {
        return state_id_;
    }


private:
    EKF ekf_;
    long a_id_, state_id_;
};


class TreeNode;
typedef shared_ptr<TreeNode> NodePtr;

class TreeNode: public enable_shared_from_this<TreeNode>
{
public:
    TreeNode(State state, int action, NodePtr parent = nullptr):state(state), parent(parent)
    {
        numVists = 0;
        isTerminal = false;
        totalReward = 0;
        selected_action = action;
        dirAngle = 0;

    }
    NodePtr getPtr()
    {
        return shared_from_this();
    }



    vector<NodePtr> children;
    NodePtr parent;
    int numVists;
    double dirAngle;
    bool isTerminal;
    double totalReward;
    State state;
    int selected_action;

};

#endif //LRAUV_MCTS_TREENODE_H
