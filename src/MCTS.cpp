//
// Created by redwan on 10/14/20.
//

#include "MCTS.h"

#define debug(x) std::cout<<x<<std::endl

MCTS::MCTS(const string &filename):field_(filename) {
    vector<double> a {-0.5, 0.0, 0.5};
    action_set_ = availableActions(a, a);
}


Traj MCTS::Simulate(const mat &xEst, const mat &PEst, const vec2 &u, int K, FlowField &field) {
    EKF Fekf;
    auto FxEst = const_cast<mat&>(xEst);
    auto FPEst = const_cast<mat&>(PEst);
    Traj traj;
    do {
        auto Fz = Fekf.measurement(FxEst, u);
        auto ud = Fekf.control_input(u);
        auto uF = field.at(FxEst(0), FxEst(1));
        auto z = Fekf.measurement(FxEst, u);
        tie(FxEst, FPEst) = Fekf.estimation(FxEst, FPEst, z, ud+uF);
        traj.x.push_back(FxEst(0));
        traj.y.push_back(FxEst(1));
    }while (--K>0);
    return traj;
}

NodePtr MCTS::Search(const vec2& goal, const mat &xEst, const mat &PEst, int timeout, NodePtr tree) {

    sample_time_ = timeout;
    State s;
    s.xEst = xEst;
    s.PEst = PEst;
    s.landmark = goal;
//    for (int i = 0; i < action_set_.size(); ++i) {
//        cout<<i <<" "<< action_set_[i] << endl;
//    }
    int action_index = 4;

    NodePtr root;
    if(tree)root = tree;
    else root = std::make_shared<TreeNode>(s, action_index, nullptr);
    root->numVists += 1;
    root = Select(root, s);
    debug("[terminate ] "<< root->isTerminal);
    root = Backpropagate(root);
    return root;

}

NodePtr MCTS::Select(NodePtr root, State s) {

    double dist = 100;
    int loop = 0;
    do {
        if(!find(root, s))
        {
//            debug("expanding node " << loop++);
            root = Expand(root, s);
            // tree modification
            root->isTerminal = (dist>1);
        }
        else
        {
            loop++;
//            debug("reusing the node " << loop);
        }
        // state update
        int best_index = root->selected_action;
        s.update(action_set_[best_index], best_index);
        root->numVists += 1;


    }while (!s.isTerminal());
    debug("[reused node ] "<< loop);
    return root;
}

NodePtr MCTS::Expand(NodePtr root, const State& s) {

    vector<double>Rewards, Angles;
    int action_index = 0;
    QNode qnode_;

    for (auto& u: action_set_)
    {
        auto node = std::make_shared<TreeNode>(s, action_index, root);
        root->children.push_back(node->getPtr());

        auto traj = Simulate(s.xEst, s.PEst, u, sample_time_, field_);
        size_t NP = traj.x.size();
        vector<double> probs = vector<double>(NP, 1.0/double(NP));
        for (int i = 0; i < NP; ++i) {
            vec2 z;
            z(0) = traj.x[i]; z(1) = traj.y[i];
            auto dz = norm(s.landmark - z, 2);
            probs[i] *= gauss_likelihood(dz, 0.5);
        }
        double reward = std::accumulate(probs.begin(), probs.end(), 0.0);
        Rewards.push_back(reward);
        qnode_[action_index++] = traj;

        double angle = fit_points(traj, s.xEst);
        Angles.push_back(angle);
    }

    double total_reward = std::accumulate(Rewards.begin(), Rewards.end(), 0.0);
    transform(Rewards.begin(), Rewards.end(), Rewards.begin(), [&](double val){return val/total_reward;});
    double max_reward = 0;
    int best_action = 0;
//    greedy search is better than UCT search
//    double explorationValue = 0.07;
    for (int j = 0; j < action_set_.size(); ++j) {
        root->children[j]->totalReward = Rewards[j];
        root->children[j]->dirAngle = Angles[j];
//        double nodeValue = Rewards[j] / root->children[j]->numVists + explorationValue * sqrt(
//                2 * log(root->numVists) / root->children[j]->numVists);
//        if(nodeValue > max_reward)
        if(Rewards[j] > max_reward)
        {
            best_action = j;
            max_reward = Rewards[j];
//            max_reward = nodeValue;
        }
    }
    return root->children[best_action];
}

NodePtr MCTS::Backpropagate(NodePtr root) {
    stack<NodePtr> tree;
    double total = 0;
    int count = 0;
    while(root)
    {
        total +=root->totalReward/ root->numVists;
        root->totalReward = total;
        tree.push(root);
        root = root->parent;
        ++count;
    }
    cout << "[Avg reward] " << total/count  << " [total reward] " << total << " [len]: "<< count<<endl;
    return tree.top();
}

bool MCTS::find(NodePtr root, State s) {

    if(! root)return false;
    NodePtr new_node = root;
    stack<NodePtr> tree;
    while (new_node)
    {
        if(new_node->state.get_id() == s.get_id())
        {
            root =  new_node;
            return true;
        }

        tree.push(new_node);
        new_node = new_node->parent;
    }
    queue<NodePtr> q;
    q.push(tree.top());
    while (!q.empty())
    {
        new_node = q.front();
        q.pop();
        if(new_node->state.get_id() == s.get_id())
        {
            root =  new_node;
            return true;
        }

        for(auto& child: new_node->children)
        {
            q.push(child);
        }
    }
    return false;
}