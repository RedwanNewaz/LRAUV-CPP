//
// Created by redwan on 10/14/20.
//

#include "MCTS.h"

MCTS::MCTS(const string &filename):field_(filename) {

}


Traj MCTS::simulate(const mat &xEst, const mat &PEst, const vec2 &u, int K, FlowField &field) {
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

ActionValue MCTS::Search(const vec2& goal, const mat &xEst, const mat &PEst, int timeout) {

    vector<double>Rewards;
    int count = 0;
    vector<double> a {-0.5, 0.0, 0.5};
    auto A = availableActions(a, a);
    QNode qnode_;
    for (auto& u: A)
    {
        auto traj = simulate(xEst, PEst, u, timeout, field_);
        size_t NP = traj.x.size();
        vector<double> probs = vector<double>(NP, 1.0/double(NP));
        for (int i = 0; i < NP; ++i) {
            vec2 z{traj.x[i], traj.y[i]};
            auto dz = norm(goal - z, 2);
            probs[i] *= gauss_likelihood(dz, 0.5);
        }
        double reward = std::accumulate(probs.begin(), probs.end(), 0.0);
        Rewards.push_back(reward);
        qnode_[count] = traj;

        //        cout<<"[reward]: "<< reward << endl;
//        string name = "a"+to_string(count);
//        plt::named_plot(name, traj.x, traj.y);
        ++count;
    }

    // select best action
    double total_reward = std::accumulate(Rewards.begin(), Rewards.end(), 0.0);
    transform(Rewards.begin(), Rewards.end(), Rewards.begin(), [&](double val){return val/total_reward;});
    double max_reward = Rewards[0];
    int best_action = 0;
    for (int j = 0; j < A.size(); ++j) {

        if(Rewards[j] > max_reward)
        {
            best_action = j;
            max_reward = Rewards[j];
        }
    }
    cout<<"[a"<<best_action<<"] = "<< Rewards[best_action] << endl;
    return {A[best_action], max_reward, best_action, qnode_};
}

