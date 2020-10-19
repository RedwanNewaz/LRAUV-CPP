//
// Created by redwan on 10/15/20.
//

#ifndef LRAUV_MCTS_MYHELPER_H
#define LRAUV_MCTS_MYHELPER_H
#include "../src/MCTS.h"
#define GOAL_RADIUS 0.8
#define ROBOT_RADIUS (0.5)
#define MAKE_VIDEO


struct problem
{
    string flow_data;
    double initial_loc[2];
    double goal_loc[2];
};


problem parse_problem(const string& filename)
{
    ifstream inFile;
    inFile.open(filename);
    if (!inFile) {
        cout << "Unable to open file";
        exit(1); // terminate with error
    }

    problem prob;
    inFile >> prob.flow_data;
    inFile >> prob.initial_loc[0]; inFile>>prob.initial_loc[1];
    inFile >> prob.goal_loc[0]; inFile>>prob.goal_loc[1];
    return prob;
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
int index_num = 0;
void sim_view(NodePtr node,  FlowField& field)
{
    auto FxEst = node->state.xEst;
    auto landmark = node->state.landmark;
    auto show_sesnors = [&]()
    {
        double radius1 = ROBOT_RADIUS - 0.2;
        double radius2 = ROBOT_RADIUS + 1.0;
        if(! node->parent) return ;
        for(auto& child: node->parent->children)
        {
            double q = child->dirAngle;
            vector<double> x {FxEst(0)+ radius1*cos(q), FxEst(0)+ radius2*cos(q) };
            vector<double> y {FxEst(1)+ radius1*sin(q), FxEst(1)+ radius2*sin(q) };
            if(child == node)
                plt::plot(x, y, "--b");
            else
                plt::plot(x, y, "--r");
        }
    };


    plt::cla();
    draw_robot(landmark, 1);
    draw_robot(FxEst, ROBOT_RADIUS);
    show_sesnors();
    track.x.push_back(FxEst(0));
    track.y.push_back(FxEst(1));

    field.plot();

//    plt::xlim(-2, 42 + 11);
//    plt::ylim(-2, 21 + 11);
    plt::xlim(-2, 22 + 11);
    plt::ylim(-2, 15 + 11);
    plt::axis("off");
//    plt::axis("square");
    plt::plot(track.x, track.y, "b");
    plt::pause(0.01);

#ifdef MAKE_VIDEO
    char* pad = "%04d";
    char s[100];
    sprintf(s, pad, atoi(to_string(index_num++).c_str()));
    auto name = "../result/video/im"+string(s)+".png";
    plt::save(name);
#endif

}

#endif //LRAUV_MCTS_MYHELPER_H
