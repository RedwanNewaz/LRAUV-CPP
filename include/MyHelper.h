//
// Created by redwan on 10/15/20.
//

#ifndef LRAUV_MCTS_MYHELPER_H
#define LRAUV_MCTS_MYHELPER_H
#include "../src/MCTS.h"





struct problem
{
    string flow_data;
    double initial_loc[2];
    double goal_loc[2];
    string back_img;
};


problem parse_problem(const string& filename)
{
    ifstream inFile;
    inFile.open(filename);
    if (!inFile) {
        cerr << "Unable to open file";
        exit(1); // terminate with error
    }

    problem prob;
    inFile >> prob.flow_data;
    inFile >> prob.initial_loc[0]; inFile>>prob.initial_loc[1];
    inFile >> prob.goal_loc[0]; inFile>>prob.goal_loc[1];
    inFile >> prob.back_img;
    return prob;
}



#endif //LRAUV_MCTS_MYHELPER_H
