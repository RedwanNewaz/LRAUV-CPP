#include <iostream>
#include "include/FlowField.h"
#include "include/RobotModel.h"
#include <despot/planner.h>
#include "lauv/LAVU.h"
void demo()
{
    char *filename = "../data/UUVV61";
    FlowField flowField(filename);

    RobotModel robot;
    mat xEst(4, 1, fill::zeros);
    mat u(2, 1);
    u(0,0) = 1.0;
    u(1, 0) = 0.1;
    int count = 0;
    do
    {
        auto z = robot.measurement(xEst, u);
        auto ud = RobotModel::control_input(u);
        auto uF = flowField.at(xEst(0),xEst(1));
        ud += uF;
        xEst = robot.transition(xEst, ud);
        cout<<trans(z) << trans(uF) <<endl;
    }while (++count<100);


}

using namespace despot;

class MyPlanner: public Planner {
public:
    MyPlanner() {
    }

    DSPOMDP* InitializeModel(option::Option* options) {
        DSPOMDP* model = NULL;
        model = new LAVU();
        return model;
    }

    World* InitializeWorld(std::string&  world_type, DSPOMDP* model, option::Option* options)
    {

        return InitializePOMDPWorld(world_type, model, options);
    }

    void InitializeDefaultParameters() {

    }

    std::string ChooseSolver(){
        return "DESPOT";
    }

    bool RunStep(Solver *solver, World *world, Logger *logger) {


        bool isTerminate = Planner::RunStep(solver, world, logger);


        return isTerminate;
    }
};

int main(int argc, char* argv[]) {
    return MyPlanner().RunEvaluation(argc, argv);
}
