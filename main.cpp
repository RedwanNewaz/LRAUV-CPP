#include "MyHelper.h"




int main()
{
    FlowField field("../data/UUVV01");
    mat xEst(4, 1, fill::zeros);
    mat PEst(4, 4, fill::zeros);
    EKF ekf;
    xEst(0) = 0;
    xEst(1) = 12;
    int sample_time = 50;

    vec2 landmark{12.50, 20.0};
    MCTS mcts("../data/UUVV01");


    double dist = 100;
    do{
        auto action = mcts.Search(landmark, xEst, PEst, sample_time);

        sim_update(xEst, action, landmark, field);

        auto u = action.u;
        auto z = ekf.measurement(xEst, u);
        auto ud = ekf.control_input(u);
        tie(xEst, PEst) = ekf.estimation(xEst, PEst, z, ud);

        vec2 current{xEst(0), xEst(1)};
        dist = norm(current-landmark, 2);

    }while (dist>1);

    
    plt::show();

    return 0;
}