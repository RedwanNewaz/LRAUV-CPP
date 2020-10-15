#include "FlowField.h"
#include "EKF.h"


int main()
{
    mat xEst(4, 1, fill::zeros);
    mat PEst(4, 4, fill::zeros);
    mat FxEst(4, 1, fill::zeros);
    mat FPEst(4, 4, fill::zeros);
    mat GxEst(4, 1, fill::zeros);

    vec2 u;
    u(0) = 0.50;
    u(1) = 0.1;
    EKF ekf, Fekf;
    std::vector<double> x, y, Fx, Fy, Gx, Gy;
    FlowField field("../data/UUVV01");

    int count = 700;
    do {
        plt::cla();
        auto z = ekf.measurement(xEst, u);
        auto ud = ekf.control_input(u);
        tie(xEst, PEst) = ekf.estimation(xEst, PEst, z, ud);

        auto Fz = Fekf.measurement(xEst, u);
        auto uF = field.at(FxEst(0)+7, FxEst(1)+2);
        tie(FxEst, FPEst) = Fekf.estimation(FxEst, FPEst, z, ud+uF);

        mat GPEst(4, 4, fill::zeros);
        tie(GxEst, GPEst) = Fekf.estimation(GxEst, GPEst, z, u);


        x.push_back(xEst(0)+7);
        y.push_back(xEst(1)+2);

        Fx.push_back(FxEst(0)+7);
        Fy.push_back(FxEst(1)+2);

        Gx.push_back(GxEst(0)+7);
        Gy.push_back(GxEst(1)+2);


        plt::named_plot("Uncertainty Awareness",x, y);
        plt::named_plot("(Uncertainty + Energy) Awareness",Fx, Fy);
        plt::named_plot("Desire",Gx, Gy);
        plt::legend();
        field.plot();
        plt::pause(0.0001);

    }while (--count>0);
    plt::show();

    return 0;
}