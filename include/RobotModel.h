//
// Created by robor on 10/7/2020.
//

#ifndef LRAUV_ROBOTMODEL_H
#define LRAUV_ROBOTMODEL_H
#include <iostream>
#include <armadillo>
using namespace std;
using namespace arma;
class RobotModel{
public:
//    TODO set GPS_NOSE, set INPUT_NOISE
    RobotModel()
    {

    }

    mat measurement(const mat& xEst, const mat& u)
    {
        mat GPS_NOISE(2, 2, fill::eye);
        mat OBS_NOISE(2, 1, fill::randn);
        GPS_NOISE = GPS_NOISE * pow(0.5, 2);
        GPS_NOISE = GPS_NOISE * OBS_NOISE;

        //    add noise to gps x-y
        auto xTrue = motion_model(xEst, u);
        auto z = observation_model(xTrue);
        z = z + GPS_NOISE;
        return z;
    }

    static mat control_input(const mat& u)
    {

        mat INPUT_NOISE(2, 2, fill::eye);
        mat TRANS_NOISE(2, 1, fill::randn);
        INPUT_NOISE = INPUT_NOISE * pow(0.523599, 2);
        INPUT_NOISE(0, 0) = 1.0;
        INPUT_NOISE = INPUT_NOISE * TRANS_NOISE;
        mat ud = u + INPUT_NOISE;
        return ud;
    }

    mat transition(const mat& xEst, const mat& u )
    {
        mat xPred = motion_model(xEst, u);
        return xPred;
    }


    [[nodiscard]] mat motion_model(const mat& x, const mat& u ) const
    {
        mat F(4,4,fill::eye);
        F(3,3) = 0;
        mat B(4, 2, fill::zeros);
        B(0, 0) = DT*cos(x(2, 0));
        B(1, 0) = DT*sin(x(2, 0));
        B(2, 1) = DT;
        B(3, 0) = 1.0;
        return F*x + B*u;
    }

    static mat observation_model(const mat& x)
    {
        mat H(2, 4, fill::zeros);
        H(0, 0) = 1;
        H(1, 1) = 1;
        return H*x;
    }


    const double DT = 0.1;
};
#endif //LRAUV_ROBOTMODEL_H
