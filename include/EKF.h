//
// Created by robor on 10/7/2020.
//

#ifndef LRAUV_EKF_H
#define LRAUV_EKF_H
#include "RobotModel.h"
class EKF : public RobotModel{
public:
    EKF()
    {

    }

    tuple<mat, mat> estimation(mat xEst, mat PEst, mat z, mat u)
    {
        mat Q(4, 4, fill::eye);
        mat R(2, 2, fill::eye);
        Q(0, 0) = Q(1, 1)= 0.1;
        Q(2, 2) = 0.0174533;

        R = pow(R, 2);
        Q = pow(Q, 2);

//        predict
        mat xPred = motion_model(xEst, u);
        mat jF = jacob_f(xEst, u);
        mat PPred = jF * PEst;
        PPred = PPred * trans(jF);
        PPred = PPred + Q;

//        update
        mat jH = jacob_h();
        mat zPred = observation_model(xPred);
        mat y = z - zPred;
        mat jHT = trans(jH);
        mat S = jH * PPred;
        S *= jHT;
        S += R;

        mat K = PPred * jHT;
        K *= inv(S);
        mat K1 = K * y;

        xEst = xPred + K1;
        mat I (4, 4, fill::eye);
        K *= jH;
        I -= K;
        PEst = I * PPred;
        return make_tuple(xEst, PEst);

    }



protected:
    /*
*     Jacobian of Motion Model

      motion model
      x_{t+1} = x_t+v*dt*cos(yaw)
      y_{t+1} = y_t+v*dt*sin(yaw)
      yaw_{t+1} = yaw_t+omega*dt
      v_{t+1} = v{t}
      so
      dx/dyaw = -v*dt*sin(yaw)
      dx/dv = dt*cos(yaw)
      dy/dyaw = v*dt*cos(yaw)
      dy/dv = dt*sin(yaw)
*/
    mat jacob_f(mat x, mat u)
    {
        mat jF(4,4,fill::eye);
        double yaw = x(2, 0);
        double v = u(0, 0);
        jF(0, 2) = -DT * v * sin(yaw);
        jF(0, 3) = DT * cos(yaw);
        jF(1, 2) = DT * v * cos(yaw);
        jF(1, 2) = DT * sin(yaw);
        return jF;
    }
    mat jacob_h()
    {
        mat jH(2, 4, fill::zeros);
        jH(0, 0) = 1;
        jH(1, 1) = 1;
        return jH;
    }


};

#endif //LRAUV_EKF_H
