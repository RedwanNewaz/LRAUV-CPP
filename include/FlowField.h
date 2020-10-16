//
// Created by robor on 10/6/2020.
//

#ifndef LRAUV_FLOWFIELD_H
#define LRAUV_FLOWFIELD_H
#include <iostream>
#include "matplotlibcpp.h"
#include <armadillo>
#include <fstream>
#include <string>
#include <cassert>

using namespace std;
using namespace arma;
namespace plt = matplotlibcpp;

class FlowField
{
public:
    FlowField(const string& filename)
    {
        std::ifstream file(filename);
        UU = load_mat<double>(file, "UU");
        VV = load_mat<double>(file, "VV");
        assert(UU.n_cols == VV.n_cols);
        assert(UU.n_rows == VV.n_rows);
//        ku = 0.2;
//        kw = 0.2;
        ku = 0.1;
        kw = 0.1;
    }

    void set(double ku, double kw)
    {
        this->ku = ku;
        this->kw = kw;
    }

    vec2 at(int x, int y )
    {
//        x += 7; y += 2;
        auto F = lookup(x, y);
        vec U(2);
        // map flow force to external disturbance
        U(0) = ku*tanh(x*x + y*y);
        U(1) = kw*atan2(F(1), F(0));
        return U;
    }

    void plot()
    {
        // u and v are respectively the x and y components of the arrows we're plotting
        std::vector<double> x, y, u, v;
        for (int i = 0; i < UU.n_rows; i++) {
            for (int j = 0; j < UU.n_cols; j++) {
                x.push_back(i);
                u.push_back(UU(i, j));
                y.push_back(j);
                v.push_back(VV(i, j));
            }
        }

        plt::quiver(x, y, u, v);
//        plt::show();
    }

private:
    Mat<double> UU, VV;
    double ku, kw;

protected:
    mat lookup(int x, int y)
    {
        vec F(2, fill::zeros);
        if( x < UU.n_rows && y < UU.n_cols  )
        {
            F(0) = UU(x, y);
            F(1) = VV(x, y);
        }
        return F;
    }
// Extract the data as an Armadillo matrix Mat of type T, if there is no data the matrix will be empty
    template<typename T>
    arma::Mat<T> load_mat(std::ifstream &file, const std::string &keyword) {
        std::string line;
        std::stringstream ss;
        bool process_data = false;
        bool has_data = false;
        while (std::getline(file, line)) {
            if (line.find(keyword) != std::string::npos) {
                process_data = !process_data;
                if (process_data == false) break;
                continue;
            }
            if (process_data) {
                ss << line << '\n';
                has_data = true;
            }
        }

        arma::Mat<T> val;
        if (has_data) {
            val.load(ss);
        }
        return val;
    }


};
#endif //LRAUV_FLOWFIELD_H
