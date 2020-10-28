//
// Created by robor on 10/23/2020.
//

#ifndef LRAUV_MCTS_SIMULATIONVIEW_H
#define LRAUV_MCTS_SIMULATIONVIEW_H
#include <iostream>
#include <armadillo>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/types.hpp>
#include "../src/TreeNode.h"
#include "DynamicObstacles.h"
#define WINDOW_HEIGHT (800)
#define WINDOW_WIDTH (800)

using namespace std;
using namespace arma;
using namespace cv;
static vector<cv::Point2f > traj;
class SimultionView
{
public:
    SimultionView(cv::Mat background, double xlim, double ylim):xlim(xlim), ylim(ylim), backgroundImg(background)
    {
        cv::resize(backgroundImg, backgroundImg, cv::Size(WINDOW_WIDTH, WINDOW_HEIGHT), 0, 0, cv::INTER_LINEAR);

    }
    void draw_circle(const mat& point, double radius,  const Scalar& color)
    {
        cv::circle(backgroundImg, transform(point), radius, color,  FILLED);
    }

    void draw_circle(const mat& point,  const Scalar& color)
    {
        center = transform(point);
        robot_(0) = point(0); robot_(1) = point(1);
        traj.push_back(center);
        cv::circle(backgroundImg, center, robot_radius, color,  FILLED);

    }

    void operator()(NodePtr node)
    {
        cv::Scalar colorBlue(0,255, 0);
        cv::Scalar colorGreen(0,100,0);
        cv::Scalar colorYellow(0,255,255);
        cv::Scalar colorRed(0,0,255);

        //show old trajectory
        if(traj.size()>1)
        for (int i = 0; i < traj.size() - 1; ++i) {
            cv::line(backgroundImg, traj[i], traj[i+1], colorBlue, 2 );
        }

        // draw goal
        draw_circle(node->state.landmark, 30, colorGreen);
        // draw robot
        draw_circle(node->state.xEst, colorYellow);
        draw_circle(node->state.xEst, 12, colorRed);

        // draw sensors
        double radius1(robot_radius - 11), radius2(robot_radius + 20);
        if(node->parent)
        for(auto& child: node->parent->children)
        {
            double q = child->dirAngle;
            cv::Point2f x(center.x+ radius1*cos(q), center.y+ radius1*sin(q) );
            cv::Point2f y (center.x+ radius2*cos(q), center.y+ radius2*sin(q) );
            if(child == node)
                cv::line(backgroundImg, x, y, colorBlue );
            else
                cv::line(backgroundImg, x, y, colorRed );
        }
    }

    void show()
    {
        cv::imshow("pic", backgroundImg);
        cv::waitKey(10);
    }

    cv::Mat get_frame()
    {
        return backgroundImg;
    }

    void show_obstacles(const vector<DynamicObstacle>& obstacles)
    {
        cv::Scalar colorBlue(255,0, 0);
        cv::Scalar colorPink(147,20, 255);
        for (auto obs:obstacles)
        {
            vec2 o = obs.get();
            auto dist = norm(o-robot_, 2);
            if(dist<=SENSING_RADIUS)
                draw_circle(obs.get(), 10, colorPink);
            else
                draw_circle(obs.get(), 10, colorBlue);
        }
    }


private:
    double xlim, ylim;
    cv::Mat backgroundImg;
    cv::Point2f center;
    vec2 robot_;
    const double robot_radius = 20;


protected:
    cv::Point2f transform( const mat& point)
    {
        Point2f p;
        p.x = WINDOW_WIDTH*(point(0)/xlim);
        p.y = WINDOW_HEIGHT-WINDOW_HEIGHT*(point(1)/ylim);
        return p;
    }


};

#endif //LRAUV_MCTS_SIMULATIONVIEW_H
