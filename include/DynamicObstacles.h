//
// Created by redwan on 10/21/20.
//

#ifndef LRAUV_MCTS_DYNAMICOBSTACLES_H
#define LRAUV_MCTS_DYNAMICOBSTACLES_H
#include <iostream>
#include <vector>
#include <armadillo>
#define SENSING_RADIUS (7)

using namespace arma;
using namespace std;
const double RADIUS = 0.5;
class DynamicObstacle{
public:
    DynamicObstacle(const vec2& p)
    {
        position = p;
        velocity(0) = 0.05 * (randu() - 0.5);
        velocity(1) = 0.05 * (randu() - 0.5);
        this->radius = RADIUS;
    }
    vec2 rotate(const vec2& v, double angle){
        vec2 u;
        u(0) = v(0) * cos(angle) - v(1)*sin(angle);
        u(1) = v(0) * sin(angle) + v(1)*cos(angle);
        return u;
    }
    void ressolve_collision(DynamicObstacle& other)
    {
        vec2 a, d;
        a(0) = velocity(0) - other.velocity(0);
        a(1) = velocity(1) - other.velocity(1);
        d = other.position - position;
        if(a(0)* d(0) + a(1)* d(1) >= 0)
        {
            double angle = atan2(d(1), d(0));
            auto u1 = rotate(velocity, angle);
            auto u2 = rotate(other.velocity, angle);
            velocity = rotate({u2(0), u1(0)}, -angle);
            other.velocity = rotate({u1(0), u2(1)}, -angle);
        }
    }
    double operator -(const DynamicObstacle& other)
    {
        return norm(other.position - position, 2);
    }

    bool operator == (const DynamicObstacle& other)
    {
        return (position(0) == other.position(0)) && (position(1) == other.position(1));
    }
    bool operator != (const DynamicObstacle& other)
    {
        return not (*this == other);
    }

    void update(vector<DynamicObstacle>& obstacles, const vec4& bb)
    {
        for(auto& o: obstacles)
        {
            if( o == *this) continue;
            if( (*this - o) < 2*radius) ressolve_collision(o);
        }

        if( (position(0) - radius) <= bb(0) || (position(0) + radius) > bb(1) )
            this->velocity(0) *= -1;
        if( (position(1) - radius) <= bb(2) || (position(1) + radius) > bb(3) )
            this->velocity(1) *= -1;
        position(0) += velocity(0);
        position(1) += velocity(1);

    }
    vec2 get()const
    {
        return position;
    }




private:
    double radius;
    double color;
    vec2 position;
    vec2 velocity;
};

inline void random_obstacles(vector<DynamicObstacle>& obstacles, const vec4& bb, int num_obstacles)
{
    if(num_obstacles<1)return; // base condition
    vec2 p;

    double deltax = bb(1) - bb(0);
    double deltay = bb(3) - bb(2);
    p(0) = bb(0)+RADIUS+ deltax *randu();
    p(1) = bb(2)+RADIUS+ deltay *randu();

    DynamicObstacle obs(p);
    for (auto& o:obstacles)
    {
        if(o - obs <=2*RADIUS)
        {
            random_obstacles(obstacles, bb, num_obstacles);
        }
    }
    obstacles.push_back(obs);
    if(--num_obstacles>0)
        random_obstacles(obstacles, bb, num_obstacles);

}

#endif //LRAUV_MCTS_DYNAMICOBSTACLES_H
