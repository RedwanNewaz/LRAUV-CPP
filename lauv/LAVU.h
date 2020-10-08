//
// Created by robor on 10/7/2020.
//

#ifndef LRAUV_LAVU_H
#define LRAUV_LAVU_H
#include <iostream>
#include <memory>
#include <armadillo>
#include <despot/interface/pomdp.h>
#include <despot/core/particle_belief.h>
#include "RobotModel.h"
#include "FlowField.h"
using namespace arma;
namespace despot
{
    class RobotState: public State, public RobotModel {
    public:
        RobotState();
        virtual ~RobotState()
        {

        }
        RobotState(int x, int y);
        void update(const mat& raw_u);
        mat measurement();
        std::string text();
        Mat<double> xEst, u;
    private:
        char *filename = "../data/UUVV61";
        unique_ptr<FlowField> flowField;

    };

    class LAVU :public BeliefMDP{
    private:
        mutable MemoryPool<RobotState> memory_pool_;
        vector<vec2>ACTION_SET;
        vec2 GOAL;

    public:
        LAVU();

        bool Step(State& s, double random_num, ACT_TYPE action, double& reward,
                  OBS_TYPE& obs) const;
        int NumStates() const;
        int NumActions() const;
        double ObsProb(OBS_TYPE obs, const State& s, ACT_TYPE a) const;

        State* CreateStartState(std::string type = "DEFAULT") const;
        Belief* InitialBelief(const State* start, std::string type = "DEFAULT") const;

        inline double GetMaxReward() const {
            return 0;
        }

        inline ValuedAction GetBestAction() const {
            return ValuedAction(1, -1); // moving north
        }
        ScenarioLowerBound* CreateScenarioLowerBound(std::string name = "DEFAULT",
                                                     std::string particle_bound_name = "DEFAULT") const override;

        void PrintState(const State& state, std::ostream& out = std::cout) const;
        void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const;
        void PrintObs(const State& state, OBS_TYPE obs, std::ostream& out = std::cout) const;
        void PrintAction(ACT_TYPE action, std::ostream& out = std::cout) const;

        State* Allocate(int state_id = -1, double weight = 0.0) const;
        State* Copy(const State* particle) const;
        void Free(State* particle) const;
        int NumActiveParticles() const;

        Belief* Tau(const Belief* belief, ACT_TYPE action, OBS_TYPE obs) const;
        void Observe(const Belief* belief, ACT_TYPE action,
                     std::map<OBS_TYPE, double>& obss) const;
        double StepReward(const Belief* belief, ACT_TYPE action) const;
        double Reward(const State& belief, ACT_TYPE action) const;

        POMCPPrior* CreatePOMCPPrior(std::string name = "DEFAULT") const;

    protected:
        double gauss_likelihood(double x, double sigma)const
        {
            return 1.0/sqrt(2.0 * M_PI * pow(sigma,2) ) *
                    exp(- pow(x, 2)/ (2 * pow(sigma, 2)));
        }

    };
}



#endif //LRAUV_LAVU_H
