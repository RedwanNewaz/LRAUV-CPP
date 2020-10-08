//
// Created by robor on 10/7/2020.
//

#include "LAVU.h"
#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/particle_belief.h>
#include <despot/solver/pomcp.h>

using namespace despot;

RobotState::RobotState()
{
    xEst = zeros(4, 1);
    u = zeros(2, 1);
    flowField = unique_ptr<FlowField>(new FlowField(filename));
}

RobotState::RobotState(int x , int y )
{
    xEst = zeros(4, 1);
    xEst(0) = x;
    xEst(1) = y;
    u = zeros(2, 1);
    flowField = unique_ptr<FlowField>(new FlowField(filename));
}

std::string RobotState::text()
{
    auto z = RobotModel::measurement(xEst, u);
    string res = "[z]: (" + to_string(z(0)) + " " + to_string(z(1)) + ") \n";
    return res;
}

mat RobotState::measurement()
{
    vec2 z_hat;
    z_hat(0) = xEst(0);
    z_hat(1) = xEst(1);
    return z_hat;
}

void RobotState::update(const mat &raw_u) {
    auto ud = RobotModel::control_input(raw_u);
    auto z = RobotModel::measurement(xEst, u);
    auto uF = flowField->at(z(0),z(1));
    u = ud + uF;
    xEst = transition(xEst, u);
}
//==================================================================================================================
LAVU::LAVU() {
    vec a(2, fill::zeros);
    ACTION_SET.push_back(a); // stay 0
    a(0) = 1;
    ACTION_SET.push_back(a); // north 1
    a(1) = 0.5;
    ACTION_SET.push_back(a); // north east 2
    a(1) = -0.5;
    ACTION_SET.push_back(a); // north west 3
    a(0) = -1;
    ACTION_SET.push_back(a); // south west 4
    a(1) = 0.5;
    ACTION_SET.push_back(a); // south east 5
    a(1) = 0;
    ACTION_SET.push_back(a); // south 6
    a(0) = 0;
    a(1) = 0.5;
    ACTION_SET.push_back(a); // east 7
    a(1) = -0.5;
    ACTION_SET.push_back(a); // west 8

    GOAL(0) = 5;
    GOAL(1) = 5;
    cout<<"[LAUV]: initialized" <<endl;
}

bool LAVU::Step(State &s, double random_num, ACT_TYPE action, double &reward, OBS_TYPE &obs) const {
    cout<<"[LAUV]: step" <<endl;
    RobotState& state = static_cast<RobotState&>(s);
    bool terminal = false;
    state.update(ACTION_SET.at(action));
    auto position = state.measurement();
    auto dz = norm(GOAL-position, 2);
    reward = -dz;
    // TODO change obs prob based on the distance and modify tau function with obs
    obs = 1;
    if (dz <= 1.0)
        terminal = true;

    return terminal;
}

int LAVU::NumStates() const {
    return 100;
}

int LAVU::NumActions() const {
    return ACTION_SET.size();
}

double LAVU::ObsProb(OBS_TYPE obs, const State &s, ACT_TYPE a) const {
    return 1.0;
}

State *LAVU::CreateStartState(std::string type) const {
    return new RobotState();;
}

ScenarioLowerBound *LAVU::CreateScenarioLowerBound(std::string name, std::string particle_bound_name) const {
    if (name == "TRIVIAL") {
        return new TrivialParticleLowerBound(this);
    } else if (name == "RANDOM" || name == "DEFAULT") {
        return new RandomPolicy(this,
                                CreateParticleLowerBound(particle_bound_name));
    }else {
        cerr << "Unsupported lower bound algorithm: " << name << endl;
        exit(-1);
    }
}



Belief* LAVU::InitialBelief(const State* start, string type) const {
    cout<<"[LAUV]: InitialBelief" <<endl;
    vector<State*> particles;
    for (int pos = 0; pos <= 1; pos++) {
        for (int i = 0; i < 1000; i++) {
            RobotState* state = static_cast<RobotState*>(Allocate(pos, 0.005));
            particles.push_back(state);
        }
    }
    return new ParticleBelief(particles, this);
}

State *LAVU::Allocate(int state_id, double weight) const {
    RobotState* particle = memory_pool_.Allocate();
    particle->state_id = state_id;
    particle->weight = weight;
    return particle;
}

void LAVU::PrintState(const State& state, ostream& out) const {
    out << state.text() << endl;
}

void LAVU::PrintBelief(const Belief& belief, ostream& out) const {
}

void LAVU::PrintObs(const State& state, OBS_TYPE obs, ostream& out) const {
    out << obs << endl;
}
;

void LAVU::PrintAction(ACT_TYPE action, ostream& out) const {

    switch (action) {
        case 0: cout << "STAY" << endl;
            break;
        case 1: cout << "NORTH" << endl;
            break;
        case 2: cout << "NORTH EAST" << endl;
            break;
        case 3: cout << "NORTH WEST" << endl;
            break;
        case 4: cout << "SOUTH WEST" << endl;
            break;
        case 5: cout << "SOUTH EAST" << endl;
            break;
        case 6: cout << "SOUTH" << endl;
            break;
        case 7: cout << "EAST" << endl;
            break;
        case 8: cout << "WEST" << endl;
            break;
    }

}



State* LAVU::Copy(const State* particle) const {
    const RobotState* new_particle = memory_pool_.Allocate();
    RobotState* _particle = const_cast<RobotState*>(new_particle);
    _particle->SetAllocated();
    return _particle;
}

void LAVU::Free(State* particle) const {
    memory_pool_.Free(static_cast<RobotState*>(particle));
}

int LAVU::NumActiveParticles() const {
    return memory_pool_.num_allocated();
}

/*
 * Implementation of particle filter
 * https://github.com/AtsushiSakai/PythonRobotics/blob/master/Localization/particle_filter/particle_filter.py
 */
Belief* LAVU::Tau(const Belief* belief, ACT_TYPE action, OBS_TYPE obs) const {


    const vector<State*>& particles =
            static_cast<const ParticleBelief*>(belief)->particles();
    const int NumStates = particles.size(); // each particle represents a state
    static vector<double> probs = vector<double>(NumStates);

    double sum = 0;
    for (int i = 0; i < particles.size(); i++) {
        RobotState* state = static_cast<RobotState*>(particles[i]);
        auto u = ACTION_SET.at(action);
        state->update(u);
        auto z = state->measurement();
        auto dz = norm(GOAL - z, 2);

        double p =  state->weight * gauss_likelihood(dz, 0.5);// sigma is chosen from the robot model sqrt(Q[0, 0] = 0.5^2)
        probs[i] += p * obs;
        sum += p;
    }

    vector<State*> new_particles;
    for (int i = 0; i < NumStates; i++) {
        if (probs[i] > 0) {
            RobotState* new_particle = static_cast<RobotState*>(Allocate(i));
            RobotState* state = static_cast<RobotState*>(particles[i]);
            // copy previous particle information to new particle
            new_particle->xEst = state->xEst;
            new_particle->u = state->u;
            new_particle->weight = probs[i] / sum;
            new_particles.push_back(new_particle);
            probs[i] = 0;
        }
    }

    return new ParticleBelief(new_particles, this, NULL, false);
}

void LAVU::Observe(const Belief* belief, ACT_TYPE action,
                     map<OBS_TYPE, double>& obss) const {
    const vector<State*>& particles =
            static_cast<const ParticleBelief*>(belief)->particles();
    // updating belief
    for (int i = 0; i < particles.size(); i++) {
        State* particle = particles[i];
        RobotState* state = static_cast<RobotState*>(particle);
        auto u = ACTION_SET.at(action);
        state->update(u);
    }
}

double LAVU::StepReward(const Belief* belief, ACT_TYPE action) const {
    const vector<State*>& particles =
            static_cast<const ParticleBelief*>(belief)->particles();

    double sum = 0;
    for (int i = 0; i < particles.size(); i++) {
        State* particle = particles[i];
        RobotState* state = static_cast<RobotState*>(particle);
        double action_weight = -10;
        if(action <= 6)
            action_weight = -1;
        vec2 pos;
        pos(0) = state->xEst(0);
        pos(1) = state->xEst(1);
        // Energy awareness
        double reward =  -norm(pos-GOAL, 2) + action_weight;
        sum += state->weight * reward;
    }

    return sum;
}

double LAVU::Reward(const State& state, ACT_TYPE action) const {
    const RobotState* robot_state = static_cast<const RobotState*>(&state);
    double action_weight = -10;
    if(action <= 6)
        action_weight = -1;
    vec2 pos;
    pos(0) = robot_state->xEst(0);
    pos(1) = robot_state->xEst(1);
    // TODO add Energy awareness
    double reward =  -norm(pos-GOAL, 2) + action_weight;
    return reward;
}
// ====================================================================================

class LAVUPOMCPPrior: public POMCPPrior {
public:
    LAVUPOMCPPrior(const DSPOMDP* model) :
            POMCPPrior(model) {
    }

    void ComputePreference(const State& state) {
        for (int a = 0; a < 7; a++) {
            legal_actions_.push_back(a);
        }

        preferred_actions_.push_back(1);
    }
};

POMCPPrior* LAVU::CreatePOMCPPrior(string name) const {
    if (name == "UNIFORM") {
        return new UniformPOMCPPrior(this);
    } else if (name == "DEFAULT") {
        return new LAVUPOMCPPrior(this);
    } else {
        cerr << "Unsupported POMCP prior: " << name << endl;
        exit(1);
        return NULL;
    }
}


