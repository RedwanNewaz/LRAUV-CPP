#include "MyHelper.h"
#include <cassert>
#include "src/TreeNode.h"
#include <queue>



int main(int argc, char* argv[])
{
    assert(argc>1 && "argument not found");
    string filename(argv[1]);

    auto prob = parse_problem(filename);
    cout << "[FlowData]: "<< prob.flow_data << endl;
    cout << "[InitalLoc]: "<<prob.initial_loc[0] << "\t" << prob.initial_loc[1] << endl;
    cout << "[GoalLoc]: "<<prob.goal_loc[0] << "\t" << prob.goal_loc[1] << endl;


    FlowField field(prob.flow_data);
    mat xEst(4, 1, fill::zeros);
    mat PEst(4, 4, fill::zeros);
    EKF ekf;
    xEst(0) = prob.initial_loc[0] ;  xEst(1) = prob.initial_loc[1] ;
    int sample_time = 50;

    vec2 landmark{prob.goal_loc[0], prob.goal_loc[1]};
    MCTS mcts(prob.flow_data);
    vec2 u{0,0};

//    plt::figure_size(16*600,9*400);
    NodePtr root = nullptr;
    unordered_map<int, ActionValue> History;
    ActionValue action;
    double dist = 100;
    do{

        State s;
        s.xEst = xEst;
        s.PEst = PEst;
        s.previous_action = ACTION_HASH[HASH(u)];


        int id = state_indx(xEst, u);
        if (History.find(id) != History.end())
        {
            action = History[id];
        }
        else
        {
            action = mcts.Search(landmark, xEst, PEst, sample_time);
            History[id] = action;

            // tree modification
            s.id = id;
            s.add_action(action.best_index);
            auto node = std::make_shared<TreeNode>(s, action.best_index, root);
            if(root)
            {
                root->children.push_back(node->getPtr());
            }
            root = node->getPtr();

        }
//        sim_update(xEst, action, landmark, field);
        u = action.u;
        auto z = ekf.measurement(xEst, u);
        auto ud = ekf.control_input(u);
        tie(xEst, PEst) = ekf.estimation(xEst, PEst, z, ud);
        vec2 current{xEst(0), xEst(1)};
        dist = norm(current-landmark, 2);

        // tree modification
        root->numVists += 1;
        root->isTerminal = (dist>1);
        root->totalReward += action.reward;

    }while (dist>1);

    root = mcts.Backpropagate(root);
    queue<NodePtr> q;
    q.push(root);
    while(!q.empty())
    {
        auto node = q.front();
        q.pop();
        cout <<"[reward]: " << node->totalReward << endl;
        for(auto& child: node->children)
        {
            q.push(child);
        }
    }
//    plt::show();


    return 0;
}