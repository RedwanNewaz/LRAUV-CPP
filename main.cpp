#include "MyHelper.h"
#include <cassert>
#include "src/MCTS.h"
#include "FlowField.h"




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
    xEst(0) = prob.initial_loc[0] ;  xEst(1) = prob.initial_loc[1] ;
    int sample_time = 50;

    vec2 landmark{prob.goal_loc[0], prob.goal_loc[1]};
    MCTS mcts(prob.flow_data);
    auto root = mcts.Search(landmark, xEst, PEst, sample_time);

    queue<NodePtr> q;
    q.push(root);
    while(!q.empty())
    {
        auto node = q.front();
        q.pop();
        sim_view(node, field);
        for(auto& child: node->children)
        {
            q.push(child);
        }
    }
    plt::show();

    return 0;
}