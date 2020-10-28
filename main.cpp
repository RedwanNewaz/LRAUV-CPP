#define _USE_MATH_DEFINES
#define _HAS_AUTO_PTR_ETC 1
#include <cmath>
#include <cstdlib>
#include "MyHelper.h"
#include <cassert>
#include "src/MCTS.h"
#include "FlowField.h"
#include "SimulationView.h"
#include <boost/program_options.hpp>


using namespace boost::program_options;

int main(int argc, char* argv[])
{
    assert(argc>1 && "argument not found");


    options_description desc{"Options"};
    desc.add_options()
            ("help,h", "Help screen")
            ("mcts", "will show only mcts output")
            ("demo", "show demo")
            ("obs",value<int>()->default_value(4), "Number of obstacles, default value 4")
            ("test",value<std::string>()->default_value("../test/01/1.txt"), "test file e.g., <../test/01/1.txt>")
            ("video",value<std::string>()->default_value("./result.avi"), "video output directory");

    variables_map vm;
    boost::program_options::store(parse_command_line(argc, argv, desc), vm);
    notify(vm);
    if (argc<2 || vm.count("help")) {
        std::cout << desc << "\n";
        return 0;
    }
    string filename, video_file;
    if(vm.count("test"))
        filename =  vm["test"].as<std::string>();
    if(vm.count("video"))
        video_file =  vm["video"].as<std::string>();
    bool mcts_only = false;
    bool demo = false;
    if(vm.count("mcts"))
        mcts_only = true;
    if(vm.count("demo"))
        demo = true;


    auto prob = parse_problem(filename);
    cout << "[FlowData]: "<< prob.flow_data << endl;
    cout << "[InitalLoc]: "<<prob.initial_loc[0] << "\t" << prob.initial_loc[1] << endl;
    cout << "[GoalLoc]: "<<prob.goal_loc[0] << "\t" << prob.goal_loc[1] << endl;
    cout << "[BackImg]: "<<prob.back_img << endl;


    FlowField field(prob.flow_data);
    mat xEst(4, 1, fill::zeros);
    mat PEst(4, 4, fill::zeros);
    xEst(0) = prob.initial_loc[0] ;  xEst(1) = prob.initial_loc[1] ;
    int sample_time = 50;

    vec2 landmark;
    landmark(0)=prob.goal_loc[0]; landmark(1)=prob.goal_loc[1];

    int num_obstcales = vm["obs"].as<int>();
    MCTS mcts(prob.flow_data, prob.back_img, num_obstcales);
    auto root = mcts.Search(landmark, xEst, PEst, sample_time);

    return 0;
}