//
// Created by redwan on 10/27/20.
//
#include "MyHelper.h"
#include "SimulationView.h"
#include <boost/program_options.hpp>


using namespace boost::program_options;

int main(int argc, char* argv[])
{

    options_description desc{"Options"};
    desc.add_options()
            ("help,h", "Help screen")
            ("mcts", "will show only mcts output")
            ("demo", "show demo")
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

    vector<DynamicObstacle> obstacles;
    vec4 bb{0, 21, 0, 29};
    int num_obstacles = 10;
    random_obstacles(obstacles, bb, num_obstacles);

    // SIMULATION
    auto background = cv::imread(prob.back_img, IMREAD_COLOR);
    int step = 1500;
    do {
        SimultionView sim_view(background, 21, 29);
        for(auto&o :obstacles)
           o.update(obstacles,bb);
        sim_view.show_obstacles(obstacles);
        if(demo)sim_view.show();
    }while (--step>0);

    return 0;
}