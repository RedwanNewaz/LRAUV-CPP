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
    MCTS mcts(prob.flow_data);
    auto root = mcts.Search(landmark, xEst, PEst, sample_time);

    queue<NodePtr> q;
    auto background = cv::imread(prob.back_img, IMREAD_COLOR);
    auto target_size = field.size();


    if(mcts_only) return 0;

    auto CV_FOURCC = VideoWriter::fourcc('X','V','I','D');
    VideoWriter video(video_file,CV_FOURCC,30, Size(WINDOW_WIDTH,WINDOW_HEIGHT));
    q.push(root);
    while(!q.empty())
    {
        auto node = q.front();
        q.pop();
        SimultionView sim_view(background, target_size.first, target_size.second);
        sim_view(node);
        if(demo)sim_view.show();
        video.write(sim_view.get_frame());
        for(auto& child: node->children)
        {
            q.push(child);
        }
    }

    video.release();


    return 0;
}