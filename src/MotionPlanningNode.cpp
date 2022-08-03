#include "Common.hpp"

// 主程序入口
int main(int argc, char** argv) {
    // 初始化日志文件
    std::string home;
    home = getenv("HOME");
    std::string log_file_path = "/motion_planning_log/";
    log_file_path = home + log_file_path;
    Tools::resetLogFile(log_file_path);
    google::InitGoogleLogging(argv[0]);
    google::SetLogDestination(google::GLOG_INFO, log_file_path.c_str());
    #ifndef NDEBUG
    // 首先本工程的目录
    std::string root_path = ros::package::getPath("motion_planning");
    std::string curve_file_path = root_path + "/curve_record/";
    // 删除文件夹
    std::string commander = "rm -rf " + curve_file_path;
    system(commander.c_str());
    // 创建文件夹
    Tools::resetLogFile(curve_file_path);
    #endif
    // 初始化ros程序
    ros::init(argc, argv, "motion_planning_node");
    ros::NodeHandle nh("~");
    std::unique_ptr<DecisionMaking::SubVehicle> motion_planning_factory_ptr(new DecisionMaking::SubVehicle(nh));
    motion_planning_factory_ptr->runMotionPlanning();

    // 关闭日志文件
    google::ShutdownGoogleLogging();
    return 0;
}