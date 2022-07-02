#include "filling_extraction/FillingExtractor.h"

void SigintHandler(int sig){
    ros::shutdown();
}

int main(int argc, char** argv){

    ros::init(argc, argv, "filling_extractor", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    signal(SIGINT, SigintHandler);
    
    FillingExtractor fe = FillingExtractor(nh);

    ros::spin();
}