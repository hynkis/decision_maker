#include <decision_maker_core.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "decision_maker");
  decision_maker::DecisionMakerNode smn(argc, argv);
  smn.run();

  return 0;
}
