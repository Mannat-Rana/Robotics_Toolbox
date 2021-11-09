#include "RoboticsToolbox.hpp"

int main()
{
    std::cout << "Starting main" << std::endl;
    rt::DHParams params {30, 1, 30, 1};
    rt::Link link1 {params, "robot1"};
    rt::Revolute rev1 {link1};
    rt::Prismatic pri1 {link1};
    std::vector<rt::Link> links {link1, rev1, pri1};
    rt::Robot bot (links, "test robot");
    bot.printDHTable();
    return 0;
}