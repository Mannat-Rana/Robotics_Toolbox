#include "RoboticsToolbox.hpp"

int main()
{
    std::cout << "Starting main" << std::endl;
    rt::DHParams params {30, 1, 30, 1};
    rt::DHParams params2 {0, 1, 0, 1};
    rt::Link link1 {params, "robot1"};
    link1.setA(10);
    rt::Revolute rev1 {link1};
    rt::Prismatic pri1 {link1};
    std::vector<rt::Link> links {link1, rev1, pri1};
    rt::Robot bot (links, "test robot");
    bot.printDHTable();
    bot.addLink(rt::Link {params2}, 1);
    bot.printDHTable();
    bot.removeLink(3);
    bot.printDHTable();
    std::cout << bot.getDOF();
    return 0;
}