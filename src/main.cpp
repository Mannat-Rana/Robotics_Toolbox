#include "RoboticsToolbox.hpp"

int main()
{   
    // Use Grubler's Formula for a system with 
    // m = 3, J = 4, N = 4, f = [1 1 1 1]
    std::cout << "DOF of the system: " << rt::getDOF(3, 4, 4, std::vector<int>{1, 1, 1, 1}) << std::endl;

    // Generate Elementary Rotation Matrices
    // Rx(30), Ry(45), Rz(60)
    std::cout << std::endl << "Rx(30):" << std::endl;
    std::cout << rt::Transforms::getRotationMatrix(rt::axis_x, 30) << std::endl;
    std::cout << std::endl << "Ry(45):" << std::endl;
    std::cout << rt::Transforms::getRotationMatrix(rt::axis_y, 45) << std::endl;
    std::cout << std::endl << "Rz(60):" << std::endl;
    std::cout << rt::Transforms::getRotationMatrix(rt::axis_z, 60) << std::endl;
    
    // Generate Link with following DH Parameters:
    // alpha = -90, a = 0, theta = 30, d = 0
    std::cout << std::endl;
    rt::DHParams exampleParams {-90, 0, 30, 0};
    rt::Link exampleLink {exampleParams, std::string {"Example Link"}};
    exampleLink.printDHTable();

    // Solve Homogeneous Transformation from DH Parameters
    std::cout << std::endl << "Corresponding Homogeneous Transformation: " << std::endl;
    std::cout << exampleLink.getHomogeneousTransform() << std::endl << std::endl;

    // Create RRP Robot by defining links and combining them into robot
    // to get forward kinematics A03
    rt::DHParams paramsLink1 {-90, 0, 30, 0};
    rt::DHParams paramsLink2 {90, 0, 45, 1};
    rt::DHParams paramsLink3 {0, 0, 0, 1};
    rt::Revolute link1 {paramsLink1, std::string {"Revolute 1"}};
    rt::Revolute link2 {paramsLink2, std::string {"Revolute 2"}};
    rt::Prismatic link3 {paramsLink3, std::string {"Prismatic 3"}};
    rt::Robot exampleRobot {std::vector<rt::Link>{link1, link2, link3},
                        std::string {"RRP Robot"}};
    exampleRobot.printDHTable();
    std::cout << std::endl << "Corresponding Forward Kinematics: " << std::endl;
    std::cout << exampleRobot.getForwardKinematics() << std::endl << std::endl;

    // Use ToolBox API to remove 3rd link and add new link 
    // to make robot RRR instead of original RRP (rename)
    rt::DHParams newParamsLink3 {0, 0, 90, 0};
    rt::Revolute newLink3 {newParamsLink3, std::string {"Revolute 3"}};
    exampleRobot.removeLink(3);
    exampleRobot.addLink(newLink3, 3);
    exampleRobot.setName(std::string {"RRR Robot"});
    exampleRobot.printDHTable();
    std::cout << std::endl << "Corresponding Forward Kinematics: " << std::endl;
    std::cout << exampleRobot.getForwardKinematics() << std::endl;

}