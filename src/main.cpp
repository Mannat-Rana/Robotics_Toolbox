#include "RoboticsToolbox.hpp"

int main()
{   
    // Use Grubler's Formula for a system with 
    // m = 3, J = 4, N = 4, f = [1 1 1 1]
    std::cout << "DOF of the system: " << rt::getDOF(3, 4, 4, std::vector<int>{1, 1, 1, 1}) << std::endl;

    // Generate Elementary Rotation Matrices
    // Rx(30), Ry(45), Rz(60)
    std::cout << std::endl << "Rx(30):" << std::endl;
    std::cout << rt::Transforms::getRotationMatrix(rt::x, 30) << std::endl;
    std::cout << std::endl << "Ry(45):" << std::endl;
    std::cout << rt::Transforms::getRotationMatrix(rt::y, 45) << std::endl;
    std::cout << std::endl << "Rz(60):" << std::endl;
    std::cout << rt::Transforms::getRotationMatrix(rt::z, 60) << std::endl;
    
    // Generate Revolute Joint with following DH Parameters:
    // alpha = -90, a = 0, theta = 30, d = 0
    std::cout << std::endl;
    rt::DHParams exampleParams {-90, 0, 30, 0};
    rt::Revolute exampleRevolute {exampleParams, std::string {"Example Revolute"}};
    exampleRevolute.printDHTable();

    // Solve Homogeneous Transformation from DH Parameters
    std::cout << std::endl << "Corresponding Homogeneous Transformation: " << std::endl;
    std::cout << exampleRevolute.getHomogeneousTransform() << std::endl << std::endl;

    // Print Revolute Joint directly
    std::cout << exampleRevolute << std::endl;

    std::vector<std::shared_ptr<rt::Link>> links {};
    links.push_back(std::make_shared<rt::Revolute>(exampleRevolute));
    // links.push_back(std::make_shared<rt::Revolute>(exampleRevolute));
    rt::Robot exampleRobot {links, "Example Robot"};
    std::cout << exampleRobot;
    std::cout << exampleRobot.getForwardKinematics() << std::endl;
}