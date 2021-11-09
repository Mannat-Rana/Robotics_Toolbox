#include "RoboticsToolbox.hpp"

int main()
{
    std::cout << "Starting main" << std::endl;
    std::cout << rt::Transforms::getRotationMatrix(rt::Axes::axis_z, 90) << std::endl;
    std::cout << "Finished main" << std::endl;
    return 0;
}