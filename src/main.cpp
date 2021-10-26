#include "RoboticsToolbox.hpp"

int main()
{
    std::cout << rt::Transforms::getRotationMatrix(rt::Axes::axis_z, 90) << std::endl;
    return 0;
}