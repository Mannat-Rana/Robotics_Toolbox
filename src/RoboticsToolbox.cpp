#include "RoboticsToolbox.hpp"

namespace rt
{
    namespace Geometry
    {
        radianAngle_t convertDegToRad(double angleInDeg)
        {
            return ((angleInDeg * Constants::pi) / 180);
        }

        degreeAngle_t convertRadToDeg(double angleInRad)
        {
            return ((angleInRad * 180) / Constants::pi);
        }
    }

    namespace Transforms
    {
        rotationMatrix_t getRotationMatrix(Axes axis, double angleInDeg)
        {
            radianAngle_t angleInRad{ Geometry::convertDegToRad(angleInDeg) };
            rotationMatrix_t mat(3,3);
            try
            {
                switch (axis)
                {
                case Axes::axis_x:
                    mat(0,0) = 1; mat(1,0) = 0;               mat(2,0) = 0;
                    mat(0,1) = 0; mat(1,1) = cos(angleInRad); mat(2,1) = -sin(angleInRad);
                    mat(0,2) = 0; mat(1,2) = sin(angleInRad); mat(2,2) = cos(angleInRad);
                    return mat;
                    break;
                case Axes::axis_y:
                    mat(0,0) = cos(angleInRad);  mat(1,0) = 0; mat(2,0) = sin(angleInRad);
                    mat(0,1) = 1;                mat(1,1) = 1; mat(2,1) = 0;
                    mat(0,2) = -sin(angleInRad); mat(1,2) = 0; mat(2,2) = cos(angleInRad);
                    return mat;
                    break;
                case Axes::axis_z:
                    mat(0,0) = cos(angleInRad); mat(1,0) = -sin(angleInRad); mat(2,0) = 0;
                    mat(0,1) = sin(angleInRad); mat(1,1) = cos(angleInRad);  mat(2,1) = 0;
                    mat(0,2) = 0;               mat(1,2) = 0;                mat(2,2) = 1;
                    return mat;
                    break;
                default:
                    throw "Illegal input, cannot compute rotation matrix. Returning identity matrix.";
                    break;
                }
            }
            catch (const char* exception)
            {
                std::cerr << "Error: " << exception << std::endl;
                mat(0,0) = 1; mat(1,0) = 0; mat(2,0) = 0;
                mat(0,1) = 0; mat(1,1) = 1; mat(2,1) = 0;
                mat(0,2) = 0; mat(1,2) = 0; mat(2,2) = 1;
                return mat;
            }
        }
    }
    

    
}
