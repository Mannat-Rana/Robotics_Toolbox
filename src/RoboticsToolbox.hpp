#pragma once
//INCLUDES HERE
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>

//TYPE ALIASES HERE
using rotationMatrix_t = Eigen::Matrix3d;
using homogenousTransform_t = Eigen::Matrix4d;
using radianAngle_t = double;
using degreeAngle_t = double;


namespace rt
{
    enum Axes
    {
        axis_x,
        axis_y,
        axis_z,
        axis_count,
    };
    namespace Constants
    {
        constexpr double pi = 3.14159265358979323846264;
    }
    namespace Geometry
    {
        /**
         * Converts Degrees to Radians.
         * 
         * @param angleinDeg is the angle in degrees that needs
         *     to be converted to radians.
         * 
         * @return the angle originally in degrees but now in 
         *     radians.
         * 
         * @exceptsafe This function does not throw exceptions.
         */
        radianAngle_t convertDegToRad(double angleInDeg);

        /**
         * Converts Radians to Degrees.
         * 
         * @param angleinRad is the angle in radians that needs
         *     to be converted to degrees.
         * 
         * @return the angle originally in radians but now in 
         *     degrees.
         * 
         * @exceptsafe This function does not throw exceptions.
         */
        degreeAngle_t convertRadToDeg(double angleInRad);
    }
    
    namespace Transforms
    {
        /**
         * Get elementary rotation matrix.
         * 
         * @param axis is the axis the rotation occurs around.
         * @param angle is the angle in degrees of the rotation.
         * 
         * @return the elementary 3x3 rotation matrix.
         * 
         * @throws "Illegal input, cannot compute rotation matrix. Returning identity matrix."
         *     if axis is invalid.
         * 
         * @exceptsafe This function offers basic exception safety.
         */
        rotationMatrix_t getRotationMatrix(Axes axis, double angle);
    }
}
