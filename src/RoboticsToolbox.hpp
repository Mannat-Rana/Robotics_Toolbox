#pragma once
//INCLUDES HERE
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>
#include <string>
#include <vector>

//TYPE ALIASES HERE
using rotationMatrix_t = Eigen::Matrix3d;
using homogenousTransform_t = Eigen::Matrix4d;
using radianAngle_t = double;
using degreeAngle_t = double;


namespace rt
{
    int getDOF(int M, int N, int J, const std::vector<int>& fi);
    enum Axes
    {
        axis_x,
        axis_y,
        axis_z,
        axis_count,
    };
    
    enum LinkType
    {
        type_revolute,
        type_prismatic,
        type_count,
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

    struct DHParams 
    {
        double alpha;
        double a;
        double theta;
        double d;
    };
    class Link
    {
        private:
            DHParams m_params;
            std::string m_name;
        public:
            Link();
            Link(const DHParams& params);
            Link(const std::string& name);
            Link(const DHParams& params, const std::string& name);
            void setAlpha(const double alpha);
            void setA(const double a);
            void setTheta(const double theta);
            void setD(const double d);
            void setName(const std::string& name);
            double getAlpha() const;
            double getA() const;
            double getTheta() const;
            double getD() const;
            std::string getName() const;
            virtual void printDHTable() const;
            homogenousTransform_t getHomogenousTransform() const;
    };
    class Revolute : public Link
    {
        private:
            rt::LinkType m_type;
        public:
            Revolute();
            Revolute(const DHParams& params);
            Revolute(const std::string& name);
            Revolute(const DHParams& params, const std::string& name);
            Revolute(const Link& link);
            virtual void printDHTable() const;
    };

    class Prismatic : public Link
    {
        private:
            rt::LinkType m_type;
        public:
            Prismatic();
            Prismatic(const DHParams& params);
            Prismatic(const std::string& name);
            Prismatic(const DHParams& params, const std::string& name);
            Prismatic(const Link& link);
            virtual void printDHTable() const;
    };
    class Robot
    {
        private:
            std::vector<Link> m_links;
            std::string m_name;
        public:
            Robot();
            Robot(const std::vector<Link>& links);
            Robot(const std::string& name);
            Robot(const std::vector<Link>& links, const std::string& name);
            std::string getName() const;
            void setName(const std::string& name);
            void addLink(const Link& link);
            homogenousTransform_t getForwardKinematics() const;
            void printDHTable() const;
    };
}
