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
    /**
     * Solves for a system's DOF with Grubler's formula.
     * 
     * @param M is either 3 or 6 depending on whether
     *      the system is planar or spacial.
     * @param N is the number of links including the 
     *      ground.
     * @param J is the number of joints in the system.
     * @param fi is a vector containing all of the 
     *      DOF's for each joint individually.
     * 
     * @return the DOF of the system as an integer.
     * 
     * @exceptsafe This function does not throw exceptions.
     */
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
         * @return the input angle in radians.
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
            /**
             * Construct a new Link object with no input parameters
             * 
             */
            Link();

            /**
             * Construct a new Link object with just DH Parameters
             * 
             * @param params the DH Parameters for the link
             */
            Link(const DHParams& params);

            /**
             * Construct a new Link object with only a name
             * 
             * @param name the name for the link 
             */
            Link(const std::string& name);

            /**
             * Construct a new Link object with DH Paramaters and a name
             * 
             * @param params the DH Parameters for the link 
             * @param name the name for the link
             */
            Link(const DHParams& params, const std::string& name);

            /**
             * Set the value for alpha for the link
             * 
             * @param alpha the desired value for the link's alpha 
             */
            void setAlpha(const double alpha);

            /**
             * Set the value for a for the link 
             * 
             * @param a the desired value for the link's a 
             */
            void setA(const double a);

            /**
             * Set the value for theta for the link
             * 
             * @param theta the desired value for the link's theta
             */
            void setTheta(const double theta);
            
            /**
             * Set the value for d for the link 
             * 
             * @param d the desired value for the link's d
             */
            void setD(const double d);

            /**
             * Name the link
             * 
             * @param name the desired name for the link
             */
            void setName(const std::string& name);
            
            /**
             * Get the value for the link's alpha
             * 
             * @return link's alpha value 
             */
            double getAlpha() const;
            /**
             * Get the value for the link's a
             * 
             * @return link's a value 
             */
            double getA() const;

            /**
             * Get the value for the link's theta
             * 
             * @return link's theta value 
             */
            double getTheta() const;

            /**
             * Get the value for the link's d
             * 
             * @return link's d value 
             */
            double getD() const;

            /**
             * Get the link's name
             * 
             * @return name of the link 
             */
            std::string getName() const;

            /**
             * Print the link's DH Table 
             * 
             */
            virtual void printDHTable() const;

            /**
             * Get the Homogenous Transform for the link
             * 
             * @return the homogeneous transformation matrix 
             *      for the link 
             */
            homogenousTransform_t getHomogenousTransform() const;
    };
    class Revolute : public Link
    {
        private:
            rt::LinkType m_type;
        public:
            /**
             * Construct a new Revolute object with no input parameters
             * 
             */
            Revolute();

            /**
             * Construct a new Revolute object with just DH Parameters
             * 
             * @param params the DH Parameters for the revolute link
             */
            Revolute(const DHParams& params);

            /**
             * Construct a new Revolute object with just the name
             * 
             * @param name the name for the revolute link
             */
            Revolute(const std::string& name);

            /**
             * Construct a new Revolute object with DH Parameters and a name
             * 
             * @param params the DH Parameters for the revolute link
             * @param name the name for the revolute link
             */
            Revolute(const DHParams& params, const std::string& name);

            /**
             * Construct a new Revolute object with a link
             * 
             * @param link the link the revolute object should
             *      get its parameters from 
             */
            Revolute(const Link& link);

            /**
             * Print the revolute joint's DH Table 
             * 
             */
            virtual void printDHTable() const;
    };

    class Prismatic : public Link
    {
        private:
            rt::LinkType m_type;
        public:
            /**
             * Construct a new Prismatic object with no input parameters
             * 
             */
            Prismatic();

            /**
             * Construct a new Prismatic object with just DH Parameters
             * 
             * @param params the DH Parameters for the prismatic link
             */
            Prismatic(const DHParams& params);

            /**
             * Construct a new Prismatic object with just the name
             * 
             * @param name the name for the prismatic link
             */
            Prismatic(const std::string& name);

            /**
             * Construct a new Prismatic object with DH Parameters and a name
             * 
             * @param params the DH Parameters for the prismatic link
             * @param name the name for the prismatic link
             */
            Prismatic(const DHParams& params, const std::string& name);

            /**
             * Construct a new Prismatic object with a link
             * 
             * @param link the link the prismatic object should
             *      get its parameters from 
             */
            Prismatic(const Link& link);

            /**
             * Print the prismatic joint's DH Table 
             * 
             */
            virtual void printDHTable() const;
    };
    class Robot
    {
        private:
            std::vector<Link> m_links;
            std::string m_name;
        public:
            /**
             * Construct an empty Robot object
             * 
             */
            Robot();

            /**
             * Construct a new Robot object with just a vector of links
             * 
             * @param links a vector of links in order for the robot 
             */
            Robot(const std::vector<Link>& links);

            /**
             * Construct a new Robot object with just a name
             * 
             * @param name the desired name for the robot
             */
            Robot(const std::string& name);

            /**
             * Construct a new Robot object with a vector of links
             *      and a name
             * 
             * @param links a vector of links in order for the robot 
             * @param name the desired name for the robot
             */
            Robot(const std::vector<Link>& links, const std::string& name);
            
            /**
             * Get the name of the robot
             * 
             * @return the name of the robot 
             */
            std::string getName() const;

            /**
             * The the robot's DOF
             * 
             * @return the DOF of the robot 
             */
            size_t getDOF() const;

            /**
             * Set the name of the robot
             * 
             * @param name the desired name for the robot 
             */
            void setName(const std::string& name);
            
            /**
             * Add a link to the robot 
             * 
             * @param link the link that needs to be added to the robot 
             * @param position the position at which the link appears
             */
            void addLink(const Link& link, const int position);

            /**
             * Remove a link from the robot 
             * 
             * @param position the position of the link that needs to be 
             *      removed 
             */
            void removeLink(const int position);

            /**
             * Get the Forward Kinematics for the robot
             * 
             * @return the homogeneous transformation matrix of the robot 
             */
            homogenousTransform_t getForwardKinematics() const;
            
            /**
             * Print the DH Table of the robot 
             * 
             */
            void printDHTable() const;
    };
}
