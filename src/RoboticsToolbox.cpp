/**
 * @file RoboticsToolbox.cpp
 * @author Mannat Rana (mrana8@asu.edu)
 * @brief CPP File for C++ Robotics Toolbox Implementation
 *        Written for MAE 547 - Modelling and Control of Robots
 *        Taught by Dr. Hyunglae Lee at Arizona State University
 * @version 0.1
 * @date 2021-12-02
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "RoboticsToolbox.hpp"

namespace rt
{
    int getDOF(int m, int N, int J, const std::vector<int>& fi)
    {
        if (fi.size() == 0)
        {
            return 0;
        }
        int DOF { 0 };
        // Sum up total degrees of freedom
        for (auto f: fi)
        {
            DOF += f;
        }
        // Apply Grubler's Formula
        DOF += m * (N - 1 - J);
        return DOF;
    }

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
            {   // Check user's desired input axis
                switch (axis)
                { // Implement formulas for elementary rotations 
                case Axes::x:
                    mat(0,0) = 1; mat(0,1) = 0;               mat(0,2) = 0;
                    mat(1,0) = 0; mat(1,1) = cos(angleInRad); mat(1,2) = -sin(angleInRad);
                    mat(2,0) = 0; mat(2,1) = sin(angleInRad); mat(2,2) = cos(angleInRad);
                    return mat;
                    break;
                case Axes::y:
                    mat(0,0) = cos(angleInRad);  mat(0,1) = 0; mat(0,2) = sin(angleInRad);
                    mat(1,0) = 1;                mat(1,1) = 1; mat(1,2) = 0;
                    mat(2,0) = -sin(angleInRad); mat(2,1) = 0; mat(2,2) = cos(angleInRad);
                    return mat;
                    break;
                case Axes::z:
                    mat(0,0) = cos(angleInRad); mat(0,1) = -sin(angleInRad); mat(0,2) = 0;
                    mat(1,0) = sin(angleInRad); mat(1,1) = cos(angleInRad);  mat(1,2) = 0;
                    mat(2,0) = 0;               mat(2,1) = 0;                mat(2,2) = 1;
                    return mat;
                    break;
                default:
                    throw "Illegal input, cannot compute rotation matrix. Returning identity matrix.";
                    break;
                }
            } // Catch in case illegal axis is inputted
            catch (const char* exception)
            {
                std::cerr << "Error: " << exception << std::endl;
                mat(0,0) = 1; mat(0,1) = 0; mat(0,2) = 0;
                mat(1,0) = 0; mat(1,1) = 1; mat(1,2) = 0;
                mat(2,0) = 0; mat(2,1) = 0; mat(2,2) = 1;
                return mat;
            }
        }
    }

    Link::Link(const DHParams& params, const std::string& name) :
     m_params {params.alpha, params.a, params.theta, params.d}, m_name {name}
    {
        // Full constructor
    }
    
    Link::Link(const DHParams& params) : Link(params, "unnamed link") 
    {
        // DH only constructor
    }

    Link::Link(const std::string& name) : Link({0,0,0,0}, name)
    {
        // Name only constructor
    }

    Link::Link() : Link({0,0,0,0}, "unnamed link")
    {
        // Empty constructor
    }

    Link::Link(const Link& l) : Link(l.m_params, l.m_name)
    {
        // Copy constructor
    } 
  
    void Link::setAlpha(const double alpha) 
    { 
        std::cout << "Assigning alpha of link " << m_name << " to " << alpha << std::endl;
        m_params.alpha = alpha;
    }
    
    void Link::setA(const double a) 
    { 
        std::cout << "Assigning a of link " << m_name << " to " << a << std::endl;
        m_params.a = a;
    }
    
    void Link::setTheta(const double theta) 
    {
        std::cout << "Assigning theta of link " << m_name << " to " << theta << std::endl; 
        m_params.theta = theta;
    }

    void Link::setD(const double d) 
    { 
        std::cout << "Assigning d of link " << m_name << " to " << d << std::endl;
        m_params.d = d; 
    }

    void Link::setName(const std::string& name) 
    { 
        std::cout << "Changing link name from " << m_name << " to " << name << std::endl;
        m_name = name; 
    }

    double Link::getAlpha() const 
    { 
        return m_params.alpha;
    }

    double Link::getA() const 
    { 
        return m_params.a; 
    }
    
    double Link::getTheta() const 
    { 
        return m_params.theta; 
    }
    
    double Link::getD() const 
    { 
        return m_params.d; 
    }

    std::string Link::getName() const 
    { 
        return m_name;
    }

    void Link::printDHTable() const
    {
        std::cout << "|\talpha\t|\ta\t|\ttheta\t|\td\t|" << std::endl;
        std::cout << "|\t" << getAlpha() << "\t|";
        std::cout << "\t" << getA() << "\t|";
        std::cout << "\t" << getTheta() << "\t|";
        std::cout << "\t" << getD() << "\t|" << std::endl;
    }

    homogenousTransform_t Link::getHomogeneousTransform() const
    {
        homogenousTransform_t transform;
        // Convert theta and alpha to radians
        radianAngle_t alphaInRad = rt::Geometry::convertDegToRad(getAlpha());
        radianAngle_t thetaInRad = rt::Geometry::convertDegToRad(getTheta());
        // Apply homogeneous transformation formula using DH Parameters
        transform(0,0) = cos(thetaInRad);
        transform(1,0) = sin(thetaInRad);
        transform(2,0) = 0;
        transform(3,0) = 0;
        transform(0,1) = -sin(thetaInRad) * cos(alphaInRad);
        transform(1,1) = cos(thetaInRad) * cos(alphaInRad);
        transform(2,1) = sin(alphaInRad);
        transform(3,1) = 0;
        transform(0,2) = sin(thetaInRad) * sin(alphaInRad);
        transform(1,2) = -cos(thetaInRad) * sin(alphaInRad);
        transform(2,2) = cos(alphaInRad);
        transform(3,2) = 0;
        transform(0,3) = getA() * cos(thetaInRad);
        transform(1,3) = getA() * sin(thetaInRad);
        transform(2,3) = getD();
        transform(3,3) = 1;
        return transform;
    }

    void Link::print(std::ostream& out) const {
        out << "Transformation: \n" << getHomogeneousTransform();
    }
 
    Link::~Link() = default;

    Revolute::Revolute(const DHParams& params, const std::string& name) : Link::Link(params, name) 
    {
        // Full constructor
    }

    Revolute::Revolute(const DHParams& params) : Link::Link(params)  
    {
        // DH Only constructor
    }

    Revolute::Revolute(const std::string& name) : Link::Link(name) 
    {
        // Name only constructor
    }

    Revolute::Revolute() : Link::Link()  
    {
        // Empty constructor
    }

    Revolute::Revolute(const Revolute& r) : Link::Link(r) 
    {
        // Copy constructor
    } 

    void Revolute::printDHTable() const
    {
        std::cout << "\n\t\t\tRevolute: " << getName() << std::endl;
        Link::printDHTable();
    }

    void Revolute::print(std::ostream& out) const {
        out << "Revolute Joint named: " << getName() << std::endl;
        Link::print(out);
    }

    Revolute::~Revolute() = default;

    Prismatic::Prismatic(const DHParams& params, const std::string& name) : Link::Link(params, name) 
    {
        // Full constructor
    }

    Prismatic::Prismatic(const DHParams& params) : Link::Link(params)  
    {
        // DH only constructor
    }

    Prismatic::Prismatic(const std::string& name) : Link::Link(name) 
    {
        // Name only constructor
    }

    Prismatic::Prismatic() : Link::Link() 
    {
        // Empty constructor
    }

    Prismatic::Prismatic(const Prismatic& p) : Link::Link(p) 
    {
        // Constructor to convert regular link to prismatic
    } 

    void Prismatic::printDHTable() const
    {
        std::cout << "\n\t\t\tPrismatic: " << getName() << std::endl;
        Link::printDHTable();
    }

    void Prismatic::print(std::ostream& out) const {
        out << "Prismatic Joint named: " << getName() << std::endl;
        Link::print(out);
    }

    Prismatic::~Prismatic() = default;

    Robot::Robot(const std::vector<std::shared_ptr<Link>>& links, const std::string& name) : m_name {name}, m_links{links} 
    {
        // Complete constructor
        std::cout << "Creating robot named " << m_name << " with " <<
        m_links.size() << " links" << std::endl;
    }

    Robot::Robot(const std::vector<std::shared_ptr<Link>>& links) : Robot(m_links, "unnamed robot")
    {
        // Link vector only constructor
    }

    Robot::Robot(const std::string& name) : m_name {name}, m_links{}
    {
        // Name only constructor
        std::cout << "Creating empty robot named " << m_name << std::endl;
    }

    Robot::Robot() : Robot("unnamed robot") 
    {
        // Empty constructor
    }

    std::string Robot::getName() const 
    { 
        return m_name; 
    }

    size_t Robot::getDOF() const 
    { 
        return m_links.size(); 
    }

    void Robot::setName(const std::string& name) 
    { 
        m_name = name; 
    }

    void Robot::addLink(std::shared_ptr<Link> link, const int position) 
    { 
        auto index { m_links.begin() + position - 1};
        m_links.insert(index, link); 
    }

    void Robot::removeLink(const int position)
    {
        auto index {m_links.begin() + position - 1 };
        m_links.erase(index);
    }

    homogenousTransform_t Robot::getForwardKinematics() const
    {
        homogenousTransform_t fKine { Eigen::Matrix4d::Identity() };
        // Loop through links while multiplying their homogeneous transformation
        for (const auto& link : m_links)
        {
            fKine *= link->getHomogeneousTransform(); 
        }
        return fKine;
    }

    void Robot::printDHTable() const
    {   
        std::cout << "\n\t\t\t\tDH Table for " << getName() << std::endl;
        std::cout << "|\tLink Frame\t|     alpha\t|\ta\t|      theta\t|\td\t|" << std::endl;
        int link_counter { 1 };
        for (const auto& link : m_links)
        {
            std::cout << "|\t    " << link_counter++ << "\t\t|";
            std::cout << "\t" << link->getAlpha() << "\t|";
            std::cout << "\t" << link->getA() << "\t|";
            std::cout << "\t" << link->getTheta() << "\t|";
            std::cout << "\t" << link->getD() << "\t|" << std::endl;
        }
    }

    void Robot::print(std::ostream& out) const {
        out << "Robot named: " << getName() << " with " << m_links.size() << " links." << std::endl;
    }

    Robot::~Robot() = default;
}
