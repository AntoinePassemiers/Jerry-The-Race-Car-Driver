/**
    module.h
    Base class for modules
    
    @author Antoine Passemiers
    @version 1.0 05/08/2019
*/

#ifndef MODULE_H__
#define MODULE_H__

#include <Eigen/Core>


class Module {
public:
    virtual size_t getNumberOfParameters() = 0;
    virtual Eigen::VectorXd getLowerBounds() = 0;
    virtual Eigen::VectorXd getUpperBounds() = 0;
    virtual Eigen::VectorXd getParameters() = 0;
    virtual void setParameters(Eigen::VectorXd parameters) = 0;
};


#endif // MODULE_H__
