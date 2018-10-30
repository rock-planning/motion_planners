#ifndef PLANNERFACTORY_HPP_
#define PLANNERFACTORY_HPP_

#include <Config.hpp>
#include <abstract/AbstractPlanner.hpp>
#include <wrapper/stomp/StompPlanner.hpp>
#include <wrapper/trajopt/TrajoptPlanner.hpp>


#if OMPL_LIB_FOUND    
    #include <wrapper/ompl/OmplPlanner.hpp>
#endif

/** \file PlannerFactory.hpp
*    \brief Factory class for the PlannerFactory class.
*/

namespace motion_planners
{
    
class PlannerFactory
{
    public:
    PlannerFactory();
    ~PlannerFactory();
    AbstractPlannerPtr getPlannerTask(motion_planners::PlannerLibrary library);
};

}// end planner

#endif
