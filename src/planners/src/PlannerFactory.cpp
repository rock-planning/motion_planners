#include "PlannerFactory.hpp"

namespace motion_planners
{

PlannerFactory::PlannerFactory()
{}

PlannerFactory::~PlannerFactory()
{}


AbstractPlannerPtr PlannerFactory::getPlannerTask(motion_planners::PlannerLibrary library)
{
	AbstractPlannerPtr planner = NULL;

	switch(library)
	{
		case STOMP:
		{
		  planner = std::shared_ptr< StompPlanner>(new StompPlanner());
		  break;
		}
		case OMPL:
		{
		    #if(OMPL_LIB_FOUND)
		    	planner = std::shared_ptr< OmplPlanner>(new OmplPlanner());
            #else
			    LOG_FATAL_S << "[PlannerFactory]: OMPL is not installed. Please select an another Planner !";			
		    #endif
		    
		    break;
		}
		default:
		{
			std::cout<<"No planner library selected"<<std::endl;
			return NULL;
		}
	}
	return planner;
}


}
