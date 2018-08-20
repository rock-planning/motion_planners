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
		    #if(OMPL_FOUND)
		    	planner = std::shared_ptr< OmplPlanner>(new OmplPlanner());
		    #endif
		    
		    if(!OMPL_FOUND)
		    {
			LOG_FATAL_S << "[PlannerFactory]: OMPL is not installed. Please select an another Planner !";			
		    }
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
