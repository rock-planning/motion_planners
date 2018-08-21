#include "wrapper/trajopt/RobotLinkWrapper.h"

RobotLinkWrapper::RobotLinkWrapper()
{

}

std::string RobotLinkWrapper::GetName() const
{
    m_link->getLinkName();
}

trajopt::geometry::Transform RobotLinkWrapper::GetTransform()
{

}

int RobotLinkWrapper::GetIndex() const
{

}

trajopt::LinkPtr RobotLinkWrapper::getParent()
{

}
