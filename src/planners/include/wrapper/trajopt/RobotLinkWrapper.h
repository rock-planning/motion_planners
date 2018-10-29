#ifndef LINKWRAPPER_H
#define LINKWRAPPER_H

#include "RobotLink.hpp"
#include "trajopt/trajoptinterface.h"
using namespace motion_planners;

typedef std::shared_ptr<RobotLink> RobotLinkPtr;

class RobotLinkWrapper : public trajopt::Link
{

    RobotLinkPtr m_link;
    int m_index;
    std::string m_name;

public:
    RobotLinkWrapper();

    // Link interface
    virtual std::string GetName() const;
    virtual trajopt::geometry::Transform GetTransform();
    virtual int GetIndex() const;
    virtual trajopt::LinkPtr getParent();
};

#endif // LINKWRAPPER_H
