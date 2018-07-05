#include <boost/test/unit_test.hpp>
#include <motion_planners/Dummy.hpp>

using namespace motion_planners;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    motion_planners::DummyClass dummy;
    dummy.welcome();
}
