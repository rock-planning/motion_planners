#define BOOST_TEST_MAIN
#if !defined( WIN32 )
    #define BOOST_TEST_DYN_LINK
#endif
#include <boost/test/unit_test.hpp>
#include <iostream>
#include "trajopt/problem_description.hpp"
#include "utils/clock.hpp"
#include "json/json.h"

using namespace trajopt;
using namespace Json;

BOOST_AUTO_TEST_CASE( la ) {
    BOOST_CHECK_EQUAL(1, 1);
    std::cout<<"Starting Planning unit test . . . ..  \n";
//    Json::Value root = readJsonFile(/*string(DATA_DIR) + */"../../data/numerical_ik1.json");
//    TrajOptProbPtr prob = ConstructProblem(root, env);
//    ASSERT_TRUE(!!prob);

//    BasicTrustRegionSQP opt(prob);
//  //  opt.addCallback(boost::bind(&PlotCosts, boost::ref(prob->getCosts()),*prob->GetRAD(), prob->GetVars(), _1));
//    opt.initialize(DblVec(prob->GetNumDOF(), 0));
//    double tStart = GetClock();
//    opt.optimize();

//    std::cout<<"Planning unit test ended in " << GetClock()-tStart <<"secs \n";

}
