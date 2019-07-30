#ifndef SQPCONFIG_H
#define SQPCONFIG_H

#include <limits>

const int TRAJOPT_INFINITY = std::numeric_limits<int>::max();

struct SQPConfig{
    double improve_ratio_threshold_, // minimum ratio true_improve/approx_improve to accept step
           min_trust_box_size_, // if trust region gets any smaller, exit and report convergence
           min_approx_improve_, // if model improves less than this, exit and report convergence
           min_approx_improve_frac_, // if model improves less than this, exit and report convergence
           max_iter_,
           trust_shrink_ratio_, // if improvement is less than improve_ratio_threshold, shrink trust region by this ratio
           trust_expand_ratio_, // see above
           cnt_tolerance_, // after convergence of penalty subproblem, if constraint violation is less than this, we're done
           max_merit_coeff_increases_, // number of times that we jack up penalty coefficient
           merit_coeff_increase_ratio_, // ratio that we increate coeff each time
           max_time_ // not yet implemented
           ;
    double merit_error_coeff_, // initial penalty coefficient
           trust_box_size_ // current size of trust region (component-wise)
           ;
    SQPConfig(){
        improve_ratio_threshold_ = .25;
        min_trust_box_size_ = 1e-4;
        min_approx_improve_= 1e-4;
        min_approx_improve_frac_ = -TRAJOPT_INFINITY;
        max_iter_ = 50;
        trust_shrink_ratio_=.1;
        trust_expand_ratio_ = 1.5;
        cnt_tolerance_ = 1e-4;
        max_merit_coeff_increases_ = 5;
        merit_coeff_increase_ratio_ = 10;
        max_time_ = TRAJOPT_INFINITY;

        merit_error_coeff_ = 10;
      //  trust_box_size_ = 1e-1;
        trust_box_size_ = 0.5;
    }

};

#endif // SQPCONFIG_H
