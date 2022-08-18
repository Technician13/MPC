#ifndef MPC_HPP
#define MPC_HPP

#include <iostream>
#include <Eigen/Dense>

/* ************************************ option start ************************************ */
/* print A */
#define DLQR_TEST_PRINT_A
/* print B */
#define DLQR_TEST_PRINT_B
/* ************************************ option end ************************************ */

class MPC
{
    private:
        /* dim of state vector */
        int dim_state;
        /* dim of control vector */
        int dim_control;
        /* sample period */
        double T;
        /* matrix A */
        Eigen::MatrixXd A;
        /* matrix B */
        Eigen::MatrixXd B;
        
    protected:
        
    public:
        MPC(Eigen::MatrixXd A_, Eigen::MatrixXd B_,
            int dim_state_, int dim_control_, double T_);
        ~MPC();
};

#endif