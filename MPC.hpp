#ifndef MPC_HPP
#define MPC_HPP

#include <iostream>
#include <Eigen/Dense>
#include <osqp.h>

/* ************************************ option start ************************************ */
/* print A */
#define MPC_TEST_PRINT_A
/* print B */
#define MPC_TEST_PRINT_B
/* print Phi */
#define MPC_TEST_PRINT_PHI
/* print Psi */
#define MPC_TEST_PRINT_PSI
/* print w1 */
#define MPC_TEST_PRINT_W1
/* print w2 */
#define MPC_TEST_PRINT_W2
/* print H */
#define MPC_TEST_PRINT_H
/* print U */
#define MPC_TEST_PRINT_G
/* print E */
#define MPC_TEST_PRINT_E
/* print Q */
#define MPC_TEST_PRINT_Q
/* print f */
#define MPC_TEST_PRINT_F
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
        /* prediction time domain */
        int horizon;
        /* matrix A */
        Eigen::MatrixXd A;
        /* matrix B */
        Eigen::MatrixXd B;
        /* matrix Phi */
        Eigen::MatrixXd Phi;
        /* matrix Psi */
        Eigen::MatrixXd Psi;
        /* matrix w1 & w2 */
        Eigen::MatrixXd w1, w2;
        /* matrix H */
        Eigen::MatrixXd H;
        /* matrix g */
        Eigen::MatrixXd g;
        /* vector E */
        Eigen::VectorXd E;
        /* matrix Q */
        Eigen::MatrixXd Q;
        /* vector f */
        Eigen::VectorXd f;

    protected:
        
    public:
        MPC(Eigen::MatrixXd A_, Eigen::MatrixXd B_,
            int dim_state_, int dim_control_, 
            double T_, int horizon_);
        ~MPC();
        void MPCInit();
        void MPCRun(Eigen::VectorXd x_cur_,
                    Eigen::VectorXd x_ref_);
};

#endif