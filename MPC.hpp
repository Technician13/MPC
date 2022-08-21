#ifndef MPC_HPP
#define MPC_HPP

#include <iostream>
#include <Eigen/Dense>
#include <osqp.h>
#include "QPSolver.hpp"

/* ************************************ option start ************************************ */
/* print A */
// #define MPC_TEST_PRINT_A
/* print B */
// #define MPC_TEST_PRINT_B
/* print Phi */
// #define MPC_TEST_PRINT_PHI
/* print Psi */
// #define MPC_TEST_PRINT_PSI
/* print w1 */
// #define MPC_TEST_PRINT_W1
/* print w2 */
// #define MPC_TEST_PRINT_W2
/* print H */
// #define MPC_TEST_PRINT_H
/* print U */
// #define MPC_TEST_PRINT_G
/* print E */
// #define MPC_TEST_PRINT_E
/* print Q */
// #define MPC_TEST_PRINT_Q
/* print f */
// #define MPC_TEST_PRINT_F
/* print CST */
// #define MPC_TEST_PRINT_CST
/* print L */
// #define MPC_TEST_PRINT_L
/* print U */
// #define MPC_TEST_PRINT_U
/* print opt result */
// #define MPC_TEST_PRINT_OPT_RES
/* ************************************ option end ************************************ */

class MPC
{
    private:
        /* dim of state vector */
        int dim_state;
        /* dim of control vector */
        int dim_control;
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
        /* matrix CST */
        Eigen::MatrixXd CST;
        /* vector l */
        Eigen::VectorXd l;
        /* vector u */
        Eigen::VectorXd u;

        QPSolver *qpsolver;

    protected:
        
    public:
        /* result of QP */
        Eigen::VectorXd res;

        MPC(int dim_state_, int dim_control_, 
            int horizon_);
        ~MPC();
        void MPCRun(Eigen::MatrixXd A_, Eigen::MatrixXd B_,
                    Eigen::MatrixXd CST_, Eigen::VectorXd l_, Eigen::VectorXd u_,
                    Eigen::VectorXd x_cur_,
                    Eigen::VectorXd x_ref_);
        Eigen::VectorXd MPCGetControl();
};

#endif