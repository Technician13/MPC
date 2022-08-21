#ifndef QPSOLVER_HPP
#define QPSOLVER_HPP

#include <iostream>
#include <Eigen/Dense>
#include <osqp.h>
#include <vector>

/* ************************************ option start ************************************ */
/* print QP info */
// #define QPSOLVER_TEST_PRINT_INFO
/* print OSQP Q info */
// #define QPSOLVER_TEST_PRINT_Q_SIZE 
// #define QPSOLVER_TEST_PRINT_Q_DATA
// #define QPSOLVER_TEST_PRINT_Q_INDICES
// #define QPSOLVER_TEST_PRINT_Q_INDPTR
/* print OSQP f info */
// #define QPSOLVER_TEST_PRINT_F_DATA
/* print OSQP CST info */
// #define QPSOLVER_TEST_PRINT_CST_SIZE 
// #define QPSOLVER_TEST_PRINT_CST_DATA
// #define QPSOLVER_TEST_PRINT_CST_INDICES
// #define QPSOLVER_TEST_PRINT_CST_INDPTR
/* print OSQP l & u info */
// #define QPSOLVER_TEST_PRINT_L_U_DATA
/* ************************************ option end ************************************ */

class QPSolver
{
    private:
        /* parameters of OSQP */
        c_float* OSQP_Q_data;
        c_int* OSQP_Q_indices;
        c_int* OSQP_Q_indptr;
        c_int OSQP_Q_size = 0;
        c_float* OSQP_f_data;
        c_float* OSQP_CST_data;
        c_int* OSQP_CST_indices;
        c_int* OSQP_CST_indptr;
        c_int OSQP_CST_size = 0;
        c_float* OSQP_l_data;
        c_float* OSQP_u_data;
        
        /* count of rows & cols of Q */
        int Q_row, Q_col;
        /* count of rows & cols of f */
        int f_row, f_col;
        /* count of rows & cols of A */
        int CST_row, CST_col;
        /* count of rows & cols of l */
        int l_row, l_col;
        /* count of rows & cols of u */
        int u_row, u_col;
        
    protected:
        
    public:
        QPSolver();
        ~QPSolver();
        Eigen::VectorXd QPSolverRun(Eigen::MatrixXd Q_,
                                    Eigen::VectorXd f_,
                                    Eigen::MatrixXd CST_,
                                    Eigen::VectorXd l_,
                                    Eigen::VectorXd u_);
        void OSQPFree();

        /* trans Eigen to OSQP */
        void CalOSQP_Q(Eigen::MatrixXd Q_);
        void CalOSQP_f(Eigen::VectorXd f_);
        void CalOSQP_CST(Eigen::MatrixXd CST_);
        void CalOSQP_l_u(Eigen::VectorXd l_,
                         Eigen::VectorXd u_);
};

#endif