#ifndef QPSOLVER_HPP
#define QPSOLVER_HPP

#include <iostream>
#include <Eigen/Dense>
#include <osqp.h>

/* ************************************ option start ************************************ */
/* print QP info */
#define QPSOLVER_TEST_PRINT_INFO
/* ************************************ option end ************************************ */

class QPSolver
{
    private:
        /* parameters of OSQP */
        c_float *P_x;
        c_int    P_nnz;
        c_int   *P_i;
        c_int   *P_p;
        c_float *q;
        c_int    A_nnz;
        c_float *A_x;
        c_int   *A_i;
        c_int   *A_p;
        c_float *l;
        c_float *u;
        c_int    n;
        c_int    m;

        /* count of rows & cols of Q */
        int Q_row, Q_col;
        /* count of rows & cols of f */
        int f_row, f_col;
        /* count of rows & cols of A */
        int A_row, A_col;
        /* count of rows & cols of l */
        int l_row, l_col;
        /* count of rows & cols of u */
        int u_row, u_col;
        
    protected:
        
    public:
        QPSolver(Eigen::MatrixXd Q,
                 Eigen::VectorXd f,
                 Eigen::MatrixXd CST,
                 Eigen::VectorXd l,
                 Eigen::VectorXd u);
        ~QPSolver();
        void QPSolverRun(Eigen::MatrixXd Q_, Eigen::VectorXd f_);
};

#endif