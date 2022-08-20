#include "QPSolver.hpp"

/* constructor */
QPSolver::QPSolver(Eigen::MatrixXd Q,
                   Eigen::VectorXd f,
                   Eigen::MatrixXd CST,
                   Eigen::VectorXd l,
                   Eigen::VectorXd u)
{
    Q_row = (int)Q.rows();
    Q_col = (int)Q.cols();
    f_row = (int)f.rows();
    f_col = (int)f.cols();
    A_row = (int)CST.rows();
    A_col = (int)CST.cols();
    l_row = (int)l.rows();
    l_col = (int)l.cols();
    u_row = (int)u.rows();
    u_col = (int)u.cols();

    #ifdef QPSOLVER_TEST_PRINT_INFO
        std::cout << "----------------------------------------- QP Info -----------------------------------------" << std::endl;
        std::cout << "This is a QP problem in this form: " << std::endl << std::endl
        << "  0.5 * U.transpose() * Q * U + U.transpos() * f  " << std::endl 
        << "             s.t.  l <= A * U <= u " << std::endl << std::endl
        << "  -- Q: " << Q_row << " x " << Q_col << std::endl
        << "  -- f: " << f_row << " x " << f_col << std::endl
        << "  -- A: " << A_row << " x " << A_col << std::endl
        << "  -- l: " << l_row << " x " << l_col << std::endl
        << "  -- u: " << u_row << " x " << u_col << std::endl;
    #endif

    //TODO

    std::cout << "QPSolver Birth Done" << std::endl;
}

/* destructor */
QPSolver::~QPSolver()
{
    std::cout << "QPSolver Die ..." << std::endl;
}

void QPSolver::QPSolverRun(Eigen::MatrixXd Q_, Eigen::VectorXd f_)
{
    
}