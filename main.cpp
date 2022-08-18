#include "MPC.hpp"

int main()
{
    double T = 0.5;
    Eigen::MatrixXd x_cur;
    x_cur.resize(2, 1);
    x_cur << 2.0,
             0.0;
    Eigen::MatrixXd A;
    A.resize(2, 2);
    A << 1.0, T,
         0.0, 1.0;
    Eigen::MatrixXd B;
    B.resize(2, 1);
    B << 0.5 * T * T,
         T;

    MPC *mpc = new MPC(A, B, 2, 1, T);

    delete mpc;
    return 0;
}