#include "MPC.hpp"

int main()
{
     double T = 0.5;
     int horizon = 3;

     Eigen::VectorXd x_cur;
     x_cur.resize(2);
     x_cur << 2.0,
              0.0;

     Eigen::VectorXd x_ref;
     x_ref.resize(2 * horizon);
     x_ref << 5.0,
              0.0,
              5.0,
              0.0,
              5.0,
              0.0;

     Eigen::MatrixXd A;
     A.resize(2, 2);
     A << 1.0, T,
          0.0, 1.0;

     Eigen::MatrixXd B;
     B.resize(2, 1);
     B << 0.5 * T * T,
          T;

     Eigen::MatrixXd CST;
     CST.setIdentity(3, 3);

     Eigen::VectorXd l;
     l.resize(3);
     l << -2.0, -2.0, -2.0;

     Eigen::VectorXd u;
     u.resize(3);
     u << 2.0, 2.0, 2.0;

     Eigen::VectorXd control;

     MPC *mpc = new MPC(2, 1, horizon);
     
     int cnt = 20;
     while(cnt > 0)
     {
          mpc->MPCRun(A, B, 
                 CST, l, u,
                 x_cur, 
                 x_ref);
          control = mpc->MPCGetControl();

          std::cout << "control is :  " << control << "    ";

          x_cur = A * x_cur + B * control;

          std::cout << x_cur(0) << "    " << x_cur(1) << std::endl;
          cnt--;
     }

     delete mpc;
     return 0;
}