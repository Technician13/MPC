#include "MPC.hpp"

void PrintMatrix(Eigen::MatrixXd mat)
{
    for(int i = 0 ; i < mat.rows() ; i++)
    {
        for(int j = 0 ; j < mat.cols() ; j++)
        {
            std::cout << mat(i, j) << "  ";
        }
        std::cout << std::endl;
    }
}

/* constructor */
MPC::MPC(Eigen::MatrixXd A_, Eigen::MatrixXd B_,
         int dim_state_, int dim_control_, 
         double T_, int horizon_)
{
    dim_state = dim_state_;
    dim_control = dim_control_;
    T = T_;
    horizon = horizon_;

    /* error checking */
    if((A_.rows()) != dim_state)
        std::cout << "A is in wrong rows !!!" << std::endl; 
    if((A_.cols()) != dim_state)
        std::cout << "A is in wrong cols !!!" << std::endl; 
    if((B_.rows()) != dim_state)
        std::cout << "B is in wrong rows !!!" << std::endl; 
    if((B_.cols()) != dim_control)
        std::cout << "B is in wrong cols !!!" << std::endl; 

    A.resize(dim_state, dim_state);
    B.resize(dim_state, dim_control);
    A = A_;
    B = B_;
    
    #ifdef MPC_TEST_PRINT_A
        std::cout << "----------------------------------------- A -----------------------------------------" << std::endl;
        PrintMatrix(A);
    #endif
    #ifdef MPC_TEST_PRINT_B
        std::cout << "----------------------------------------- B -----------------------------------------" << std::endl;
        PrintMatrix(B);
    #endif

    std::cout << "MPC Birth Done" << std::endl;
}

/* destructor */
MPC::~MPC()
{
    std::cout << "MPC Die ..." << std::endl;
}

void MPC::MPCInit()
{
    Phi.setZero(horizon * dim_state, dim_state);
    Psi.setZero(horizon * dim_state, horizon * dim_control);
    Eigen::MatrixXd temp_phi = A;
    Eigen::MatrixXd temp_psi;
    
    for(int i = 0 ; i < horizon ; i++)
    {   
        temp_psi = temp_phi;

        /* load Phi ------------------------------------------------------------------------ */
        Phi.block(i * dim_state, 0, dim_state, dim_state) = temp_phi;

        /* load Psi ------------------------------------------------------------------------ */
        for(int j = 0 ; j <= i ; j++)
        {
            temp_psi = temp_psi * A.inverse();
            Psi.block(i * dim_state, j * dim_control, dim_state, dim_control) = temp_psi * B;
        }

        temp_phi *= A;
    }

    /* load H & U ------------------------------------------------------------------------ */
    w1.setZero(dim_state, dim_state);
    w2.setZero(dim_control, dim_control);
    H.setZero(horizon * dim_state, horizon * dim_state);
    g.setZero(horizon * dim_control, horizon * dim_control);

    /* ************************************ option start ************************************ */
    w1(0, 0) = 1.0;
    w1(1, 1) = 1.0;

    w2(0, 0) = 1.0;
    /* ************************************ option end ************************************ */

    for(int i = 0 ; i < horizon ; i++)
    {
        H.block(i * dim_state, i * dim_state, dim_state, dim_state) = w1;
        g.block(i * dim_control, i * dim_control, dim_control, dim_control) = w2;
    }

    E.setZero(horizon * dim_state);
    Q.setZero(horizon * dim_control, horizon * dim_control);
    f.setZero(horizon * dim_control);

    #ifdef MPC_TEST_PRINT_PHI
        std::cout << "----------------------------------------- PHI -----------------------------------------" << std::endl;
        std::cout << "Phi is a " << Phi.rows() << " x " << Phi.cols() << " matrix" << std::endl;
        PrintMatrix(Phi);
    #endif
    #ifdef MPC_TEST_PRINT_PSI
        std::cout << "----------------------------------------- PSI -----------------------------------------" << std::endl;
        std::cout << "Psi is a " << Psi.rows() << " x " << Psi.cols() << " matrix" << std::endl;
        PrintMatrix(Psi);
    #endif
    #ifdef MPC_TEST_PRINT_W1
        std::cout << "----------------------------------------- w1 -----------------------------------------" << std::endl;
        std::cout << "w1 is a " << w1.rows() << " x " << w1.cols() << " matrix" << std::endl;
        PrintMatrix(w1);
    #endif
    #ifdef MPC_TEST_PRINT_W2
        std::cout << "----------------------------------------- w2 -----------------------------------------" << std::endl;
        std::cout << "w2 is a " << w2.rows() << " x " << w2.cols() << " matrix" << std::endl;
        PrintMatrix(w2);
    #endif
    #ifdef MPC_TEST_PRINT_H
        std::cout << "----------------------------------------- H -----------------------------------------" << std::endl;
        std::cout << "H is a " << H.rows() << " x " << H.cols() << " matrix" << std::endl;
        PrintMatrix(H);
    #endif
    #ifdef MPC_TEST_PRINT_G
        std::cout << "----------------------------------------- g -----------------------------------------" << std::endl;
        std::cout << "g is a " << g.rows() << " x " << g.cols() << " matrix" << std::endl;
        PrintMatrix(g);
    #endif

    std::cout << "MPC Init Done" << std::endl;
} 

void MPC::MPCRun(Eigen::VectorXd x_cur_,
                 Eigen::VectorXd x_ref_)
{
    /* error checking */
    if((x_cur_.size()) != dim_state)
        std::cout << "x_cur is in wrong size !!!" << std::endl; 
    if((x_ref_.size()) != horizon * dim_state)
        std::cout << "x_ref is in wrong size !!!" << std::endl; 

    /* load E ------------------------------------------------------------------------ */
    E = Phi * x_cur_ - x_ref_;

    /* load Q & f ------------------------------------------------------------------------ */
    Q = 2.0 * (Psi.transpose() * H * Psi + g);
    f = 2.0 * Psi.transpose() * H.transpose() * E;

    /* QP problem objective function: 0.5 * U.transpose() * Q * U + U.transpos() * f ------------------------------------------------------------------------ */


    #ifdef MPC_TEST_PRINT_E
        std::cout << "----------------------------------------- E -----------------------------------------" << std::endl;
        for(int i = 0 ; i < horizon * dim_state ; i++)
            std::cout << E(i) << std::endl;
    #endif
    #ifdef MPC_TEST_PRINT_Q
        std::cout << "----------------------------------------- Q -----------------------------------------" << std::endl;
        std::cout << "Q is a " << Q.rows() << " x " << Q.cols() << " matrix" << std::endl;
        PrintMatrix(Q);
    #endif
    #ifdef MPC_TEST_PRINT_F
        std::cout << "----------------------------------------- f -----------------------------------------" << std::endl;
        for(int i = 0 ; i < horizon * dim_control ; i++)
            std::cout << f(i) << std::endl;
    #endif
}