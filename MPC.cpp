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
         int dim_state_, int dim_control_, double T_)
{
    dim_state = dim_state_;
    dim_control = dim_control_;
    T = T_;

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

    #ifdef DLQR_TEST_PRINT_A
        std::cout << "----------------------------------------- A -----------------------------------------" << std::endl;
        PrintMatrix(A);
    #endif
    #ifdef DLQR_TEST_PRINT_B
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