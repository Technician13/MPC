#include "QPSolver.hpp"

int NearZero(double val_)
{
    return (val_ < 1e-10 && val_ > -1e-10);
}

/* constructor */
QPSolver::QPSolver()
{
    std::cout << "QPSolver Birth Done" << std::endl;
}

/* destructor */
QPSolver::~QPSolver()
{
    std::cout << "QPSolver Die ..." << std::endl;
}

Eigen::VectorXd QPSolver::QPSolverRun(Eigen::MatrixXd Q_,
                                      Eigen::VectorXd f_,
                                      Eigen::MatrixXd CST_,
                                      Eigen::VectorXd l_,
                                      Eigen::VectorXd u_)
{
    Q_row = (int)Q_.rows();
    Q_col = (int)Q_.cols();
    f_row = (int)f_.rows();
    f_col = (int)f_.cols();
    CST_row = (int)CST_.rows();
    CST_col = (int)CST_.cols();
    l_row = (int)l_.rows();
    l_col = (int)l_.cols();
    u_row = (int)u_.rows();
    u_col = (int)u_.cols();

    #ifdef QPSOLVER_TEST_PRINT_INFO
        std::cout << "----------------------------------------- QP Info -----------------------------------------" << std::endl;
        std::cout << "This is a QP problem in this form: " << std::endl << std::endl
        << "  0.5 * U.transpose() * Q * U + U.transpos() * f  " << std::endl 
        << "             s.t.  l <= A * U <= u " << std::endl << std::endl
        << "  -- Q  : " << Q_row << " x " << Q_col << std::endl
        << "  -- f  : " << f_row << " x " << f_col << std::endl
        << "  -- CST: " << CST_row << " x " << CST_col << std::endl
        << "  -- l  : " << l_row << " x " << l_col << std::endl
        << "  -- u  : " << u_row << " x " << u_col << std::endl;
    #endif

    Eigen::VectorXd res;

    OSQPWorkspace* work = nullptr;
    OSQPSettings* settings = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));
    OSQPData* data = (OSQPData*)c_malloc(sizeof(OSQPData));

    OSQP_CST_size = 0;
    OSQP_Q_size = 0;
    CalOSQP_Q(Q_);
    CalOSQP_f(f_);
    CalOSQP_CST(CST_);
    CalOSQP_l_u(l_, u_);

    res.resize(CST_col);

    if (data)
    {
        data->n = (c_int)CST_col;
        data->m = (c_int)CST_row;
        data->P = csc_matrix(data->n, data->n, OSQP_Q_size, OSQP_Q_data, OSQP_Q_indices, OSQP_Q_indptr);
        data->q = OSQP_f_data;
        data->A = csc_matrix(data->m, data->n, OSQP_CST_size, OSQP_CST_data, OSQP_CST_indices, OSQP_CST_indptr);
        data->l = OSQP_l_data;
        data->u = OSQP_u_data;
    }

    if (settings != nullptr)
    {
        /* ************************************ option start ************************************ */
        osqp_set_default_settings(settings);
        settings->polish = true;
        settings->scaled_termination = true;
        settings->verbose = false;
        settings->max_iter = 2000;
        settings->eps_abs = 0.000001;
        settings->eps_rel = 0.000001;
        /* ************************************ option end ************************************ */
    }
    work = osqp_setup(data, settings);

    osqp_solve(work);

    auto status = work->info->status_val;
    if (status < 0 || (status != 1 && status != 2))
        std::cout << "Failed optimization status:  " << work->info->status << std::endl;
    else if (work->solution == nullptr)
        std::cout << "The solution from OSQP is nullptr" << std::endl;

    for (int i = 0 ; i < CST_col ; i++)
        res(i) = work->solution->x[i];

    OSQPFree();

    if (data) 
    {
        if (data->A) 
            c_free(data->A);
        if (data->P) 
            c_free(data->P);
        c_free(data);
    }

    if (settings) 
        c_free(settings);

    c_free(work);

    return res;
}

void QPSolver::OSQPFree()
{
    c_free(OSQP_Q_data);
    c_free(OSQP_Q_indices);
    c_free(OSQP_Q_indptr);
    c_free(OSQP_f_data);
    c_free(OSQP_CST_data);
    c_free(OSQP_CST_indices);
    c_free(OSQP_CST_indptr);
    c_free(OSQP_l_data);
    c_free(OSQP_u_data);
}

void QPSolver::CalOSQP_Q(Eigen::MatrixXd Q_)
{
    std::vector<c_float> Q_data;
    std::vector<c_int> Q_indices;
    std::vector<c_int> Q_indptr;
    int ind_Q = 0;

    for (int i = 0; i < Q_col; i++)
    {
        Q_indptr.push_back(ind_Q);
        for (int j = 0 ; j < Q_row ; j++)
        {
            if (!NearZero(Q_(j, i)))
            {
                Q_indices.push_back(j);
                Q_data.push_back(Q_(j, i));
                ind_Q++;
            }
        }
    }
    Q_indptr.push_back(ind_Q);
    
    OSQP_Q_data = (c_float*)c_malloc(sizeof(c_float) * Q_data.size());
    OSQP_Q_indices = (c_int*)c_malloc(sizeof(c_int) * Q_indices.size());
    OSQP_Q_indptr = (c_int*)c_malloc(sizeof(c_int) * Q_indptr.size());


    OSQP_Q_size = Q_data.size();
    #ifdef QPSOLVER_TEST_PRINT_Q_SIZE
        std::cout << "----------------------------------------- OSQP Q size -----------------------------------------" << std::endl;
        std::cout << OSQP_Q_size << std::endl;
    #endif


    #ifdef QPSOLVER_TEST_PRINT_Q_DATA
        std::cout << "----------------------------------------- OSQP Q data -----------------------------------------" << std::endl;
    #endif
    for (int i = 0 ; i < Q_data.size() ; i++)
    {
        OSQP_Q_data[i] = Q_data[i];
        #ifdef QPSOLVER_TEST_PRINT_Q_DATA
            std::cout << OSQP_Q_data[i] << "    ";
        #endif
    }
    #ifdef QPSOLVER_TEST_PRINT_Q_DATA
        std::cout << std::endl;
    #endif


    #ifdef QPSOLVER_TEST_PRINT_Q_INDICES
        std::cout << "----------------------------------------- OSQP Q indices -----------------------------------------" << std::endl;
    #endif
    for (int i = 0 ; i < Q_indices.size() ; i++)
    {
        OSQP_Q_indices[i] = Q_indices[i];
        #ifdef QPSOLVER_TEST_PRINT_Q_INDICES
            std::cout << OSQP_Q_indices[i] << "    ";
        #endif
    }
    #ifdef QPSOLVER_TEST_PRINT_Q_INDICES
        std::cout << std::endl;
    #endif


    #ifdef QPSOLVER_TEST_PRINT_Q_INDPTR
        std::cout << "----------------------------------------- OSQP Q indptr -----------------------------------------" << std::endl;
    #endif
    for (int i = 0 ; i < Q_indptr.size() ; i++)
    {
        OSQP_Q_indptr[i] = Q_indptr[i];
        #ifdef QPSOLVER_TEST_PRINT_Q_INDPTR
            std::cout << OSQP_Q_indptr[i] << "    ";
        #endif
    }
    #ifdef QPSOLVER_TEST_PRINT_Q_INDPTR
        std::cout << std::endl;
    #endif
}

void QPSolver::CalOSQP_f(Eigen::VectorXd f_)
{
    OSQP_f_data = (c_float*)c_malloc(sizeof(c_float) * f_row);

    #ifdef QPSOLVER_TEST_PRINT_F_DATA
        std::cout << "----------------------------------------- OSQP F data -----------------------------------------" << std::endl;
    #endif
    for (int i = 0; i < f_row; i++)
    {
        OSQP_f_data[i] = f_(i);
        #ifdef QPSOLVER_TEST_PRINT_F_DATA
            std::cout << OSQP_f_data[i] << std::endl;
        #endif
    }
}

void QPSolver::CalOSQP_CST(Eigen::MatrixXd CST_)
{
    std::vector<c_float> CST_data;
    std::vector<c_int> CST_indices;
    std::vector<c_int> CST_indptr;

    int ind_CST = 0;
    for (int i = 0 ; i < CST_col ; i++)
    {
        CST_indptr.push_back(ind_CST);
        for (int j = 0 ; j < CST_row ; j++)
        {
            if (!NearZero(CST_(j, i)))
            {
                CST_indices.push_back(j);
                CST_data.push_back(CST_(j, i));
                ind_CST++;
            }
        }
    }
    CST_indptr.push_back(ind_CST);

    OSQP_CST_data = (c_float*)c_malloc(sizeof(c_float) * CST_data.size());
    OSQP_CST_indices = (c_int*)c_malloc(sizeof(c_int) * CST_indices.size());
    OSQP_CST_indptr = (c_int*)c_malloc(sizeof(c_int) * CST_indptr.size());

    OSQP_CST_size = CST_data.size();
    #ifdef QPSOLVER_TEST_PRINT_CST_SIZE
        std::cout << "----------------------------------------- OSQP CST size -----------------------------------------" << std::endl;
        std::cout << OSQP_CST_size << std::endl;
    #endif


    #ifdef QPSOLVER_TEST_PRINT_CST_DATA
        std::cout << "----------------------------------------- OSQP CST data -----------------------------------------" << std::endl;
    #endif
    for (int i = 0 ; i < CST_data.size() ; i++)
    {
        OSQP_CST_data[i] = CST_data[i];
        #ifdef QPSOLVER_TEST_PRINT_CST_DATA
            std::cout << OSQP_CST_data[i] << "    ";
        #endif
    }
    #ifdef QPSOLVER_TEST_PRINT_CST_DATA
        std::cout << std::endl;
    #endif


    #ifdef QPSOLVER_TEST_PRINT_CST_INDICES
        std::cout << "----------------------------------------- OSQP CST indices -----------------------------------------" << std::endl;
    #endif
    for (int i = 0 ; i < CST_indices.size() ; i++)
    {
        OSQP_CST_indices[i] = CST_indices[i];
        #ifdef QPSOLVER_TEST_PRINT_CST_INDICES
            std::cout << OSQP_CST_indices[i] << "    ";
        #endif
    }
    #ifdef QPSOLVER_TEST_PRINT_CST_INDICES
        std::cout << std::endl;
    #endif


    #ifdef QPSOLVER_TEST_PRINT_CST_INDPTR
        std::cout << "----------------------------------------- OSQP CST indptr -----------------------------------------" << std::endl;
    #endif
    for (int i = 0 ; i < CST_indptr.size() ; i++)
    {
        OSQP_CST_indptr[i] = CST_indptr[i];
        #ifdef QPSOLVER_TEST_PRINT_CST_INDPTR
            std::cout << OSQP_CST_indptr[i] << "    ";
        #endif
    }
    #ifdef QPSOLVER_TEST_PRINT_CST_INDPTR
        std::cout << std::endl;
    #endif
}


void QPSolver::CalOSQP_l_u(Eigen::VectorXd l_,
                           Eigen::VectorXd u_)
{
    OSQP_l_data = (c_float*)c_malloc(sizeof(c_float) * l_row);
    OSQP_u_data = (c_float*)c_malloc(sizeof(c_float) * u_row);

    #ifdef QPSOLVER_TEST_PRINT_L_U_DATA
        std::cout << "----------------------------------------- OSQP L & U data -----------------------------------------" << std::endl;
    #endif
    for (int i = 0 ; i < l_row ; i++)
    {
        OSQP_l_data[i] = l_(i);
        OSQP_u_data[i] = u_(i);
        #ifdef QPSOLVER_TEST_PRINT_L_U_DATA
            std::cout << OSQP_l_data[i] << "    " << OSQP_u_data[i] << std::endl;
        #endif
    }
}