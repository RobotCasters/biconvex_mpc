# include "motion_planner/biconvex.hpp"


namespace motion_planner{

    BiConvexMP::BiConvexMP(double m, double dt, double T, int n_eff):
        m_(m), centroidal_dynamics(m, dt, T, n_eff), 
        prob_data_x(9, int(T/dt)+1), prob_data_f(3*n_eff, int(T/dt)), 
        fista_x(), fista_f(){
            horizon_ = T;
            dt_ = dt;

            dyn_violation.resize(9*(int (T/dt)+1)); 
            dyn_violation.setZero();
            P_k_.resize(9*(int (T/dt)+1)); 
            P_k_.setZero();

            // setting starting line search params
            fista_x.set_l0(2.25e6);
            fista_f.set_l0(506.25);

            //Use Second Order Cone Projection
            fista_f.set_soc_true();

            //Set number of variables and constraints for osqp-eigen
            #ifdef USE_OSQP
                //osqp_x();
                //osqp_f();
                std::cout << "OSQP found" << std::endl;
                osqp_x.data()->setNumberOfVariables(9*(int (T/dt)+1));
                osqp_x.data()->setNumberOfConstraints( 2 * 9*(int (T/dt)+1));

                osqp_f.data()->setNumberOfVariables(3*n_eff*int (T/dt));
                osqp_f.data()->setNumberOfConstraints(2 * 3*n_eff*int (T/dt));
            #endif
    };

    void BiConvexMP::optimize(Eigen::VectorXd x_init, int no_iters){
        // updating x_init
        centroidal_dynamics.update_x_init(x_init);

        for (unsigned i = 0; i < no_iters; ++i){
            // We need to look into this line...it causes a very high dynamic violation...
            //maxit = init_maxit/(int(i)/10 + 1);

            // optimizing for F
            // std::cout << "optimizing F..." << std::endl;
            centroidal_dynamics.compute_x_mat(prob_data_x.x_k);
            prob_data_f.set_data(centroidal_dynamics.A_x, centroidal_dynamics.b_x, P_k_, rho_);
            fista_f.optimize(prob_data_f, maxit, tol);

            // optimizing for X
            // std::cout << "optimizing X..." << std::endl;
            centroidal_dynamics.compute_f_mat(prob_data_f.x_k);
            prob_data_x.set_data(centroidal_dynamics.A_f, centroidal_dynamics.b_f, P_k_, rho_);
            fista_x.optimize(prob_data_x, maxit, tol);

            dyn_violation = centroidal_dynamics.A_f * prob_data_x.x_k - centroidal_dynamics.b_f;
            P_k_ += dyn_violation;

            if (dyn_violation.norm() < exit_tol){
                // std::cout << "breaking outerloop due to norm ..." << std::endl;
                break;
            };       
        }
        // std::cout << "Maximum iterations reached " << std::endl << "Final norm: " << dyn_violation.norm() << std::endl;
    }

    void BiConvexMP::update_cost_x(Eigen::VectorXd X_ter, Eigen::VectorXd X_ter_nrml) {
        //Change for loop to prob_data_x.num_vars_ - (2*prob_data.x.state)
        //TODO: See if you can user the Eigen Sparse innerloop functionality to update faster...

        //First loop: Loop through all of horizon except the last two knot points
        for (unsigned int i = 0; i < (prob_data_x.num_vars_-(2*9))/9; ++i) {
            prob_data_x.Q_.coeffRef(i,i) = prob_data_x.Q_.coeffRef(i+9, i+9);
            prob_data_x.q_[i] = prob_data_x.q_[i+9];
        } 

        //Second loop: Loop through second to last knot point: 
        //TODO: THis could also change to dividing this cost by some weighting factor (i.e. if the final cost is
        // just multifplied by a weighting factor, we can divide by this weighting factor)
        for (unsigned int i = 0; i < 9; ++i) {
            prob_data_x.Q_.coeffRef(prob_data_x.num_vars_-(2*9)+i,prob_data_x.num_vars_-(2*9)+i) = X_ter_nrml[i];
            prob_data_x.q_[prob_data_x.num_vars_-(2*9)+i] = X_ter_nrml[i];
        }

        //Set Terminal Constraint (last knot point): 
        for (unsigned int i = 0; i < 9; ++i) {
            prob_data_x.Q_.coeffRef(prob_data_x.num_vars_-(9)+i, prob_data_x.num_vars_-(9)+i) = X_ter[i];
            prob_data_x.q_[prob_data_x.num_vars_-(9)+i] = X_ter[i];
        }
    }

    void BiConvexMP::update_bounds_x(Eigen::VectorXd lb_fin, Eigen::VectorXd ub_fin) {
        //Shift bounds
        //prob_data_x.lb_.segment(1, prob_data_x.num_vars_-9) = prob_data_x.lb_.segment(9, prob_data_x.num_vars_)
        //prob_data_x.ub_.segment(1, prob_data_x.num_vars_-9) = prob_data_x.ub_.segment(9, prob_data_x.num_vars_)

        prob_data_x.lb_.head(prob_data_x.num_vars_-9) = prob_data_x.lb_.tail(prob_data_x.num_vars_-9);
        prob_data_x.ub_.head(prob_data_x.num_vars_-9) = prob_data_x.ub_.tail(prob_data_x.num_vars_-9);

        //Update new bounds (end of bound vectors)
        prob_data_x.lb_.tail(9) = lb_fin;
        prob_data_x.ub_.tail(9) = ub_fin;
    }

    // void BiConvexMP::update_constraints_x() {
    //     for (unsigned int t = 0; t < centroidal_dynamics.n_col_-1; ++t) {
    //         centroidal_dynamics.A_f.coeffRef(9*t+6, 9*t+1) = centroidal_dynamics.A_f.coeffRef(9*(t+1)+6, 9*(t+1)+1);
    //         centroidal_dynamics.A_f.coeffRef(9*t+6, 9*t+2) = centroidal_dynamics.A_f.coeffRef(9*(t+2)+6, 9*(t+2)+9);
            
    //         centroidal_dynamics.A_f.coeffRef(9*t+7, 9*t+0) = centroidal_dynamics.A_f.coeffRef(9*(t+1)+7, 9*(t+1)+0);
    //         centroidal_dynamics.A_f.coeffRef(9*t+7, 9*t+2) = centroidal_dynamics.A_f.coeffRef(9*(t+1)+7, 9*(t+1)+2);
            
    //         centroidal_dynamics.A_f.coeffRef(9*t+8, 9*t+0) = centroidal_dynamics.A_f.coeffRef(9*(t+1)+8, 9*(t+1)+0);
    //         centroidal_dynamics.A_f.coeffRef(9*t+8, 9*t+1) = centroidal_dynamics.A_f.coeffRef(9*(t+1)+8, 9*(t+1)+1);

    //         //TODO: Use eigen::segment functionality
    //         centroidal_dynamics.b_f[9*t+3] = centroidal_dynamics.b_f[9*(t+1)+3];
    //         centroidal_dynamics.b_f[9*t+4] = centroidal_dynamics.b_f[9*(t+1)+4];
    //         centroidal_dynamics.b_f[9*t+5] = centroidal_dynamics.b_f[9*(t+1)+5];
    //         centroidal_dynamics.b_f[9*t+6] = centroidal_dynamics.b_f[9*(t+1)+6];
    //         centroidal_dynamics.b_f[9*t+7] = centroidal_dynamics.b_f[9*(t+1)+7];
    //         centroidal_dynamics.b_f[9*t+8] = centroidal_dynamics.b_f[9*(t+1)+8];
    //     }

    //     //Update final constraint
    //     auto last_state = centroidal_dynamics.n_col_ - 1;
    //     centroidal_dynamics.A_f.coeffRef(9*last_state + 6, 9*last_state + 1) = 0.0;
    //     centroidal_dynamics.A_f.coeffRef(9*last_state + 6, 9*last_state + 2) = 0.0;
        
    //     centroidal_dynamics.A_f.coeffRef(9*last_state + 7, 9*last_state + 0) = 0.0;
    //     centroidal_dynamics.A_f.coeffRef(9*last_state + 7, 9*last_state + 2) = 0.0;
        
    //     centroidal_dynamics.A_f.coeffRef(9*last_state + 8, 9*last_state + 0) = 0.0;
    //     centroidal_dynamics.A_f.coeffRef(9*last_state + 8, 9*last_state + 1) = 0.0;

    //     centroidal_dynamics.b_f.tail(9) = Eigen::VectorXd::Zero(9);

    //     //If any of the incoming contacts are in contact
    //     if (centroidal_dynamics.cnt_arr_.row(centroidal_dynamics.n_col_).sum() > 0) {
    //         for (unsigned int i = 0; i < centroidal_dynamics.n_eff_; ++i) {
    //             centroidal_dynamics.A_f.coeffRef(9*last_state + 6, 9*last_state + 1) = 0.0;
    //             centroidal_dynamics.A_f.coeffRef(9*last_state + 6, 9*last_state + 2) = 0.0;
                
    //             centroidal_dynamics.A_f.coeffRef(9*last_state + 7, 9*last_state + 0) = 0.0;
    //             centroidal_dynamics.A_f.coeffRef(9*last_state + 7, 9*last_state + 2) = 0.0;
                
    //             centroidal_dynamics.A_f.coeffRef(9*last_state + 8, 9*last_state + 0) = 0.0;
    //             centroidal_dynamics.A_f.coeffRef(9*last_state + 8, 9*last_state + 1) = 0.0;

    //             centroidal_dynamics.b_f.tail(9) = Eigen::VectorXd::Zero(9);
    //         }
    //     }
    // }

    void BiConvexMP::shift_horizon() {
        //Update new contacts

        //update X constraints

        //update X bounds

        //update X cost function

        //Update previous solutions
        prob_data_x.x_k.head(prob_data_x.num_vars_ - prob_data_x.state_) = \
            prob_data_x.x_k.tail(prob_data_x.num_vars_ - prob_data_x.state_);
        prob_data_x.x_k.tail(prob_data_x.state_) = Eigen::VectorXd::Zero(prob_data_x.state_);

        prob_data_f.x_k.head(prob_data_f.num_vars_ - prob_data_f.state_) = \
            prob_data_f.x_k.tail(prob_data_f.num_vars_ - prob_data_f.state_);
        prob_data_f.x_k.tail(prob_data_f.state_) = Eigen::VectorXd::Zero(prob_data_f.state_);

        P_k_.head(9*(int(horizon_/dt_))) = P_k_.tail(9*(int(horizon_/dt_)));
        P_k_.tail(9) = Eigen::VectorXd::Zero(9);
    }

};
