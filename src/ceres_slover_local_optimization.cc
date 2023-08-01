#include <iostream>
#include <ceres/ceres.h>
#include "utils.h"
#include "error.h"


void SolveLocalOptimization(LocalOptimizationProblem &local_opt_problem);

int main(int argc, char **argv) {
    
    if (argc != 4) {
        std::cout << "usage: ceres_slover_local_optimization sim_data_p.txt sim_data_T.txt sim_data_D.txt" << std::endl;
        return 1;
    }

    LocalOptimizationProblem local_opt_problem(argv[1],argv[2],argv[3]);

    SolveLocalOptimization(local_opt_problem);

    return 0;
}

void SolveLocalOptimization(LocalOptimizationProblem &local_opt_problem) {
    // std::cout << "Here!" << std::endl;
    const int point_block_size = local_opt_problem.point_block_size();
    const int transform_block_size = local_opt_problem.transform_block_size();

    double *estimations_D_k_2to_k_1_ = local_opt_problem.estimations_D_k_2to_k_1();
    double *estimations_D_k_1to_k_ = local_opt_problem.estimations_D_k_1to_k();

    // const double * estimations_D_k_2to_k_1_ = local_opt_problem.estimations_D_k_2to_k_1();
    // const double * estimations_D_k_1to_k_ = local_opt_problem.estimations_D_k_1to_k();

    // Observations is 2 * num_observations long array observations
    // [u_1, u_2, ... u_n], where each u_i is two dimensional, the x
    // and y position of the observation.
    const double *observations_p_k_2 = local_opt_problem.observations_p_k_2();
    const double p = observations_p_k_2[0];
    const double *observations_p_k_1 = local_opt_problem.observations_p_k_1();
    const double *observations_p_k = local_opt_problem.observations_p_k();

    const double *observations_T_k_2to_k_1 = local_opt_problem.observations_T_k_2to_k_1();
    const double *observations_T_k_1to_k = local_opt_problem.observations_T_k_1to_k();
    

    for (int i = 0; i<200; i++){
        std::cout << observations_p_k_2[i] << std::endl;
        std::cout << "Here!" << std::endl;

    }

    ceres::Problem problem;

    // Add the Feature Observation Residual Blocks to Problem
    for (int i = 0; i < local_opt_problem.num_landmark(); ++i) {
        ceres::CostFunction *cost_function_fo_1;
        ceres::CostFunction *cost_function_fo_2;     

        // Each Residual block takes a point and a camera as input
        // and outputs a 2 dimensional Residual
        cost_function_fo_1 = FeatureObservationError::Create(
            observations_p_k_2[2 * i + 0], observations_p_k_2[2 * i + 1],
            observations_p_k_1[2 * i + 0], observations_p_k_1[2 * i + 1]);
        cost_function_fo_2 = FeatureObservationError::Create(
            observations_p_k_1[2 * i + 0], observations_p_k_1[2 * i + 1],
            observations_p_k[2 * i + 0], observations_p_k[2 * i + 1]);

        // If enabled use Huber's loss function.
        ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);

        // // Each observation corresponds to a pair of a camera and a point
        // // which are identified by camera_index()[i] and point_index()[i]
        // // respectively.
        // double *transform = transform + transform_block_size * local_opt_problem.transform_index()[i];
        // double *point = points + point_block_size * local_opt_problem.point_index()[i];

        problem.AddResidualBlock(cost_function_fo_1, loss_function, estimations_D_k_2to_k_1_);
        problem.AddResidualBlock(cost_function_fo_2, loss_function, estimations_D_k_1to_k_);
    }

    // Add the Motion Smooth Residual Blocks to Problem
    // ceres::CostFunction *cost_function_ms;
    // cost_function_ms = MotionSmoothError::Create(observations_T_k_2to_k_1[0], observations_T_k_1to_k[0]);
    // ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);
    // problem.AddResidualBlock(cost_function_ms, loss_function, estimations_D_k_2to_k_1_, estimations_D_k_1to_k_);

    // show some information here ...
    std::cout << "local optimization problem file loaded..." << std::endl;
    std::cout << "local optimization problem have " << local_opt_problem.num_transform() << " transforms and "
              << local_opt_problem.num_landmark() << " landmarks. " << std::endl;
    std::cout << "Forming " << local_opt_problem.num_observations() << " observations. " << std::endl;
    std::cout << "Solving ceres local optimization ... " << std::endl;

    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;

    ceres::Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << "\n";
}