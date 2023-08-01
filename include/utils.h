// #pragma once
#include <iostream>
#include <string.h>
#include <vector>




void readDataFromFile(const std::string& filename, std::vector<std::vector<double>>& data);

class LocalOptimizationProblem {
public:
    /// load bal data from text file
    explicit LocalOptimizationProblem(const std::string &filename_p ,const std::string &filename_t, const std::string &filename_d);

    ~LocalOptimizationProblem() {
        // delete[] point_index_;
        // delete[] transform_index_;

        delete[] observations_p_k_2_;
        delete[] observations_p_k_1_;
        delete[] observations_p_k_;
        delete[] observations_T_k_2to_k_1_;
        delete[] observations_T_k_1to_k_;
        delete[] estimations_D_k_2to_k_1_;
        delete[] estimations_D_k_1to_k_;

    }

    int transform_block_size() const { return 16; }

    int point_block_size() const { return 2; }

    int num_transform() const { return 2; }

    int num_landmark() const { return landmark_size_; }

    int num_observations() const { return 3 * landmark_size_; }


    // const int *point_index() const { return point_index_; }

    // const int *transform_index() const { return transform_index_; }


    const double *observations_p_k_2() const { return observations_p_k_2_;}

    const double *observations_p_k_1() const { return observations_p_k_1_; }

    const double *observations_p_k() const { return observations_p_k_; }

    const double *observations_T_k_2to_k_1() const { return observations_T_k_2to_k_1_; }

    const double *observations_T_k_1to_k() const { return observations_T_k_1to_k_; }


    // const double *parameters() const { return parameters_; }

    double *estimations_D_k_2to_k_1() const { return estimations_D_k_2to_k_1_; }

    double *estimations_D_k_1to_k() const { return estimations_D_k_1to_k_; }

    // const double *camera_for_observation(int i) const {
    //     return transforms() + transform_index_[i] * transform_block_size();
    // }

    // const double *point_for_observation(int i) const {
    //     return points() + point_index_[i] * point_block_size();
    // }

private:
    // void CameraToAngelAxisAndCenter(const double *camera,
    //                                 double *angle_axis,
    //                                 double *center) const;

    // void AngleAxisAndCenterToCamera(const double *angle_axis,
    //                                 const double *center,
    //                                 double *camera) const;

    int landmark_size_;
    // int num_points_;
    int num_observations_;
    int num_transform_;
    int num_parameters_;
    bool use_quaternions_;

    // int *point_index_;      // 每个observation对应的point index
    // int *transform_index_;     // 每个observation对应的camera index

    double *observations_p_k_2_;
    double *observations_p_k_1_;
    double *observations_p_k_;
    double *observations_T_k_2to_k_1_;
    double *observations_T_k_1to_k_;
    double *estimations_D_k_2to_k_1_;
    double *estimations_D_k_1to_k_;



};
