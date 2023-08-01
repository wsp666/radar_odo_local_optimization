#include "ceres/ceres.h"


class FeatureObservationError {
public:
    FeatureObservationError(double observation_x_1, double observation_y_1, double observation_x_2, double observation_y_2) 
    : observed_x_1(observation_x_1),observed_y_1(observation_y_1),observed_x_2(observation_x_2),observed_y_2(observation_y_2) {}
                                                                           
    template<typename T>
    bool operator()(const T *const transfrom,T *residuals) const {
        T predictions[2];


        Eigen::Matrix<T,4,4> mat_transfrom_;
        Eigen::Map<Eigen::Matrix<T,4,4>>(mat_transfrom_.data(), 4, 4) = \
        Eigen::Map<const Eigen::Matrix<T, 4, 4, Eigen::RowMajor>>(transfrom);

        Eigen::Matrix<T,4,1> mat_landmark_coord;
        mat_landmark_coord << T(observed_x_1), T(observed_y_1), T(0.0), T(1.0);

        Eigen::Matrix<T,4,1> mat_prediction_coord = mat_transfrom_ * mat_landmark_coord;

        predictions[0] = mat_prediction_coord[0]/mat_landmark_coord[3];
        predictions[1] = mat_prediction_coord[1]/mat_landmark_coord[3];
        

        residuals[0] = predictions[0] - T(observed_x_2);
        residuals[1] = predictions[1] - T(observed_y_2);

        return true;
    }

    // template<typename T>
    // static inline bool OdometryTransformLandmark(const T *transfrom, const T *landmark_coord, T *predictions) {
    //     Eigen::Matrix<T,4,4> mat_transfrom_;
    //     Eigen::Map<Eigen::Matrix<T,4,4>>(mat_transfrom_.data(), 4, 4) = \
    //     Eigen::Map<const Eigen::Matrix<T, 4, 4, Eigen::RowMajor>>(transfrom);

    //     Eigen::Matrix<T,4,1> mat_landmark_coord;
    //     mat_landmark_coord << landmark_coord[0], landmark_coord[1], 0, 1;

    //     Eigen::Matrix<T,4,1> mat_prediction_coord = mat_transfrom_ * mat_landmark_coord;

    //     predictions[0] = mat_prediction_coord[0]/mat_landmark_coord[3];
    //     predictions[1] = mat_prediction_coord[1]/mat_landmark_coord[3];

    //     return true;
    // }

    static ceres::CostFunction *Create(const double observed_x_1, const double observed_y_1, 
                                        const double observed_x_2, const double observed_y_2) {
        return (new ceres::AutoDiffCostFunction<FeatureObservationError, 2, 16>(
            new FeatureObservationError(observed_x_1, observed_y_1, observed_x_2, observed_y_2)));
    }

private:
    double observed_x_1;
    double observed_y_1;
    double observed_x_2;
    double observed_y_2;
};


// class MotionSmoothError {
// public:
//     MotionSmoothError(double observation_x, double observation_y) : observed_x(observation_x),
//                                                                            observed_y(observation_y) {}

//     template<typename T>
//     bool operator()(const T *const camera,
//                     const T *const point,
//                     T *residuals) const {
//         // camera[0,1,2] are the angle-axis rotation
//         T predictions[2];
//         CamProjectionWithDistortion(camera, point, predictions);
//         residuals[0] = predictions[0] - T(observed_x);
//         residuals[1] = predictions[1] - T(observed_y);

//         return true;
//     }

//     static ceres::CostFunction *Create(const double observed_x, const double observed_y) {
//         return (new ceres::AutoDiffCostFunction<MotionSmoothError, 6, 9, 3>(
//             new MotionSmoothError(observed_x, observed_y)));
//     }

// private:
//     double observed_x;
//     double observed_y;
// };
