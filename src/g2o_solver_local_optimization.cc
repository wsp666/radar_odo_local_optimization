#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <array>

#include <eigen3/Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>

#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/robust_kernel_impl.h>

#include <sophus/se3.hpp>

using namespace std;
using namespace Eigen;
using Sophus::SE3d;
using Sophus::SO3d;

typedef Matrix<double, 6, 6> Matrix6d;

// 给定误差求J_R^{-1}的近似
Matrix6d JRInv(const SE3d &e)
{
    Matrix6d J;
    J.block(0, 0, 3, 3) = SO3d::hat(e.so3().log());
    J.block(0, 3, 3, 3) = SO3d::hat(e.translation());
    J.block(3, 0, 3, 3) = Matrix3d::Zero(3, 3);
    J.block(3, 3, 3, 3) = SO3d::hat(e.so3().log());
    // J = J * 0.5 + Matrix6d::Identity();
    J = Matrix6d::Identity(); // try Identity if you want
    return J;
}

// 李代数顶点
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 8, 4> Martrix8_4;

// 自身位姿顶点
class VertexSE3LieAlgebraPose : public g2o::BaseVertex<6, SE3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual bool read(istream &is) override { return true; }

    virtual bool write(ostream &os) const override { return true; }

    virtual void setToOriginImpl() override
    {
        _estimate = SE3d();
    }

    // 左乘更新
    virtual void oplusImpl(const double *update) override
    {
        Vector6d upd;
        upd << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = SE3d::exp(upd) * _estimate;
    }
};

class EdgePose2Landmark : public g2o::BaseUnaryEdge<4, Eigen::Vector4d, VertexSE3LieAlgebraPose>
{
    virtual bool read(istream &in) { return true; }

    virtual bool write(ostream &out) const { return true; }

    //残差维度2维？4维？ TODO
    virtual void computeError() override
    {
        SE3d v1 = (static_cast<VertexSE3LieAlgebraPose *>(_vertices[0]))->estimate();
        Eigen::Vector4d p_k_1, p_k;
        p_k_1 << _measurement[0], _measurement[1], double(0.0), double(1.0);
        p_k << _measurement[2], _measurement[3], double(0.0), double(1.0);
        _error = v1 * p_k_1 - p_k;
    }

    // use numeric derivatives
};

class EdgePose2Motion : public g2o::BaseBinaryEdge<6, Eigen::Matrix<double, 8, 4>,
                                                   VertexSE3LieAlgebraPose, VertexSE3LieAlgebraPose>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual bool read(istream &is) override
    {
        return true;
        ;
    }

    virtual bool write(ostream &os) const override
    {
        return true;
        ;
    }

    virtual void computeError() override
    {
        // D_k_2_to_k_1
        SE3d v1 = (static_cast<VertexSE3LieAlgebraPose *>(_vertices[0]))->estimate();
        // D_k_1_to_k
        SE3d v2 = (static_cast<VertexSE3LieAlgebraPose *>(_vertices[1]))->estimate();

        Vector6d T_k_2_to_k_1, T_k_1_to_k;
        T_k_2_to_k_1 << _measurement.block<4, 4>(0, 0);
        T_k_1_to_k << _measurement.block<4, 4>(4, 0);

        Eigen::Matrix3d T_k_2_to_k_1_R = T_k_2_to_k_1.block<3, 3>(0, 0);
        Eigen::Vector3d T_k_2_to_k_1_t = T_k_2_to_k_1.block<3, 1>(0, 3);
        Eigen::Quaterniond rotation_T_k_2_to_k_1(T_k_2_to_k_1_R);
        rotation_T_k_2_to_k_1.norm();
        Sophus::SE3d SE3_T_k_2_to_k_1(rotation_T_k_2_to_k_1, T_k_2_to_k_1_t);

        Eigen::Matrix3d T_k_1_to_k_R = T_k_1_to_k.block<3, 3>(0, 0);
        Eigen::Vector3d T_k_1_to_k_t = T_k_1_to_k.block<3, 1>(0, 3);
        Eigen::Quaterniond rotation_T_k_1_to_k(T_k_1_to_k_R);
        rotation_T_k_1_to_k.norm();
        Sophus::SE3d SE3_T_k_1_to_k(rotation_T_k_1_to_k, T_k_1_to_k_t);

        std::cout << "SE3_T_k_2_to_k_1:\n"
                  << SE3_T_k_2_to_k_1.matrix() << std::endl;
        std::cout << "SE3_T_k_1_to_k:\n"
                  << SE3_T_k_1_to_k.matrix() << std::endl;
        std::cout << "v1:\n"
                  << v1.matrix() << std::endl;
        std::cout << "v2:\n"
                  << v2.matrix() << std::endl;

        _error = (v2 * SE3_T_k_2_to_k_1 * v1.inverse() * SE3_T_k_1_to_k.inverse()).log();
    }

    // 雅可比计算；省略不写则g2o将自动求导
    // virtual void linearizeOplus() override {
    //     SE3d v1 = (static_cast<VertexSE3LieAlgebraPose *> (_vertices[0]))->estimate();
    //     SE3d v2 = (static_cast<VertexSE3LieAlgebraPose *> (_vertices[1]))->estimate();
    //     Matrix6d J = JRInv(SE3d::exp(_error));
    //     // 尝试把J近mat_arr_D_d_k_1tok_似为I？
    //     _jacobianOplusXi = -J * v2.inverse().Adj();
    //     _jacobianOplusXj = J * v2.inverse().Adj();
    // }
};

void readData(const std::string &filename_p, const std::string &filename_T, const std::string &filename_D,
              std::vector<std::vector<double>> &p_data, std::vector<std::vector<double>> &T_data,
              std::vector<std::vector<double>> &D_data, int &landmark_size);

void readDataFromFile(const std::string &filename, std::vector<std::vector<double>> &data);

void copyToEigenMatrix(const std::vector<std::vector<double>> &data, Eigen::Matrix<double, 8, 4> &matrix);

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        std::cout << "usage: ceres_slover_local_optimization sim_data_p.txt sim_data_T.txt sim_data_D.txt" << std::endl;
        return 1;
    }

    std::vector<std::vector<double>> p_data, T_data, D_data;
    int landmark_size;
    readData(argv[1], argv[2], argv[3], p_data, T_data, D_data, landmark_size);

    Martrix8_4 mat_T;
    copyToEigenMatrix(T_data, mat_T);
    // std::cout << "\n mat_T:\n" << mat_T << std::endl;

    std::vector<std::vector<double>> est_D_d_k_2tok_1, est_D_d_k_1tok;
    Eigen::Matrix4d D_k_2to_k_1, D_k_1to_k;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            D_k_2to_k_1(i, j) = D_data[i][j];
            D_k_1to_k(i, j) = D_data[i + 4][j];
        }
    }
    Eigen::Matrix3d D_k_2to_k_1_R = D_k_2to_k_1.block<3, 3>(0, 0);
    Eigen::Vector3d D_k_2to_k_1_t = D_k_2to_k_1.block<3, 1>(0, 3);
    Eigen::Matrix3d D_k_1to_k_R = D_k_1to_k.block<3, 3>(0, 0);
    Eigen::Vector3d D_k_1to_k_t = D_k_1to_k.block<3, 1>(0, 3);
    // std::cout << "D_k_2to_k_1:\n" << D_k_2to_k_1 << std::endl << "D_k_1to_k:\n" << D_k_1to_k << std::endl;
    // std::cout << "D_k_2to_k_1_R:\n" << D_k_2to_k_1_R <<std::endl;
    // std::cout << "D_k_2to_k_1_t:\n" << D_k_2to_k_1_t <<std::endl;
    // std::cout << "D_k_1to_k_R:\n" << D_k_1to_k_R <<std::endl;
    // std::cout << "D_k_1to_k_t:\n" << D_k_1to_k_t <<std::endl;

    Eigen::Quaterniond rotation_D_k_2tok_1(D_k_2to_k_1_R);
    rotation_D_k_2tok_1.norm();
    Sophus::SE3d SE3_D_k_2tok_1(rotation_D_k_2tok_1, D_k_2to_k_1_t);

    Eigen::Quaterniond rotation_D_k_1tok(D_k_1to_k_R);
    rotation_D_k_1tok.norm();
    Sophus::SE3d SE3_D_k_1tok(rotation_D_k_1tok, D_k_1to_k_t);

    std::vector<Eigen::Vector4d> p_data_k_2_k_1, p_data_k_1_k;
    for (const auto &row : p_data)
    {
        Eigen::Vector4d tmp_vector_1, tmp_vector_2;
        tmp_vector_1 << row[0], row[1], row[2], row[3];
        tmp_vector_2 << row[2], row[3], row[4], row[5];
        p_data_k_2_k_1.push_back(tmp_vector_1);
        p_data_k_1_k.push_back(tmp_vector_2);
    }
    // std::cout << "p_data_k_2_k_1:" << std::endl;
    // for (std::vector<Eigen::Vector4d>::iterator it = p_data_k_2_k_1.begin(); it != p_data_k_2_k_1.end(); ++it) {
    //     Eigen::Vector4d vector = *it;
    //     for (int i = 0; i < 4; ++i) {
    //         std::cout << vector(i) << " ";
    //     }
    //     std::cout << std::endl;
    // }
    //  std::cout << "p_data_k_1_k:" << smat_Ttd::endl;
    // for (std::vector<Eigen::Vector4d>::iterator it = p_data_k_1_k.begin(); it != p_data_k_1_k.end(); ++it) {
    //     Eigen::Vector4d vector = *it;
    //     for (int i = 0; i < 4; ++i) {
    //         std::cout << vector(i) << " ";
    //     }
    //     std::cout << std::endl;
    // }

    // 设定g2o
    //整个优化问题的残差维度2维？4维？还是李代数的6维？将残差维度设置为动态情况下（第二维设置为-1）疑似无效 TODO
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, -1>> BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
    // auto solver = new g2o::OptimizationAlgorithmDogleg(
    //     std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
    // auto solver = new g2o::OptimizationAlgorithmGaussNewton(
    //     std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer; // 图模型
    optimizer.setAlgorithm(solver); // 设置求解器

    optimizer.setVerbose(true); // 打开调试输出

    int vertexCnt = 0, edgeCnt = 0; // 顶点和边的数量

    // add Vertex
    VertexSE3LieAlgebraPose *v1 = new VertexSE3LieAlgebraPose();
    v1->setId(vertexCnt++);
    v1->setEstimate(SE3_D_k_2tok_1);
    optimizer.addVertex(v1);
    VertexSE3LieAlgebraPose *v2 = new VertexSE3LieAlgebraPose();
    v2->setId(vertexCnt++);
    v2->setEstimate(SE3_D_k_1tok);
    optimizer.addVertex(v2);

    // add Edge
    for (std::vector<Eigen::Vector4d>::iterator it = p_data_k_2_k_1.begin(); it != p_data_k_2_k_1.end(); ++it)
    {
        EdgePose2Landmark *e = new EdgePose2Landmark();
        e->setId(edgeCnt++);
        // e->setVertex(0, optimizer.vertices()[1]);
        e->setVertex(0, v1);
        // std::cout << "*it:" << *it << std::endl<< std::endl<< std::endl;
        e->setMeasurement(*it);
        e->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(e);
    }
    for (std::vector<Eigen::Vector4d>::iterator it = p_data_k_1_k.begin(); it != p_data_k_1_k.end(); ++it)
    {
        EdgePose2Landmark *e = new EdgePose2Landmark();
        e->setId(edgeCnt++);
        // e->setVertex(0, optimizer.vertices()[1]);
        // std::cout << "*it:" << *it << std::endl;
        e->setVertex(0, v2);
        e->setMeasurement(*it);
        e->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(e);
    }
    // EdgePose2Motion *e = new EdgePose2Motion();
    // e -> setId( edgeCnt++ );
    // e->setVertex(0, optimizer.vertices()[0]);
    // e->setVertex(0, optimizer.vertices()[1]);
    // e->setMeasurement(mat_T);
    // optimizer.addEdge(e);
    // std::cout << "Add the dynamic motion constrian!" << std::endl;

    cout << "Add total " << vertexCnt << " vertices, " << edgeCnt << " edges to the graph" << endl;
    cout << "optimizing ..." << endl;
    optimizer.initializeOptimization();
    optimizer.optimize(100);

    SE3d v1_estimate = v1->estimate();
    cout << "SE3_D_k_2tok_1 estimated model:\n " << v1_estimate.matrix() << endl;
    SE3d v2_estimate = v2->estimate();
    cout << "SE3_D_k_1tok estimated model:\n " << v2_estimate.matrix() << endl;
    return 0;
}

void readData(const std::string &filename_p, const std::string &filename_T, const std::string &filename_D,
              std::vector<std::vector<double>> &p_data, std::vector<std::vector<double>> &T_data,
              std::vector<std::vector<double>> &D_data, int &landmark_size)
{
    readDataFromFile(filename_p, p_data);
    std::cout << "Load landmark coords success! " << std::endl;
    readDataFromFile(filename_T, T_data);
    std::cout << "Load motion transformation success! " << std::endl;
    readDataFromFile(filename_D, D_data);
    std::cout << "Load odometry transformation success! " << std::endl;

    // Print the p_data size
    landmark_size = p_data.size();
    std::cout << "landmark size: " << landmark_size << std::endl;
}

void readDataFromFile(const std::string &filename, std::vector<std::vector<double>> &data)
{
    std::ifstream file(filename);
    if (!file)
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::vector<double> row;
        std::stringstream ss(line);
        double value;
        while (ss >> value)
        {
            row.push_back(value);
        }
        data.push_back(row);
    }

    file.close();
}

void copyToEigenMatrix(const std::vector<std::vector<double>> &data, Eigen::Matrix<double, 8, 4> &matrix)
{
    // 检查数据尺寸是否匹配
    if (data.size() != 8 || data[0].size() != 4)
    {
        std::cout << "数据尺寸不匹配！" << std::endl;
        return;
    }

    for (int i = 0; i < 8; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            matrix(i, j) = data[i][j];
        }
    }
}