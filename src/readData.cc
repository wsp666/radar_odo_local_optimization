#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <array>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <sophus/se3.hpp>

void readDataFromFile(const std::string& filename, std::vector<std::vector<double>>& data)
{
    std::ifstream file(filename);
    if (!file)
    {
        std::cout << "Failed to open file: " << filename << std::endl;
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

int main()
{
    //load data from txt files
    std::string filename_p = "../data/sim_data_p.txt";
    std::vector<std::vector<double>> p_data;

    std::string filename_T = "../data/sim_data_T.txt";
    std::vector<std::vector<double>> T_data;

    std::string filename_D = "../data/sim_data_D.txt";
    std::vector<std::vector<double>> D_data;

    readDataFromFile(filename_p, p_data);
    std::cout << "Load landmark coords success! " << std::endl;
    readDataFromFile(filename_T, T_data);
    std::cout << "Load motion transformation success! " << std::endl;
    readDataFromFile(filename_D, D_data);
    std::cout << "Load odometry transformation success! " << std::endl;

    // Print the p_data size
    int landmark_size = p_data.size();
    std::cout << "landmark size: " << landmark_size << std::endl;

    std::vector<std::array<double, 2>> p_data_k_2, p_data_k_1, p_data_k;
    std::vector<std::vector<double>> obs_T_d_k_2tok_1, obs_T_d_k_1tok;
    std::vector<std::vector<double>> est_D_d_k_2tok_1, est_D_d_k_1tok;

    //Generate landmark observation coordinates at time (k-2),(k-1),(k)
    for (const auto& row : p_data)
    {
        std::array<double,2> rowData_k_2 = {row[0], row[1]};
        std::array<double,2> rowData_k_1 = {row[2], row[3]};
        std::array<double,2> rowData_k = {row[4], row[5]};

        p_data_k_2.push_back(rowData_k_2);
        p_data_k_1.push_back(rowData_k_1);
        p_data_k.push_back(rowData_k);
    }

    //Generate Dynamic object motions at time (k-2) to (k-1),(k-1) to (k)
    for (int i = 0; i < 4; ++i) {
        obs_T_d_k_2tok_1.push_back(T_data[i]);
    }
    for (int i = 4; i < 8; ++i) {
        obs_T_d_k_1tok.push_back(T_data[i]);
    }

    //Generate Odometry estimation at time (k-2) to (k-1),(k-1) to (k)
    for (int i = 0; i < 4; ++i) {
        est_D_d_k_2tok_1.push_back(D_data[i]);
    }
    for (int i = 4; i < 8; ++i) {
        est_D_d_k_1tok.push_back(D_data[i]);
    }
    {
    // ***********print the p-data**********************
    
    // for (const auto& row : p_data)
    // {
    //     for (const auto& value : row)
    //     {
    //         std::cout << value << "\t";
    //     }
    //     std::cout << std::endl;
    // }
    std::cout << std::endl << "landmark observation coordinates at time (k-2)" << std::endl;
    for(const auto& row : p_data_k_2)
    {
        std::cout << row[0] << "\t" << row[1] << std::endl; 
    }
    std::cout << std::endl << "landmark observation coordinates at time (k-1)" << std::endl;
    for(const auto& row : p_data_k_1)
    {
        std::cout << row[0] << "\t" << row[1] << std::endl; 
    }
    std::cout << std::endl << "landmark observation coordinates at time (k)" << std::endl;
    for(const auto& row : p_data_k)
    {
        std::cout << row[0] << "\t" << row[1] << std::endl; 
    }

    // ***********print the T-data**********************
    std::cout << std::endl << "Dynamic object motions at time (k-2) to (k-1)" << std::endl;
    for (const auto& row : obs_T_d_k_2tok_1) {
        for (const auto& value : row) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl << "Dynamic object motions at time (k-1) to (k-2)" << std::endl;
    for (const auto& row : obs_T_d_k_1tok) {
        for (const auto& value : row) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }

    // ***********print the D-data**********************
    std::cout << std::endl << "Dynamic object motions at time (k-2) to (k-1)" << std::endl;
    for (const auto& row : est_D_d_k_2tok_1) {
        for (const auto& value : row) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl << "Dynamic object motions at time (k-1) to (k-2)" << std::endl;
    for (const auto& row : est_D_d_k_1tok) {
        for (const auto& value : row) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }
    }
    // 创建7个动态分配的double类型数组
    double* arr_p_k_2_ = new double[p_data_k_2.size() * 2];
    double* arr_p_k_1_ = new double[p_data_k_1.size() * 2];
    double* arr_p_k_ = new double[p_data_k.size() * 2];
    double* arr_T_d_k_2tok_1_ = new double[obs_T_d_k_2tok_1.size() * 16];
    double* arr_T_d_k_1tok_ = new double[obs_T_d_k_1tok.size() * 16];
    double* arr_D_d_k_2tok_1_ = new double[est_D_d_k_2tok_1.size() * 16];
    double* arr_D_d_k_1tok_ = new double[est_D_d_k_1tok.size() * 16];


    int index = 0;
    for (const auto& element : p_data_k_2) {
    arr_p_k_2_[index++] = element[0];
    arr_p_k_2_[index++] = element[1];
    }

    index = 0;
    for (const auto& element : p_data_k_1) {
    arr_p_k_1_[index++] = element[0];
    arr_p_k_1_[index++] = element[1];
    }

    index = 0;
    for (const auto& element : p_data_k) {
    arr_p_k_[index++] = element[0];
    arr_p_k_[index++] = element[1];
    }

    index = 0;
    for (const auto& row : obs_T_d_k_2tok_1) {
    for (const auto& element : row) {
        arr_T_d_k_2tok_1_[index++] = element;
    }
    }

    index = 0;
    for (const auto& row : obs_T_d_k_1tok) {
    for (const auto& element : row) {
        arr_T_d_k_1tok_[index++] = element;
    }
    }

    index = 0;
    for (const auto& row : est_D_d_k_2tok_1) {
    for (const auto& element : row) {
        arr_D_d_k_2tok_1_[index++] = element;
    }
    }

    index = 0;
    for (const auto& row : est_D_d_k_1tok) {
    for (const auto& element : row) {
        arr_D_d_k_1tok_[index++] = element;
    }
    }

    Eigen::Matrix4d mat_arr_D_d_k_1tok_;
    Eigen::Map<Eigen::Matrix4d>(mat_arr_D_d_k_1tok_.data(), 4, 4) = \
    Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(arr_D_d_k_1tok_);
    std::cout << "mat_arr_D_d_k_1tok_:\n" << mat_arr_D_d_k_1tok_ <<std::endl;

    Eigen::Matrix3d mat_arr_D_d_k_1tok_R = mat_arr_D_d_k_1tok_.block<3, 3>(0, 0);
    // std::cout << "mat_arr_D_d_k_1tok_R:\n" << mat_arr_D_d_k_1tok_R <<std::endl;

    Eigen::Vector3d mat_arr_D_d_k_1tok_t = mat_arr_D_d_k_1tok_.block<3, 1>(0, 3);
    // std::cout << "mat_arr_D_d_k_1tok_t:\n" << mat_arr_D_d_k_1tok_t <<std::endl;

    Eigen::Quaterniond rotation_D_d_k_1tok(mat_arr_D_d_k_1tok_R);
    rotation_D_d_k_1tok.norm();
    Sophus::SE3d SE3_D_d_k_1tok(rotation_D_d_k_1tok,mat_arr_D_d_k_1tok_t);

    // 输出李群SE3 & 李代数se3
    std::cout << "SE3: \n" << SE3_D_d_k_1tok.matrix() << std::endl;
    std::cout << "se3: \n" << SE3_D_d_k_1tok.log() << std::endl;

    // Eigen::Vector3d vector_se3;
    // vector_se3 << SE3_D_d_k_1tok.log();
    // vector_se3 = (Sophus::SE3d)vector_se3;
    // Sophus::SE3d newSE3 = Sophus::SE3d::hat(vector_se3)
    // std::cout << "se3: \n" << vector_se3  << std::endl;

    // 在不再需要这些数组时释放内存
    delete[] arr_p_k_2_;
    delete[] arr_p_k_1_;
    delete[] arr_p_k_;
    delete[] arr_T_d_k_2tok_1_;
    delete[] arr_T_d_k_1tok_;
    delete[] arr_D_d_k_2tok_1_;
    delete[] arr_D_d_k_1tok_;

    return 0;
}
