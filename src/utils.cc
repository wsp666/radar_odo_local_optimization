#include <cstdio>
#include <fstream>
#include <iostream>
#include <vector>
// #include <Eigen/Core>
// #include <Eigen/Dense>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include "utils.h"

template<typename T>
void FscanfOrDie(FILE *fptr, const char *format, T *value) {
    int num_scanned = fscanf(fptr, format, value);
    if (num_scanned != 1)
        std::cerr << "Invalid UW data file. ";
}

void readDataFromFile(const std::string& filename, std::vector<std::vector<double>>& data)
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


LocalOptimizationProblem::LocalOptimizationProblem(const std::string &filename_p, 
                                                    const std::string &filename_T,
                                                    const std::string &filename_D) 
{
    //load data from txt files
    // std::string filename_p = "../data/sim_data_p.txt";
    std::vector<std::vector<double>> p_data;

    // std::string filename_T = "../data/sim_data_T.txt";
    std::vector<std::vector<double>> T_data;
    
    // std::string filename_D = "../data/sim_data_D.txt";
    std::vector<std::vector<double>> D_data;

    readDataFromFile(filename_p, p_data);
    std::cout << "Load landmark coords success! " << std::endl;
    readDataFromFile(filename_T, T_data);
    std::cout << "Load motion transformation success! " << std::endl;
    readDataFromFile(filename_D, D_data);
    std::cout << "Load odometry transformation success! " << std::endl;


    // Print the p_data size
    landmark_size_ = p_data.size();
    std::cout << "landmark size: " << landmark_size_ << std::endl;

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

    //print loaded data
    {
    // // ***********print the p-data**********************
    
    // // for (const auto& row : p_data)
    // // {
    // //     for (const auto& value : row)
    // //     {
    // //         std::cout << value << "\t";
    // //     }
    // //     std::cout << std::endl;
    // // }
    // std::cout << std::endl << "landmark observation coordinates at time (k-2)" << std::endl;
    // for(const auto& row : p_data_k_2)
    // {
    //     std::cout << row[0] << "\t" << row[1] << std::endl; 
    // }
    // std::cout << std::endl << "landmark observation coordinates at time (k-1)" << std::endl;
    // for(const auto& row : p_data_k_1)
    // {
    //     std::cout << row[0] << "\t" << row[1] << std::endl; 
    // }
    // std::cout << std::endl << "landmark observation coordinates at time (k)" << std::endl;
    // for(const auto& row : p_data_k)
    // {
    //     std::cout << row[0] << "\t" << row[1] << std::endl; 
    // }

    // // ***********print the T-data**********************
    // std::cout << std::endl << "Dynamic object motions at time (k-2) to (k-1)" << std::endl;
    // for (const auto& row : obs_T_d_k_2tok_1) {
    //     for (const auto& value : row) {
    //         std::cout << value << " ";
    //     }
    //     std::cout << std::endl;
    // }
    // std::cout << std::endl << "Dynamic object motions at time (k-1) to (k-2)" << std::endl;
    // for (const auto& row : obs_T_d_k_1tok) {
    //     for (const auto& value : row) {
    //         std::cout << value << " ";
    //     }
    //     std::cout << std::endl;
    // }
    // // ***********print the D-data**********************
    // std::cout << std::endl << "Odometry estimation at time (k-2) to (k-1)" << std::endl;
    // for (const auto& row : est_D_d_k_2tok_1) {
    //     for (const auto& value : row) {
    //         std::cout << value << " ";
    //     }
    //     std::cout << std::endl;
    // }
    // std::cout << std::endl << "Odometry estimation at time (k-1) to (k)" << std::endl;
    // for (const auto& row : est_D_d_k_1tok) {
    //     for (const auto& value : row) {
    //         std::cout << value << " ";
    //     }
    //     std::cout << std::endl;
    // }
    }

    //store the data in array
    {
    std::cout << "p_data_k_2.size() * 2:   " << p_data_k_2.size() * 2 << std::endl; 
    std::cout << "est_D_d_k_2tok_1.size() * 4:   " << est_D_d_k_2tok_1.size() * 4 << std::endl; 


    double* observations_p_k_2_ = new double[p_data_k_2.size() * 2];
    double* observations_p_k_1_ = new double[p_data_k_1.size() * 2];
    double* observations_p_k_ = new double[p_data_k.size() * 2];
    double* observations_T_k_2to_k_1_ = new double[obs_T_d_k_2tok_1.size() * 4];
    double* observations_T_k_1to_k_ = new double[obs_T_d_k_1tok.size() * 4];
    double* estimations_D_k_2to_k_1_ = new double[est_D_d_k_2tok_1.size() * 4];
    double* estimations_D_k_1to_k_ = new double[est_D_d_k_2tok_1.size() * 4];


    int index = 0;
    for (const auto& element : p_data_k_2) {
        observations_p_k_2_[index++] = element[0];
        observations_p_k_2_[index++] = element[1];
    }

    index = 0;
    for (const auto& element : p_data_k_1) {
        observations_p_k_1_[index++] = element[0];
        observations_p_k_1_[index++] = element[1];
    }

    index = 0;
    for (const auto& element : p_data_k) {
        observations_p_k_[index++] = element[0];
        observations_p_k_[index++] = element[1];
    }

    index = 0;
    for (const auto& row : obs_T_d_k_2tok_1) {
        for (const auto& element : row) {
            observations_T_k_2to_k_1_[index++] = element;
        }
    }

    index = 0;
    for (const auto& row : obs_T_d_k_1tok) {
        for (const auto& element : row) {
            observations_T_k_1to_k_[index++] = element;
        }
    }
    index = 0;
    for (const auto& row : est_D_d_k_2tok_1) {
    for (const auto& element : row) {
        estimations_D_k_2to_k_1_[index++] = element;
    }
    }

    index = 0;
    for (const auto& row : est_D_d_k_1tok) {
    for (const auto& element : row) {
        estimations_D_k_1to_k_[index++] = element;
    }
    }
    // for (int i = 0; i<16; i++){
    //     std::cout << estimations_D_k_2to_k_1_[i] << std::endl;
    //     // std::cout << "Here!" << std::endl;
    // }
    }

}

