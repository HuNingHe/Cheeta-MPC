//
// Created by HuNing-He on 22-6-12.
//
#include "save_data_to_file.h"

void print_vec3_to_file(const Vec3<double> &data, std::string file_name) {
    std::fstream f;
    f.open(file_name, std::ios::out | std::ios::app);
    if (f.fail()) {
        std::cerr << "create file error!" << std::endl;
    }
    f << data[0] << "\t" << data[1] << "\t" << data[2] << "\n";
    f.close();
}

void print_vec_to_file(const DVec<double> &data, std::string file_name) {
    std::fstream f;
    f.open(file_name, std::ios::out | std::ios::app);
    if (f.fail()) {
        std::cerr << "create file error!" << std::endl;
    }
    for (int i = 0; i < data.rows(); ++i) {
        f << data[i] << "\t";
    }
    f << "\n";
    f.close();
}