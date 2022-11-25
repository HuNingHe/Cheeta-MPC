/*!
 * @file: LinearAlgebra.h
 * @authors: HuNing-He
 * @date: 2022-11-21
 * @reference: 现代机器人学-机构、规划与控制 By Kevin
 * @copyright (c) 2022 HuNing-He. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */
#pragma once

#include <cmath>
#include <type_traits>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>
#include "MathTypes.h"

/*!
 * Compute the pseudo inverse of a matrix
 * @param matrix : input matrix
 * @param invMatrix : output matrix
 * @param sigmaThreshold : threshold for singular values being zero
 */
template<typename T>
void pseudoInverse(DMat<T> const &matrix, DMat<T> &invMatrix, double sigmaThreshold = 1e-4) {
    if ((1 == matrix.rows()) && (1 == matrix.cols())) {
        invMatrix.resize(1, 1);
        if (matrix.coeff(0, 0) > sigmaThreshold) {
            invMatrix.coeffRef(0, 0) = 1.0 / matrix.coeff(0, 0);
        } else {
            invMatrix.coeffRef(0, 0) = 0.0;
        }
        return;
    }
    Eigen::JacobiSVD<DMat<T>> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);

    int const nrows(svd.singularValues().rows());
    DMat<T> invS;
    invS = DMat<T>::Zero(nrows, nrows);
    for (int ii(0); ii < nrows; ++ii) {
        if (svd.singularValues().coeff(ii) > sigmaThreshold) {
            invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
        }
    }
    invMatrix = svd.matrixV() * invS * svd.matrixU().transpose();
}
