/*!
 * @file: Orientation.h
 * @authors: HuNing-He
 * @date: 2022-11-21
 * @reference: <<Modern robotics: planning and control>> (chinese version) By Kevin
 * @copyright (c) 2022 HuNing-He. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */
#pragma once
#include <cmath>
#include <type_traits>
#include "MathTypes.h"

namespace Ori {
    template<typename T>
    Mat3<typename T::Scalar> crossMatrix(const Eigen::MatrixBase<T> &v) {
        static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3, "must have 3x1 vector");
        Mat3<typename T::Scalar> m;
        m << 0, -v(2), v(1),
             v(2), 0, -v(0),
             -v(1), v(0), 0;
        return m;
    }

    template <typename T>
    Vec3<typename T::Scalar> skewMatToVec(const Eigen::MatrixBase<T>& m) {
        static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3, "Must have 3x3 matrix");
        return 0.5 * Vec3<typename T::Scalar>(m(2, 1) - m(1, 2), m(0, 2) - m(2, 0), (m(1, 0) - m(0, 1)));
    }

    /*!
     * @param r must be in SO3, otherwise wrong
     * Convert a coordinate transformation matrix to an orientation quaternion.
     * refer to Kevin's book(chinese version) in page 53, <<Modern robotics: planning and control>> (chinese version)
     */
    template<typename T>
    Quat<T> rotMatToQuat(const RotMat<T> &r) {
        static_assert(std::is_floating_point<T>::value, "must use floating point value");
        T tr = r.trace();
        Quat<T> q;
        if (r.isIdentity(1e-4)) {
            q.w() = 1;
            q.x() = 0;
            q.y() = 0;
            q.z() = 0;
        } else if (abs(tr + 1.0) < 1e-4) {
            T S = sqrt(2 * (r(2, 2) + 1));
            q.w() = 0;
            q.x() = r(0, 2) / S;
            q.y() = r(1, 2) / S;
            q.z() = (r(2, 2) + 1) / S;
        } else {
            T theta = acos(0.5 * (tr - 1));
            RotMat<T> w = (r - r.transpose()) / (2 * sin(theta));
            q.w() = cos(theta / 2.0);
            q.x() = w(2, 1) * sin(theta / 2.0);
            q.y() = w(0, 2) * sin(theta / 2.0);
            q.z() = w(1, 0) * sin(theta / 2.0);
        }
        q.normalize();
        return q;
    }

    /*!
     * Convert a quaternion to RPY.  Uses ZYX extrinsic order, returns
     * angles in (roll, pitch, yaw).
     */
    template<typename T>
    Vec3<T> quatToRPY(const Quat<T> &qua) {
        static_assert(std::is_floating_point<T>::value, "must use floating point value");

        Vec3<T> rpy;
        double q[4] = {qua.w(), qua.x(), qua.y(), qua.z()};
        T as = std::min(2.0 * (q[0] * q[2] - q[1] * q[3]), .99999);
        rpy(2) = std::atan2(2 * (q[1] * q[2] + q[0] * q[3]), pow(q[0], 2) + pow(q[1], 2) - pow(q[2], 2) - pow(q[3], 2));
        rpy(1) = std::asin(as);
        rpy(0) = std::atan2(2 * (q[2] * q[3] + q[0] * q[1]), pow(q[0], 2) - pow(q[1], 2) - pow(q[2], 2) + pow(q[3], 2));
        return rpy;
    }

    /*!
     * Uses ZYX extrinsic order
     */
    template<typename T>
    Quat<T> rpyToQuat(const Vec3<T> &rpy) {
        static_assert(std::is_floating_point<T>::value, "must use floating point value");
        Quat<T> q = Eigen::AngleAxis<T>(rpy[2], Vec3<T>::UnitZ()) *
                    Eigen::AngleAxis<T>(rpy[1], Vec3<T>::UnitY()) *
                    Eigen::AngleAxis<T>(rpy[0], Vec3<T>::UnitX());
        return q;
    }

    /*!
     * Get the rotation matrix coincide with euler angle
     */
    template<typename T>
    Mat3<T> rpyToRotMat(const Vec3<T> &rpy) {
        static_assert(std::is_floating_point<T>::value, "must use floating point value");
        Quat<T> q = rpyToQuat(rpy);
        return q.matrix();
    }

    template<typename T>
    Vec3<typename T::Scalar> rotMatToRPY(const Eigen::MatrixBase<T> &R) {
        static_assert(T::ColsAtCompileTime == 3 && T::RowsAtCompileTime == 3, "Must have 3x3 matrix");
        Quat<typename T::Scalar> q = rotMatToQuat(R);
        Vec3<typename T::Scalar> rpy = quatToRPY(q);
        return rpy;
    }

    /*!
     * Convert a quaternion to so3.
     */
    template<typename T>
    Vec3<T> quatToso3(const Quat<T> &q) {
        static_assert(std::is_floating_point<T>::value, "must use floating point value");
        Vec3<T> so3;
        T theta = 2.0 * std::acos(q.w());

        so3[0] = theta * q.x() / std::sin(theta / 2.0);
        so3[1] = theta * q.y() / std::sin(theta / 2.0);
        so3[2] = theta * q.z() / std::sin(theta / 2.0);
        return so3;
    }

    template<typename T>
    Quat<T> so3ToQuat(Vec3<T> &so3) {
        static_assert(std::is_floating_point<T>::value, "must use floating point value");
        Quat<T> quat;
        T theta = sqrt(pow(so3[0], 2) + pow(so3[1], 2) + pow(so3[2], 2));

        if (fabs(theta) < 1e-6) {
            quat.w() = 1.;
            quat.x() = 0;
            quat.y() = 0;
            quat.z() = 0;
            return quat;
        }

        quat.w() = cos(theta / 2.0);
        quat.x() = so3[0] / theta * sin(theta / 2.0);
        quat.y() = so3[1] / theta * sin(theta / 2.0);
        quat.z() = so3[2] / theta * sin(theta / 2.0);
        return quat;
    }
}
