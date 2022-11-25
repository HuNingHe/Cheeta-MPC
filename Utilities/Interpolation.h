/*! @file Interpolation.h
 *  @brief Utility functions to interpolate between two values
 *  @details: https://zhuanlan.zhihu.com/p/72595018 and https://www.guyuehome.com/25388
 */

#ifndef PROJECT_INTERPOLATION_H
#define PROJECT_INTERPOLATION_H

#include <cassert>
#include <type_traits>
#include "cmath"
#include "MathTypes.h"

namespace Interpolate {
    /*!
     * Cubic bezier interpolation between y0 and yf.  x is between 0 and 1
     */
    template<typename y_t, typename x_t>
    y_t cubicBezier(y_t y0, y_t yf, x_t x) {
        static_assert(std::is_floating_point<x_t>::value, "must use floating point value");
        assert(x >= x_t(0) && x <= x_t(1));
        y_t yDiff = yf - y0;
        x_t bezier = x * x * x + x_t(3) * (x * x * (x_t(1) - x));
        return y0 + bezier * yDiff;
    }

    /*!
     * Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and 1
     */
    template<typename y_t, typename x_t>
    y_t cubicBezierFirstDerivative(y_t y0, y_t yf, x_t x) {
        static_assert(std::is_floating_point<x_t>::value, "must use floating point value");
        assert(x >= x_t(0) && x <= x_t(1));
        y_t yDiff = yf - y0;
        x_t bezier = x_t(6) * x * (x_t(1) - x);
        return bezier * yDiff;
    }

    /*!
     * Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and 1
     */
    template<typename y_t, typename x_t>
    y_t cubicBezierSecondDerivative(y_t y0, y_t yf, x_t x) {
        static_assert(std::is_floating_point<x_t>::value, "must use floating point value");
        assert(x >= x_t(0) && x <= x_t(1));
        y_t yDiff = yf - y0;
        x_t bezier = x_t(6) - x_t(12) * x;
        return bezier * yDiff;
    }

    Vec3<double> cycloid(const Vec3<double>& y0, const Vec3<double>& yf, double x) {
        assert(x >= 0 && x <= 1);
        Vec3<double> yDiff = yf - y0;
        double cycloid = x - sin(2 * M_PI * x) / (2 * M_PI);
        Vec3<double> array = cycloid * yDiff + y0;
        if (x < 0.5) {
            cycloid = 2 * x - sin(4 * M_PI * x) / (2 * M_PI);
        } else {
            cycloid = 2 - 2 * x + sin(4 * M_PI * x) / (2 * M_PI);
        }
        array[2] = cycloid * yDiff[2] + y0[2];
        return array;
    }

    Vec3<double> cycloidFirstDerivative(const Vec3<double>& y0, const Vec3<double>& yf, double x) {
        assert(x >= 0 && x <= 1);
        Vec3<double> yDiff = yf - y0;
        double cycloid = 1 - cos(2 * M_PI * x);
        Vec3<double> array = cycloid * yDiff;
        if (x < 0.5) {
            cycloid = 2 - cos(4 * M_PI * x) * 2;
        } else {
            cycloid = cos(4 * M_PI * x) * 2 - 2;
        }
        array[2] = cycloid * yDiff[2];
        return array;
    }

    Vec3<double> cycloidSecondDerivative(const Vec3<double>& y0, const Vec3<double>& yf, double x) {
        assert(x >= 0 && x <= 1);
        Vec3<double> yDiff = yf - y0;
        double cycloid = 2 * M_PI * sin(2 * M_PI * x);
        Vec3<double> array = cycloid * yDiff;
        if (x < 0.5) {
            cycloid = sin(4 * M_PI * x) * 8 * M_PI;
        } else {
            cycloid = -sin(4 * M_PI * x) * 8 * M_PI;
        }
        array[2] = cycloid * yDiff[2];
        return array;
    }

} // namespace Interpolate

#endif // PROJECT_INTERPOLATION_H
