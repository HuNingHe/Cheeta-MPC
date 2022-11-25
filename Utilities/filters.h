#ifndef FILTERS_
#define FILTERS_

#include <cmath>
#include <cstring>
#include <type_traits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

template<typename T>
class filter {
public:
    filter(void) {}

    virtual ~filter(void) {}

    virtual void input(T input_value) = 0;

    virtual T output(void) = 0;

    virtual void clear(void) = 0;
};

template<typename T>
class moving_average_filter : public filter<T> {
private:
    T *buffer_;
    int num_data_;
    int idx_;
    T sum_;

public:
    explicit moving_average_filter(int num_data) : num_data_(num_data), idx_(0.0), sum_(0.0) {
        static_assert(std::is_floating_point<T>::value, "must use floating point value when using filter");
        buffer_ = new T[num_data_];
        memset((void *) buffer_, 0.0, sizeof(T) * num_data_);
    }

    ~moving_average_filter() override {
        delete[] buffer_;
    }

    /*! Update the moving window sum using Neumaier's algorithm.
     * For more details please refer to: https://en.wikipedia.org/wiki/Kahan_summation_algorithm#Further_enhancements
     * @value: The new value to be added to the window.
     */
    void neumaier_sum(T value) {
        double correction_ = 0;
        T new_sum = sum_ + value;
        // If sum_ is bigger, low-order digits of value are lost.
        if (fabs(sum_) >= fabs(value)) {
            correction_ += (sum_ - new_sum) + value;
        } else {
            // low-order digits of sum are lost
            correction_ += (value - new_sum) + sum_;
        }
        sum_ = new_sum + correction_;
    }

    // this part is amazing
    virtual void input(T input_value) {
        neumaier_sum(-buffer_[idx_]);
        neumaier_sum(input_value);

        buffer_[idx_] = input_value;
        ++idx_;
        idx_ %= num_data_;
    }

    virtual T output(void) {
        return sum_ / num_data_;
    }

    virtual void clear(void) {
        sum_ = 0.0;
        memset((void *) buffer_, 0.0, sizeof(T) * num_data_);
    }
};

template
class moving_average_filter<double>;

template
class moving_average_filter<float>;
/*============================================================================*/

/*!
 * First Order IIR Filter
 * @details : https://zhuanlan.zhihu.com/p/51097798
 * @tparam T : type of the data to be filtered
 * @tparam T2 : floating point type for the cutoff/sample frequencies, gain
 */
template<typename T>
class FirstOrderIIRFilter : public filter<T> {
private:
    T _state;
    T _state_pre = 0;
    T _input_pre = 0;
    T _alpha;

public:
    /*!
     * Create a new first order filter
     * @param cutoffFrequency : cutoff frequency of filter
     * @param sampleFrequency : sample frequency (rate at which update is called)
     * @param initialialValue : initial value stored in the filter
     */
    FirstOrderIIRFilter(T cutoffFrequency, T sampleFrequency, T &initialValue) {
        _alpha = tan(M_PI * cutoffFrequency / sampleFrequency);
        _state_pre = initialValue;
    }

    /*!
     * Create a new first order filter
     * @param alpha : filter parameter
     * @param initialValue : initial value
     */
    FirstOrderIIRFilter(T alpha, T &initialValue) : _state_pre(initialValue), _alpha(alpha) {}

    virtual void input(T input_value) {
        _state = _alpha / (_alpha + 1) * input_value + _input_pre / (_alpha + 1) + (_alpha - 1) / (1 + _alpha) * _state_pre;
        _state_pre = _state;
        _input_pre = input_value;
    }

    virtual T output(void) {
        return _state;
    }

    /*!
     * Reset the filter to zero.
     */
    virtual void clear(void) {
        _state = 0.0;
        _alpha = 0.0;
    }
};

template
class FirstOrderIIRFilter<double>;

template
class FirstOrderIIRFilter<float>;
/*============================================================================*/

/*!
 * First Order Filter
 * @tparam T : type of the data to be filtered
 * @tparam T2 : floating point type for the cutoff/sample frequencies, gain
 */
template<typename T, typename T2>
class FirstOrderLowPassFilter : public filter<T> {
private:
    T _state;
    T2 _alpha;

public:
    /*!
     * Create a new first order filter
     * @param cutoffFrequency : cutoff frequency of filter
     * @param sampleFrequency : sample frequency (rate at which update is called)
     * @param initialialValue : initial value stored in the filter
     */
    FirstOrderLowPassFilter(T2 cutoffFrequency, T2 sampleFrequency, T &initialValue) {
        _alpha = cutoffFrequency / (cutoffFrequency + sampleFrequency);
        _state = initialValue;
    }

    /*!
     * Create a new first order filter
     * @param alpha : filter parameter
     * @param initialValue : initial value
     */
    FirstOrderLowPassFilter(T2 alpha, T &initialValue) : _state(initialValue), _alpha(alpha) {}

    virtual void input(T input_value) {
        _state = _alpha * input_value + (T2(1) - _alpha) * _state;
    }

    virtual T output(void) {
        return _state;
    }

    /*!
     * Reset the filter to zero.
     */
    virtual void clear(void) {
        _state = 0.0;
        _alpha = 0.0;
    }
};

template
class FirstOrderLowPassFilter<double, double>;

template
class FirstOrderLowPassFilter<float, double>;

#endif