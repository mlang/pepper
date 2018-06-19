#ifndef DSP_H_DEFINED
#define DSP_H_DEFINED

#include <boost/math/constants/constants.hpp>
#include <boost/units/quantity.hpp>
#include <boost/units/systems/si/frequency.hpp>

// https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
template<typename T>
class ExponentiallyWeightedMovingAverage {
  T Average;
  T const Weight, OneMinusWeight;

public:
  explicit ExponentiallyWeightedMovingAverage(T Weight, T Initial = T(0))
  : Average(Initial), Weight(Weight), OneMinusWeight(T(1) - Weight) {}

  ExponentiallyWeightedMovingAverage &operator()(T Value) {
    Average = Value * Weight + Average * OneMinusWeight;

    return *this;
  }

  operator T() const noexcept { return Average; }
};

template<typename T> using EWMA = ExponentiallyWeightedMovingAverage<T>;

template<typename T>
class EdgeDetect {
  T previousDifference = T(0);
  T threshold;
  EWMA<T> fastAverage, slowAverage;
public:
  EdgeDetect(T threshold = 0.2)
  : threshold(threshold)
  , fastAverage(0.25), slowAverage(0.0625)
  {}
  bool operator()(T value) {
    auto const difference = fastAverage(value) - slowAverage(value);
    auto const rising = previousDifference < threshold && difference > threshold;
    previousDifference = difference;
    return rising;
  }
};

template<typename T>
using frequency = boost::units::quantity<boost::units::si::frequency, T>;

using boost::units::si::hertz;

frequency<float> operator"" _Hz(unsigned long long v) {
  return float(v) * hertz;
}

frequency<float> operator"" _Hz(long double v) {
  return float(v) * hertz;
}

class Bandpass {};
constexpr Bandpass bandpass{};
class Highpass {};
constexpr Highpass highpass{};
class Highshelf {};
constexpr Highshelf highshelf{};
class Lowpass {};
constexpr Lowpass lowpass{};
class Lowshelf {};
constexpr Lowshelf lowshelf{};

template<typename T> class Biquad {
  static_assert(std::is_floating_point<T>::value,
                "T must be a floating point type");

  T b0, b1, b2,
        a1, a2,
        s1, s2;

public:
  Biquad() : Biquad(1, 0, 0, 0, 0) {}

  Biquad(T b0, T b1, T b2, T a1, T a2)
  : b0{b0}, b1{b1}, b2{b2}, a1{a1}, a2{a2}, s1{0}, s2{0} {}

  Biquad(Bandpass, frequency<T> sampleRate, frequency<T> cutoff, T q)
  : s1{0}, s2{0} {
    T const k = std::tan(boost::math::constants::pi<T>() * (cutoff / sampleRate));
    T const k2 = k * k;
    T const norm = T(1) / (T(1) + k / q + k2);
    b0 = k / q * norm;
    b1 = T(0);
    b2 = -b0;
    a1 = T(2) * (k2 - T(1)) * norm;
    a2 = (T(1) - k / q + k2) * norm;
  }

  Biquad(Highpass, frequency<T> sampleRate, frequency<T> cutoff, T q)
  : s1{0}, s2{0} {
    T const k = std::tan(boost::math::constants::pi<T>() * (cutoff / sampleRate));
    T const k2 = k * k;
    T const norm = T(1) / (T(1) + k / q + k2);
    b0 = T(1) * norm;
    b1 = T(-2) * b0;
    b2 = b0;
    a1 = T(2) * (k2 - T(1)) * norm;
    a2 = (T(1) - k / q + k2) * norm;
  }

  Biquad(Highshelf, frequency<T> sampleRate, frequency<T> cutoff, T gain)
  : s1{0}, s2{0} {
    T const k = std::tan(boost::math::constants::pi<T>() * (cutoff / sampleRate));
    T const k2 = k * k;
    T const v = std::exp(std::fabs(gain) *
                         (T(1) / T(20)) *
                         boost::math::constants::ln_ten<T>());
    if (gain >= 0) {
      T const norm = T(1) / (T(1) + boost::math::constants::root_two<T>() * k + k2);
      b0 = (v + std::sqrt(T(2) * v) * k + k2) * norm;
      b1 = T(2) * (k2 - v) * norm;
      b2 = (v - std::sqrt(T(2) * v) * k + k2) * norm;
      a1 = T(2) * (k2 - T(1)) * norm;
      a2 = (T(1) - boost::math::constants::root_two<T>() * k + k2) * norm;
    } else {
      T const norm = T(1) / (v + std::sqrt(T(2) * v) * k + k2);
      b0 = (1 + boost::math::constants::root_two<T>() * k + k2) * norm;
      b1 = T(2) * (k2 - T(1)) * norm;
      b2 = (T(1) - boost::math::constants::root_two<T>() * k + k2) * norm;
      a1 = T(2) * (k2 - v) * norm;
      a2 = (v - std::sqrt(T(2) * v) * k + k2) * norm;
    }
  }

  Biquad(Lowpass, frequency<T> sampleRate, frequency<T> cutoff, T q)
  : s1{0}, s2{0} {
    T const k = std::tan(boost::math::constants::pi<T>() * (cutoff / sampleRate));
    T const k2 = k * k;
    T const norm = T(1) / (T(1) + k / q + k2);
    b0 = k2 * norm;
    b1 = T(2) * b0;
    b2 = b0;
    a1 = T(2) * (k2 - T(1)) * norm;
    a2 = (T(1) - k / q + k2) * norm;
  }

  Biquad(Lowshelf, frequency<T> sampleRate, frequency<T> cutoff, T gain)
  : s1{0}, s2{0} {
    T const k = std::tan(boost::math::constants::pi<T>() * (cutoff / sampleRate));
    T const k2 = k * k;
    T const v = std::exp(std::fabs(gain) *
                         (T(1) / T(20)) *
                         boost::math::constants::ln_ten<T>());
    if (gain >= 0) {
      T const norm = T(1) / (T(1) + boost::math::constants::root_two<T>() * k + k2);
      b0 = (T(1) + std::sqrt(T(2) * v) * k + v * k2) * norm;
      b1 = T(2) * (v * k2 - T(1)) * norm;
      b2 = (T(1) - std::sqrt(T(2) * v) * k + v * k2) * norm;
      a1 = T(2) * (k2 - T(1)) * norm;
      a2 = (T(1) - boost::math::constants::root_two<T>() * k + k2) * norm;
    } else {
      T const norm = T(1) / (T(1) + std::sqrt(T(2) * v) * k + v * k2);
      b0 = (1 + boost::math::constants::root_two<T>() * k + k2) * norm;
      b1 = T(2) * (k2 - T(1)) * norm;
      b2 = (T(1) - boost::math::constants::root_two<T>() * k + k2) * norm;
      a1 = T(2) * (v * k2 - T(1)) * norm;
      a2 = (T(1) - std::sqrt(T(2) * v) * k + v * k2) * norm;
    }
  }

  T operator()(T in) {
    T out = std::fma(b0, in, s1);
    s1 = s2 + b1 * in - a1 * out;
    s2 = b2 * in - a2 * out;

    return out;
  }
};

#endif // DSP_H_DEFINED