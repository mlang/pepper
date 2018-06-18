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

template<typename T> class Biquad {
  T b0, b1, b2,
        a1, a2,
        s1, s2;
public:
  constexpr Biquad() : b0{1}, b1{0}, b2{0}, a1{0}, a2{0}, s1{0}, s2{0} {}
  constexpr Biquad(T b0, T b1, T b2, T a1, T a2)
  : b0{b0}, b1{b1}, b2{b2}, a1{a1}, a2{a2}, s1{0}, s2{0} {}

  using frequency = boost::units::quantity<boost::units::si::frequency, T>;
  
  static Biquad bandpass(frequency sampleRate, frequency cutoff, T q) {
    auto const k = std::tan(boost::math::constants::pi<T>()
			    * (cutoff / sampleRate));
    auto const k2 = k * k;
    auto const norm = T(1) / (T(1) + k / q + k2);
    auto const b0 = k / q * norm;
    return {
      b0, T(0), -b0,
      T(2) * (k2 - T(1)) * norm, (T(1) - k / q + k2) * norm
    };
  }

  static constexpr Biquad highpass(frequency sampleRate, frequency cutoff, T q) {
    auto const k = std::tan(boost::math::constants::pi<T>()
			    * (cutoff / sampleRate));
    auto const k2 = k * k;
    auto const norm = T(1) / (T(1) + k / q + k2);
    auto const b0 = T(1) * norm;
    return {
      b0, T(-2) * b0, b0,
      T(2) * (k2 - T(1)) * norm, (T(1) - k / q + k2) * norm
    };
  }

  static Biquad highshelf(frequency sampleRate, frequency cutoff, T gain) {
    auto const k = std::tan(boost::math::constants::pi<T>()
			    * (cutoff / sampleRate));
    auto const k2 = k * k;
    auto const v = std::exp(std::fabs(gain) *
			    (T(1) / T(20)) *
			    boost::math::constants::ln_ten<T>());
    if (gain >= 0) {
      auto const norm = T(1) /
	(T(1) + boost::math::constants::root_two<T>() * k + k2);
      auto const b0 = (v + std::sqrt(T(2) * v) * k + k2) * norm;
      auto const b1 = T(2) * (k2 - v) * norm;
      auto const b2 = (v - std::sqrt(T(2) * v) * k + k2) * norm;
      auto const a1 = T(2) * (k2 - T(1)) * norm;
      auto const a2 = (T(1) - boost::math::constants::root_two<T>() * k + k2) *
	norm;
      return { b0, b1, b2, a1, a2 };
    } else {
      auto const norm = T(1) / (v + std::sqrt(T(2) * v) * k + k2);
      auto const b0 = (1 + boost::math::constants::root_two<T>() * k + k2) *
	norm;
      auto const b1 = T(2) * (k2 - T(1)) * norm;
      auto const b2 = (T(1) - boost::math::constants::root_two<T>() * k + k2) *
	norm;
      auto const a1 = T(2) * (k2 - v) * norm;
      auto const a2 = (v - std::sqrt(T(2) * v) * k + k2) * norm;
      return { b0, b1, b2, a1, a2 };
    }
  }

  static Biquad lowpass(frequency sampleRate, frequency cutoff, T q) {
    auto const k = std::tan(boost::math::constants::pi<T>()
			    * (cutoff / sampleRate));
    auto const k2 = k * k;
    auto const norm = T(1) / (T(1) + k / q + k2);
    auto const b0 = k2 * norm;
    return {
      b0, T(2) * b0, b0,
      T(2) * (k2 - T(1)) * norm, (T(1) - k / q + k2) * norm
    };
  }

  static Biquad lowshelf(frequency sampleRate, frequency cutoff, T gain) {
    auto const k = std::tan(boost::math::constants::pi<T>()
			    * (cutoff / sampleRate));
    auto const k2 = k * k;
    auto const v = std::exp(std::fabs(gain) *
			    (T(1) / T(20)) *
			    boost::math::constants::ln_ten<T>());
    if (gain >= 0) {
      auto const norm = T(1) /
	(T(1) + boost::math::constants::root_two<T>() * k + k2);
      auto const b0 = (T(1) + std::sqrt(T(2) * v) * k + v * k2) * norm;
      auto const b1 = T(2) * (v * k2 - T(1)) * norm;
      auto const b2 = (T(1) - std::sqrt(T(2) * v) * k + v * k2) * norm;
      auto const a1 = T(2) * (k2 - T(1)) * norm;
      auto const a2 = (T(1) - boost::math::constants::root_two<T>() * k + k2) *
	norm;
      return { b0, b1, b2, a1, a2 };
    } else {
      auto const norm = T(1) / (T(1) + std::sqrt(T(2) * v) * k + v * k2);
      auto const b0 = (1 + boost::math::constants::root_two<T>() * k + k2) *
	norm;
      auto const b1 = T(2) * (k2 - T(1)) * norm;
      auto const b2 = (T(1) - boost::math::constants::root_two<T>() * k + k2) *
	norm;
      auto const a1 = T(2) * (v * k2 - T(1)) * norm;
      auto const a2 = (T(1) - std::sqrt(T(2) * v) * k + v * k2) * norm;
      return { b0, b1, b2, a1, a2 };
    }
  }

  T operator()(T in) {
    auto const out = b0 * in + s1;
    s1 = s2 + b1 * in - a1 * out;
    s2 = b2 * in - a2 * out;

    return out;
  }
};

#endif // DSP_H_DEFINED
