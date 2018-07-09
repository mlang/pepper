#ifndef DSP_H_DEFINED
#define DSP_H_DEFINED

#include "units.h"
#include <boost/circular_buffer.hpp>
#include <boost/math/constants/constants.hpp>
#include <numeric>

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

  Biquad(Bandpass,
	 units::frequency::hertz_t sampleRate,
	 units::frequency::hertz_t cutoff,
	 T q)
  : s1{0}, s2{0} {
    T const k = std::tan(boost::math::constants::pi<T>() * T(cutoff / sampleRate));
    T const k2 = k * k;
    T const norm = T(1) / (T(1) + k / q + k2);
    b0 = k / q * norm;
    b1 = T(0);
    b2 = -b0;
    a1 = T(2) * (k2 - T(1)) * norm;
    a2 = (T(1) - k / q + k2) * norm;
  }

  Biquad(Highpass,
	 units::frequency::hertz_t sampleRate,
	 units::frequency::hertz_t cutoff, T q)
  : s1{0}, s2{0} {
    T const k = std::tan(boost::math::constants::pi<T>() * T(cutoff / sampleRate));
    T const k2 = k * k;
    T const norm = T(1) / (T(1) + k / q + k2);
    b0 = T(1) * norm;
    b1 = T(-2) * b0;
    b2 = b0;
    a1 = T(2) * (k2 - T(1)) * norm;
    a2 = (T(1) - k / q + k2) * norm;
  }

  Biquad(Highshelf,
	 units::frequency::hertz_t sampleRate,
	 units::frequency::hertz_t cutoff,
         units::dimensionless::dB_t gain)
  : s1{0}, s2{0} {
    T const k = std::tan(boost::math::constants::pi<T>() * T(cutoff / sampleRate));
    T const k2 = k * k;
    T const v = std::exp(units::math::fabs(gain) *
                         (T(1) / T(20)) *
                         boost::math::constants::ln_ten<T>());
    if (gain >= 0.0) {
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

  Biquad(Lowpass,
	 units::frequency::hertz_t sampleRate,
	 units::frequency::hertz_t cutoff, T q)
  : s1{0}, s2{0} {
    T const k = std::tan(boost::math::constants::pi<T>() * T(cutoff / sampleRate));
    T const k2 = k * k;
    T const norm = T(1) / (T(1) + k / q + k2);
    b0 = k2 * norm;
    b1 = T(2) * b0;
    b2 = b0;
    a1 = T(2) * (k2 - T(1)) * norm;
    a2 = (T(1) - k / q + k2) * norm;
  }

  Biquad(Lowshelf,
	 units::frequency::hertz_t sampleRate,
	 units::frequency::hertz_t cutoff,
         units::dimensionless::dB_t gain)
  : s1{0}, s2{0} {
    T const k = std::tan(boost::math::constants::pi<T>() * T(cutoff / sampleRate));
    T const k2 = k * k;
    T const v = std::exp(units::math::fabs(gain) *
                         (T(1) / T(20)) *
                         boost::math::constants::ln_ten<T>());
    if (gain >= 0.0) {
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

namespace interpolate {
  double none(double) { return 0; }
  double linear(double value) { return value; }
  double cosine(double value) {
    return (1.0 - std::cos(value * boost::math::constants::pi<double>())) / 2.0;
  }
  double logarithmic(double v) { return std::pow(0.1, (1.0 - v) * 5); }
  double inverted_parabola(double v) { return 1.0 - (1.0 - v) * (1.0 - v); }
}

template<typename T>
class Interpolator {
  T p0;
  size_t written = 0;
  struct point {
    size_t frames;
    double (*interp)(double);
    T level;
  };
  boost::circular_buffer<point> points;

public:
  Interpolator(size_t capacity, T zero = T(0))
  : p0(zero)
  , points(capacity)
  {}

  bool add_point(size_t frames, T level,
		 double (*interpolation)(double) = interpolate::linear) {
    if (!points.full()) {
      points.push_back(point{frames, interpolation, level});
      return true;
    }

    return false;
  }
  bool empty() const { return points.empty(); }
  size_t points_available() const { return points.capacity() - points.size(); }
  T last_point() const { return points.empty()? p0: points.back().level; }
  size_t pending_frames() const {
    return std::accumulate(points.begin(), points.end(), 0,
      [](size_t frames, point const &p) { return frames + p.frames; }
    ) - written;
  }
  template<typename OutputIterator>
  OutputIterator generate_n(OutputIterator frame, size_t size) {
    while (size && !points.empty()) {
      auto const &p1 = points.front();
      auto const n = std::min(size, p1.frames - written);
      frame = std::generate_n(frame, n,
	[&p1, this]{
	  auto const progress = p1.interp(double(written++) / p1.frames); // [p0, p1)
	  return T((1.0f - progress) * p0) + T(progress * p1.level);
	}
      );
      size -= n;
      if (written == p1.frames) {
        p0 = p1.level;
        points.pop_front();
        written = 0;
      }
    }
    return std::fill_n(frame, size, p0);
  }
  T operator()() {
    if (!points.empty()) {
      auto const &p1 = points.front();
      auto const progress = p1.interp(double(written++) / p1.frames); // [p0, p1)
      auto const value = T((1.0f - progress) * p0) + T(progress * p1.level);
      if (written == p1.frames) {
        p0 = p1.level;
        points.pop_front();
        written = 0;
      }
      return value;
    }
    return p0;
  }
};

#endif // DSP_H_DEFINED
