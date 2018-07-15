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

template<typename T> struct interpolate {
  using signature = T(T, size_t, size_t, T);
  static T none(T p0, size_t n, size_t d, T p1) {
    return n < d? p0: p1;
  }
  static T linear(T p0, size_t n, size_t d, T p1) {
    auto const mu = double(n) / d;
    return (1.0 - mu) * p0 + mu * p1;
  }
  static T cosine(T p0, size_t n, size_t d, T p1) {
    auto const mu = (1.0 - std::cos((double(n) / d) * boost::math::constants::pi<double>())) / 2.0;
    return (1.0 - mu) * p0 + mu * p1;
  }
  static T exponential(T p0, size_t n, size_t d, T p1) {
    return p0 * std::pow(double(p1) / p0, double(n) / d);
  }
  static T logarithmic(T p0, size_t n, size_t d, T p1) {
    auto mu = double(n) / d;
    if (p0 < p1) mu = 1.0 - mu;
    mu = std::pow(std::pow(2, -10), mu);
    return (1.0 - mu) * std::min(p0, p1) + mu * std::max(p0, p1);
  }
  static T inverted_parabola(T p0, size_t n, size_t d, T p1) {
    auto const mu = 1.0 - (double(n) / d);
    auto const mu2 = mu * mu;
    return mu2 * p0 + (1.0 - mu2) * p1;
  }
};

template<typename T>
class Interpolator {
  T p0;
  size_t written = 0;
  struct point {
    size_t frames;
    typename interpolate<T>::signature *interp;
    T level;
  };
  boost::circular_buffer<point> points;

public:
  Interpolator(size_t capacity, T zero = T(0))
  : p0(zero)
  , points(capacity)
  {}

  bool add_point(size_t frames, T level,
		 typename interpolate<T>::signature *interpolation = interpolate<T>::linear) {
    if (!points.full()) {
      points.push_back(point{frames, interpolation, level});
      return true;
    }

    return false;
  }

  void reset_to(size_t frames, T level) {
    while (!points.empty() && !points.front().frames) {
      p0 = points.front().level;
      points.pop_front();
    }
    if (points.empty()) {
      add_point(frames, level);
    } else {
      auto &p1 = points.front();
      p0 = p1.interp(p0, written, p1.frames, p1.level);
      written = 0;
      p1.frames = frames;
      p1.level = level;
      points.erase(std::next(points.begin()), points.end());
    }
  }
  bool empty() const { return points.empty(); }

  size_t points_available() const { return points.capacity() - points.size(); }

  T last_point() const { return points.empty()? p0: points.back().level; }

  size_t pending_frames() const {
    return std::accumulate(points.begin(), points.end(), 0,
      [](size_t frames, point const &p) { return frames + p.frames; }
    ) - written;
  }

  T operator()() {
    while (!points.empty()) {
      auto const &p1 = points.front();

      if (p1.frames) {
        auto value = p1.interp(p0, written++, p1.frames, p1.level); // [p0, p1)
        if (written == p1.frames) {
          p0 = p1.level;
          points.pop_front();
          written = 0;
        }
        return value;
      }

      p0 = p1.level;
      points.pop_front();
    }

    return p0;
  }
};

#endif // DSP_H_DEFINED
