#ifndef SALT_H_DEFINED
#define SALT_H_DEFINED
#include "units.h"
#include <Bela.h>

namespace Salt {

using namespace units::literals;

constexpr auto analogPeakToPeak = 10_V;

constexpr int nPins = 4;

constexpr int trigOutPins[nPins] = { 0, 5, 12, 13 };
constexpr int trigInPins[nPins] = { 15, 14, 1, 3 };
constexpr int sw1Pin = 6;
constexpr int ledPins[nPins] = { 2, 4, 8, 9 };
constexpr int pwmPin = 7;
constexpr int gNumButtons = nPins;

constexpr int buttonPins[gNumButtons] = {
  sw1Pin, trigInPins[1], trigInPins[2], trigInPins[3]
};

bool setup(BelaContext *bela) {
  if ((bela->flags & BELA_FLAG_INTERLEAVED) == BELA_FLAG_INTERLEAVED) {
    std::cerr << "This project only works in non-interleaved mode."
              << std::endl;
    return false;
  }
  pinMode(bela, 0, pwmPin, OUTPUT);
  for(auto pin: trigOutPins) {
    pinMode(bela, 0, pin, OUTPUT);
  }
  for(auto pin: trigInPins) {
    pinMode(bela, 0, pin, INPUT);
  }
  for(auto pin: ledPins) {
    pinMode(bela, 0, pin, INPUT);
  }

  return true;
}

inline float const *audioIn(BelaContext *bela, unsigned int channel) {
  return &bela->audioIn[channel * bela->audioFrames];
}

template<unsigned int channel> float *audioOut(BelaContext *bela) {
  return &bela->audioOut[channel * bela->audioFrames];
}

inline float *audioOut(BelaContext *bela, unsigned int channel) {
  return &bela->audioOut[channel * bela->audioFrames];
}

template<unsigned int channel> float const *analogIn(BelaContext *bela) {
  return &bela->analogIn[channel * bela->analogFrames];
}

inline float const *analogIn(BelaContext *bela, unsigned int channel) {
  return &bela->analogIn[channel * bela->analogFrames];
}

template<unsigned int channel> float *analogOut(BelaContext *bela) {
  return &bela->analogOut[channel * bela->analogFrames];
}

inline float *analogOut(BelaContext *bela, unsigned int channel) {
  return &bela->analogOut[channel * bela->analogFrames];
}

template<class T, class Compare>
constexpr T const &clamp(T const &v, T const &lo, T const &hi, Compare comp) {
  return comp(v, lo)? lo: comp(hi, v)? hi: v;
}

template<typename T>
constexpr T const &clamp(T const &v, T const &lo, T const &hi) {
  return clamp(v, lo, hi, std::less<T>());
}

inline units::frequency::hertz_t to_frequency(
  float level,
  units::frequency::hertz_t base = 16.3516_Hz,
  units::voltage::volt_t octave = 1_V
) {
  return base * std::pow(2.0, level / (octave / analogPeakToPeak));
}

inline float to_analog(
  units::frequency::hertz_t const &freq,
  units::frequency::hertz_t base = 16.3516_Hz,
  units::voltage::volt_t octave = 1_V
) {
  return clamp(
    units::unit_cast<float>(
      units::math::log2(freq / base) / (analogPeakToPeak / octave) + 0.5
    ), 0.5f, 1.0f
  );
}

inline constexpr float to_analog(units::voltage::volt_t const &volt) {
  return clamp(units::unit_cast<float>(volt / analogPeakToPeak + 0.5), 0.0f, 1.0f);
}

} // namespace Salt

#endif // SALT_H_DEFINED
