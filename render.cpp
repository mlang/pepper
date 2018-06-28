// Pepper -- Braille display support for the Salt programmable eurorack module
//
// Copyright Mario Lang, 2018
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE or copy at http://boost.org/LICENSE_1_0.txt)
//------------------------------------------------------------------------------//
#include "AuxTask.h"
#include "DSP.h"
#include "RTPipe.h"
#include "RTQueue.h"
#include "mpark_variant.h"
#include "units.h"
#include <Bela.h>
#include <DigitalChannelManager.h>
#include <algorithm>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/hana/functional/overload.hpp>
#include <boost/optional.hpp>
#include <boost/serialization/vector.hpp>
#include <brlapi.h>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <lilv/lilvmm.hpp>
#include <lv2/lv2plug.in/ns/ext/buf-size/buf-size.h>
#include <memory>
#include <poll.h>
#include <sstream>
#include <utility>
#include <vector>
//-*--*---*----*-----*------*-------*--------*-------*------*-----*----*---*--*-//

using boost::archive::text_iarchive;
using boost::archive::text_oarchive;
using boost::hana::overload;
using boost::none;
using boost::optional;
using mpark::variant;                   // for TriviallyCopyable variants
using mpark::visit;
using std::make_unique;
using std::string;
using std::to_string;
using std::unique_ptr;
using std::vector;
using units::frequency::hertz_t;
using units::literals::operator""_ms;
using units::literals::operator""_Hz;
using units::literals::operator""_V;
using units::time::second_t;
using units::unit_cast;
using units::voltage::volt_t;

// No locks are used at all.  Instead, all communication between the RT and
// non-RT thread is done via (typed) messages.  The RT thread uses a pipe to
// wake up the non-RT thread, while the non-RT thread uses a RT Queue to notify
// the RT thread that things need to change.  A (trivially copyable) variant is
// used to handle different types of requests (non-RT -> RT) and
// messages (RT -> non-RT).
//
// The RT thread can be in a number of different modes, each of which has a
// corresponding "tab" in the (non-RT) display thread.  Switching between tabs
// will automatically activate the corresponding mode.

namespace {

constexpr auto analogPeakToPeak = 10_V;

enum class ModeIdentifier {
  AnalogueOscillator, FMOscillator,
  AudioLevelMeter, Sequencer
};

class Song {
  vector<vector<int>> pattern;
  vector<vector<int>> song;
public:
  Song() : pattern {
    {1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0},
    {1, 0, 0, 0, 1, 0, 0, 0}
  }, song {
    {1, 1, 1, 1},
    {0, 0}
  } {}
  unsigned int length() const {
    unsigned int max = 0;
    for (auto const &track: song) {
      unsigned int ticks = 0;
      for (auto index: track) {
        ticks += pattern[index].size();
      }
      if (ticks > max) {
        max = ticks;
      }
    }
    return max;
  }
  template<typename F> void triggersAt(unsigned int position, F f) {
    unsigned int channel = 0;
    for (auto const &track: song) {
      unsigned int pos = 0;
      for (auto index: track) {
        auto const &pat = pattern[index];
        if (position < pos + pat.size()) {
          if (pat[position - pos] == 1) {
            f(channel);
          }
        } else {
          pos += pat.size();
        }
        if (pos > position) {
          break;
        }
      }
      channel += 1;
    }
  }
  vector<vector<int>> &patterns() { return pattern; }
  template<typename Archive>
  void serialize(Archive &archive, unsigned int /*version*/) {
    archive & pattern;
    archive & song;
  }
};

void save(Song const &song, string const &filename) {
  std::ofstream ofs(filename);
  text_oarchive oa(ofs);
  oa << song;
}

void load(Song &song, string const &filename) {
  std::ifstream ifs(filename);
  if (ifs.good()) {
    text_iarchive ia(ifs);
    ia >> song;
  } else {
    std::cout << filename << " does not exist, saving..." << std::endl;
    save(song, filename);
  }
}

enum class Command { PrevPlugin, NextPlugin };

struct UpdateSong {
  Song *pointer;
};

using Request = variant<Command, UpdateSong>;

struct ModeChanged {
  ModeIdentifier mode;
};

struct LevelsChanged {
  float l, r, lp, rp;
  float analog[8];
};

struct TempoChanged {
  float bpm;
};
    
struct SongLoaded {
  Song const *pointer;
};
  
struct PositionChanged {
  unsigned int position;
};

struct SongUpdated {
  Song *pointer;
};

using Message = variant<
  ModeChanged,
  LevelsChanged,
  TempoChanged, SongLoaded, PositionChanged, SongUpdated
>;

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

class Salt {
public:
  explicit Salt(BelaContext *bela) {
    pinMode(bela, 0, pwmPin, OUTPUT);
    for(auto pin : trigOutPins) {
      pinMode(bela, 0, pin, OUTPUT);
    }
    for(auto pin : trigInPins) {
      pinMode(bela, 0, pin, INPUT);
    }
    for(auto pin : ledPins) {
      pinMode(bela, 0, pin, INPUT);
    }
  }
};

class Pepper;

class Display {
  unique_ptr<char[]> brlapiHandle;
  brlapi_handle_t *handle() {
    return static_cast<brlapi_handle_t *>(
      static_cast<void *>(brlapiHandle.get())
    );
  }
  int fd = -1;
  unsigned int width = 0;
  bool connected = false;
  bool connect();
  void writeText(string const &text, int cursor) {
    if (connected) {
      brlapi__writeText(handle(),
                        cursor > static_cast<int>(width)? BRLAPI_CURSOR_OFF: cursor,
                        text.c_str());
    }
  }
  RTPipe updatePipe;
  void doPoll();
  void keyPressed(brlapi_keyCode_t keyCode);
  Pepper &pepper;
  class Tab {
    string name;
  protected:
    unsigned int x = 0, y = 0;
    struct LineInfo {
      string text;
      optional<int> cursor;

      LineInfo() = default;
      explicit LineInfo(
        string text, optional<int> cursor = none
      ) : text(std::move(text)), cursor(cursor) {}
    };
    vector<LineInfo> lines;
  public:
    explicit Tab(string name, size_t lines = 0)
    : name(std::move(name))
    , lines(lines)
    {}
    void draw(Display &display) {
      if (y == 0) {
        display.writeText(name, BRLAPI_CURSOR_OFF);
      } else {
        auto const &line = lines[y - 1];
        auto const text = line.text.substr(
          std::min(static_cast<string::size_type>(x), line.text.length())
        );
        int cursor = BRLAPI_CURSOR_OFF;
        if (line.cursor) {
          int cell = line.cursor.value() - x;
          if (cell >= 0 && cell < static_cast<int>(display.width)) {
            cursor = cell + 1;
          }
        }
        display.writeText(text, cursor);
      }
    }
    unsigned int line() const { return y; }
    void lineUp(Display &display) {
      if (y > 0) {
        y -= 1;
        draw(display);
      }
    }
    void lineDown(Display &display) {
      if (y < lines.size()) {
        y += 1;
        draw(display);
      }
    }
    void windowLeft(Display &display) {
      if (x > 0) {
        x -= std::min(display.width, x);
        draw(display);
      }
    }
    void windowRight(Display &display) {
      x += display.width;
      draw(display);
    }
    virtual void click(unsigned int /*cell*/, Display & /*display*/);
  };
  class AnalogueOscillatorTab : public Tab {
  public:
    AnalogueOscillatorTab() : Tab("AnalogueOsc") {}
    void click(unsigned int /*cell*/, Display & /*display*/) {}
  };
  class FMOscillatorTab : public Tab {
  public:
    FMOscillatorTab() : Tab("FMOsc") {}
    void click(unsigned int /*cell*/, Display & /*display*/) {}
  };
  class LevelMeterTab : public Tab {
  public:
    LevelMeterTab() : Tab("metering...", 12) {}
    void operator()(LevelsChanged const &level) {
      lines[0].text = "L: " + to_string(level.l);
      lines[1].text = "R: " + to_string(level.r);
      lines[2].text = "PeakL: " + to_string(level.lp);
      lines[3].text = "PeakR: " + to_string(level.rp);
      for (unsigned long i = 0; i < 8; i++) {
        lines[i+4].text = "A" + to_string(i) + ": " + to_string(level.analog[i]);
      }
    }
    void click(unsigned int /*cell*/, Display & /*display*/) override {}
  };
  class SequencerTab : public Tab {
    Song song;
  public:
    SequencerTab() : Tab("Sequencer", 2) {
      lines[0].text = "Not running";
    }
    void operator()(TempoChanged const &tempo) {
      if (tempo.bpm == 0.0f) {
        lines[0].text = "Not running";
      } else {
        lines[0].text = "BPM: " +
          to_string(static_cast<int>(std::round(tempo.bpm)));
      }
    }
    void operator()(Song const *song) {
      this->song = *song;
      drawSong();
    }
    void drawSong() {
      lines.resize(2);
      for (auto const &pattern: this->song.patterns()) {
        string rep;
        for (auto v: pattern) {
          char c = ' ';
          if (v == 1) {
            c = '%';
          }
          rep += c;
        }
        lines.emplace_back(rep);
      }
    }
    void operator()(PositionChanged const &changed) {
      lines[1].cursor = changed.position;
    }
    void click(unsigned int cell, Display &display) override;
  };
  vector<unique_ptr<Tab>> tabs;
  ModeIdentifier currentMode = ModeIdentifier::Sequencer;
  Tab &currentTab() { return *tabs[static_cast<int>(currentMode)]; }
  void redraw() {
    currentTab().draw(*this);
  }
  LevelMeterTab &levelMeterTab() {
    return dynamic_cast<LevelMeterTab&>(
      *tabs[static_cast<int>(ModeIdentifier::AudioLevelMeter)]
    );
  }
  SequencerTab &sequencerTab() {
    return dynamic_cast<SequencerTab&>(
      *tabs[static_cast<int>(ModeIdentifier::Sequencer)]
    );
  }
  friend class Tab;
public:
  explicit Display(Pepper &pepper);
  ~Display() {
    if (connected) {
      brlapi__leaveTtyMode(handle());
      brlapi__closeConnection(handle());
    }
  }
  AuxTask<NonRT, decltype(&Display::doPoll)> poll;
  void write(Message const &msg) {
    updatePipe.write(msg);
  }
};

class Mode;

class Pepper : Salt {
  Lilv::World lilv;
  vector<unique_ptr<Mode>> plugins;
  int index = -1;
  Display braille;
  DigitalChannelManager digital;
  static void digitalChanged(bool state, unsigned int /*unused*/, void *data) {
    if (state) {
      static_cast<Pepper *>(data)->nextPlugin();
    }
  }
  void prevPlugin() {
    if (index != 0) {
      index -= 1;
    } else {
      index = plugins.size() - 1;
    }
    modeChanged();
  }
  void nextPlugin() {
    index = (index + 1) % plugins.size();
    modeChanged();
  }
  inline void modeChanged();
  RTQueue commandQueue;
  void requestReceived(Request & /*req*/);
public:
  explicit Pepper(BelaContext * /*bela*/);
  ~Pepper();
  void render(BelaContext * /*bela*/);
  void sendCommand(Command const &cmd) {
    Request req { cmd };
    commandQueue.write(req);
  }
  void sendRequest(Request req) {
    commandQueue.write(req);
  }
  void updateDisplay(Message const &msg) {
    braille.write(msg);
  }
};

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

class Mode {
protected:
  Pepper &pepper;
public:
  explicit Mode(Pepper &pepper) : pepper(pepper) {}
  virtual ~Mode() = default;

  virtual ModeIdentifier mode() const = 0;
  virtual void activate() = 0;
  virtual void run(BelaContext *bela) = 0;
  virtual void deactivate() = 0;
};

LV2_Feature hardRTCapable = { LV2_CORE__hardRTCapable, nullptr };
LV2_Feature fixedBlockSize = { LV2_BUF_SIZE__fixedBlockLength, nullptr };

LV2_Feature *features[3] = {
  &hardRTCapable, &fixedBlockSize, nullptr
};

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
    unit_cast<double>(
      units::math::log2(freq / base) / (analogPeakToPeak / octave) + 0.5
    ), 0.5, 1.0
  );
}

inline constexpr float to_analog(units::voltage::volt_t const &volt) {
  return clamp(unit_cast<double>(volt / analogPeakToPeak + 0.5), 0.0, 1.0);
}

class LV2Plugin : public Mode {
protected:
  Lilv::Instance *instance;
  vector<float> minValue, maxValue, defValue, value;
public:
  LV2Plugin(Pepper &pepper, BelaContext *bela, Lilv::World &lilv, Lilv::Plugin p)
  : Mode(pepper)
  , instance(Lilv::Instance::create(p, bela->audioSampleRate, features)) {
    static LilvNode *lv2_core__sampleRate = lilv.new_uri(LV2_CORE__sampleRate);
    auto const count = p.get_num_ports();
    minValue.resize(count); maxValue.resize(count); defValue.resize(count);
    p.get_port_ranges_float(&minValue.front(), &maxValue.front(),
                            &defValue.front());
    for (unsigned int i = 0; i < count; i++) {
      if (p.get_port_by_index(i).has_property(lv2_core__sampleRate)) {
        minValue[i] *= bela->audioSampleRate;
        maxValue[i] *= bela->audioSampleRate;
      }
    }
    value = defValue;
  }
  void activate() override { instance->activate(); }
  void deactivate() override { instance->deactivate(); }
protected:
  void controlFromAnalog(unsigned p, float v) {
    value[p] = map(v, 0, 1, minValue[p], maxValue[p]);
  }
  void connectAudioIn(BelaContext *bela,
                      unsigned lv2PortIndex, unsigned channel) {
    connectAudio(bela, lv2PortIndex, channel,
                 const_cast<float *>(bela->audioIn));
  }
  void connectAudioOut(BelaContext *bela,
                       unsigned lv2PortIndex, unsigned channel) {
    connectAudio(bela, lv2PortIndex, channel, bela->audioOut);
  }
private:
  void connectAudio(BelaContext *bela,
                    unsigned lv2PortIndex, unsigned channel, float *buffer) {
    instance->connect_port(lv2PortIndex, &buffer[bela->audioFrames * channel]);
  }
};

class AnalogueOscillator final : public LV2Plugin {
  EWMA<float> freqAverage;
public:
  static constexpr const char *uri =
    "http://plugin.org.uk/swh-plugins/analogueOsc";
  ModeIdentifier mode() const override {
    return ModeIdentifier::AnalogueOscillator;
  }
  AnalogueOscillator
  (Pepper &pepper, BelaContext *bela, Lilv::World &lilv, Lilv::Plugin p)
  : LV2Plugin(pepper, bela, lilv, p)
  , freqAverage(0.25) {
    auto const count = p.get_num_ports();
    for (unsigned int i = 0; i < count; i++) {
      switch (i) {
      case 4:
        connectAudioOut(bela, 4, 0);
        break;
      default:
        instance->connect_port(i, &value[i]);
        break;
      }
    }
  }
  void run(BelaContext *bela) override {
    std::for_each(analogIn<0>(bela), analogIn<0>(bela) + bela->analogFrames,
                  freqAverage);
    value[1] = unit_cast<float>(to_frequency(freqAverage));
    controlFromAnalog(0, analogReadNI(bela, 0, 1)); // Waveform
    controlFromAnalog(2, analogReadNI(bela, 0, 2)); // Warmth
    controlFromAnalog(3, analogReadNI(bela, 0, 3)); // Instability

    instance->run(bela->audioFrames);

    // Duplicate output to both channels
    std::copy_n(audioOut<0>(bela), bela->audioFrames, audioOut<1>(bela));
  }
};

class FMOscillator final : public LV2Plugin {
public:
  static constexpr const char *uri = "http://plugin.org.uk/swh-plugins/fmOsc";
  ModeIdentifier mode() const override { return ModeIdentifier::FMOscillator; }
  FMOscillator
  (Pepper &pepper, BelaContext *bela, Lilv::World &lilv, Lilv::Plugin p)
  : LV2Plugin(pepper, bela, lilv, p) {
    auto const count = p.get_num_ports();
    for (unsigned int i = 0; i < count; i++) {
      switch (i) {
      case 2:
        connectAudioOut(bela, i, 0);
        break;
      case 1:
        connectAudioIn(bela, i, 0);
        break;
      default:
        instance->connect_port(i, &value[i]);
        break;
      }
    }
  }
  void run(BelaContext *bela) override {
    controlFromAnalog(0, analogReadNI(bela, 0, 0)); // Waveform

    instance->run(bela->audioFrames);

    // Duplicate output to both channels
    std::copy_n(audioOut<0>(bela), bela->audioFrames, audioOut<1>(bela));
  }
};

class AudioLevelMeter : public Mode {
  struct AudioChannel {
    Biquad<float> dcblock;
    float localLevel = 0, peakLevel = 0;

    explicit AudioChannel(unsigned int sr)
      : dcblock(highpass, hertz_t(sr), 5_Hz, 0.5)
    {}

    void operator()(float sample) {
      float const level = std::fabs(dcblock(sample));
      if (level > localLevel) {
        localLevel = level;
      } else {
        localLevel *= localDecayRate;
      }
      if (level > peakLevel) {
        peakLevel = level;
      } else {
        peakLevel *= peakDecayRate;
      }
    }
  };
  vector<AudioChannel> audio;
  unsigned int blockCount = 0;
  static constexpr float const localDecayRate = 0.99, peakDecayRate = 0.999;
public:
  AudioLevelMeter(Pepper &pepper, BelaContext *bela)
  : Mode(pepper)
  , audio(bela->audioInChannels, AudioChannel(bela->audioSampleRate))
  {}

  ModeIdentifier mode() const override {
    return ModeIdentifier::AudioLevelMeter;
  }

  void activate() override {}
  void deactivate() override {}

  void run(BelaContext *bela) override {
    for (unsigned int channel = 0; channel < bela->audioInChannels; ++channel) {
      auto * const samples = audioIn(bela, channel);
      std::for_each(samples, samples + bela->audioFrames, audio[channel]);
      std::copy_n(samples, bela->audioFrames, audioOut(bela, channel));
    }

    blockCount++;
    if (blockCount % 100 == 0) {
      Message msg {
        LevelsChanged {
          audio[0].localLevel, audio[1].localLevel,
          audio[0].peakLevel, audio[1].peakLevel,
          { analogReadNI(bela, 0, 0)
          , analogReadNI(bela, 0, 1)
          , analogReadNI(bela, 0, 2)
          , analogReadNI(bela, 0, 3)
          , analogReadNI(bela, 0, 4)
          , analogReadNI(bela, 0, 5)
          , analogReadNI(bela, 0, 6)
          , analogReadNI(bela, 0, 7)
          }
        }
      };
      pepper.updateDisplay(msg);
    };
  }
};

class AnalogOut {
  hertz_t sampleRate;
  unsigned int channel;
  float zero;
  float level = zero;
  size_t offset = 0, length = 0;
public:
  AnalogOut(BelaContext *bela, unsigned int channel, volt_t zero = 0.0_V)
  : sampleRate(bela->analogSampleRate)
  , channel(channel)
  , zero(to_analog(zero))
  {}

  void set_for(unsigned int frame, second_t duration, volt_t level = 5.0_V) {
    this->offset = frame;
    this->length = sampleRate * duration;
    this->level = to_analog(level);
  }

  void run(BelaContext *bela) {
    auto * const samples = analogOut(bela, channel);
    auto * const begin = samples + offset;
    auto const size = std::min(bela->analogFrames - offset, length);
    auto * const end = begin + size;

    std::fill(samples, begin, zero);
    std::fill(begin, end, level);
    std::fill(end, samples + bela->analogFrames, zero);

    length -= size;
    offset = 0;
  }
};

class Clock {
  optional<int> offset;
  optional<int> length;
public:
  void tick(unsigned int frame) {
    offset = frame;
  }
  template<typename BPM>
  void update(BelaContext *bela, BPM bpm) {
    if (offset) {
      if (length) {
        length = length.value() + offset.value();
        bpm(60.0f /
            (static_cast<float>(length.value()) /
             static_cast<float>(bela->analogSampleRate)) /
            4.0f);
      }
      length = bela->analogFrames - offset.value();
      offset = none;
    } else {
      if (length) {
        length = length.value() + bela->analogFrames;
        if (length.value() > bela->analogSampleRate) {
          bpm(0.0f);
          length = none;
        }
      }
    }
  }
};

class Sequencer final : public Mode {
  EdgeDetect<float> clockRising, resetRising;
  Clock clock;
  Song song;
  unsigned int position = 0;
  vector<AnalogOut> analogOut;
public:
  Sequencer(Pepper &pepper, BelaContext *bela)
  : Mode(pepper)
  , clockRising(0.2), resetRising(0.2)
  , position(0)
  {
    load(song, "default.pepper");
    pepper.updateDisplay(Message { SongLoaded { &song } });
    for (unsigned int channel = 0; channel < bela->analogOutChannels; ++channel) {
      analogOut.emplace_back(bela, channel);
    }
  }
  ModeIdentifier mode() const override { return ModeIdentifier::Sequencer; }
  void activate() override {}
  void deactivate() override {}
  void setSong(Song *newSong) {
    song = std::move(*newSong);
    position = std::min(position, song.length());
  }
  void run(BelaContext *bela) override {
    for (unsigned int frame = 0; frame < bela->analogFrames; ++frame) {
      if (resetRising(analogIn<1>(bela)[frame])) {
        position = 0;
      }
      if (clockRising(analogIn<0>(bela)[frame])) {
        clock.tick(frame);
        song.triggersAt(position, [this, frame](unsigned int channel) {
            analogOut[channel].set_for(frame, 5_ms, 4.0_V);
        });
        pepper.updateDisplay(Message { PositionChanged { position } });
        position = song.length() > 0? (position + 1) % song.length(): 0;
      }
    }
    for (auto &channel: analogOut) {
      channel.run(bela);
    }
    clock.update(bela, [this](float bpm) {
      pepper.updateDisplay(Message { TempoChanged { bpm } });
    });
  }
};

} // anonymous namespace

// Implementation

Pepper::Pepper(BelaContext *bela)
: Salt(bela)
, braille(*this)
, commandQueue("command-queue", 128)
{
  braille.poll();
  digital.setCallback(digitalChanged);
  digital.setCallbackArgument(buttonPins[3], this);
  digital.manage(buttonPins[3], INPUT, true);

  plugins.push_back(make_unique<Sequencer>(*this, bela));
  plugins.emplace_back(make_unique<AudioLevelMeter>(*this, bela));

  lilv.load_all();
  auto lv2plugins = lilv.get_all_plugins();
  for (auto i = lv2plugins.begin(); !lv2plugins.is_end(i); i = lv2plugins.next(i)) {
    auto lv2plugin = Lilv::Plugin(lv2plugins.get(i));
    auto name = Lilv::Node(lv2plugin.get_name());
    auto uri = Lilv::Node(lv2plugin.get_uri());
    if (uri.is_uri() && strcmp(uri.as_string(), AnalogueOscillator::uri) == 0) {
      plugins.push_back(make_unique<AnalogueOscillator>(*this, bela, lilv, lv2plugin));
      std::cout << "Loaded " << name.as_string() << std::endl;
    } else if (uri.is_uri() && strcmp(uri.as_string(), FMOscillator::uri) == 0) {
      plugins.push_back(make_unique<FMOscillator>(*this, bela, lilv, lv2plugin));
      std::cout << "Loaded " << name.as_string() << std::endl;
    }
  }

  if (!this->plugins.empty()) {
    index = 0;
  }
  for (auto &plugin: plugins) {
    plugin->activate();
  }
  modeChanged();
}

void Pepper::requestReceived(Request &req) {
  visit(
    overload(
      [this](Command cmd) {
        switch (cmd) {
        case Command::PrevPlugin:
          prevPlugin();
          break;
        case Command::NextPlugin:
          nextPlugin();
          break;
        default:
          fprintf(stderr, "Unknown command %d\n", static_cast<int>(cmd));
        }
      },
      [this](UpdateSong const &song) {
        for (auto &plugin: plugins) {
          if (plugin->mode() == ModeIdentifier::Sequencer) {
            dynamic_cast<Sequencer&>(*plugin).setSong(song.pointer);
          }
        }
        updateDisplay(Message { SongUpdated { song.pointer } });
      }),
    req
  );
}

void Pepper::modeChanged() {
  Message msg(ModeChanged { plugins[index]->mode() });
  braille.write(msg);
}

Display::Display(Pepper &pepper)
: brlapiHandle(new char[brlapi_getHandleSize()])
, connected(connect())
, updatePipe("update-display", 0x10000)
, pepper(pepper)
, poll("brlapi-poll", &Display::doPoll, this)
{
  auto const modes = static_cast<int>(ModeIdentifier::Sequencer) + 1;
  //tabs.resize(modes);
  for (int i = 0; i < modes; ++i) {
    switch (ModeIdentifier(i)) {
    case ModeIdentifier::AnalogueOscillator:
      tabs.push_back(make_unique<AnalogueOscillatorTab>());
      break;
    case ModeIdentifier::AudioLevelMeter:
      tabs.push_back(make_unique<LevelMeterTab>());
      break;
    case ModeIdentifier::FMOscillator:
      tabs.push_back(make_unique<FMOscillatorTab>());
      break;
    case ModeIdentifier::Sequencer:
      tabs.push_back(make_unique<SequencerTab>());
      break;
    }
  }
  if (connected) {
    writeText("Welcome!", BRLAPI_CURSOR_OFF);
  }
}

bool Display::connect() {
  brlapi_settings_t settings = BRLAPI_SETTINGS_INITIALIZER;
  fd = brlapi__openConnection(handle(), &settings, &settings);
  if (fd != -1) {
    unsigned int x, y;
    brlapi__getDisplaySize(handle(), &x, &y);
    width = x;
    auto tty = brlapi__enterTtyMode(handle(), 1, "");
    if (tty != -1) {
      return true;
    }

    brlapi__closeConnection(handle());
  }
  return false;
}

void Display::doPoll() {
  pollfd fds[] = {
    { fd, POLLIN, 0 },
    { updatePipe.open(), POLLIN, 0 }
  };

  while (!gShouldStop) {
    int ret = ::poll(fds, std::distance(std::begin(fds), std::end(fds)), 10);
    if (ret < 0) {
      perror("poll");
      break;
    }

    if (ret == 0) {
      continue;
    }

    for (auto const &item: fds) {
      if ((item.revents & POLLIN) == 0) {
        continue;
      }

      if (item.fd == fd) {
        brlapi_keyCode_t keyCode;
        while (brlapi__readKey(handle(), 0, &keyCode) == 1) {
          keyPressed(keyCode);
        }
      } else if (item.fd == updatePipe.fileDescriptor()) {
        Message msg;
        if (read(updatePipe.fileDescriptor(), &msg, sizeof(Message)) ==
            sizeof(Message)) {
          visit(
            overload(
              [this](ModeChanged const &changed) {
                currentMode = changed.mode;
              },
              [this](LevelsChanged const &level) {
                levelMeterTab()(level);
              },
              [this](TempoChanged const &tempo) {
                sequencerTab()(tempo);
              },
              [this](SongLoaded const &song) {
                sequencerTab()(song.pointer);
              },
              [this](PositionChanged const &position) {
                sequencerTab()(position);
              },
              [this](SongUpdated const &song) {
                delete song.pointer;
              }),
            msg);
          redraw();
        }
      }
    }
  }
}

void Display::keyPressed(brlapi_keyCode_t keyCode) {
  brlapi_expandedKeyCode_t key{};
  if (brlapi_expandKeyCode(keyCode, &key) == 0) {
    switch (key.type) {
    case BRLAPI_KEY_TYPE_CMD:
      switch (key.command) {
      case BRLAPI_KEY_CMD_FWINLT:
        if (currentTab().line() == 0) {
          pepper.sendCommand(Command::PrevPlugin);
        } else {
          currentTab().windowLeft(*this);
        }
        return;
      case BRLAPI_KEY_CMD_FWINRT:
        if (currentTab().line() == 0) {
          pepper.sendCommand(Command::NextPlugin);
        } else {
          currentTab().windowRight(*this);
        }
        return;
      case BRLAPI_KEY_CMD_LNUP:
        currentTab().lineUp(*this);
        return;
      case BRLAPI_KEY_CMD_LNDN:
        currentTab().lineDown(*this);
        return;
      case BRLAPI_KEY_CMD_ROUTE:
        currentTab().click(key.argument, *this);
        return;
      default:
        break;
      }
      break;
    default:
      break;
    }
  }
  std::stringstream str;
  str << key.type << " " << key.command << " " << key.argument << " " << key.flags;
  writeText(str.str(), BRLAPI_CURSOR_OFF);
}

void Display::SequencerTab::click(unsigned int cell, Display &display) {
  if (y >= 3 && y < 3 + song.patterns().size()) {
    auto &pattern = song.patterns()[y - 3];
    if (x+cell < pattern.size()) {
      pattern[x+cell] = pattern[x+cell] == 0? 1: 0;
      drawSong();
      display.redraw();
      display.pepper.sendRequest(UpdateSong { new Song(song) });
    }
  }
}

void Pepper::render(BelaContext *bela) {
  {
    char buffer[128];
    ssize_t const size = commandQueue.receive(buffer, 128);
    switch (size) {
    case sizeof(Request): {
      Request req;
      std::memcpy(&req, buffer, size);
      requestReceived(req);
      break;
    }
    default:
      break;
    }
  }
  digital.processInput(bela->digital, bela->digitalFrames);
  plugins[index]->run(bela);
}

Pepper::~Pepper() {
  for (auto &plugin: plugins) {
    plugin->deactivate();
  }
}

// Bela

static unique_ptr<Pepper> p;

void Bela_userSettings(BelaInitSettings *settings)
{
  settings->interleave = 0;
}

bool setup(BelaContext *bela, void * /*userData*/)
{
  if ((bela->flags & BELA_FLAG_INTERLEAVED) == BELA_FLAG_INTERLEAVED) {
    std::cerr << "This project only works in non-interleaved mode."
              << std::endl;
    return false;
  }
  try {
    p = make_unique<Pepper>(bela);
  } catch (std::exception &e) {
    std::cerr << e.what() << std::endl;
  }
  return bool(p);
}

void render(BelaContext *bela, void * /*userData*/)
{
  p->render(bela);
}

void cleanup(BelaContext * /*bela*/, void * /*userData*/)
{
  p = nullptr;
}
