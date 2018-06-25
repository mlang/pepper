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
#include <Bela.h>
#include <DigitalChannelManager.h>
#include <algorithm>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/hana/functional/overload.hpp>
#include <boost/optional.hpp>
#include <boost/serialization/vector.hpp>
#include <brlapi.h>
#include <chrono>
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

using namespace std::literals::chrono_literals;

namespace {

enum class ModeIdentifier {
  AnalogueOscillator, FMOscillator,
  AudioLevelMeter, Sequencer
};

class Song {
  std::vector<std::vector<int>> pattern;
  std::vector<std::vector<int>> song;
public:
  Song() : pattern {
    {1, 0, 0, 0, 1, 0, 1, 0},
    {1, 0, 0, 0, 1, 0, 0, 0}
  }, song{
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
          continue;
        }
      }
      channel += 1;
    }
  }
  std::vector<std::vector<int>> &patterns() { return pattern; }
  template<typename Archive>
  void serialize(Archive &archive, unsigned int /*version*/) {
    archive & pattern;
    archive & song;
  }
};

void save(Song const &song, const std::string& filename) {
  std::ofstream ofs(filename);
  boost::archive::text_oarchive oa(ofs);
  oa << song;
}

void load(Song &song, const std::string& filename) {
  std::ifstream ifs(filename);
  if (ifs.good()) {
    boost::archive::text_iarchive ia(ifs);
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

using Request = mpark::variant<Command, UpdateSong>;

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

using Message = mpark::variant<
  ModeChanged, LevelsChanged, TempoChanged, SongLoaded, PositionChanged, SongUpdated
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
  std::unique_ptr<char[]> brlapiHandle;
  brlapi_handle_t *handle() {
    return static_cast<brlapi_handle_t *>(
      static_cast<void *>(brlapiHandle.get())
    );
  }
  int fd = -1;
  bool connected = false;
  unsigned int width = 0;
  bool connect();
  void writeText(std::string const &text, int cursor) {
    if (connected) {
      brlapi__writeText(handle(), cursor > static_cast<int>(width)? width: cursor,
			text.c_str());
    }
  }
  RTPipe updatePipe;
  void doPoll();
  void keyPressed(brlapi_keyCode_t keyCode);
  Pepper &pepper;
  class TabBase {
    std::string name;
  protected:
    unsigned int x = 0, y = 0;
    struct LineInfo {
      std::string text;
      int cursor = BRLAPI_CURSOR_OFF;

      LineInfo() = default;
      explicit LineInfo(
        std::string text, int cursor = BRLAPI_CURSOR_OFF
      ) : text(std::move(text)), cursor(cursor) {}
    };
    std::vector<LineInfo> lines;
  public:
    explicit TabBase(std::string name, size_t lines = 0)
    : name(std::move(name))
    , lines(lines)
    {}
    void draw(Display &display) {
      if (y == 0) {
        display.writeText(name, BRLAPI_CURSOR_OFF);
      } else {
        auto const &line = lines[y - 1];
        display.writeText(
	  line.text.substr(std::min(static_cast<std::string::size_type>(x),
				    line.text.length())),
	  line.cursor
	);
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
  };
  class AnalogueOscillatorTab : public TabBase {
  public:
    AnalogueOscillatorTab() : TabBase("AnalogueOsc") {}
    void click(unsigned int /*cell*/, Display & /*display*/) {}
  };
  class FMOscillatorTab : public TabBase {
  public:
    FMOscillatorTab() : TabBase("FMOsc") {}
    void click(unsigned int /*cell*/, Display & /*display*/) {}
  };
  class LevelMeterTab : public TabBase {
  public:
    LevelMeterTab() : TabBase("metering...", 12) {}
    void operator()(LevelsChanged const &level) {
      lines[0].text = "L: " + std::to_string(level.l);
      lines[1].text = "R: " + std::to_string(level.r);
      lines[2].text = "PeakL: " + std::to_string(level.lp);
      lines[3].text = "PeakR: " + std::to_string(level.rp);
      for (unsigned long i = 0; i < 8; i++) {
        lines[i+4].text = "A" + std::to_string(i) + ": " +
          std::to_string(level.analog[i]);
      }
    }
    void click(unsigned int /*cell*/, Display & /*display*/) {}
  };
  class SequencerTab : public TabBase {
    Song song;
  public:
    SequencerTab() : TabBase("Sequencer", 2) {}
    void operator()(TempoChanged const &tempo) {
      if (tempo.bpm == 0.0f) {
        lines[0].text = "Not running";
      } else {
        lines[0].text = "BPM: " +
          std::to_string(static_cast<int>(std::round(tempo.bpm)));
      }
    }
    void operator()(Song const *song) {
      this->song = *song;
      drawSong();
    }
    void drawSong() {
      lines.resize(2);
      for (auto const &pattern: this->song.patterns()) {
        std::string rep;
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
      lines[1].cursor = changed.position + 1;
      lines[1].text = std::string(changed.position, ' ') + '=';
    }
    void click(unsigned int cell, Display &display);
  };
  using Tab = mpark::variant<
    AnalogueOscillatorTab, FMOscillatorTab,
    LevelMeterTab, SequencerTab
  >;
  std::vector<Tab> tabs;
  ModeIdentifier currentMode = ModeIdentifier::Sequencer;
  Tab &currentTab() { return tabs[static_cast<int>(currentMode)]; }
  void redraw() {
    mpark::visit([this](auto &tab) { tab.draw(*this); }, currentTab());
  }
  LevelMeterTab &levelMeterTab() {
    return mpark::get<LevelMeterTab>(
      tabs[static_cast<int>(ModeIdentifier::AudioLevelMeter)]
    );
  }
  SequencerTab &sequencerTab() {
    return mpark::get<SequencerTab>(
      tabs[static_cast<int>(ModeIdentifier::Sequencer)]
    );
  }
  friend class TabBase;
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
  std::vector<std::unique_ptr<Mode>> plugins;
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

class LV2Plugin : public Mode {
protected:
  Lilv::Instance *instance;
  std::vector<float> minValue, maxValue, defValue, value;
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
public:
  static constexpr const char *uri =
    "http://plugin.org.uk/swh-plugins/analogueOsc";
  ModeIdentifier mode() const override {
    return ModeIdentifier::AnalogueOscillator;
  }
  AnalogueOscillator
  (Pepper &pepper, BelaContext *bela, Lilv::World &lilv, Lilv::Plugin p)
  : LV2Plugin(pepper, bela, lilv, p) {
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
    controlFromAnalog(1, analogReadNI(bela, 0, 0)); // Frequency
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

    explicit AudioChannel(unsigned int sr) : dcblock(highpass, sr * hertz, 5_Hz, 0.5) {}

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
  std::vector<AudioChannel> audio;
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

template<typename Rep, typename Period> constexpr Rep to_samples(
  std::chrono::duration<Rep, Period> duration, unsigned int sampleRate
) {
  return duration.count() * Period::num * sampleRate / Period::den;
}

class AnalogOut {
  unsigned int channel;
  unsigned int sampleRate;
  size_t offset = 0, length = 0;
  float level{};
public:
  AnalogOut(BelaContext *bela, unsigned int channel)
  : channel(channel), sampleRate(bela->analogSampleRate) {}
  template<typename Rep, typename Period>
  void set_for(unsigned int frame, float level,
               std::chrono::duration<Rep, Period> duration) {
    this->offset = frame;
    this->length = to_samples(duration, sampleRate);
    this->level = level;
  }
  void run(BelaContext *bela) {
    auto * const samples = analogOut(bela, channel);
    auto * const begin = samples + offset;
    auto const size = std::min(bela->analogFrames - offset, length);
    auto * const end = begin + size;

    std::fill(samples, begin, 0);
    std::fill(begin, end, level);
    std::fill(end, samples + bela->analogFrames, 0);

    length -= size;
    offset = 0;
  }
};

class Clock {
  boost::optional<int> offset;
  boost::optional<int> length;
public:
  void tick(unsigned int frame) {
    offset = frame;
  }
  template<typename BPM>
  void run(BelaContext *bela, BPM bpm) {
    if (offset) {
      if (length) {
        length = length.value() + offset.value();
        bpm(60.0f /
            (static_cast<float>(length.value()) /
             static_cast<float>(bela->analogSampleRate)) /
            4.0f);
      }
      length = bela->analogFrames - offset.value();
      offset = boost::none;
    } else {
      if (length) {
        length = length.value() + bela->analogFrames;
        if (length.value() > bela->analogSampleRate) {
          bpm(0.0f);
          length = boost::none;
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
  std::vector<AnalogOut> analogOut;
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
          analogOut[channel].set_for(frame, 0.9, 1ms);
        });
        pepper.updateDisplay(Message { PositionChanged { position } });
        position = song.length() > 0? (position + 1) % song.length(): 0;
      }
    }
    for (auto &channel: analogOut) {
      channel.run(bela);
    }
    clock.run(bela, [this](float bpm) {
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

  plugins.emplace_back(new Sequencer(*this, bela));
  plugins.emplace_back(new AudioLevelMeter(*this, bela));

  lilv.load_all();
  auto lv2plugins = lilv.get_all_plugins();
  for (auto i = lv2plugins.begin(); !lv2plugins.is_end(i); i = lv2plugins.next(i)) {
    auto lv2plugin = Lilv::Plugin(lv2plugins.get(i));
    auto name = Lilv::Node(lv2plugin.get_name());
    auto uri = Lilv::Node(lv2plugin.get_uri());
    if (uri.is_uri() && strcmp(uri.as_string(), AnalogueOscillator::uri) == 0) {
      plugins.emplace_back(new AnalogueOscillator(*this, bela, lilv, lv2plugin));
      std::cout << "Loaded " << name.as_string() << std::endl;
    } else if (uri.is_uri() && strcmp(uri.as_string(), FMOscillator::uri) == 0) {
      plugins.emplace_back(new FMOscillator(*this, bela, lilv, lv2plugin));
      std::cout << "Loaded " << name.as_string() << std::endl;
    }
  }

  if (!this->plugins.empty()) {
    index = 0;
  }
  for (auto &plugin: plugins) { plugin->activate();
  modeChanged();
}
}

void Pepper::requestReceived(Request &req) {
  mpark::visit(
    boost::hana::overload(
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
      tabs.push_back(AnalogueOscillatorTab{});
      break;
    case ModeIdentifier::AudioLevelMeter:
      tabs.push_back(LevelMeterTab{});
      break;
    case ModeIdentifier::FMOscillator:
      tabs.push_back(FMOscillatorTab{});
      break;
    case ModeIdentifier::Sequencer:
      tabs.push_back(SequencerTab{});
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

  while (gShouldStop == 0) {
    int ret = ::poll(fds, std::distance(std::begin(fds), std::end(fds)),
                     std::chrono::milliseconds(10).count());
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
          mpark::visit(
            boost::hana::overload(
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
        if (mpark::visit([](auto const &tab) { return tab.line(); }, currentTab())
            == 0) {
          pepper.sendCommand(Command::PrevPlugin);
        } else {
	  mpark::visit([this](auto &tab) { tab.windowLeft(*this); }, currentTab());
	}
        return;
      case BRLAPI_KEY_CMD_FWINRT:
        if (mpark::visit([](auto const &tab) { return tab.line(); }, currentTab())
            == 0) {
          pepper.sendCommand(Command::NextPlugin);
        } else {
	  mpark::visit([this](auto &tab) { tab.windowRight(*this); }, currentTab());
	}
        return;
      case BRLAPI_KEY_CMD_LNUP:
        mpark::visit([this](auto &tab) { tab.lineUp(*this); }, currentTab());
        return;
      case BRLAPI_KEY_CMD_LNDN:
        mpark::visit([this](auto &tab) { return tab.lineDown(*this); },
                     currentTab());
        return;
      case BRLAPI_KEY_CMD_ROUTE:
        mpark::visit(
	  [this, &key](auto &tab) { tab.click(key.argument, *this); },
	  currentTab());
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
  if (y > 2 && y < 2 + song.patterns().size()) {
    auto &pattern = song.patterns()[y - 3];
    if (cell < pattern.size()) {
      pattern[cell] = pattern[cell] == 0? 1: 0;
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
  for (auto &plugin: plugins) { plugin->deactivate();
}
}

// Bela

static std::unique_ptr<Pepper> p;

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
    p = std::make_unique<Pepper>(bela);
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
