#include "AuxTask.h"
#include "EdgeDetect.h"
#include "RTPipe.h"
#include "RTQueue.h"
#include "mpark_variant.h"
#include <Bela.h>
#include <DigitalChannelManager.h>
#include <brlapi.h>
#include <lilv/lilvmm.hpp>
#include <lv2/lv2plug.in/ns/ext/buf-size/buf-size.h>
#include <poll.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

using namespace std::literals::chrono_literals;

enum class Command { PrevPlugin, NextPlugin };

enum class ModeIdentifier {
  AnalogueOscillator, FMOscillator,
  AudioLevelMeter, Sequencer
};

struct ModeChanged {
  ModeIdentifier mode;
};

struct LevelsChanged {
  float l, r, lp, rp;
  float analog[8];
};

using Message = mpark::variant<ModeChanged, LevelsChanged>;

  static constexpr int nPins = 4;

  static constexpr int trigOutPins[nPins] = { 0, 5, 12, 13 };
  static constexpr int trigInPins[nPins] = { 15, 14, 1, 3 };
  static constexpr int sw1Pin = 6;
  static constexpr int ledPins[nPins] = { 2, 4, 8, 9 };
  static constexpr int pwmPin = 7;
  static constexpr int gNumButtons = nPins;

  static constexpr int buttonPins[gNumButtons] = {
    sw1Pin, trigInPins[1], trigInPins[2], trigInPins[3]
  };

class Salt {
public:
  Salt(BelaContext *bela) {
    pinMode(bela, 0, pwmPin, OUTPUT);
    for(unsigned int i = 0; i < nPins; ++i) {
      pinMode(bela, 0, trigOutPins[i], OUTPUT);
      pinMode(bela, 0, trigInPins[i], INPUT);
      pinMode(bela, 0, ledPins[i], OUTPUT);
    }
  }
};

namespace {

class Pepper;

class Display {
  std::unique_ptr<char[]> brlapiHandle;
  brlapi_handle_t *handle() {
    return reinterpret_cast<brlapi_handle_t *>(brlapiHandle.get());
  }
  int fd = -1;
  bool connected = false;
  bool connect() {
    brlapi_settings_t settings = BRLAPI_SETTINGS_INITIALIZER;
    fd = brlapi__openConnection(handle(), &settings, &settings);
    if (fd != -1) {
      unsigned int x, y;
      brlapi__getDisplaySize(handle(), &x, &y);
      auto tty = brlapi__enterTtyMode(handle(), 1, "");
      if (tty != -1) {
        return true;
      }

      brlapi__closeConnection(handle());
    }
    return false;
  }
    
  std::string previousText;
  void writeText(std::string text, int cursor = BRLAPI_CURSOR_OFF) {
    if (connected) {
      if (text != previousText) {
        brlapi__writeText(handle(), cursor, text.data());
        previousText = text;
      }
    }
  }
  RTPipe updatePipe;
  void doPoll();
  void keyPressed(brlapi_keyCode_t keyCode);
  void updated(ModeChanged changed) {
    currentMode = changed.mode;
    tabs[static_cast<int>(currentMode)].draw(*this);
  }
  void updated(LevelsChanged changed) {
    tabs[static_cast<int>(ModeIdentifier::AudioLevelMeter)].updated(changed);
  }
  Pepper &pepper;
  class Tab {
    std::string name;
    std::vector<std::string> lines;
    unsigned int x = 0, y = 0;
  public:
    Tab() : name("<unnamed>") {}
    Tab(std::string name) : name(name) {}
    void draw(Display &display) {
      if (y == 0) {
        display.writeText(name);
      } else {
        display.writeText(lines[y - 1]);
      }
    }
    unsigned int line() const { return y; }
    void updated(LevelsChanged const &level) {
    }
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
  };
  std::vector<Tab> tabs;
  ModeIdentifier currentMode = ModeIdentifier::Sequencer;
public:
  Display(Pepper &pepper)
  : brlapiHandle(new char[brlapi_getHandleSize()])
  , connected(connect())
  , updatePipe("update-display", 0x10000)
  , pepper(pepper)
  , poll("brlapi-poll", &Display::doPoll, this)
  {
    auto const modes = static_cast<int>(ModeIdentifier::Sequencer) + 1;
    tabs.resize(modes);
    for (int i = 0; i < modes; ++i) {
      switch (ModeIdentifier(i)) {
      case ModeIdentifier::AnalogueOscillator:
        tabs[i] = Tab("AnalogueOSC");
        break;
      case ModeIdentifier::AudioLevelMeter:
        tabs[i] = Tab("metering...");
        break;
      case ModeIdentifier::FMOscillator:
        tabs[i] = Tab("FM Oscillator");
        break;
      case ModeIdentifier::Sequencer:
        tabs[i] = Tab("Sequencer");
        break;
      }
    }
    if (connected) writeText("Welcome!");
  }
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
  static void digitalChanged(bool state, unsigned int, void *data) {
    if (state) {
      static_cast<Pepper *>(data)->nextPlugin();
    }
  }
  void prevPlugin() {
    if (index) {
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
  void commandReceived(Command command);
public:
  Pepper(BelaContext *);
  ~Pepper();
  void render(BelaContext *);
  void sendCommand(Command const &cmd) {
    commandQueue.write(cmd);
  }
};

static inline float const *audioInChannel(BelaContext *bela, unsigned int channel) {
  return &bela->audioIn[channel * bela->audioFrames];
}

template<unsigned int channel> static inline float *
audioOutChannel(BelaContext *bela) {
  return &bela->audioOut[channel * bela->audioFrames];
}
static inline float *
audioOutChannel(BelaContext *bela, unsigned int channel) {
  return &bela->audioOut[channel * bela->audioFrames];
}

class Mode {
protected:
  Pepper &parent;
public:
  Mode(Pepper &parent) : parent(parent) {}
  virtual ~Mode() = default;

  virtual ModeIdentifier mode() const = 0;
  virtual void activate() = 0;
  virtual void run(BelaContext *bela) = 0;
  virtual void deactivate() = 0;
};

static LV2_Feature hardRTCapable = { LV2_CORE__hardRTCapable, nullptr };
static LV2_Feature fixedBlockSize = { LV2_BUF_SIZE__fixedBlockLength, nullptr };

static LV2_Feature *features[3] = {
  &hardRTCapable, &fixedBlockSize, nullptr
};

class LV2Plugin : public Mode {
protected:
  Lilv::Instance *instance;
  std::vector<float> minValue, maxValue, defValue, value;
public:
  LV2Plugin(Pepper &parent, BelaContext *bela, Lilv::World &lilv, Lilv::Plugin p)
  : Mode(parent)
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
  void connectAudioIn(BelaContext *bela, unsigned lv2PortIndex, unsigned channel) {
    connectAudio(bela, lv2PortIndex, channel, const_cast<float *>(bela->audioIn));
  }
  void connectAudioOut(BelaContext *bela, unsigned lv2PortIndex, unsigned channel) {
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
  static constexpr const char *uri = "http://plugin.org.uk/swh-plugins/analogueOsc";
  ModeIdentifier mode() const override {
    return ModeIdentifier::AnalogueOscillator;
  }
  AnalogueOscillator
  (Pepper &parent, BelaContext *bela, Lilv::World &lilv, Lilv::Plugin p)
  : LV2Plugin(parent, bela, lilv, p) {
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
    std::copy_n(audioOutChannel<0>(bela), bela->audioFrames, audioOutChannel<1>(bela));
  }
};

class FMOscillator final : public LV2Plugin {
public:
  static constexpr const char *uri = "http://plugin.org.uk/swh-plugins/fmOsc";
  ModeIdentifier mode() const override { return ModeIdentifier::FMOscillator; }
  FMOscillator(Pepper &parent, BelaContext *bela, Lilv::World &lilv, Lilv::Plugin p)
  : LV2Plugin(parent, bela, lilv, p) {
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
    std::copy_n(audioOutChannel<0>(bela), bela->audioFrames,
                audioOutChannel<1>(bela));
  }
};

class Biquad {
  double B0, B1, B2, A1, A2;
  double X[2], Y[2];
public:
  Biquad()
  : B0(0.99949640), B1(-1.99899280), B2(B0), A1(-1.99899254), A2(0.99899305)
  , X{0, 0}, Y{0, 0}
  {}
  double operator()(double in) {
    float out = B0 * in + B1 * X[0] + B2 * X[1] - A1 * Y[0] - A2 * Y[1];

    X[1] = X[0];
    X[0] = in;
    Y[1] = Y[0];
    Y[0] = out;
    return out;
  }
};

class AudioLevelMeter : public Mode {
  struct Channel {
    Biquad dcblock;
    float localLevel = 0, peakLevel = 0;
    Channel() : dcblock() {}
    void operator()(float sample) {
      float const level = std::abs(dcblock(sample));
      if (level > localLevel)
        localLevel = level;
      else
        localLevel *= localDecayRate;
      if (level > peakLevel)
        peakLevel = level;
      else
        peakLevel *= peakDecayRate;
    }
  };
  std::vector<Channel> data;
  static constexpr float const localDecayRate = 0.99, peakDecayRate = 0.999;
public:
  AudioLevelMeter(Pepper &parent, BelaContext *bela)
  : Mode(parent), data(bela->audioInChannels) {}
  ModeIdentifier mode() const override { return ModeIdentifier::AudioLevelMeter; }
  void activate() override {}
  void deactivate() override {}
  void run(BelaContext *bela) override {
    for (unsigned int channel = 0; channel < bela->audioInChannels; ++channel) {
      auto const samples = audioInChannel(bela, channel);
      std::for_each(samples, samples + bela->audioFrames, data[channel]);
      std::copy_n(samples, bela->audioFrames, audioOutChannel(bela, channel));
    }
  }
};

class AnalogOut {
  unsigned int channel;
  unsigned int sampleRate;
  size_t offset = 0, length = 0;
  float level;
public:
  AnalogOut(BelaContext *bela, unsigned int channel)
  : channel(channel), sampleRate(bela->analogSampleRate) {}
  template<typename Rep, typename Period>
  void set_for(unsigned int frame, float level,
               std::chrono::duration<Rep, Period> duration) {
    this->offset = frame;
    this->length = duration.count() * Period::num * sampleRate / Period::den;
    this->level = level;
  }
  void run(BelaContext *bela) {
    auto * const samples = audioOutChannel(bela, channel);
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

class Sequencer final : public Mode {
  EdgeDetect<float> clockRising, resetRising;
  unsigned int position = 0;
  std::vector<int> pattern;
  std::vector<AnalogOut> analogOut;
public:
  Sequencer(Pepper &pepper, BelaContext *bela)
  : Mode(pepper)
  , clockRising(0.2), resetRising(0.2)
  , pattern{1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 1}
  , analogOut()
  {
    for (unsigned int channel = 0; channel < bela->analogOutChannels; ++channel) {
      analogOut.emplace_back(bela, channel);
    }
  }
  ModeIdentifier mode() const override { return ModeIdentifier::Sequencer; }
  void activate() override {}
  void deactivate() override {}
  void run(BelaContext *bela) override {
    for (unsigned int frame = 0; frame < bela->analogFrames; ++frame) {
      if (resetRising(analogReadNI(bela, frame, 1))) {
        position = 0;
      }
      if (clockRising(analogReadNI(bela, frame, 0))) {
        if (pattern[position] == 1) {
          analogOut[0].set_for(frame, 0.5, 5ms);
        }
        position = (position + 1) % pattern.size();
      }
    }
    for (auto &channel: analogOut) channel.run(bela);
  }
};

}

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

  if (!this->plugins.empty()) index = 0;
  modeChanged();
  for (auto &plugin: plugins) plugin->activate();
}

void Pepper::commandReceived(Command command) {
  switch (command) {
  case Command::PrevPlugin:
    prevPlugin();
    break;
  case Command::NextPlugin:
    nextPlugin();
    break;
  default:
    fprintf(stderr, "Unknown command %d\n", static_cast<int>(command));
  }
}

void Pepper::modeChanged() {
  Message msg(ModeChanged { plugins[index]->mode() });
  braille.write(msg);
}

void Display::doPoll() {
  pollfd fds[] = {
    { fd, POLLIN, 0 },
    { updatePipe.open(), POLLIN, 0 }
  };

  while (!gShouldStop) {
    int ret = ::poll(fds, std::distance(std::begin(fds), std::end(fds)),
                     std::chrono::milliseconds(10).count());
    if (ret < 0) {
      perror("poll");
      break;
    }

    if (ret == 0) continue;

    for (auto const &item: fds) {
      if (!(item.revents & POLLIN)) continue;

      if (item.fd == fd) {
        brlapi_keyCode_t keyCode;
        while (brlapi__readKey(handle(), 0, &keyCode) == 1) {
          keyPressed(keyCode);
        }
      } else if (item.fd == updatePipe.fileDescriptor()) {
        Message msg;
        read(updatePipe.fileDescriptor(), &msg, sizeof(Message));
        mpark::visit([this](auto &content) { updated(content); }, msg);
      }
    }
  }
}

void Display::keyPressed(brlapi_keyCode_t keyCode) {
  brlapi_expandedKeyCode_t key;
  if (brlapi_expandKeyCode(keyCode, &key) == 0) {
    switch (key.type) {
    case BRLAPI_KEY_TYPE_CMD:
      switch (key.command) {
      case BRLAPI_KEY_CMD_FWINLT:
        if (tabs[static_cast<int>(currentMode)].line() == 0) {
          pepper.sendCommand(Command::PrevPlugin);
        }
        break;
      case BRLAPI_KEY_CMD_FWINRT:
        if (tabs[static_cast<int>(currentMode)].line() == 0) {
          pepper.sendCommand(Command::NextPlugin);
        }
        break;
      case BRLAPI_KEY_CMD_LNUP:
        tabs[static_cast<int>(currentMode)].lineUp(*this);
        break;
      case BRLAPI_KEY_CMD_LNDN:
        tabs[static_cast<int>(currentMode)].lineDown(*this);
        break;
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
  writeText(str.str());
}

void Pepper::render(BelaContext *bela) {
  {
    char buffer[128];
    ssize_t const size = commandQueue.receive(buffer, 128);
    switch (size) {
    case sizeof(Command): {
      Command cmd;
      memcpy(&cmd, buffer, size);
      commandReceived(cmd);
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
  for (auto &plugin: plugins) plugin->deactivate();
}

// Bela

static std::unique_ptr<Pepper> p;

void Bela_userSettings(BelaInitSettings *settings)
{
  settings->interleave = 0;
}

bool setup(BelaContext *bela, void *)
{
  if ((bela->flags & BELA_FLAG_INTERLEAVED) == BELA_FLAG_INTERLEAVED) {
    std::cerr << "This project only works in non-interleaved mode." << std::endl;
    return false;
  }
  try {
    p = std::make_unique<Pepper>(bela);
  } catch (std::exception &e) {
    std::cerr << e.what() << std::endl;
  }
  return bool(p);
}

void render(BelaContext *bela, void *)
{
  p->render(bela);
}

void cleanup(BelaContext *, void *)
{
  p = nullptr;
}
