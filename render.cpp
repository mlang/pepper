#include "AuxTask.h"
#include <Bela.h>
#include <brlapi.h>
#include <algorithm>
#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>
#include <lilv/lilvmm.hpp>
#include <lv2/lv2plug.in/ns/ext/buf-size/buf-size.h>

constexpr int nPins = 4;

constexpr int trigOutPins[nPins] = {0, 5, 12, 13};
constexpr int trigInPins[nPins] = {15, 14, 1, 3};
constexpr int sw1Pin = 6;
constexpr int ledPins[nPins] = {2, 4, 8, 9};
constexpr int pwmPin = 7;
constexpr int gNumButtons = nPins;

constexpr int buttonPins[gNumButtons] = {
  sw1Pin, trigInPins[1], trigInPins[2], trigInPins[3]
};

class Display {
  std::unique_ptr<char[]> brlapiHandle;
  brlapi_handle_t *handle() {
    return reinterpret_cast<brlapi_handle_t *>(brlapiHandle.get());
  }
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
  int fd;
    
  void writeText(std::string text, int cursor = BRLAPI_CURSOR_OFF) {
    if (connected) brlapi__writeText(handle(), cursor, text.data());
  }
  void doRun() {
    while (!gShouldStop) {
      fd_set rfds, efds;
      struct timeval tv;
      FD_ZERO(&rfds);
      FD_ZERO(&efds);
      FD_SET(fd, &rfds);
      FD_SET(fd, &efds);
      tv.tv_sec = 0;
      tv.tv_usec = 10000;
      if (select(fd+1, &rfds, nullptr, &efds, &tv) > 0) {
        brlapi_keyCode_t keyCode;
        while (brlapi__readKey(handle(), 0, &keyCode) == 1) {
          keyPressed(keyCode);
        }
      }
    }
  }
  void keyPressed(brlapi_keyCode_t &keyCode) {
    std::stringstream str;
    str << keyCode;
    writeText(str.str());
  }
public:
  Display()
  : brlapiHandle(new char[brlapi_getHandleSize()])
  , connected(connect())
  , run("brlapi-run", &Display::doRun, this) {
    if (connected) writeText("Welcome!");
  }
  ~Display() {
    if (connected) {
      brlapi__leaveTtyMode(handle());
      brlapi__closeConnection(handle());
    }
  }
  AuxTask<NonRT, decltype(&Display::doRun)> run;
};

static LV2_Feature hardRTCapable = { LV2_CORE__hardRTCapable, NULL };
static LV2_Feature buf_size_features[1] = {
  { LV2_BUF_SIZE__fixedBlockLength, NULL },
};

LV2_Feature* features[3] = {
  &hardRTCapable, &buf_size_features[0], nullptr
};

class Mode {
public:
  Mode() = default;
  virtual ~Mode() = default;

  virtual void activate() = 0;
  virtual void run(BelaContext *bela) = 0;
  virtual void deactivate() = 0;
};

class pepper {
  Lilv::World lilv;
  std::vector<std::unique_ptr<Mode>> plugins;
  int index = -1;
  Display braille;
public:
  pepper(BelaContext *bela);
  ~pepper() { for (auto &plugin: plugins) plugin->deactivate(); }
  void run(BelaContext *bela) {
    if (index >= 0 && index < plugins.size()) {
      plugins[index]->run(bela);
    }
  }
};

class LV2Plugin : public Mode {
protected:
  Lilv::Instance *instance;
  std::vector<float> minValue, maxValue, defValue, value;
public:
  LV2Plugin(BelaContext *bela, Lilv::World &lilv, Lilv::Plugin p)
  : Mode(), instance(Lilv::Instance::create(p, bela->audioSampleRate, features)) {
    static LilvNode *lv2_core__sampleRate = lilv.new_uri(LV2_CORE__sampleRate);
    auto const count = p.get_num_ports();
    minValue.resize(count); maxValue.resize(count); defValue.resize(count);
    p.get_port_ranges_float(&minValue.front(), &maxValue.front(), &defValue.front());
    for (int i = 0; i < count; i++) {
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
  void connectAudio(BelaContext *bela, unsigned lv2PortIndex, unsigned channel, float *buffer) {
    instance->connect_port(lv2PortIndex, &buffer[bela->audioFrames * channel]);
  }
};

class AnalogueOscillator final : public LV2Plugin {
public:
  static constexpr const char *uri = "http://plugin.org.uk/swh-plugins/analogueOsc";
  AnalogueOscillator(BelaContext *bela, Lilv::World &lilv, Lilv::Plugin p)
  : LV2Plugin(bela, lilv, p) {
    auto const count = p.get_num_ports();
    for (int i = 0; i < count; i++) {
      switch (i) {
      case 4:
        connectAudioIn(bela, 4, 0);
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
    std::copy_n(&bela->audioOut[0 * bela->audioFrames], bela->audioFrames,
                &bela->audioOut[1 * bela->audioFrames]);
  }
};

pepper::pepper(BelaContext *bela)
: braille() {
  braille.run();
  lilv.load_all();
  auto lv2plugins = lilv.get_all_plugins();
  for (auto i = lv2plugins.begin(); !lv2plugins.is_end(i); i = lv2plugins.next(i)) {
    auto lv2plugin = Lilv::Plugin(lv2plugins.get(i));
    auto name = Lilv::Node(lv2plugin.get_name());
    std::cout << "Seen " << name.as_string() << std::endl;
    auto uri = Lilv::Node(lv2plugin.get_uri());
    if (uri.is_uri() && strcmp(uri.as_string(), AnalogueOscillator::uri) == 0) {
      plugins.emplace_back(new AnalogueOscillator(bela, lilv, lv2plugin));
      std::cout << "Loaded " << name.as_string() << std::endl;
    }
  }
  if (!this->plugins.empty()) index = 0;
  for (auto &plugin: plugins) plugin->activate();
}

// Bela

static std::unique_ptr<pepper> p;

void Bela_userSettings(BelaInitSettings *settings) {
  settings->interleave = 0;
}

bool setup(BelaContext *context, void *userData) {
  if ((context->flags & BELA_FLAG_INTERLEAVED) == BELA_FLAG_INTERLEAVED) {
    std::cerr << "This project only works in non-interleaved mode." << std::endl;
    return false;
  }
  pinMode(context, 0, pwmPin, OUTPUT);
  for(unsigned int i = 0; i < nPins; ++i) {
    pinMode(context, 0, trigOutPins[i], OUTPUT);
    pinMode(context, 0, trigInPins[i], INPUT);
    pinMode(context, 0, ledPins[i], OUTPUT);
  }
  p = std::unique_ptr<pepper>(new pepper(context));
  return bool(p);
}

void render(BelaContext *context, void *) {
  p->run(context);
}

void cleanup(BelaContext *context, void *userData) {
  p = nullptr;
}
