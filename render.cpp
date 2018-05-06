#include <Bela.h>
#include <iostream>
#include <memory>
#include <vector>
#include <lilv/lilvmm.hpp>
#include <lv2/lv2plug.in/ns/ext/buf-size/buf-size.h>

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

class LV2Plugin : public Mode {
protected:
  Lilv::Instance *instance;
  std::vector<float> minValue, maxValue, defValue, value;
public:
  LV2Plugin(BelaContext *bela, Lilv::World &lilv, Lilv::Plugin p)
  : Mode(), instance(Lilv::Instance::create(p, bela->audioSampleRate, features)) {
    auto const count = p.get_num_ports();
    minValue.resize(count); maxValue.resize(count); defValue.resize(count); value.resize(count);
    p.get_port_ranges_float(&minValue.front(), &maxValue.front(), &defValue.front());
    for (int i = 0; i < count; i++) {
      auto port = p.get_port_by_index(i);
      if (port.has_property(lilv.new_uri(LV2_CORE__sampleRate))) {
        minValue[i] *= bela->audioSampleRate;
        maxValue[i] *= bela->audioSampleRate;
      }
      value[i] = defValue[i];
    }
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
  void connectAudio(BelaContext *bela, unsigned lv2PortIndex, unsigned channel, float *buffer) {
    instance->connect_port(lv2PortIndex, &buffer[bela->audioFrames * channel]);
  }
};

class VocProc final : public LV2Plugin {
public:
  VocProc(BelaContext *bela, Lilv::World &lilv, Lilv::Plugin p)
  : LV2Plugin(bela, lilv, p) {
    auto const count = p.get_num_ports();
    for (int i = 0; i < count; i++) {
      switch (i) {
      case 0:
      case 1:
        connectAudioIn(bela, i, i);
        break;
      case 2:
        connectAudioOut(bela, i, 0);
        break;
      default:
        instance->connect_port(i, &value[i]);
        break;
      }
    }
  }
  void run(BelaContext *bela) override {
    float analog[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    for (int frame = 0; frame < bela->analogFrames; frame++) {
      for (int channel = 0; channel < bela->analogInChannels; channel++) {
        analog[channel] += analogReadNI(bela, frame, channel);
      }
    }
    for (int channel = 0; channel < bela->analogInChannels; channel++) {
      analog[channel] /= bela->analogFrames;
    }
    controlFromAnalog(3, analog[0]); // Pitch Factor
    controlFromAnalog(4, analog[1]); // Robotize/Whisperize
    controlFromAnalog(8, analog[2]); // Threshold
    controlFromAnalog(9, analog[3]); // Attack
    controlFromAnalog(10, analog[4]); // Transpose

    instance->run(bela->audioFrames);

    for (int frame = 0; frame < bela->audioFrames; frame++) {
      audioWriteNI(bela, frame, 1, audioReadNI(bela, frame, 0));
    }
  }
};

class pepper {
  Lilv::World lilv;
  std::vector<std::unique_ptr<Mode>> plugins;
  int index = -1;
public:
  pepper(BelaContext *bela) {
    lilv.load_all();
    auto lv2plugins = lilv.get_all_plugins();
    for (auto i = lv2plugins.begin(); !lv2plugins.is_end(i); i = lv2plugins.next(i)) {
      auto lv2plugin = Lilv::Plugin(lv2plugins.get(i));
      auto name = Lilv::Node(lv2plugin.get_name());
      std::cout << "Seen " << name.as_string() << std::endl;
      if (name.is_string() && name.as_string() == std::string("VocProc")) {
        plugins.emplace_back(new VocProc(bela, lilv, lv2plugin));
        std::cout << "Loaded " << name.as_string() << std::endl;
      }
    }
    if (!this->plugins.empty()) index = 0;
    for (auto &plugin: plugins) plugin->activate();
  }
  ~pepper() { for (auto &plugin: plugins) plugin->deactivate(); }
  void run(BelaContext *bela) {
    if (index >= 0 && index < plugins.size()) {
      plugins[index]->run(bela);
    }
  }
};

// Bela

std::unique_ptr<pepper> p;

void Bela_userSettings(BelaInitSettings *settings)
{
  settings->interleave = 0;
}

bool setup(BelaContext *context, void *userData)
{
  if ((context->flags & BELA_FLAG_INTERLEAVED) == BELA_FLAG_INTERLEAVED) {
    std::cerr << "This project only works in non-interleaved mode, "
              << "a custom main is required." << std::endl;
    return false;
  }
  p = std::unique_ptr<pepper>(new pepper(context));
  return true;
}

void render(BelaContext *context, void *userData)
{
  p->run(context);
}

void cleanup(BelaContext *context, void *userData)
{
  p = nullptr;
}
