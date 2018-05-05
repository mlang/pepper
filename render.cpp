#include <Bela.h>
#include <iostream>
#include <memory>
#include <vector>
#include <lilv/lilvmm.hpp>
#include <lv2/lv2plug.in/ns/ext/buf-size/buf-size.h>

static LV2_Feature buf_size_features[1] = {
  { LV2_BUF_SIZE__fixedBlockLength, NULL },
};
LV2_Feature* features[2] = {
  &buf_size_features[0], nullptr
};

class Plugin {
protected:
  Lilv::Instance *instance;
public:
  Plugin(Lilv::Plugin p, BelaContext *bela)
  : instance(Lilv::Instance::create(p, bela->audioSampleRate, features)) {
  }
  virtual ~Plugin() = default;
  void activate() { instance->activate(); }
  virtual void run(BelaContext *) = 0;
  void deactivate() { instance->deactivate(); }
};

class VocProc : public Plugin {
  std::vector<std::vector<float>> port;
  std::vector<float> minValue, maxValue, defValue;
public:
  VocProc(Lilv::Plugin p, BelaContext *bela)
  : Plugin(p, bela) {
    auto const count = p.get_num_ports();
    port.resize(count);
    minValue.resize(count); maxValue.resize(count); defValue.resize(count);
    p.get_port_ranges_float(&minValue.front(), &maxValue.front(), &defValue.front());
    for (int i = 0; i < count; i++) {
      //auto port = p.get_port_by_index(i);
      if (i < 3) {
        port[i].resize(bela->audioFrames);
      } else {
        port[i] = { defValue[i] };
      }
      instance->connect_port(i, &port[i].front());
    }
  }
  void run(BelaContext *bela) override {
    for (int frame = 0; frame < bela->audioFrames; frame++) {
      port[0][frame] = audioRead(bela, frame, 0);
      port[1][frame] = audioRead(bela, frame, 1);
    }
    instance->run(bela->audioFrames);
    for (int frame = 0; frame < bela->audioFrames; frame++) {
      auto const sample = port[2][frame];
      audioWrite(bela, frame, 0, sample);
      audioWrite(bela, frame, 1, sample);
    }
  }
};

class pepper {
  Lilv::World lilv;
  std::vector<std::unique_ptr<Plugin>> plugins;
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
        plugins.emplace_back(new VocProc(lv2plugin, bela));
        std::cout << "Loaded " << name.as_string() << std::endl;
      }
    }
    if (!this->plugins.empty()) index = 0;
    for (auto &plugin: plugins) plugin->activate();
  }
  ~pepper() {
    for (auto &plugin: plugins) plugin->deactivate();
  }
  void run(BelaContext *bela) {
    if (index >= 0 && index < plugins.size()) {
      plugins[index]->run(bela);
    }
  }
};

std::unique_ptr<pepper> p;

bool setup(BelaContext *context, void *userData)
{
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

