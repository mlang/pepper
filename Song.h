#ifndef SONG_H_DEFINED
#define SONG_H_DEFINED

#include "DSP.h"
#include "units.h"
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <vector>

enum class interpol { none, linear, cosine, sine };

class Song;
namespace boost { namespace serialization {
template<typename Archive> void serialize(Archive &, Song &, const unsigned int);
}}

class Song {
  using CVTrack = std::map<size_t, std::tuple<float, interpol>>;
  using TriggerTrack = std::set<size_t>;
  std::vector<CVTrack> cvTracks;
  std::vector<TriggerTrack> triggerTracks;
  size_t size;
  template<typename Archive>
  friend void boost::serialization::serialize(
    Archive &, Song &, const unsigned int
  );
public:
  Song()
  : cvTracks(4), triggerTracks(4), size(64)
  {
    cvTracks[0][0] = {0.6f, interpol::none};
    cvTracks[0][8] = {0.7f, interpol::linear};
    cvTracks[0][16] = {0.6f, interpol::none};
    for (size_t i = 0; i < size; i += 4) triggerTracks[0].insert(i);
  }
  size_t length() const noexcept { return size; }
  void length(size_t length) { size = length; }
  decltype(auto) cv() { return cvTracks; }
  decltype(auto) trigger() { return triggerTracks; }
  void flipTrigger(size_t track, size_t position) {
    if (triggerTracks[track].count(position)) {
      triggerTracks[track].erase(position);
    } else {
      triggerTracks[track].insert(position);
    } 
  }
  template<typename Analog, typename Digital>
  void playAt(unsigned int position, unsigned int offset, units::time::second_t sps,
          Analog &analog, Digital &digital) {
    if (position < size) {
      unsigned int cvChannel = 0;
      unsigned int triggerChannel = 0;
      for (auto const &cv: cvTracks) {
        auto const p = cv.find(position);
        if (p != cv.end()) {
          bool const last = std::next(p) == cv.end();
          auto next = last? cv.begin(): std::next(p);
          auto const level = std::get<0>(p->second);
          auto const nextLevel = std::get<0>(next->second);
          analog[cvChannel].reset_to(offset, level);
          interpolate<float>::signature *interp = nullptr;
          switch (std::get<1>(p->second)) {
          case interpol::linear:
            interp = &interpolate<float>::linear;
            break;
          }
          if (next != p && level != nextLevel && interp) {
            unsigned int length = (size - p->first + next->first) % size;
            analog[cvChannel].add_point(length*sps, nextLevel, interp);
          }
          analog[cvChannel+1].set_for(offset, units::voltage::volt_t(4), units::time::millisecond_t(5));
        }
        cvChannel += 2;
        if (cvChannel >= analog.size()) {
          break;
        }
      }

      for (auto const &trigger: triggerTracks) {
        auto p = trigger.find(position);
        if (p != trigger.end()) {
          digital[triggerChannel].set_for(offset * 2, units::time::millisecond_t(5));
        }
        triggerChannel += 1;
        if (triggerChannel == digital.size()) {
          break;
        }
      }
    }
  }
};

void save(Song const &song, std::string const &filename);
void load(Song &song, std::string const &filename);

#endif // SONG_H_DEFINED
