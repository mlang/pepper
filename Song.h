#ifndef SONG_H_DEFINED
#define SONG_H_DEFINED

#include "DSP.h"
#include "units.h"
#include <map>
#include <set>
#include <string>
#include <vector>

enum class interpol { none, linear, cosine, sine };

struct Note {
  uint8_t value;
  interpol interp;
};

class Song {
  size_t size;
  using CVTrack = std::map<size_t, Note>;
  using TriggerTrack = std::set<size_t>;
  std::vector<CVTrack> cvTracks;
  std::vector<TriggerTrack> triggerTracks;
public:
  Song()
  : size(64), cvTracks(4), triggerTracks(4)
  {
    cvTracks[0][0] = {12, interpol::none};
    cvTracks[0][8] = {24, interpol::linear};
    cvTracks[0][16] = {12, interpol::none};
    for (size_t i = 0; i < size; i += 4) triggerTracks[0].insert(i);
  }
  size_t length() const noexcept { return size; }
  void length(size_t length) { size = length; }
  size_t length(CVTrack::const_iterator begin, CVTrack::const_iterator end,
                CVTrack::const_iterator pos
  ) const {
    if (pos != end) {
      bool const is_last = std::next(pos) == end;
      if (pos == begin && is_last) return size;

      return (size - pos->first + (is_last? begin: std::next(pos))->first) % size;
    }

    return 0;
  } 
  std::vector<CVTrack> &cv() { return cvTracks; }
  std::vector<CVTrack> const &cv() const { return cvTracks; }
  std::vector<TriggerTrack> &trigger() { return triggerTracks; }
  std::vector<TriggerTrack> const &trigger() const { return triggerTracks; }

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
	  constexpr auto vsemi = units::voltage::volt_t(1) / 12;
          bool const last = std::next(p) == cv.end();
          auto next = last? cv.begin(): std::next(p);
          analog[cvChannel].reset_to(offset, vsemi * p->second.value);
          interpolate<float>::signature *interp = nullptr;
          switch (p->second.interp) {
          case interpol::linear:
            interp = &interpolate<float>::linear;
            break;
          case interpol::cosine:
            interp = &interpolate<float>::cosine;
            break;
          case interpol::none:
            break;
          }
          if (next != p && p->second.value != next->second.value && interp) {
            size_t ticks = length(cv.begin(), cv.end(), p);
            analog[cvChannel].add_point(ticks * sps,
					vsemi * next->second.value, interp);
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
        if (triggerChannel >= digital.size()) {
          break;
        }
      }
    }
  }
  template<typename Archive> void serialize(Archive &archive, unsigned int) {
    archive & size;
    archive & triggerTracks;
    archive & cvTracks;
  }
};

void save(Song const &song, std::string const &filename);
void load(Song &song, std::string const &filename);

inline void dump(Song const &song) {
  std::cout << "Song {" << "\n" << "  size = " << song.length() << "\n";
  std::cout << "  CV size = " << song.cv().size() << "\n";
  for (auto const &cv : song.cv()) {
    std::cout << "  CV {" << "\n";
    for (auto const &pair: cv) {
      std::cout << "    [" << pair.first << "] = Note { " << int(pair.second.value) << " " << static_cast<int>(pair.second.interp) << "}" << "\n";
    }
    std::cout << "  }" << "\n";
  }
  std::cout << "}" << std::endl;
}

#endif // SONG_H_DEFINED
