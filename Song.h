#ifndef SONG_H_DEFINED
#define SONG_H_DEFINED

#include "DSP.h"
#include "units.h"
#include <boost/operators.hpp>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <vector>

enum class interpol { none, linear, cosine, sine };

class midinote_t : boost::totally_ordered<midinote_t
                 , boost::incrementable<midinote_t>> {
  uint8_t value;
public:
  midinote_t() : value(0) {}
  midinote_t(uint8_t value) : value(value) {}
  midinote_t(midinote_t const &) = default;
  midinote_t &operator=(uint8_t value) { this->value = value; return *this; }
  midinote_t &operator=(midinote_t const &) = default;
  midinote_t &operator++() { ++value; return *this; }
  operator uint8_t() const noexcept { return value; }
  units::voltage::volt_t voct(
    units::voltage::volt_t volt = units::voltage::volt_t(1),
    size_t semitones = 12
  ) const {
    return volt / semitones * value;
  }
  bool operator<(midinote_t const &rhs) const {
    return this->value < rhs.value;
  }
  bool operator==(midinote_t const &rhs) const {
    return this->value == rhs.value;
  }
  template<typename Archive> void serialize(Archive &archive, const unsigned int) {
    archive & value;
  }
};

class Song {
  using CVTrack = std::map<size_t, std::tuple<midinote_t, interpol>>;
  using TriggerTrack = std::set<size_t>;
  std::vector<CVTrack> cvTracks;
  std::vector<TriggerTrack> triggerTracks;
  size_t size;
public:
  Song()
  : cvTracks(4), triggerTracks(4), size(64)
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
          auto const note = std::get<0>(p->second);
          auto const nextNote = std::get<0>(next->second);
          analog[cvChannel].reset_to(offset, note.voct());
          interpolate<float>::signature *interp = nullptr;
          switch (std::get<1>(p->second)) {
          case interpol::linear:
            interp = &interpolate<float>::linear;
            break;
          }
          if (next != p && note != nextNote && interp) {
            size_t ticks = length(cv.begin(), cv.end(), p);
            analog[cvChannel].add_point(ticks * sps, nextNote.voct(), interp);
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

#endif // SONG_H_DEFINED
