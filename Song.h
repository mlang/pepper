#ifndef SONG_H_DEFINED
#define SONG_H_DEFINED

#include <map>
#include <set>
#include <string>
#include <vector>
#include <iostream>

enum class interpol { none, linear, cosine, sine };

struct Note {
  int value;
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

  template<typename Archive> void serialize(Archive &archive, unsigned int) {
    archive & size;
    archive & triggerTracks;
    archive & cvTracks;
  }
};

void save(Song const &song, std::string const &filename);
void load(Song &song, std::string const &filename);

#endif // SONG_H_DEFINED
