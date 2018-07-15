#include "Song.h"
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/vector.hpp>
#include <iostream>

namespace boost { namespace serialization {

template<uint N>
struct serialize_tuple {
  template<class Archive, typename... Args>
  static void call(Archive & ar, std::tuple<Args...> & t, const unsigned int version) {
    serialize_tuple<N-1>::call(ar, t, version);
    ar & std::get<N-1>(t);
  }
};

template<>
struct serialize_tuple<0> {
  template<class Archive, typename... Args>
  static void call(Archive &, std::tuple<Args...> &, const unsigned int /*version*/) {
  }
};
    
template<class Archive, typename... Args>
void serialize(Archive &ar, std::tuple<Args...> &t, const unsigned int version) {
  serialize_tuple<sizeof...(Args)>::call(ar, t, version);
}
    
template<typename Archive>
void serialize(Archive &archive, ::Song &song, unsigned int /*version*/) {
  archive & song.cvTracks;
  archive & song.triggerTracks;
  archive & song.size;
}

}}

void save(Song const &song, std::string const &filename) {
  std::ofstream ofs(filename);
  boost::archive::text_oarchive oa(ofs);
  oa << song;
}

void load(Song &song, std::string const &filename) {
  std::ifstream ifs(filename);
  if (ifs.good()) {
    boost::archive::text_iarchive ia(ifs);
    ia >> song;
  } else {
    std::cout << filename << " does not exist, saving..." << std::endl;
    save(song, filename);
  }
}
