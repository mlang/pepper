#include "Song.h"
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/vector.hpp>
#include <fstream>
#include <iostream>

namespace boost { namespace serialization {

template<typename Archive>
void serialize(Archive &archive, ::Note &note, const unsigned int) {
  archive & note.value;
  archive & note.interp;
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
