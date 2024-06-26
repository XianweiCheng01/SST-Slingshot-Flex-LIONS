/**
Copyright 2009-2022 National Technology and Engineering Solutions of Sandia,
LLC (NTESS).  Under the terms of Contract DE-NA-0003525, the U.S. Government
retains certain rights in this software.

Sandia National Laboratories is a multimission laboratory managed and operated
by National Technology and Engineering Solutions of Sandia, LLC., a wholly
owned subsidiary of Honeywell International, Inc., for the U.S. Department of
Energy's National Nuclear Security Administration under contract DE-NA0003525.

Copyright (c) 2009-2022, NTESS

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Questions? Contact sst-macro-help@sandia.gov
*/

#ifndef SERIALIZE_LIST_H
#define SERIALIZE_LIST_H

#include <list>
#include <deque>
#include <sprockit/serializer.h>

namespace sprockit {
namespace pvt {

template <class Container, class T>
void  serialize_container(Container& v, serializer& ser){
  typedef typename Container::iterator iterator;
  switch(ser.mode())
  {
  case serializer::SIZER: {
    size_t size = v.size();
    ser.size(size);
    iterator it, end = v.end();
    for (it=v.begin(); it != end; ++it){
      T& t = *it;
      serialize<T>()(t, ser);
    }
    break;
  }
  case serializer::PACK: {
    size_t size = v.size();
    ser.pack(size);
    iterator it, end = v.end();
    for (it=v.begin(); it != end; ++it){
      T& t = *it;
      serialize<T>()(t, ser);
    }
    break;
  }
  case serializer::UNPACK: {
    size_t size;
    ser.unpack(size);
    for (int i=0; i < size; ++i){
      T t;
      serialize<T>()(t, ser);
      v.push_back(t);
    }
    break;
  }
  }
}

}

template <class T>
class serialize <std::list<T> > {
 typedef std::list<T> List; 
public:
 void operator()(List& v, serializer& ser) {
   pvt::serialize_container<List,T>(v,ser);
 }
};

template <class T>
class serialize <std::deque<T> > {
 typedef std::deque<T> DQ; 
public:
 void operator()(DQ& v, serializer& ser) {
   pvt::serialize_container<DQ,T>(v,ser);
 }
};


}

#endif // SERIALIZE_LIST_H
