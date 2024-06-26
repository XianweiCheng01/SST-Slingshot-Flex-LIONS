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

#ifndef SSTMAC_SOFTWARE_THREADING_STACKALLOC_H_INCLUDED
#define SSTMAC_SOFTWARE_THREADING_STACKALLOC_H_INCLUDED

#include <cstring>
#include <vector>
#include <sprockit/sim_parameters_fwd.h>

namespace sstmac {
namespace sw {

/**
 * A management type to handle dividing mmap-ed memory for use
 * as ucontext stack(s).  This is basically a very simple malloc
 * which allocates uniform-size chunks (with the NX bit unset)
 * and sets guard pages on each side of the allocated stacks.
 *
 * This allocator does not return memory to the system until it is
 * deleted, but regions can be allocated and free-d repeatedly.
 */
class StackAlloc
{
 public:
  class chunk;
  struct chunk_set {
    std::vector<chunk*> allocations;
    std::vector<void*> available;
    ~chunk_set(){
      clear();
    }
    void clear();
  };
 private:
  static chunk_set chunks_;
  /// Each chunk is of this suggested size.
  static size_t suggested_chunk_;
  /// Each stack request is of this size:
  static size_t stacksize_;
  /// Optionally added a protected stack between each stack we return
  static bool protect_stacks_;

 public:
  static size_t stacksize() {
    return stacksize_;
  }

  ~StackAlloc(){
    clear();
  }

  static size_t chunksize() {
    return suggested_chunk_;
  }

  static void init(SST::Params& params);

  static void* alloc();

  static void free(void*);

  static void clear();

};

}
} // end of namespace sstmac

#endif
