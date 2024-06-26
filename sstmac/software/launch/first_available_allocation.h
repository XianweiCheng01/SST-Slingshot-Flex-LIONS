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

#ifndef FIRSTAVAILABLEALLOCATION_H
#define FIRSTAVAILABLEALLOCATION_H

#include <sstmac/software/launch/node_allocator.h>

namespace sstmac {
namespace sw {

class FirstAvailableAllocation : public NodeAllocator
{
 public:
  SST_ELI_REGISTER_DERIVED(
    NodeAllocator,
    FirstAvailableAllocation,
    "macro",
    "first_available",
    SST_ELI_ELEMENT_VERSION(1,0,0),
    "Allocate the first set of nodes from the list of available nodes."
    "In most cases, allocating from the available node list will give "
    "you a regular, contiguous allocation")

  FirstAvailableAllocation(SST::Params& params) :
    NodeAllocator(params)
  {
  }

  std::string toString() const override {
    return "first available allocator";
  }

  ~FirstAvailableAllocation() throw () override;

  bool allocate(
    int nnode_requested,
    const ordered_node_set& available,
    ordered_node_set& allocation) const override;
  
  bool allocate(int nnode_requested,
    const std::vector<int>& available_proc,
    const std::vector<int>& available_mem,
    const std::vector<int>& available_storage,
   int required_proc,
   int required_mem,
   int required_storage,
   std::vector<NodeId>& n_allocation,
    ordered_node_set& allocation) const override {}/**/

};


}
} // end of namespace sstmac


#endif // FIRSTAVAILABLEALLOCATION_H
