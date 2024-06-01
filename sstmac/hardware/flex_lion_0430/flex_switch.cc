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

#ifndef flex_switch_CC
#define flex_switch_CC

#include <sstmac/hardware/switch/network_switch.h>
#include <sstmac/hardware/flex_lion/flex_switch.h>
#include <sstmac/hardware/node/node.h>
#include <sstmac/hardware/topology/topology.h>
#include <sstmac/hardware/interconnect/interconnect.h>
#include <sstmac/hardware/nic/nic.h>
#include <sstmac/common/event_manager.h>
#include <sprockit/util.h>
#include <sprockit/sim_parameters.h>
#include <sprockit/keyword_registration.h>
#include <sstmac/software/launch/launch_event.h>
#include <random>
#include <sstmac/hardware/flex_lion/flex_port.h>

//MakeDebugSlot(logp)
RegisterKeywords(
{ "num_waves", "Total number of waves" },
);

namespace sstmac {
namespace hw {


OptSwitch::OptSwitch(uint32_t cid, SST::Params& params) :
  ConnectableComponent(cid, params)
{
}

OptSwitch::~OptSwitch()
{
}

FleXSwitch::FleXSwitch(uint32_t cid, SST::Params& params) :
  OptSwitch(cid, params)
{
  //std::cout << "Test 0 in FleXSwitch with cid: " << cid << std::endl;
  fflush(stdout);

  SST::Params topParams;

  //int my_id_ = params.find<int>("id", 0);
  int num_wave = params.find<int>("num_waves", 16);

  nic_ports_.resize(num_wave);
  for (int i=0; i<num_wave; i++)
  {
    nic_ports_[i].resize(num_wave);
    for (int j=0; j<num_wave; j++) {
      //nic_ports_[i][j] = sprockit::create<FleXPort>("macro", "flex_port", i, j, this);
      SST::Params port_params = params.get_scoped_params("flex_port");
      //nic_ports_[i][j] = sprockit::create<OptPort>("macro", "flex_port", i, j, this);
      //nic_ports_[i][j] = loadSub<OptPort>("flex_port", "port", i, j, this);
      port_params.insert("port_id", std::to_string(i));
      port_params.insert("wave_id", std::to_string(j));
      port_params.insert("port_name", sprockit::sprintf("port.%d.%d",i,j));
      //std::cout << "Test 1 in FleXSwitch with cid: " << cid << " " << i << " " << j << std::endl;
      fflush(stdout);
      //nic_ports_[i][j] = loadSub<OptPort>("flex_port", "port", i*num_wave+j, port_params, this);
      //nic_ports_[i][j] = loadAnonymousSubComponent<OptPort>("flex_port", "port", i*num_wave+j, ComponentInfo::SHARE_PORTS | ComponentInfo::SHARE_STATS | ComponentInfo::INSERT_STATS,
      //nic_ports_[i][j] = loadAnonymousSubComponent<OptPort>("flex_port", "port", i*num_wave+j, 0x7,
      nic_ports_[i][j] = loadAnonymousSubComponent<OptPort>("macro.flex_port", "FleXPort", i*num_wave+j, 0x7,
        port_params, this);
    }
  }


}

FleXSwitch::~FleXSwitch()
{
  // JJW 4/10/19 these are now owned by the interconnect
  //for (auto* link : nic_links_){
  //  delete link;
  //}
}

void
FleXSwitch::sendEvent(int port, int wave, Event *ev)
{
  NicEvent* nev = dynamic_cast<NicEvent*>(ev);
  NetworkMessage* msg = nev->msg();
  delete nev;
  //std::cout << "Test 0 in FleXSwitch::sendEvent with port_id: " << port << ", wave_id: " << wave << ", msg: " << msg << ", at " << now().nsec() << std::endl;
  nic_ports_[wave][port]->locSend(msg);
}

void FleXSwitch::init(unsigned int phase)
{
/**
  int wave_size = nic_ports_.size();
  if (phase < 2) {
    for (int i=0; i<wave_size; i++) {
      for (int j=0; j<wave_size; j++) {
        std::cout << "Test 0 in init of flex_switch.cc i: " << i << ", j: " << j << ", phase: " << phase << std::endl;
        fflush(stdout);
        nic_ports_[i][j]->doInit(phase);
      }
    }
  }
/**/
}

}
}

#endif // simple_switch_CC
