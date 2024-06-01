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
#include <sstmac/hardware/flex_lion/double_switch.h>
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
#include <sstmac/hardware/flex_lion/slingshot_port.h>

//MakeDebugSlot(logp)
RegisterKeywords(
{ "num_waves", "Total number of waves" },
);

namespace sstmac {
namespace hw {


/*OptSwitch::OptSwitch(uint32_t cid, SST::Params& params) :
  ConnectableComponent(cid, params)
{
}

OptSwitch::~OptSwitch()
{
}*/

DoubleSwitch::DoubleSwitch(uint32_t cid, SST::Params& params) :
  OptSwitch(cid, params)
{
  //std::cout << "Test 0 in FleXSwitch with cid: " << cid << std::endl;
  fflush(stdout);

  //int my_id_ = params.find<int>("id", 0);
  num_wave_ = params.find<int>("num_waves", 16);

  switch_name_ = params.find<std::string>("switch_name");

  AWGR_ports_.resize(num_wave_);
  RCS_ports_.resize(num_wave_);
  params.find_array("reconfig_map", reconfig_map_);
  //reconfig_map_.resize(num_wave_);

  for (int i=0; i<num_wave_; i++)
  {
    AWGR_ports_[i].resize(num_wave_-1);
    RCS_ports_[i].resize(num_wave_-1);
    //reconfig_map_[i] = (i%2==0)?(i+1):(i-1);
    //int idx_j = 0;
    for (int j=0; j<num_wave_-1; j++) {
      //if (i == j) {
      //  continue;
      //}
      //nic_ports_[i][j] = sprockit::create<FleXPort>("macro", "flex_port", i, j, this);
      SST::Params port_params = params.get_scoped_params("double_port");
      //nic_ports_[i][j] = sprockit::create<OptPort>("macro", "flex_port", i, j, this);
      //nic_ports_[i][j] = loadSub<OptPort>("flex_port", "port", i, j, this);
      port_params.insert("port_id", std::to_string(i));
      port_params.insert("wave_id", std::to_string(j));
      port_params.insert("port_name", switch_name_+sprockit::sprintf("port_%d_%d",i,j));
      port_params.insert("real_port_name", sprockit::sprintf("port_%d_%d",i,j));
      //std::cout << "Test 1 in FleXSwitch with cid: " << cid << " " << i << " " << j << std::endl;
      fflush(stdout);

      SST::Params port_r_params = params.get_scoped_params("double_port");
      port_r_params.insert("port_id", std::to_string(i+num_wave_));
      port_r_params.insert("wave_id", std::to_string(j));
      port_r_params.insert("port_name", switch_name_+sprockit::sprintf("port_r_%d_%d",i,j));
      port_r_params.insert("real_port_name", sprockit::sprintf("port_r_%d_%d",i,j));
      //nic_ports_[i][j] = loadSub<OptPort>("flex_port", "port", i*num_wave+j, port_params, this);
      //nic_ports_[i][j] = loadAnonymousSubComponent<OptPort>("flex_port", "port", i*num_wave+j, ComponentInfo::SHARE_PORTS | ComponentInfo::SHARE_STATS | ComponentInfo::INSERT_STATS,
      //nic_ports_[i][j] = loadAnonymousSubComponent<OptPort>("flex_port", "port", i*num_wave+j, 0x7,
      AWGR_ports_[i][j] = loadAnonymousSubComponent<OptPort>("macro.slingshot_port", "AWGRPort", i*num_wave_+j, 0x7,
        port_params, this);
      RCS_ports_[i][j] = loadAnonymousSubComponent<OptPort>("macro.slingshot_port", "RCSPort", i*num_wave_+j, 0x7,
        port_r_params, this);
      //idx_j++;
    }
  }


}

DoubleSwitch::~DoubleSwitch()
{
  // JJW 4/10/19 these are now owned by the interconnect
  //for (auto* link : nic_links_){
  //  delete link;
  //}
}

void
DoubleSwitch::input_port(int port, int wave, NetworkMessage *msg)
{
  int real_port = port%num_wave_;
  if (port < num_wave_) {
    int real_wave = (real_port > wave)?wave:(wave+1);
    real_port = real_port - ((real_port > wave)?1:0);
    //std::cout << "Test 0 in input_port of double_switch.cc real_port " << real_port << " real_wave " << real_wave << std::endl;
    AWGR_ports_[real_wave][real_port]->locSend(msg);
  } else {
    int dst_port = reconfig_map_[real_port];
    //std::cout << "Test 1 in input_port of double_switch.cc real_port " << real_port << " dst_port " << dst_port << " wave " << wave << std::endl;
    RCS_ports_[dst_port][wave]->locSend(msg);
  }
  //int real_wave = (real_port > wave)?wave:(wave+1) + (port < num_wave_)?0:num_wave_;
}

void DoubleSwitch::init(unsigned int phase)
{
}

}
}

#endif // simple_switch_CC
