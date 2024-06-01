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

#ifndef FLEX_SWITCH_H
#define FLEX_SWITCH_H

#include <sstmac/common/event_handler.h>
#include <sstmac/common/event_scheduler.h>
#include <sstmac/common/rng.h>
#include <sstmac/sst_core/integrated_component.h>
#include <sstmac/hardware/common/flow_fwd.h>
#include <sstmac/hardware/network/network_message_fwd.h>
//#include <sstmac/hardware/node/node_fwd.h>
#include <sstmac/hardware/common/connection.h>
#include <sstmac/hardware/interconnect/interconnect_fwd.h>
#include <sstmac/hardware/topology/topology_fwd.h>
#include <unordered_map>
#include <sstmac/hardware/flex_lion/flex_port_fwd.h>

#include <sprockit/factory.h>
#include <sprockit/debug.h>

namespace sstmac {
namespace hw {

/**
 * @brief Implements a switch that does very basic congestion modeling
 *        using the LogGP model.  See "LogGP in Theory and Practice"
 *        by Hoefler and Schneider.
 */
class OptSwitch : public ConnectableComponent
{
 public:
  SST_ELI_DECLARE_BASE(OptSwitch)
  SST_ELI_DECLARE_DEFAULT_INFO()
  SST_ELI_DECLARE_CTOR(uint32_t, SST::Params&)

  //SST_ELI_DOCUMENT_SUBCOMPONENT_SLOTS(
  //    {"flex_port", "The optical port", "sstmac::OptPort"},
  //)
 protected:
  OptSwitch(uint32_t cid, SST::Params& params);

 public:
  ~OptSwitch();

  void setup() override {}

  void init(unsigned int phase) override {}

  std::string toString() const override {
    return "Optical switch";
  }

  virtual void sendEvent(int port, int wave, Event* ev) = 0;

  void connectOutput(int  /*src_outport*/, int  /*dst_inport*/, EventLink::ptr&&  /*link*/) override {
    sprockit::abort("should never be called on Merlin NIC");
  }

  void connectInput(int  /*src_outport*/, int  /*dst_inport*/, EventLink::ptr&&  /*link*/) override {
    sprockit::abort("should never be called on Merlin NIC");
  }

  LinkHandler* creditHandler(int  /*port*/) override {
    sprockit::abort("should never be called on Merlin NIC");
    return nullptr;
  }

  LinkHandler* payloadHandler(int  /*port*/) override {
    sprockit::abort("should never be called on Merlin NIC");
    return nullptr;
  }
};

class FleXSwitch :
  public OptSwitch
{

 public:
  SST_ELI_REGISTER_DERIVED_COMPONENT(
    OptSwitch,
    FleXSwitch,
    "macro",
    "flex_switch",
    SST_ELI_ELEMENT_VERSION(1,0,0),
    "A switch that implements a basic delay model with no congestion modeling",
    COMPONENT_CATEGORY_NETWORK)

  SST_ELI_DOCUMENT_PORTS(
    {"port_%(num_wave_)d_%(num_wave_)d", "port and wave of opt switch", {}}
  )

 public:
  FleXSwitch(uint32_t cid, SST::Params& params);

  ~FleXSwitch();

  void init(unsigned int phase) override;

  std::string toString() const override {
    return "FleX switch";
  }

  void sendEvent(int port, int wave, Event* ev) override;

  void connectOutput(int  /*src_outport*/, int  /*dst_inport*/, EventLink::ptr&&  /*link*/) override {
    sprockit::abort("should never be called on Merlin NIC");
  }

  void connectInput(int  /*src_outport*/, int  /*dst_inport*/, EventLink::ptr&&  /*link*/) override {
    sprockit::abort("should never be called on Merlin NIC");
  }

  LinkHandler* creditHandler(int  /*port*/) override {
    sprockit::abort("should never be called on Merlin NIC");
    return nullptr;
  }

  LinkHandler* payloadHandler(int  /*port*/) override {
    sprockit::abort("should never be called on Merlin NIC");
    return nullptr;
  }

 private:
  //int my_id_;
  //std::vector<std::vector<FleXPort*> > nic_ports_;
  std::vector<std::vector<OptPort*> > nic_ports_;
};



}
}


#endif // SIMPLE_SWITCH_H