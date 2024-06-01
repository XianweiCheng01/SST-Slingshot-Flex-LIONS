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

#ifndef SLINGSHOT_SWITCH_H
#define SLINGSHOT_SWITCH_H

//#include <sstmac/hardware/nic/nic.h>
//#include <sstmac/hardware/topology/structured_topology.h>

//#include <sstmac/hardware/network/network_message.h>
//#include <sstmac/hardware/node/node.h>
#include <sstmac/common/event_callback.h>

//#include <sstmac/common/event_handler.h>
//#include <sstmac/common/event_scheduler.h>
//#include <sstmac/common/rng.h>
//#include <sstmac/sst_core/integrated_component.h>
//#include <sstmac/hardware/common/flow_fwd.h>
//#include <sstmac/hardware/network/network_message_fwd.h>
//#include <sstmac/hardware/node/node_fwd.h>
//#include <sstmac/hardware/common/connection.h>
//#include <sstmac/hardware/interconnect/interconnect_fwd.h>
//#include <sstmac/hardware/topology/topology_fwd.h>
//#include <unordered_map>
//#include <sstmac/hardware/flex_lion/flex_port_fwd.h>

//#include <sstmac/hardware/flex_lion/flex_switch.h>
#include <sstmac/hardware/flex_lion/opt_switch.h>

#include <deque>
#include <map>

//#include <functional>
//#include <sprockit/factory.h>
//#include <sprockit/debug.h>

namespace sstmac {
namespace hw {

/**
 * @brief Implements a model for the FleX LION network
 */
//class FleXNIC : public NIC
class SlingshotSwitch : 
  public OptSwitch
{
 public:
//#if SSTMAC_INTEGRATED_SST_CORE
/*  SST_ELI_REGISTER_SUBCOMPONENT_DERIVED(
    FleXNIC,
    "macro",
    "flex_nic",
    SST_ELI_ELEMENT_VERSION(1,0,0),
    "implements a nic for AWGR / FleX LION",
    sstmac::hw::NIC)
*/

  SST_ELI_REGISTER_DERIVED_COMPONENT(
    OptSwitch,
    SlingshotSwitch,
    "macro",
    "slingshot_switch",
    SST_ELI_ELEMENT_VERSION(1,0,0),
    "A switch that implements a basic delay model with no congestion modeling",
    COMPONENT_CATEGORY_NETWORK)

  
  SST_ELI_DOCUMENT_PORTS(
    {"local_%(node_id)d_%(link_id)d", "port connect to local node", {}},
    {"port_%(layer)d_%(port)d_%(wave)d", "port and wave of inter-rack communication", {}},
    {"port_r_%(layer)d_%(port)d_%(wave)d", "reconfiguable port and wave of inter-rack communication", {}},
    {"ctrl_%(layer)d", "control port to communicate with optical switch", {}}
  )
  /**/
  /*
  SST_ELI_DOCUMENT_SUBCOMPONENT_SLOTS(
      {"local_%(node_id)d_%(link_id)d", "The electronic local port", "sstmac::SlingshotPort"},
      {"port_%(layer)d_%(port)d_%(wave)d", "The optical global port", "sstmac::SlingshotPort"},
      {"port_r_%(layer)d_%(port)d_%(wave)d", "The optical global port", "sstmac::SlingshotPort"},
      {"ctrl_%(layer)d", "The electronic control port", "sstmac::SlingshotPort"}
  )
  */
/*
#else
  SST_ELI_REGISTER_DERIVED(
    NIC,
    FleXNIC,
    "macro",
    "flex",
    SST_ELI_ELEMENT_VERSION(1,0,0),
    "implements a nic for AWGR / FleX LION")
#endif
*/
/**/
  SlingshotSwitch(uint32_t cid, SST::Params& params);

  ~SlingshotSwitch();

  std::string toString() const override {
    return "Slingshot switch";
  }

  //std::string switchName() override {
  //  return switch_name_;
  //}

  void input_port(int port_id, int wave_id, NetworkMessage* msg) override;

  void input_queue(int s_id, int inport, NetworkMessage* msg, bool outside_packet);
  void crossbar(int s_id, int outport, int outlink, int inport);
  void forward(int s_id, int outport, NetworkMessage* msg);

  void output_port(int layer_id, int port_id, int wave_id, int config_id, NetworkMessage* msg);

  int input_routing(int s_id, NetworkMessage* msg, bool &high_pri, bool outside_packet);

  int earliest_port(int s_id, int outlink);

  int proper_outlink(int s_id, int outport);

  bool check_inter_link(int s_id, int dst_id, int layer, bool larger, NetworkMessage* msg, bool &high_pri, bool outside_packet);

  int earliest_inter_port(int s_id, int dst_id, int layer, bool larger, NetworkMessage* msg, bool &high_pri, bool outside_packet);

  void input_queue_update(int s_id, int inport, int outlink, uint64_t num_bytes) {
    input_queues_byte_[s_id][outlink] -= num_bytes;
    input_queues_packet_[s_id][outlink]--;
  }

  //void sendManagerMsg(NetworkMessage* msg) override;

  //void inject_next_packet(int dst_addr, bool extra_wave);

  //void mtlHandle(Event* ev) override;

  //std::string toString() const override {
  //  return "FleX LION nic";
  //}

  void connectOutput(int  /*src_outport*/, int  /*dst_inport*/, EventLink::ptr&&  /*link*/) override {
    sprockit::abort("should never be called on Slingshot Switch");
  }

  void connectInput(int  /*src_outport*/, int  /*dst_inport*/, EventLink::ptr&&  /*link*/) override {
    sprockit::abort("should never be called on Slingshot Switch");
  }

  LinkHandler* creditHandler(int  /*port*/) override {
    sprockit::abort("should never be called on Slingshot Switch");
    return nullptr;
  }

  LinkHandler* payloadHandler(int  /*port*/) override {
    sprockit::abort("should never be called on Slingshot Switch");
    return nullptr;
  }

  //void updateRouting(int aid, std::string app_name) override;

  //bool clockTic(uint64_t/*Cycle_t*/) override;

/**/
 protected:
  /**
    Start the message sending and inject it into the network
    @param payload The network message to send
  */
/**/

  void init(unsigned int phase) override;

  void finish() override;

  //void inject_with_port(NetworkMessage* msg, int remap_id, TimeDelta delay);
/**/
 protected:

  int num_links_per_node_;
  int num_nodes_per_switch_;
  int num_links_per_switch_;
  int num_switches_;
  int num_nodes_;
  int num_local_links_;
  int num_ports_per_switch_;

  //int num_global_links_;
  int num_global_ports_;

  int num_links_layer_0_;
  int num_ports_per_link_layer_0_;
  int num_ports_layer_0_;

  int num_links_layer_1_;
  int num_ports_per_link_layer_1_;
  int num_ports_layer_1_;

  int num_links_layer_2_;
  int num_ports_per_link_layer_2_;
  int num_ports_layer_2_;

  int ToR_id_;

  int layer_0_idx_;
  int layer_1_idx_;
  int layer_2_idx_;

  int thr_inter_rack_;
  int thr_intra_rack_;

  bool adaptive_routing_;
  bool bandwidth_steering_;
  bool dynamic_steering_;

  TimeDelta inj_byte_delay_;
  TimeDelta forward_byte_delay_;

  TimeDelta inj_lat_;
  TimeDelta routing_lat_;

  //int flit_extra_lat_;

  TimeDelta inter_lat_;

  std::string switch_name_;

  std::vector<std::vector<OptPort*>> local_links_;

  std::vector<std::vector<OptPort*>> layer_0_links_;
  std::vector<std::vector<OptPort*>> layer_1_links_;
  std::vector<std::vector<OptPort*>> layer_2_links_;

  std::vector<std::vector<OptPort*>> layer_0_r_links_;
  std::vector<std::vector<OptPort*>> layer_1_r_links_;
  std::vector<std::vector<OptPort*>> layer_2_r_links_;

  std::vector<std::vector<int>> layer_0_config_;
  std::vector<std::vector<int>> layer_1_config_;
  std::vector<std::vector<int>> layer_2_config_;

  std::vector<std::vector<int>> layer_0_bw_steering_;
  std::vector<std::vector<int>> layer_1_bw_steering_;
  std::vector<std::vector<int>> layer_2_bw_steering_;

  std::vector<std::vector<int>> layer_0_shortest_routing_;
  std::vector<std::vector<int>> layer_1_shortest_routing_;
  std::vector<std::vector<int>> layer_2_shortest_routing_;

  int layer_0_ds_dst_;

  std::vector<std::vector<std::vector<std::deque<NetworkMessage*>>>> input_queues_;//switch, inport, outport
  std::vector<std::vector<long long>> input_queues_byte_;//switch, inport, outport
  std::vector<std::vector<long long>> input_queues_packet_;//switch, inport, outport
  std::vector<std::vector<long long>> inport_queues_byte_;//switch, inport, outport
  std::vector<std::vector<long long>> inport_queues_packet_;//switch, inport, outport

  std::vector<long long> global_queues_byte_;
  std::vector<long long> global_queues_packet_;

  std::vector<std::vector<Timestamp>> next_xbar_in_free_;//inport available
  std::vector<std::vector<Timestamp>> next_xbar_out_free_;//outport available
  std::vector<Timestamp> next_forward_layer_0_free_;//forward available
  std::vector<Timestamp> next_forward_layer_1_free_;//forward available
  std::vector<Timestamp> next_forward_layer_2_free_;//forward available
  std::vector<std::vector<int>> port_pri_;//outport priority port

  std::vector<long long> stat_inter_rack_byte_counting_;
  std::vector<long long> stat_inter_rack_packet_counting_;
  std::vector<long long> stat_inter_rack_lat_counting_;
  //std::vector<std::map<long long, long long>> stat_inter_rack_lat_mapping_;

  std::vector<std::vector<std::vector<long long>>> stat_inter_rack_link_byte_counting_;
  std::vector<std::vector<std::vector<long long>>> stat_inter_rack_link_packet_counting_;

  std::vector<std::vector<std::vector<long long>>> stat_switch_ports_byte_counting_;
  std::vector<std::vector<std::vector<long long>>> stat_switch_ports_packet_counting_;

  std::vector<int> layer_0_bw_steering_id_;

  std::vector<std::vector<double>> stat_switch_queue_byte_acc_;//time x queue byte size accumulation
  std::vector<std::vector<double>> stat_switch_queue_packet_acc_;//time x queue packet size accumulation
  std::vector<std::vector<long long>> stat_switch_queue_non_idle_acc_;//idle time for 
  //std::vector<double> stat_switch_out_idle_;//outport idle for no packets
  //std::vector<double> stat_switch_out_rt_;//outport idle for routing

  std::vector<std::vector<long long>> each_queue_last_time_;

  long long stat_ele_forward_counting_;
  long long stat_opt_forward_counting_;
};

}
} // end of namespace sstmac.

#endif // SIMPLE_NIC_H
