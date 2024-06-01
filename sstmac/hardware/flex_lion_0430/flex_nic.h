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

#ifndef FLEX_NIC_H
#define FLEX_NIC_H

#include <sstmac/hardware/nic/nic.h>
#include <sstmac/hardware/topology/structured_topology.h>
//#include <sstmac/hardware/network/network_message.h>
//#include <sstmac/hardware/node/node.h>
//#include <sstmac/common/event_callback.h>

namespace sstmac {
namespace hw {

/**
 * @brief Implements a model for the FleX LION network
 */
class FleXNIC :
  public NIC
{
 public:
#if SSTMAC_INTEGRATED_SST_CORE
  SST_ELI_REGISTER_SUBCOMPONENT_DERIVED(
    FleXNIC,
    "macro",
    "flex_nic",
    SST_ELI_ELEMENT_VERSION(1,0,0),
    "implements a nic for AWGR / FleX LION",
    sstmac::hw::NIC)
#else
  SST_ELI_REGISTER_DERIVED(
    NIC,
    FleXNIC,
    "macro",
    "flex",
    SST_ELI_ELEMENT_VERSION(1,0,0),
    "implements a nic for AWGR / FleX LION")
#endif
/**/
  FleXNIC(uint32_t id, SST::Params& params, Node* parent);

  ~FleXNIC() override;

  void sendManagerMsg(NetworkMessage* msg) override;

  void inject_next_packet(int dst_addr, bool extra_wave);

  void mtlHandle(Event* ev) override;

  std::string toString() const override {
    return "FleX LION nic";
  }

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

  void updateRouting(int aid, std::string app_name) override;

  bool clockTic(uint64_t/*Cycle_t*/) override;

/**/
 protected:
  /**
    Start the message sending and inject it into the network
    @param payload The network message to send
  */
/**/
  int routing_port(int dst) {

    int tmp_addr = dst;
    int pod_idx = tmp_addr % num_pod_;
    if (pod_idx != pod_idx_) {
      return pod_idx;
    }

    int col_idx = 0;
    tmp_addr = tmp_addr / num_pod_;
    if (num_col_ >= 2) {
      col_idx = tmp_addr % num_col_;
      tmp_addr = tmp_addr / num_col_;
    }
    if (col_idx != col_idx_) {
      return col_idx + num_pod_;
    }

    int row_idx = 0;
    if (num_row_ >= 2) {
      row_idx = tmp_addr % num_row_;
    }
    //if (row_idx != row_idx_) {
    return row_idx + num_pod_ + num_col_;
    //}
  }

  void init(unsigned int phase) override;

  void doSend(NetworkMessage* msg) override;

  void inject_with_port(NetworkMessage* msg, int remap_id, TimeDelta delay);
/**/
 protected:
  TimeDelta inj_byte_delay_;

  TimeDelta inj_lat_;

  std::vector<Timestamp> next_out_free_;
  std::vector<Timestamp> extra_next_out_free_;

  Timestamp next_in_free_;

  int num_core_;
  int num_pod_;
  int num_col_;
  int num_row_;

  int used_node_;

  int pod_idx_;
  int col_idx_;
  int row_idx_;

  bool double_wave_;

  std::map<int, bool> app_nic_done_;

  std::vector<EventLink*> intra_pod_link_;
  std::vector<EventLink*> intra_col_link_;
  std::vector<EventLink*> intra_row_link_;

  std::vector<std::queue<NetworkMessage*>> d_inj_queue_;

  std::vector<long long int> stat_lat_packets_;
  std::vector<long long int> stat_num_packets_;

  std::vector<long long int> loc_dataflow_;
  std::vector<long long int> loc_packet_;

  std::vector<long long int> new_data_d_inj_;
  std::vector<long long int> new_data_r_inj_;
  std::vector<long long int> bp_data_inj_;

  std::vector<long long int> new_packet_d_inj_;
  std::vector<long long int> new_packet_r_inj_;
  std::vector<long long int> bp_packet_inj_;

  //std::vector<int> routing_table_; // topology?

  //routing algorithm
  std::vector<std::vector<int> > reconfigure_info_;

  //routing algorithm
  std::vector<std::vector<int> > configure_info_;

  //hardware configuration
  std::vector<int> remap_id_;

  //hardware configuration
  std::vector<std::vector<int> > waves_group_;

};

}
} // end of namespace sstmac.

#endif // SIMPLE_NIC_H
