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

#include <sstmac/hardware/flex_lion/flex_nic.h>
//#include <sstmac/hardware/logp/logp_switch.h>
#include <sstmac/hardware/network/network_message.h>
#include <sstmac/hardware/node/node.h>
#include <sstmac/software/process/operating_system.h>
#include <sstmac/common/event_callback.h>
#include <sprockit/util.h>
#include <sprockit/sim_parameters.h>
#include <sstmac/hardware/interconnect/interconnect.h>
#include <sprockit/keyword_registration.h>

using namespace SST;

RegisterKeywords(
{ "num_ports", "Total number of ports" },
{ "num_pod", "number of ports used for intra-pod connection" },
{ "num_col", "number of ports used for inter-pod connection column" },
{ "num_row", "number of ports used for inter-pod connection row" },
{ "num_core", "number of cors in each rack" },
{ "double_wave", "decide weather the number of wavelength double or not" },
{ "used_node", "number of nodes in using" },
);

namespace sstmac {
namespace hw {

//num_ports, num_pod, num_col, num_row (the name for each port)
//bandwitdth / next_free_time for each port
//sendExecutionEvent(next_free_time, schedule_next_packet, port_id);
//injection queues for each dest/ routing algorithm will decide the port
//when reconfig execute the schedule_next_packet?

FleXNIC::FleXNIC(uint32_t id, SST::Params& params, Node* node) :
  NIC(id, params, node)
{
  SST::Params inj_params = params.get_scoped_params("injection");
  inj_byte_delay_ = TimeDelta(inj_params.find<SST::UnitAlgebra>("bandwidth").getValue().inverse().toDouble());
  inj_lat_ = TimeDelta(inj_params.find<SST::UnitAlgebra>("latency").getValue().toDouble());

  int num_ports = params.find<int>("num_ports", 16);
  next_out_free_.resize(num_ports);

  used_node_ = params.find<int>("used_node", 16);
  double_wave_ = params.find<bool>("double_wave", false);
  if (double_wave_) {
    //std::cout << "Test 0 in FleXNIC with port: " << num_ports << std::endl;
    extra_next_out_free_.resize(num_ports);
  }
  d_inj_queue_.resize(num_ports);

  new_data_d_inj_.resize(num_ports, 0);
  new_data_r_inj_.resize(num_ports, 0);
  bp_data_inj_.resize(num_ports, 0);

  new_packet_d_inj_.resize(num_ports, 0);
  new_packet_r_inj_.resize(num_ports, 0);
  bp_packet_inj_.resize(num_ports, 0);

  stat_lat_packets_.resize(num_ports, 0);
  stat_num_packets_.resize(num_ports, 0);

  reconfigure_info_.resize(num_ports);
  configure_info_.resize(num_ports);
  remap_id_.resize(num_ports);
  waves_group_.resize(num_ports);

  for (int i=0; i<num_ports; i++) {
    reconfigure_info_[i].push_back(i);
    //configure_info_[i].push_back(i);
    remap_id_[i] = i;
    waves_group_[i].push_back(i);
  }

  /*port_configuration_.resize(num_ports);
  for (int i=0; i<num_ports; i++) {
    port_configuration_[i] = (i == my_addr_)?0:1;
  }*/

  num_core_ = params.find<int>("num_core", 16);
  //next_in_free_.resize(num_ports);

  num_pod_ = params.find<int>("pod", 16);
  intra_pod_link_.resize(num_pod_);
  for (int i=0; i<num_pod_; i++) {
    std::string name = sprockit::sprintf("pod_port_%d", i);
    intra_pod_link_[i] = new EventLink(name, TimeDelta(), configureLink(name, SharedBaseComponent::timeConverter(), newLinkHandler(this, &FleXNIC::mtlHandle)));
  }

  num_col_ = params.find<int>("col", 0);
  intra_col_link_.resize(num_col_);
  for (int i=0; i<num_col_; i++) {
    std::string name = sprockit::sprintf("col_port_%d", i);
    intra_col_link_[i] = new EventLink(name, TimeDelta(), configureLink(name, SharedBaseComponent::timeConverter(), newLinkHandler(this, &FleXNIC::mtlHandle)));
  }

  num_row_ = params.find<int>("row", 0);
  intra_row_link_.resize(num_row_);
  for (int i=0; i<num_row_; i++) {
    std::string name = sprockit::sprintf("row_port_%d", i);
    intra_row_link_[i] = new EventLink(name, TimeDelta(), configureLink(name, SharedBaseComponent::timeConverter(), newLinkHandler(this, &FleXNIC::mtlHandle)));
  }

  int total_ranks = ((num_pod_==0)?1:num_pod_)*((num_col_==0)?1:num_col_)*((num_row_==0)?1:num_row_);
  loc_dataflow_.resize(total_ranks, 0);
  loc_packet_.resize(total_ranks, 0);

  int tmp_addr = my_addr_;
  pod_idx_ = tmp_addr % num_pod_;
  tmp_addr = tmp_addr / num_pod_;
  if (num_col_ < 2) {
    col_idx_ = 0;
  } else {
    col_idx_ = tmp_addr % num_col_;
    tmp_addr = tmp_addr / num_col_;
  }
  if (num_row_ < 2) {
    row_idx_ = 0;
  } else {
    row_idx_ = tmp_addr % num_row_;
  }

  //configureLogPLinks();
}

FleXNIC::~FleXNIC()
{
}

void FleXNIC::updateRouting(int aid, std::string app_name)
{
#if 1
  /*std::ifstream dm_file;
  dm_file.open(app_name);
  dm_file.close();*/
  if (app_nic_done_.find(aid) == app_nic_done_.end()) {
    top_->updateRouting(aid, app_name, reconfigure_info_, configure_info_, remap_id_, waves_group_, my_addr_);
    app_nic_done_[aid] = true;

  /**/
  std::cout << "Node id: " << my_addr_ << std::endl;
  std::cout << "reconfigure_info_: " << std::endl;
  for (int i=0; i<reconfigure_info_.size(); i++) {
    std::cout << i << " :";
    for (int j=0; j<reconfigure_info_[i].size(); j++) {
      std::cout << " " << reconfigure_info_[i][j];
    }
    std::cout << std::endl;
    if (reconfigure_info_[i].size() == 0 && my_addr_ != i) {
      std::cout << "Warning in clockTic of flex_lion/flex_nic.cc" << std::endl;
    }
  }
  std::cout << "configure_info_: " << std::endl;
  for (int i=0; i<configure_info_.size(); i++) {
    std::cout << i << " :";
    for (int j=0; j<configure_info_[i].size(); j++) {
      std::cout << " " << configure_info_[i][j];
    }
    std::cout << std::endl;
  }
  std::cout << "waves_group_: " << std::endl;
  for (int i=0; i<waves_group_.size(); i++) {
    std::cout << i << " :";
    for (int j=0; j<waves_group_[i].size(); j++) {
      std::cout << " " << waves_group_[i][j];
    }
    std::cout << std::endl;
  }
  std::cout << "remap_id_: " << std::endl;
  for (int i=0; i<remap_id_.size(); i++) {
    std::cout << " " << remap_id_[i];
  }
  std::cout << std::endl;
  fflush(stdout);
  /**/
  }
#endif
}

bool FleXNIC::clockTic(uint64_t/*Cycle_t*/)
{
#if 1
  for (int i=0; i<loc_dataflow_.size(); i++)
  {
    //top_->update_dataflow(my_addr_, i, loc_dataflow_[i], loc_packet_[i]);
    loc_dataflow_[i] = 0;
    loc_packet_[i] = 0;
  }
  //std::cout << "FlexNIC Time: " << now().nsecRounded() << " ";
  top_->current_topo(reconfigure_info_, configure_info_, remap_id_, waves_group_, my_addr_);

  parent()->printCount();

/**/
  std::cout << "Inject_data_new_dir " << my_addr_;
  for (int i=0; i<new_data_d_inj_.size(); i++)
  {
    std::cout << " " << new_data_d_inj_[i];
    //new_data_d_inj_[i] = 0;
  }
  std::cout << std::endl;

  std::cout << "Inject_data_new_re " << my_addr_;
  for (int i=0; i<new_data_r_inj_.size(); i++)
  {
    std::cout << " " << new_data_r_inj_[i];
    //new_data_r_inj_[i] = 0;
  }
  std::cout << std::endl;

  std::cout << "Inject_data_bypass " << my_addr_;
  for (int i=0; i<bp_data_inj_.size(); i++)
  {
    std::cout << " " << bp_data_inj_[i];
    //bp_data_inj_[i] = 0;
  }
  std::cout << std::endl;

  std::cout << "Inject_packet_new_dir " << my_addr_;
  for (int i=0; i<new_packet_d_inj_.size(); i++)
  {
    std::cout << " " << new_packet_d_inj_[i];
    //new_packet_d_inj_[i] = 0;
  }
  std::cout << std::endl;

  std::cout << "Inject_packet_new_re " << my_addr_;
  for (int i=0; i<new_packet_r_inj_.size(); i++)
  {
    std::cout << " " << new_packet_r_inj_[i];
    //new_packet_r_inj_[i] = 0;
  }
  std::cout << std::endl;

  std::cout << "Inject_packet_bypass " << my_addr_;
  for (int i=0; i<bp_packet_inj_.size(); i++)
  {
    std::cout << " " << bp_packet_inj_[i];
    //bp_packet_inj_[i] = 0;
  }
  std::cout << std::endl;

  std::cout << "total_packet_latency " << my_addr_;
  for (int i=0; i<stat_lat_packets_.size(); i++)
  {
    std::cout << " " << stat_lat_packets_[i];
    //stat_lat_packets_[i] = 0;
  }
  std::cout << std::endl;

  std::cout << "total_packet_number " << my_addr_;
  for (int i=0; i<stat_num_packets_.size(); i++)
  {
    std::cout << " " << stat_num_packets_[i];
    //stat_num_packets_[i] = 0;
  }
  std::cout << std::endl;
/**/

  /**
  std::cout << "Node id: " << my_addr_ << std::endl;
  std::cout << "reconfigure_info_: " << std::endl;
  for (int i=0; i<reconfigure_info_.size(); i++) {
    std::cout << i << " :";
    for (int j=0; j<reconfigure_info_[i].size(); j++) {
      std::cout << " " << reconfigure_info_[i][j];
    }
    std::cout << std::endl;
    if (reconfigure_info_[i].size() == 0 && my_addr_ != i) {
      std::cout << "Warning in clockTic of flex_lion/flex_nic.cc" << std::endl;
    }
  }
  std::cout << "configure_info_: " << std::endl;
  for (int i=0; i<configure_info_.size(); i++) {
    std::cout << i << " :";
    for (int j=0; j<configure_info_[i].size(); j++) {
      std::cout << " " << configure_info_[i][j];
    }
    std::cout << std::endl;
  }
  std::cout << "waves_group_: " << std::endl;
  for (int i=0; i<waves_group_.size(); i++) {
    std::cout << i << " :";
    for (int j=0; j<waves_group_[i].size(); j++) {
      std::cout << " " << waves_group_[i][j];
    }
    std::cout << std::endl;
  }
  std::cout << "remap_id_: " << std::endl;
  for (int i=0; i<remap_id_.size(); i++) {
    std::cout << " " << remap_id_[i];
  }
  std::cout << std::endl;
  /**/
  //std::cout << std::endl;
  //update the reconfigure_info_ and remap_id_ (routing table)
#endif
  return false;
}

void FleXNIC::init(unsigned int phase)
{
/**
  switch (phase) {
    case 0:
    {
        for (int i=0; i<num_pod_; i++) {
          Event* ev = new NicEvent();
          intra_pod_link_[i]->sendInitData(ev);
        }
        for (int i=0; i<num_col_; i++) {
          Event* ev = new NicEvent();
          intra_col_link_[i]->sendInitData(ev);
        }
        for (int i=0; i<num_row_; i++) {
          Event* ev = new NicEvent();
          intra_row_link_[i]->sendInitData(ev);
        }
        break;
    }
    case 1:
    {
        for (int i=0; i<num_pod_; i++) {
          Event* ev = intra_pod_link_[i]->recvInitData();
	  delete ev;
        }
        for (int i=0; i<num_col_; i++) {
          Event* ev = intra_col_link_[i]->recvInitData();
	  delete ev;
        }
        for (int i=0; i<num_row_; i++) {
          Event* ev = intra_row_link_[i]->recvInitData();
          delete ev;
        }
        break;
    }
    default:
    {
        break;
    }
  }
/**/
}

/**/
void
FleXNIC::mtlHandle(Event *ev)
{
  Timestamp now_ = now();
  NicEvent* nev = static_cast<NicEvent*>(ev);
  NetworkMessage* msg = nev->msg();
  delete nev;
    fflush(stdout);
  if (msg->toaddr() != my_addr_) {
    //might influence the performance
    //improve the priority of reroute packet?
    //std::cout << "Test 0 in FleXNIC::mtlHandle of flex_nic.cc addr: " << msg->toaddr() << ", my_addr: " << my_addr_ << " " << msg << " start: " << msg->injectionStarted().nsec() << " now: " << now_.nsec() << std::endl;
    doSend(msg);
    return;
  }

  stat_lat_packets_[msg->fromaddr()] += (now_ - msg->injectionStarted()).nsec();
  stat_num_packets_[msg->fromaddr()] += 1;
    //std::cout << "Test 0 in FleXNIC::mtlHandle of flex_nic.cc addr: " << msg->toaddr() << ", my_addr: " << my_addr_ << " " << msg << " start: " << msg->injectionStarted().nsec() << " now: " << now_.nsec() << std::endl;

  if (msg->byteLength() < negligibleSize_){
    recvMessage(msg);
  } else {
    TimeDelta time_to_recv = inj_byte_delay_*msg->byteLength()/num_core_;
    Timestamp recv_start = now_ - time_to_recv;
    if (recv_start > next_in_free_){
      next_in_free_ = now_;
      recvMessage(msg);
    } else {
      next_in_free_ += time_to_recv;
      sendExecutionEvent(next_in_free_, newCallback(this, &NIC::recvMessage, msg));
    }
  }
}

void
FleXNIC::sendManagerMsg(NetworkMessage* msg)
{
    if (msg->toaddr() == my_addr_){
      //std::cout << "Test 0 in sendManagerMsg: " << my_addr_ << " to: " << msg->toaddr() << std::endl;
      intranodeSend(msg);
    } else {
      //std::cout << "Test 1 in sendManagerMsg: " << my_addr_ << " to: " << msg->toaddr() << std::endl;
      msg->setInjectionStarted(now());
      msg->setExtraHops(extra_hops_);
      doSend(msg);
    }
}

//////decouple the queue and wavelength!!!!!
//////dst_addr->routing->queue(ori_wavelengh)->reconfig_routing->target_ports(available_wavelength)
//////available_wavelength->target_ports->potential_queues(ori_wavelength)

void
FleXNIC::doSend(NetworkMessage* msg)
{
  int dst_addr = msg->toaddr();
  if (msg->doExtraHop()) {
    msg->reduceHops();
    //dst_addr = (dst_addr + 1)%used_node_;
      if ((dst_addr+1)%used_node_ == my_addr_) {
        dst_addr = (dst_addr + used_node_ - 1)%used_node_;
      } else {
        dst_addr = (dst_addr + 1)%used_node_;
      }
  }
  int dst_port = routing_port(dst_addr);

  //vector redirect_port = reconfigure_info_[dst_port]
  // port_group[]

  //if (my_addr_ == msg->fromaddr() && dst_addr == msg->toaddr()) {
  //  std::cout << "There might be something wrong " << my_addr_ << " " << dst_addr << " " << msg << std::endl;
  //}

  Timestamp now_ = now();
  d_inj_queue_[dst_port].push(msg);

  if (my_addr_ == msg->fromaddr()) {//////injection time?
    top_->update_dataflow(my_addr_, msg->toaddr(), msg->byteLength());
    loc_dataflow_[msg->toaddr()] += msg->byteLength();
    loc_packet_[msg->toaddr()]++;
  }

  std::vector<int> additional_ports = reconfigure_info_[dst_port];//ori_wavelength --> target_ports

  /**
  Timestamp earliest_time = now();
  int earliest_port = -1;
  bool extra_wave = false;
  /**/

#if 1
  //int target_port = -1;
  int total_ports = additional_ports.size();
  int start_i = 0;

  if (total_ports > 0) {
    start_i = random()%total_ports;
  }

  for (int idx_i=0; idx_i < total_ports; idx_i++)
  {
    //if (additional_ports[(start_i+idx_i)%total_ports] == msg->fromaddr()) {
    //  continue;
    //}
    //std::vector<int> avail_waves = waves_group_[target_port];//available_wavelengthes of target_ports
    std::vector<int> avail_waves = waves_group_[additional_ports[(start_i+idx_i)%total_ports]];//available_wavelengthes of target_ports
    int total_waves = avail_waves.size();
    //int start_j = 0;
    int start_j = random()%total_waves;
    //if (total_waves > 0) {
    //  start_j = random()%total_waves;
    //}
    int avail_wave = -1;
    for (int idx_j=0; idx_j < total_waves; idx_j++)
    {
      avail_wave = avail_waves[(start_j+idx_j)%total_waves];
      /**/
      if (now_ >= next_out_free_[avail_wave]) {
        inject_next_packet(avail_wave, false);
        return;
      }
      /**/
        /**
        if (earliest_time >= next_out_free_[avail_wave]) {
            earliest_port = avail_wave;
            earliest_time = next_out_free_[avail_wave];
            extra_wave = false;
        }
        /**/
      if (double_wave_) {
        /**/
        if (now_ >= extra_next_out_free_[avail_wave]) {
          inject_next_packet(avail_wave, true);
          return;
        }
        /**/
          /**
	  if (earliest_time >= extra_next_out_free_[avail_wave]) {
              earliest_port = avail_wave;
              earliest_time = extra_next_out_free_[avail_wave];
              extra_wave = true;
          }
	  /**/
      }
    }
  }
#endif

#if 0
  for (int target_port : additional_ports)
  {
    if (target_port == msg->fromaddr()) {
      continue;
    }
    std::vector<int> avail_waves = waves_group_[target_port];//available_wavelengthes of target_ports
    //std::vector<int> avail_waves = waves_group_[additional_ports[(start_i+idx_i)%total_ports]];//available_wavelengthes of target_ports
    //int total_waves = avail_waves.size();
    //int start_j = 0;
    //int start_j = random()%total_waves;
    //if (total_waves > 0) {
    //  start_j = random()%total_waves;
    //}
    //int avail_wave = -1;
    for (int avail_wave : avail_waves)
    {
      //avail_wave = avail_waves[(start_j+idx_j)%total_waves];
      /**/
      if (now_ >= next_out_free_[avail_wave]) {
        inject_next_packet(avail_wave, false);
        return;
      }
      /**/
        /**
        if (earliest_time >= next_out_free_[avail_wave]) {
            earliest_port = avail_wave;
            earliest_time = next_out_free_[avail_wave];
            extra_wave = false;
        }
        /**/
      if (double_wave_) {
        /**/
        if (now_ >= extra_next_out_free_[avail_wave]) {
          inject_next_packet(avail_wave, true);
          return;
        }
        /**/
          /**
	  if (earliest_time >= extra_next_out_free_[avail_wave]) {
              earliest_port = avail_wave;
              earliest_time = extra_next_out_free_[avail_wave];
              extra_wave = true;
          }
	  /**/
      }
    }
  }
#endif
  /**
  if (earliest_port != -1) {
      inject_next_packet(earliest_port, extra_wave);
  }
  /**/
}

  void FleXNIC::inject_with_port(NetworkMessage* msg, int remap_id, TimeDelta delay) {

    if (msg->needsAck() && !msg->hasAcked()){
      msg->setHasAcked(true);
      NetworkMessage* acker = msg->cloneInjectionAck();
    //std::cout << "Test 0 in inject_with_port m: " << msg << ", node: " << my_addr_ << ", src: "<< msg->fromaddr() << ", dst: " << msg->toaddr() << ", port: " << remap_id << " " << acker << std::endl;
    fflush(stdout);
      auto ack_ev = newCallback(parent_, &Node::handle, acker);
      parent_->sendExecutionEvent(now(), ack_ev);
    }

    //std::cout << "Test in inject_with_port m: " << msg << ", node: " << my_addr_ << ", src: "<< msg->fromaddr() << ", dst: " << msg->toaddr() << ", port: " << remap_id << ", time: " << now().nsec() << std::endl;

    //TimeDelta delay = now() - now();
    if (remap_id < num_pod_) {
      intra_pod_link_[remap_id]->send(delay, new NicEvent(msg));
    } else if (remap_id < (num_pod_ + num_col_)) {
      intra_col_link_[remap_id - num_pod_]->send(delay, new NicEvent(msg));
    } else {
      intra_row_link_[remap_id - num_pod_ - num_col_]->send(delay, new NicEvent(msg));
    }
  }

void
FleXNIC::inject_next_packet(int dst_port, bool extra_wave)
{
  int remap_id = remap_id_[dst_port];///////get the target_port which the wavelength belong to
  Timestamp now_ = now();

  std::vector<int> src_waves = configure_info_[remap_id];
  for (int src_wave : src_waves)
  {
    if (d_inj_queue_[src_wave].size() > 0) {
      NetworkMessage* msg = d_inj_queue_[src_wave].front();
      d_inj_queue_[src_wave].pop();
      uint64_t num_bytes = msg->byteLength();
      TimeDelta time_to_inject = inj_lat_ + inj_byte_delay_ * num_bytes;

      //std::cout  << "Packet number in node_port: " << parent_->addr() << " " << src_wave << " " << d_inj_queue_[src_wave].size() << " packet_sz: " << num_bytes << " time: " << time_to_inject.nsec() << " at: " << now().nsec() << " packet: " << msg << std::endl;
      fflush(stdout);

      if (my_addr_ != msg->fromaddr()) {
	bp_data_inj_[remap_id] += num_bytes;
	bp_packet_inj_[remap_id] += 1;
      } else {
	if (remap_id != msg->toaddr()) {
	  new_data_r_inj_[remap_id] += num_bytes;
	  new_packet_r_inj_[remap_id] += 1;
	} else {
      //std::cout  << "Test Packet number in node_port: " << parent_->addr() << " " << src_wave << " " << d_inj_queue_[src_wave].size() << " packet_sz: " << num_bytes << " time: " << time_to_inject.nsec() << " at: " << now().nsec() << " packet: " << msg << std::endl;
	  new_data_d_inj_[remap_id] += num_bytes;
	  new_packet_d_inj_[remap_id] += 1;
	}
      }

      Timestamp next_inject = now_ + time_to_inject;
      if (extra_wave) {
        extra_next_out_free_[dst_port] = next_inject;
      } else {
        next_out_free_[dst_port] = next_inject;
      }
      sendExecutionEvent(next_inject, newCallback(this, &FleXNIC::inject_next_packet, dst_port, extra_wave));

      inject_with_port(msg, remap_id, time_to_inject);
      return;
    }
  }

  if (d_inj_queue_[remap_id].size() > 0) {
    NetworkMessage* msg = d_inj_queue_[remap_id].front();
    d_inj_queue_[remap_id].pop();
    uint64_t num_bytes = msg->byteLength();
    TimeDelta time_to_inject = inj_lat_ + inj_byte_delay_ * num_bytes;

    //std::cout  << "Packet number in node_port: " << parent_->addr() << " " << remap_id << " " << d_inj_queue_[remap_id].size() << " packet_sz: " << num_bytes << " time: " << time_to_inject.nsec() << " at: " << now().nsec() << " packet: " << msg << std::endl;
    fflush(stdout);

      if (my_addr_ != msg->fromaddr()) {
	bp_data_inj_[remap_id] += num_bytes;
	bp_packet_inj_[remap_id] += 1;
      } else {
	if (remap_id != msg->toaddr()) {
	  new_data_r_inj_[remap_id] += num_bytes;
	  new_packet_r_inj_[remap_id] += 1;
	} else {
    //std::cout  << "Test Packet number in node_port: " << parent_->addr() << " " << remap_id << " " << dst_port << " size: " << d_inj_queue_[remap_id].size() << " packet_sz: " << num_bytes << " time: " << time_to_inject.nsec() << " at: " << now().nsec() << " packet: " << msg << std::endl;
	  new_data_d_inj_[remap_id] += num_bytes;
	  new_packet_d_inj_[remap_id] += 1;
	}
      }

    Timestamp next_inject = now_ + time_to_inject;
    if (extra_wave) {
      extra_next_out_free_[dst_port] = next_inject;
    } else {
      next_out_free_[dst_port] = next_inject;
    }
    sendExecutionEvent(next_inject, newCallback(this, &FleXNIC::inject_next_packet, dst_port, extra_wave));

    inject_with_port(msg, remap_id, time_to_inject);
    return;
  }

  //std::cout << "Warning in inject_next_port of flex_lion/flex_lion.cc with node " << my_addr_ << " wave " << dst_port << " to port " << remap_id << " has no more packets to send" << std::endl;
}
/**/

}
}
