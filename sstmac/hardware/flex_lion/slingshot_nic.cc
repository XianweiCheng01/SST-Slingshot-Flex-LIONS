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

#include <sstmac/hardware/flex_lion/slingshot_nic.h>
//#include <sstmac/hardware/logp/logp_nic.h>
//#include <sstmac/hardware/logp/logp_switch.h>
#include <sstmac/hardware/network/network_message.h>
#include <sstmac/hardware/node/node.h>
#include <sstmac/software/process/operating_system.h>
#include <sstmac/common/event_callback.h>
#include <sprockit/util.h>
#include <sprockit/sim_parameters.h>
#include <sstmac/hardware/interconnect/interconnect.h>
#include <sprockit/keyword_registration.h>

RegisterKeywords(
  {"num_links", "number of linksof each node"},
);

namespace sstmac {
namespace hw {

SlingshotNIC::SlingshotNIC(uint32_t id, SST::Params& params, Node* node) :
  NIC(id, params, node)
{
  SST::Params inj_params = params.get_scoped_params("injection");
  inj_byte_delay_ = TimeDelta(inj_params.find<SST::UnitAlgebra>("bandwidth").getValue().inverse().toDouble());
  inj_lat_ = TimeDelta(inj_params.find<SST::UnitAlgebra>("latency").getValue().toDouble());

  num_links_ = params.find<int>("num_links", 4);

  next_out_free_.resize(num_links_);
  port_links_.resize(num_links_);
  //next_in_free_.resize(num_links_);//time to received of each

  for (int i=0; i<num_links_; i++) {
    std::string name = sprockit::sprintf("port_link_%d", i);
    port_links_[i] = new EventLink(name, TimeDelta(), configureLink(name, SharedBaseComponent::timeConverter(), newLinkHandler(this, &SlingshotNIC::mtlHandle)));
  }

}

SlingshotNIC::~SlingshotNIC()
{
}

void SlingshotNIC::updateRouting(int aid, std::string app_name)
{
}

void
SlingshotNIC::sendManagerMsg(NetworkMessage* msg)
{
    if (msg->toaddr() == my_addr_){
      //std::cout << "Test 0 in sendManagerMsg: " << my_addr_ << " to: " << msg->toaddr() << std::endl;
      intranodeSend(msg);
    } else {
      //std::cout << "Test 1 in sendManagerMsg: " << my_addr_ << " to: " << msg->toaddr() << std::endl;
      doSend(msg);
    }
}

void
SlingshotNIC::mtlHandle(Event *ev)
{
    //std::cout << "Test 0 in mtlHandle of logp/logp_nic.cc" << std::endl;
    fflush(stdout);
  Timestamp now_ = now();
  NicEvent* nev = static_cast<NicEvent*>(ev);
  NetworkMessage* msg = nev->msg();
  delete nev;
  recvMessage(msg);
  /*
  if (msg->byteLength() < negligibleSize_){
    //std::cout << "Test 1 in mtlHandle of logp/logp_nic.cc" << std::endl;
    fflush(stdout);
    recvMessage(msg);
  } else {
    //TimeDelta time_to_recv = inj_byte_delay_*msg->byteLength();
    //Timestamp recv_start = now_ - time_to_recv;
    if (recv_start > next_in_free_){
    //std::cout << "Test 2 in mtlHandle of logp/logp_nic.cc" << std::endl;
    fflush(stdout);
      next_in_free_ = now_;
      recvMessage(msg);
    } else {
    //std::cout << "Test 3 in mtlHandle of logp/logp_nic.cc" << std::endl;
    fflush(stdout);
      next_in_free_ += time_to_recv;
      sendExecutionEvent(next_in_free_, newCallback(this, &NIC::recvMessage, msg));
    }
  }
    //std::cout << "Test 4 in mtlHandle of logp/logp_nic.cc" << std::endl;
    fflush(stdout);
  */
}

void
SlingshotNIC::doSend(NetworkMessage* msg)
{
    //std::cout << "Test 0 in doSend of logp/logp_nic.cc" << std::endl;
  msg->setInjectionStarted(now());
    fflush(stdout);
  uint64_t num_bytes = msg->byteLength();
  Timestamp now_ = now();
  int earliest_idx = earliest_link();
  Timestamp start_send = now_ >= next_out_free_[earliest_idx] ? now_ : next_out_free_[earliest_idx];
  nic_debug("logp injection queued at %8.4e, sending at %8.4e for %s",
            now_.sec(), start_send.sec(), msg->toString().c_str());

  //TimeDelta time_to_inject = inj_lat_ + inj_byte_delay_ * num_bytes;
  TimeDelta time_to_inject = inj_byte_delay_ * num_bytes;
  next_out_free_[earliest_idx] = start_send + time_to_inject;

  if (msg->needsAck()){
    //std::cout << "Test 1 in doSend of logp/logp_nic.cc" << std::endl;
    fflush(stdout);
    NetworkMessage* acker = msg->cloneInjectionAck();
    auto ack_ev = newCallback(parent_, &Node::handle, acker);
    parent_->sendExecutionEvent(next_out_free_[earliest_idx], ack_ev);
  }

  TimeDelta extra_delay = start_send - now_;

  //std::cout << "Test 2 in doSend of flex_lion/slingshot_nic.cc: " << earliest_idx << " now: " << now_.nsec() << " " << (extra_delay+inj_lat_).nsec() << std::endl;
  fflush(stdout);
  //logp_link_->send(extra_delay, new NicEvent(msg));
  port_links_[earliest_idx]->send(extra_delay+inj_lat_, new NicEvent(msg));
    //std::cout << "Test 3 in doSend of logp/logp_nic.cc" << std::endl;
    fflush(stdout);
}

int SlingshotNIC::earliest_link() {
  int earliest_idx = 0;
  Timestamp earliest = next_out_free_[0];
  for (int idx=1; idx<num_links_; idx++) {
    if (earliest > next_out_free_[idx]) {
      earliest = next_out_free_[idx];
      earliest_idx = idx;
    }
  }
  return earliest_idx;
}

//void
//LogPNIC::connectOutput(int  /*src_outport*/, int  /*dst_inport*/, EventLink::ptr&& link)
//{
//  logp_link_ = std::move(link);
//}

//void
//LogPNIC::connectInput(int /*src_outport*/, int /*dst_inport*/, 
//                      EventLink::ptr&& /*link*/)
//{
  //nothing needed
//}

//LinkHandler*
//LogPNIC::payloadHandler(int  /*port*/)
//{
//  return newLinkHandler(this, &NIC::mtlHandle);
//}

}
}
