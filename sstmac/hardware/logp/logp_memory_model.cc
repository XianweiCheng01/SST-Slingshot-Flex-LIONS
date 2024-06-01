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

#include <sstmac/hardware/logp/logp_memory_model.h>
#include <sstmac/hardware/node/node.h>
#include <sstmac/software/libraries/compute/compute_event.h>
#include <sprockit/sim_parameters.h>
#include <sprockit/util.h>

namespace sstmac {
namespace hw {

LogPMemoryModel::~LogPMemoryModel()
{
  if (link_) delete link_;
}

LogPMemoryModel::LogPMemoryModel(uint32_t id, SST::Params& params, Node* node)
  : MemoryModel(id, params, node),
  num_mem_access_(nullptr)
{

  lat_ = TimeDelta(params.find<SST::UnitAlgebra>("latency").getValue().toDouble());

  auto bw = params.find<SST::UnitAlgebra>("bandwidth").getValue();
  if (bw.toDouble() == 0){
    spkt_abort_printf("Zero or missing memory.bandwidth parameter");
  }
  min_byte_delay_ = TimeDelta(bw.inverse().toDouble());
  link_ = new Link(min_byte_delay_, lat_);
  std::string subname = sprockit::sprintf("Mem.%d", node->addr());
  num_mem_access_ = registerStatistic<uint64_t>(params, "num_mem_access", subname);
}

void
LogPMemoryModel::accessFlow(uint64_t bytes, TimeDelta byte_request_delay, Callback* cb)
{
  mem_debug("simple model: doing access of %ld bytes", bytes);

  num_mem_access_->addData(bytes);
  parent_node_->memCount(bytes);

  TimeDelta delta_t = link_->newAccess(now(), bytes, byte_request_delay);
  //std::cout << "Test in LogPMemory id: " << parent_node_->addr() << " with bytes: " << bytes << " time: " << delta_t << std::endl;
  parent_node_->sendDelayedExecutionEvent(delta_t, cb);
}

void
LogPMemoryModel::accessRequest(int  /*linkId*/, Request * /*req*/)
{
  spkt_abort_printf("LogP does not support single memory requests - only flow-level calls");
}

TimeDelta
LogPMemoryModel::Link::newAccess(Timestamp now, uint64_t size, TimeDelta byte_request_delay)
{
  TimeDelta actual_byte_delay = byte_request_delay + byte_delay_;
  Timestamp base = std::max(now, last_access_);
  //TimeDelta access = lat_ + actual_byte_delay * size;


  //////req_bw (byte_delay_ / actual_byte_delay) /  start (base) /  end
  ///// if (start_bw > req_bw) modify (start_bw) time (last_access_ and lat) not change /// else fill in (modify the last_access_)
  /**/
  double req_bw = byte_delay_.nsec() / actual_byte_delay.nsec();
  double total_size = req_bw * (actual_byte_delay.sec() * size + lat_.sec());
  double start_time = base.sec();
  double start_bw = startBW(start_time);
  TimeDelta n_access = TimeDelta();
  TimeDelta delta = base - now;
  while (total_size > 0) {
    //std::cout << "Test 0 in newAccess with " << now.sec() << " " << last_access_.sec() << " base: " << base.sec() << " " << byte_request_delay << " " << byte_delay_ << " " << actual_byte_delay << " req_bw: " << req_bw << " start_bw: " << start_bw << " total_size: " << total_size << " delta: " << delta << std::endl;
    if (start_bw > req_bw) {
      delta += TimeDelta(total_size/req_bw);
      auto iter = bw_record_.find(now.sec()+delta.sec());
      if (iter != bw_record_.end()) {
        iter->second += req_bw;
      } else {
        bw_record_[now.sec()+delta.sec()] = req_bw;
      }
      total_size = 0;
    } else {
      if(bw_record_.size() == 0) {
        n_access += TimeDelta(total_size/start_bw);
        delta += TimeDelta(total_size/start_bw);
        total_size = 0;
      } else {
        double rest_size = start_bw * (bw_record_.begin()->first - start_time);
        if(rest_size >= total_size) {
          n_access += TimeDelta(total_size/start_bw);
          delta += TimeDelta(total_size/start_bw);
          total_size = 0;
        } else {
          n_access += TimeDelta(rest_size/start_bw);
          delta += TimeDelta(rest_size/start_bw);
          total_size -= rest_size;
          start_time = bw_record_.begin()->first;
          start_bw = startBW(start_time);
        }
      }
    }
    //std::cout << "Test 1 in newAccess with " << now.sec() << " " << last_access_.sec() << " base: " << base.sec() << " " << byte_request_delay << " " << byte_delay_ << " " << actual_byte_delay << " req_bw: " << req_bw << " start_bw: " << start_bw << " total_size: " << total_size << " delta: " << delta << std::endl;
  }
  last_access_ = base + n_access;
  return delta;
  /**
  while (total_size > 0) {
    if (start_bw == req_bw) {
      total_size = 0;
      if(bw_record_.size() == 0) {
        last_access_ = base + access;
      } else {
        last_access_ = bw_record_.begin()->first;
        iter = bw_record_.find(base.sec()+access.sec());
        if (iter == bw_record_.end()) {
          bw_record_[base.sec()+access.sec()] = req_bw;
        } else {
          iter->second += req_bw;
        }
        bw_record_.erase(bw_record.begin());
      }
      delta += access;
    } else if (start_bw > req_bw) {
      total_size = 0;
      iter = bw_record_.find(base.sec()+access.sec());
      if (iter == bw_record_.end()) {
        bw_record_[base.sec()+access.sec()] = req_bw;
      } else {
        iter->second += req_bw;
      }
      delta += access;
    } else {
      //double total_size = req_bw * access.sec();
      //double start_time = base.sec();
      double duration = bw_record_.begin()->first() - start_time;
      if (total_size > duration * start_bw) {
        delta += TimeDelta(duration);
        total_size -= duration * start_bw;
      } else {
        total_size = 0;
        delta += TimeDelta(total_size/start_bw);
        last_access_ = 
      }
    }
  }

  /**/


  //TimeDelta n_access = TimeDelta(req_bw);
  /*if (byte_request_delay > TimeDelta()) {
    return (base - now + access);
  }*/

  /*
  last_access_ = base + access;
  TimeDelta delta = last_access_ - now;
  std::cout << "Test_0_in_accessFlow_of_logp_memory_model_with_delay: " << delta.nsec() << " base: " << base.nsec() << " last: " << last_access_.nsec() <<std::endl;
  return delta;
  */
}



}
} /* namespace sstmac */
