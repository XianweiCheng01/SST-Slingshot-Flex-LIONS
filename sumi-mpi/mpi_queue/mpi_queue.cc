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

#include <sumi-mpi/mpi_api.h>
#include <sumi-mpi/mpi_queue/mpi_queue.h>
#include <sumi-mpi/mpi_queue/mpi_queue_recv_request.h>
#include <sumi-mpi/mpi_queue/mpi_queue_probe_request.h>
#include <sumi-mpi/mpi_status.h>
#include <sumi-mpi/mpi_protocol/mpi_protocol.h>
#include <sstmac/software/process/app.h>
#include <sstmac/software/process/operating_system.h>
#include <sstmac/software/process/thread.h>

#include <unusedvariablemacro.h>

#include <sprockit/sim_parameters.h>
#include <sprockit/factory.h>
#include <sprockit/debug.h>
#include <sprockit/errors.h>
#include <sprockit/statics.h>
#include <sprockit/keyword_registration.h>
#include <sprockit/util.h>
#include <stdint.h>

RegisterNamespaces("traffic_matrix", "num_messages");
RegisterKeywords(
{ "smp_single_copy_size", "the minimum size of message for single-copy protocol" },
{ "max_eager_msg_size", "the maximum size for using eager pt2pt protocol" },
{ "max_vshort_msg_size", "the maximum size for mailbox protocol" },
);

DeclareDebugSlot(mpi_all_sends);
RegisterDebugSlot(mpi_all_sends);

using namespace std::placeholders;

namespace sumi {

static sprockit::NeedDeletestatics<MpiQueue> del_statics;

bool
MpiQueue::sortbyseqnum::operator()(MpiMessage* a, MpiMessage*b) const
{
  return (a->seqnum() < b->seqnum());
}

MpiQueue::MpiQueue(SST::Params& params, int task_id,
                   MpiApi* api, CollectiveEngine*  /*engine*/) :
  queue_(api->parent()->os()),
  taskid_(task_id),
  api_(api)
{
  max_vshort_msg_size_ = params.find<SST::UnitAlgebra>("max_vshort_msg_size", "512B").getRoundedValue();
  max_eager_msg_size_ = params.find<SST::UnitAlgebra>("max_eager_msg_size", "8192B").getRoundedValue();
  use_put_window_ = params.find<bool>("use_put_window", false);

  protocols_.resize(MpiProtocol::NUM_PROTOCOLS);
  protocols_[MpiProtocol::EAGER0] = new Eager0(params, this);
  protocols_[MpiProtocol::EAGER1] = new Eager1(params, this);
  protocols_[MpiProtocol::RENDEZVOUS_GET] = new RendezvousGet(params, this);
  protocols_[MpiProtocol::DIRECT_PUT] = new DirectPut(params, this);

  pt2pt_cq_ = api_->allocateCqId();
  coll_cq_ = api_->allocateCqId();

  api_->allocateCq(pt2pt_cq_, std::bind(&progress_queue::incoming, &queue_, pt2pt_cq_, _1));
  api_->allocateCq(coll_cq_, std::bind(&progress_queue::incoming, &queue_, coll_cq_, _1));
}

struct init_struct {
  int pt2pt_cq_id;
  int coll_cq_id;
  bool all_equal;
};

void
MpiQueue::init()
{
  auto init_fxn = [](void* out, const void* src, int nelems){
    auto* outs= (init_struct*)out;
    auto* srcs = (init_struct*)src;
    for (int i=0; i < nelems; ++i){
      auto& out = outs[i];
      auto& src = srcs[i];
      out.all_equal = src.all_equal
          && out.pt2pt_cq_id == src.pt2pt_cq_id
          && out.coll_cq_id == src.coll_cq_id;
    }
  };
  init_struct init;
  init.coll_cq_id = coll_cq_;
  init.pt2pt_cq_id = pt2pt_cq_;
  init.all_equal = true;
  auto cmsg = api_->engine()->allreduce(&init, &init, 1, sizeof(init_struct), 0, init_fxn,
                            Message::default_cq);
  if (cmsg == nullptr){
    cmsg = api_->engine()->blockUntilNext(Message::default_cq);
  }

  if (cmsg->tag() != 0){
    spkt_abort_printf("got bad collective done message in mpi_queue::init");
  }
  delete cmsg;

  if (!init.all_equal){
    spkt_abort_printf("MPI_Init: procs did not agree on completion queue ids");
  }
}

void
MpiQueue::deleteStatics()
{
}

sstmac::Timestamp
MpiQueue::now() const {
  return api_->now();
}

MpiQueue::~MpiQueue() throw ()
{
  for (auto* prot : protocols_){
    if (prot) delete prot;
  }
}

void
MpiQueue::send(MpiRequest *key, int count, MPI_Datatype type,
  int dest, int tag, MpiComm *comm, void *buffer)
{
  MpiType* typeobj = api_->typeFromId(type);
  if (typeobj->packed_size() < 0){
    spkt_throw_printf(sprockit::ValueError,
      "MPI_Datatype %s has negative size %ld",
      api_->typeStr(type).c_str(), typeobj->packed_size());
  }
  uint64_t bytes = count * uint64_t(typeobj->packed_size());

  int prot_id = MpiProtocol::RENDEZVOUS_GET;
  if (use_put_window_) {
    prot_id = MpiProtocol::DIRECT_PUT;
  } else if (bytes <= max_vshort_msg_size_) {
    prot_id = MpiProtocol::EAGER0;
  } else if (bytes <= max_eager_msg_size_) {
    prot_id = MpiProtocol::EAGER1;
  }

  MpiProtocol* prot = protocols_[prot_id];

  mpi_queue_debug("%f: starting send count=%d, type=%s, dest=%d, tag=%d, comm=%s, prot=%s",
    float(now().nsec()),
    count, api_->typeStr(type).c_str(), int(dest),
    int(tag), api_->commStr(comm).c_str(),
    prot->toString().c_str());

  TaskId dst_tid = comm->peerTask(dest);
  prot->start(buffer, comm->rank(), dest, dst_tid, count, typeobj,
              tag, comm->id(), next_outbound_[dst_tid]++, key);

}

MpiMessage*
MpiQueue::findMatchingRecv(MpiQueueRecvRequest* req)
{
  for (auto it = need_recv_match_.begin(); it != need_recv_match_.end(); ++it) {
    MpiMessage* mess = *it;
    if (req->matches(mess)) {
      mpi_queue_debug("%f: test_recv_erase size:%d matched recv tag=%s,src=%s on comm=%s to send %s",
        float(now().nsec()),
        need_recv_match_.size(),
        api_->tagStr(req->tag_).c_str(), 
        api_->srcStr(req->source_).c_str(),
        api_->commStr(req->comm_).c_str(),
        mess->toString().c_str());

      need_recv_match_.erase(it);
      return mess;
    }
  }
  mpi_queue_debug("%f: test_send_push size:%d could not match recv tag=%s, src=%s to any of %d sends on comm=%s",
    float(now().nsec()),
    need_send_match_.size(),
    api_->tagStr(req->tag_).c_str(), 
    api_->srcStr(req->source_).c_str(),
    need_recv_match_.size(),
    api_->commStr(req->comm_).c_str());

  need_send_match_.push_back(req);
  return nullptr;
}

//
// Receive data.
//
void
MpiQueue::recv(MpiRequest* key, int count,
                MPI_Datatype type,
                int source, int tag,
                MpiComm* comm,
                void* buffer)
{
  mpi_queue_debug("%f: starting recv count=%d, type=%s, src=%s, tag=%s, comm=%s, buffer=%p",
        float(now().nsec()),
        count, api_->typeStr(type).c_str(), api_->srcStr(source).c_str(),
        api_->tagStr(tag).c_str(), api_->commStr(comm).c_str(), buffer);

  MpiQueueRecvRequest* req = new MpiQueueRecvRequest(api_->now(), key, this,
                            count, type, source, tag, comm->id(), buffer);
  MpiMessage* mess = findMatchingRecv(req);
  if (mess) {
    auto* protocol = protocols_[mess->protocol()];
    protocol->incoming(mess, req);
  }
}

void
MpiQueue::finalizeRecv(MpiMessage* msg, MpiQueueRecvRequest* req)
{
  req->key_->complete(msg);
  if (req->recv_buffer_ != req->final_buffer_){
    req->type_->unpack_recv(req->recv_buffer_, req->final_buffer_, msg->count());
    delete[] req->recv_buffer_;
  }
  delete req;
}

//
// Ask for a notification when a message with the given signature arrives.
//
void
MpiQueue::probe(MpiRequest* key, MpiComm* comm,
                 int source, int tag)
{
  mpi_queue_debug("%f: starting probe src=%s, tag=%s, comm=%s",
    float(now().nsec()),
    api_->srcStr(source).c_str(), api_->tagStr(tag).c_str(),
    api_->commStr(comm).c_str());

  mpi_queue_probe_request* req = new mpi_queue_probe_request(key, comm->id(), source, tag);
  // Figure out whether we already have a matching message.
  for (auto it = need_recv_match_.begin(); it != need_recv_match_.end(); ++it) {
    MpiMessage* mess = *it;
    if (req->matches(mess)){
      // We're good to go.
      req->complete(mess);
      return;
    }
  }
  // If we get here, we still need to wait for the message.
  probelist_.push_back(req);
}

//
// Immediate-mode probe for a message with the given signature.
//
bool
MpiQueue::iprobe(MpiComm* comm,
                  int source,
                  int tag,
                  MPI_Status* stat)
{
  mpi_queue_debug("%f: starting immediate probe src=%s, tag=%s, comm=%s",
    float(now().nsec()),
    api_->srcStr(source).c_str(), api_->tagStr(tag).c_str(),
    api_->commStr(comm).c_str());

  mpi_queue_probe_request req(NULL, comm->id(), source, tag);
  for (auto it = need_recv_match_.begin(); it != need_recv_match_.end(); ++it) {
    MpiMessage* mess = *it;
    if (req.matches(mess)) {
      // This is it
      if (stat != MPI_STATUS_IGNORE) mess->buildStatus(stat);
      return true;
    }
  }
  return false;
}

MpiQueueRecvRequest*
MpiQueue::findMatchingRecv(MpiMessage* message)
{
  auto end = need_send_match_.end();
  for (auto it = need_send_match_.begin(); it != end;) {
    auto* req = *it;
    auto tmp = it++;
    if (req->isCancelled()) {
      mpi_queue_debug("%f: test_send_erase size:%d cancelled send tag=%s,src=%s on comm=%s",
        float(now().nsec()),
        need_send_match_.size(),
        api_->tagStr(req->tag_).c_str(), 
        api_->srcStr(req->source_).c_str(),
        api_->commStr(req->comm_).c_str());
      need_send_match_.erase(tmp);
    } else if (req->matches(message)) {
      mpi_queue_debug("%f: test_send_erase size:%d matched send tag=%s,src=%s on comm=%s to send %s",
        float(now().nsec()),
        need_send_match_.size(),
        api_->tagStr(req->tag_).c_str(), 
        api_->srcStr(req->source_).c_str(),
        api_->commStr(req->comm_).c_str(),
        message->toString().c_str());
      need_send_match_.erase(tmp);
      return req;
    }
  }
  mpi_queue_debug("%f: test_recv_push size:%d going to size %d add message %s",
    float(now().nsec()),
    need_recv_match_.size(),
    need_send_match_.size(),
    message->toString().c_str());
  need_recv_match_.push_back(message);
  return nullptr;
}

void
MpiQueue::incomingCollectiveMessage(sumi::Message* m)
{
  //std::cout << "Test 0 in incomingCollectiveMessage with m: " << m << ", src: " << m->fromaddr() << ", dst: " << m->toaddr() << std::endl;
  fflush(stdout);

  CollectiveWorkMessage* msg = safe_cast(CollectiveWorkMessage, m);
  //std::cout << "Test 1 in incomingCollectiveMessage" << std::endl;
  fflush(stdout);

  CollectiveDoneMessage* cmsg = api_->engine()->incoming(msg);
  //std::cout << "Test 2 in incomingCollectiveMessage" << std::endl;
  fflush(stdout);

  if (cmsg){
    MpiComm* comm = safe_cast(MpiComm, cmsg->dom());
  //std::cout << "Test 3 in incomingCollectiveMessage" << std::endl;
  fflush(stdout);

    MpiRequest* req = comm->getRequest(cmsg->tag());
  //std::cout << "Test 4 in incomingCollectiveMessage" << std::endl;
  fflush(stdout);

    CollectiveOpBase* op = req->collectiveData();
  //std::cout << "Test 5 in incomingCollectiveMessage" << std::endl;
  fflush(stdout);

    api_->finishCollective(op);
  //std::cout << "Test 6 in incomingCollectiveMessage" << std::endl;
  fflush(stdout);

    req->complete();
  //std::cout << "Test 7 in incomingCollectiveMessage" << std::endl;
  fflush(stdout);

    delete cmsg;
  }
}

//
// Handle a new incoming message.  Can be either an eager send (with data)
// or a new handshake request
//
void
MpiQueue::incomingPt2ptMessage(Message* m)
{
  mpi_queue_debug("%f: incoming new message %s",
    float(now().nsec()),
    m->toString().c_str());

  MpiMessage* message = safe_cast(MpiMessage, m);

  if (message->stage() > 0 || message->isNicAck()){
    //already matched - pass that on through
    handlePt2ptMessage(message);
    return;
  }

  TaskId tid(message->sender());
  auto& next_inbound = next_inbound_[tid];
  if (message->seqnum() == next_inbound){
    mpi_queue_debug("%f: seqnum for task %d matched expected seqnum %d and advanced to next seqnum",
        float(now().nsec()),
        int(tid), int(next_inbound));


    handlePt2ptMessage(message);
    ++next_inbound;

    // Handle any messages that have been freed by the arrival of this one
    if (!held_[tid].empty()) {
      hold_list_t::iterator it = held_[tid].begin(), end = held_[tid].end();
      while (it != end) {
        MpiMessage* mess = *it;
        mpi_queue_debug("%f: handling out-of-order message for task %d, seqnum %d",
            float(now().nsec()),
            int(tid), mess->seqnum());
        if (mess->seqnum() <= next_inbound_[tid]) {
          //it = held_[tid].erase(it);
          it++;
          handlePt2ptMessage(mess);
          ++next_inbound;
        } else {
          break;
        }
      }
      // Now erase all the completed messages from the held queue.
      held_[tid].erase(held_[tid].begin(), it);
    }
  } else if (message->seqnum() < next_inbound){
    spkt_abort_printf("message sequence went backwards on %s from %d",
                      message->toString().c_str(), next_inbound);
  } else {
    mpi_queue_debug("%f: message arrived out-of-order with seqnum %d, didn't match expected %d for task %d",
         float(now().nsec()),
         message->seqnum(), int(next_inbound), int(tid));
    held_[tid].insert(message);
  }
}

void
MpiQueue::notifyProbes(MpiMessage* message)
{
  auto pit = probelist_.begin();
  auto pend = probelist_.end();
  while (pit != pend) {
    auto tmp = pit++;
    mpi_queue_probe_request* preq = *tmp;
    if (preq->matches(message)){
      probelist_.erase(tmp);
      preq->complete(message);
      delete preq;
    }
  }
}

void
MpiQueue::handlePt2ptMessage(MpiMessage* message)
{
  MpiProtocol* protocol = protocols_[message->protocol()];
  protocol->incoming(message);
}

void
MpiQueue::incomingMessage(sumi::Message* msg)
{
  //std::cout << "Test 0 in mpiqueue::incomingMessage " << msg << std::endl;
  fflush(stdout);

  if (msg->cqId() == pt2pt_cq_){
  //std::cout << "Test 1 in mpiqueue::incomingMessage " << msg << std::endl;
  fflush(stdout);
    incomingPt2ptMessage(msg);
  } else if (msg->cqId() == coll_cq_){
  //std::cout << "Test 2 in mpiqueue::incomingMessage " << msg << std::endl;
  fflush(stdout);
    incomingCollectiveMessage(msg);
  } else {
    spkt_abort_printf("Got bad completion queue %d for %s",
                      msg->cqId(), msg->toString().c_str());
  }
  //std::cout << "Test 3 in mpiqueue::incomingMessage" << std::endl;
  fflush(stdout);

}

void
MpiQueue::nonblockingProgress()
{
  sumi::Message* msg = api_->poll(false); //do not block
  while (msg){
  //std::cout << "Test in nonblockingProcess before incomingMessage " << msg << std::endl;
  fflush(stdout);

    incomingMessage(msg);
    msg = api_->poll(false);
  }
}

sstmac::Timestamp
MpiQueue::progressLoop(MpiRequest* req)
{
  if (!req || req->isComplete()) {
    return api_->now();
  }

  mpi_queue_debug("%f: entering progress loop",
    float(now().nsec())
  );

  sstmac::Timestamp wait_start = api_->now();
  //std::cout << "Test 0 in mpiqueue::progressLoop " << wait_start.nsecRounded() << std::endl;
  fflush(stdout);

  while (!req->isComplete()) {
  //std::cout << "Test 0-1 in mpiqueue::progressLoop" << std::endl;
  fflush(stdout);
    mpi_queue_debug("%f: blocking on progress loop",
    float(now().nsec())
    );
    sumi::Message* msg = queue_.find_any();
  //std::cout << "Test 1 in mpiqueue::progressLoop: " << msg << std::endl;
  fflush(stdout);

    if (!msg){
      spkt_abort_printf("polling returned null message");
    }
    req->setWaitStart(wait_start);
    incomingMessage(msg);
  //std::cout << "Test 2 in mpiqueue::progressLoop" << std::endl;
  fflush(stdout);
  }
  //std::cout << "Test 3 in mpiqueue::progressLoop" << std::endl;
  fflush(stdout);


  mpi_queue_debug("%f: finishing progress loop",
    float(now().nsec())
  );
  return api_->now();
}

bool
MpiQueue::atLeastOneComplete(const std::vector<MpiRequest*>& req)
{
  mpi_queue_debug("%f: checking if any of %d requests is done",
    float(now().nsec()),
    (int)req.size());
  for (int i=0; i < (int) req.size(); ++i) {
    if (req[i] && req[i]->isComplete()) {
      mpi_queue_debug("%f: request is done",
        float(now().nsec())
      );
      return true;
    }
  }
  return false;
}

void
MpiQueue::startProgressLoop(const std::vector<MpiRequest*>& reqs)
{
  mpi_queue_debug("%f: starting progress loop",
    float(now().nsec())
  );
  while (!atLeastOneComplete(reqs)) {
    mpi_queue_debug("%f: blocking on progress loop",
      float(now().nsec())
    );
    sumi::Message* msg = queue_.find_any();
    if (!msg){
      spkt_abort_printf("polling returned null message");
    }
  //std::cout << "Test 0 in mpiqueue::startProgressLoop " << msg << std::endl;
  fflush(stdout);
    incomingMessage(msg);
  }
  mpi_queue_debug("%f: finishing progress loop",
    float(now().nsec())
  );
}

void
MpiQueue::forwardProgress(double timeout)
{
  mpi_queue_debug("%f: starting forward progress with timeout=%f",
    float(now().nsec()),
    timeout);
  sumi::Message* msg = queue_.find_any(true, timeout); //block until timeout
  if (msg){
  //std::cout << "Test 0 in mpiqueue::forwardProgress " << msg << std::endl;
  fflush(stdout);
    incomingMessage(msg);
  }
}

void
MpiQueue::startProgressLoop(
  const std::vector<MpiRequest*>& req,
  sstmac::TimeDelta  /*timeout*/)
{
  startProgressLoop(req);
}

void
MpiQueue::finishProgressLoop(const std::vector<MpiRequest*>&)
{
}

void
MpiQueue::memcopy(uint64_t bytes)
{
  //std::cout << "Test in memcopy of sumi-mpi/mpi_queue/mpi_queue.cc before memcopyDelay ";
  api_->memcopyDelay(bytes);
}

void
MpiQueue::bufferUnexpected(MpiMessage* msg)
{
  //std::cout << "Test in bufferUnexpected of sumi-mpi/mpi_queue/mpi_queue.cc before memcopyDelay " << msg->toString().c_str() << " ";
  CallGraphAppend(MPIQueueBufferUnexpectedMessage);
  api_->memcopyDelay(msg->payloadBytes());
}

}
