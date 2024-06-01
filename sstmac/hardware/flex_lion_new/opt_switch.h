
#ifndef OPT_SWITCH_H
#define OPT_SWITCH_H

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


class OptSwitch : public ConnectableComponent
{
 public:
  SST_ELI_DECLARE_BASE(OptSwitch)
  SST_ELI_DECLARE_DEFAULT_INFO()
  SST_ELI_DECLARE_CTOR(uint32_t, SST::Params&)

  //SST_ELI_DOCUMENT_SUBCOMPONENT_SLOTS(
  //    {"opt_port", "The optical port", "sstmac::OptPort"},
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

  //virtual void input_queue(int s_id, int inport, NetworkMessage* msg) = 0;
  virtual void sendEvent(int port, int wave, Event* ev) = 0;
  //virtual void crossbar(int s_id, int real_outport, int outport) = 0;
  //virtual void forward(int s_id, int outport, NetworkMessage* msg) = 0;

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

}
}


#endif // simple_switch_CC
