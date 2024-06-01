
#ifndef COMPONENTS_FLEX_PORT_H
#define COMPONENTS_FLEX_PORT_H

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
#include <sstmac/hardware/flex_lion/flex_switch_fwd.h>

#include <sprockit/factory.h>
#include <sprockit/debug.h>
namespace sstmac {
namespace hw {
#if 0
class OptPort : public ConnectableComponent
{
  public:
    SST_ELI_DECLARE_BASE(OptPort)
    SST_ELI_DECLARE_DEFAULT_INFO()
    SST_ELI_DECLARE_CTOR(int, int, FleXSwitch*)

    OptPort(int port, int wave, FleXSwitch* parent);

    virtual ~OptPort();

    virtual void sendEvent(Event* ev);
    virtual void locSend(NetworkMessage* msg);

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
#endif

class OptPort : public SubComponent
{
 public:
#if SSTMAC_INTEGRATED_SST_CORE
  SST_ELI_REGISTER_SUBCOMPONENT_API(sstmac::hw::OptPort, hw::OptSwitch*)
#else
  SST_ELI_DECLARE_BASE(OptPort)
  SST_ELI_DECLARE_DEFAULT_INFO()
  SST_ELI_DECLARE_CTOR(uint32_t,SST::Params&,hw::OptSwitch*)
  //SST_ELI_DECLARE_CTOR(int, int, hw::FleXSwitch*)
#endif

    //OptPort(int port, int wave, FleXSwitch* parent);
    OptPort(uint32_t cid, SST::Params& params, hw::OptSwitch* parent);

    virtual ~OptPort();

    virtual void sendEvent(Event* ev) = 0;
    virtual void doInit(unsigned int phase) = 0;
    virtual void locSend(NetworkMessage* msg) = 0;
};

class FleXPort : public OptPort
//class FleXPort : public ConnectableComponent
{
  public:
#if SSTMAC_INTEGRATED_SST_CORE
  SST_ELI_REGISTER_SUBCOMPONENT_DERIVED(
    FleXPort,
    "macro",
    "flex_port",
    SST_ELI_ELEMENT_VERSION(1,0,0),
    "implements a memory model that models nothing",
    sstmac::hw::OptPort)
#else
  SST_ELI_REGISTER_DERIVED(
    OptPort,
    FleXPort,
    "macro",
    "flex",
    SST_ELI_ELEMENT_VERSION(1,0,0),
    "implements a memory model that models nothing")
#endif
    /**
    SST_ELI_REGISTER_COMPONENT(
      FleXPort,
      "macro",
      "flex_port",
      SST_ELI_ELEMENT_VERSION(1,0,0),
      "A optical port that only for one wavelength",
      COMPONENT_CATEGORY_NETWORK)
    /**/
    /*
    SST_ELI_REGISTER_DERIVED(
      OptPort,
      FleXPort,
      "macro",
      "flex_port",
      SST_ELI_ELEMENT_VERSION(1,0,0),
      "A optical port that only for one wavelength")
    */

    SST_ELI_DOCUMENT_PARAMS(
      {"port_id", "id of port", "-1"},
      {"wave_id", "id of wave", "-1"}
    )

  public:
    FleXPort(uint32_t cid, SST::Params& params, hw::OptSwitch* parent);
    //FleXPort(int port_id, int wave_id, FleXSwitch* parent);

    std::string toString() const override {
      return "FleX port";
    }

    ~FleXPort() {
    }
    void sendEvent(Event* ev) override;
    void locSend(NetworkMessage* msg) override;
    void doInit(unsigned int phase) override;

  private:
    int port_id_;
    int wave_id_;

    EventLink* loc_link_;
    OptSwitch* parent_;
};

}
}
#endif
