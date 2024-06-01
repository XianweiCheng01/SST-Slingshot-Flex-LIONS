
#ifndef COMPONENTS_FLEX_PORT_H
#define COMPONENTS_FLEX_PORT_H

#include <sstmac/hardware/flex_lion/opt_port.h>

namespace sstmac {
namespace hw {

class SlingshotPort : public OptPort
//class FleXPort : public ConnectableComponent
{
  public:
#if SSTMAC_INTEGRATED_SST_CORE
  SST_ELI_REGISTER_SUBCOMPONENT_DERIVED(
    SlingshotPort,
    "macro",
    "slingshot_port",
    SST_ELI_ELEMENT_VERSION(1,0,0),
    "implements a ",
    sstmac::hw::OptPort)
#else
  SST_ELI_REGISTER_DERIVED(
    OptPort,
    SlingshotPort,
    "macro",
    "slingshot",
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
    SlingshotPort(uint32_t cid, SST::Params& params, hw::OptSwitch* parent);
    //FleXPort(int port_id, int wave_id, FleXSwitch* parent);

    std::string toString() const override {
      return "Slingshot port";
    }

    ~SlingshotPort() {
    }
    void inputEvent(Event* ev) override;
    void locSend(NetworkMessage* msg) override;
    //void outputEvent(NetworkMessage* msg) override;
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
