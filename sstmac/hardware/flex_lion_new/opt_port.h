
#ifndef COMPONENTS_OPT_PORT_H_
#define COMPONENTS_OPT_PORT_H_

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

}
}

#endif
