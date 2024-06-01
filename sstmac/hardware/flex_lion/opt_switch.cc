
#ifndef opt_switch_CC
#define opt_switch_CC

#include <sstmac/hardware/switch/network_switch.h>
#include <sstmac/hardware/flex_lion/opt_switch.h>
#include <sstmac/hardware/node/node.h>
#include <sstmac/hardware/topology/topology.h>
#include <sstmac/hardware/interconnect/interconnect.h>
#include <sstmac/hardware/nic/nic.h>
#include <sstmac/common/event_manager.h>
#include <sprockit/util.h>
#include <sprockit/sim_parameters.h>
#include <sprockit/keyword_registration.h>
#include <sstmac/software/launch/launch_event.h>
#include <random>
#include <sstmac/hardware/flex_lion/opt_port.h>

namespace sstmac {

namespace hw {

OptSwitch::OptSwitch(uint32_t cid, SST::Params& params) :
  ConnectableComponent(cid, params)
{
}

OptSwitch::~OptSwitch()
{
}

}

}
#endif // simple_switch_CC
