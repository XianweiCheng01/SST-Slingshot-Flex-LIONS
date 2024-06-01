
#ifndef MEMORYMODEL_H_
#define MEMORYMODEL_H_

#include <sstmac/hardware/common/connection.h>
#include <sprockit/factory.h>
#include <sprockit/debug.h>

#include <sstmac/hardware/node/node_fwd.h>
#include <sstmac/hardware/memory/memory_id.h>
#include <sstmac/sst_core/integrated_component.h>

namespace sstmac {
namespace hw {

class OptPort : public SubComponent
{
 public:
#if SSTMAC_INTEGRATED_SST_CORE
  SST_ELI_REGISTER_SUBCOMPONENT_API(sstmac::hw::OptPort, hw::FleXSwitch*)
#else
  SST_ELI_DECLARE_BASE(OptPort)
  SST_ELI_DECLARE_DEFAULT_INFO()
  //SST_ELI_DECLARE_CTOR(uint32_t,SST::Params&,hw::FleXSwitch*)
  SST_ELI_DECLARE_CTOR(int, int, hw::FleXSwitch*)
#endif
};

}
}
