


#include <sstmac/hardware/flex_lion/opt_port.h>
#include <sstmac/hardware/flex_lion/opt_switch.h>
#include <sstmac/hardware/nic/nic.h>

namespace sstmac {
namespace hw {

  //OptPort::OptPort(int port, int wave, FleXSwitch* parent)
  OptPort::OptPort(uint32_t cid, SST::Params& params, OptSwitch* parent) :
  //SubComponent(cid, "port", parent)
  SubComponent(cid, params.find<std::string>("port_name", "Port"), parent)
  {
  }

  OptPort::~OptPort() {
  }


}
}

