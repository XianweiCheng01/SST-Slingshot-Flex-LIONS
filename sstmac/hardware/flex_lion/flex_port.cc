#include <sstmac/hardware/flex_lion/flex_port.h>
#include <sstmac/hardware/flex_lion/flex_switch.h>
#include <sstmac/hardware/nic/nic.h>

namespace sstmac {
namespace hw {

/*
  //OptPort::OptPort(int port, int wave, FleXSwitch* parent)
  OptPort::OptPort(uint32_t cid, SST::Params& params, OptSwitch* parent) :
  //SubComponent(cid, "port", parent)
  SubComponent(cid, params.find<std::string>("port_name", "Port"), parent)
  {
  }

  OptPort::~OptPort() {
  }
*/


  FleXPort::FleXPort(uint32_t cid, SST::Params& params, OptSwitch* parent) :
  OptPort(cid, params, parent),
  //FleXPort::FleXPort(int port_id, int wave_id, FleXSwitch* parent) :
  //port_id_(port_id),
  //wave_id_(wave_id),
  parent_(parent)
  {
  //std::cout << "Test -1 in FleXPort with port_id: " << port_id_ << ", wave_id: " << wave_id_ << std::endl;
  fflush(stdout);
    port_id_ = params.find<int>("port_id", 0);
    wave_id_ = params.find<int>("wave_id", 0);
    std::string name = sprockit::sprintf("port_%d_%d", port_id_, wave_id_);
  //std::cout << "Test 0 in FleXPort with port_id: " << port_id_ << ", wave_id: " << wave_id_ << std::endl;
  fflush(stdout);
    loc_link_ = new EventLink(name, TimeDelta(), configureLink(name, SharedBaseComponent::timeConverter(), newLinkHandler(this, &FleXPort::inputEvent)));
  }

  void FleXPort::inputEvent(Event* ev)
  {
    NicEvent* nev = dynamic_cast<NicEvent*>(ev);
    NetworkMessage* msg = nev->msg();
    delete nev;
    parent_->input_port(port_id_, wave_id_, msg);
  }

  void FleXPort::locSend(NetworkMessage* msg)
  {
    loc_link_->send(now()-now(), new NicEvent(msg));
  }

  void FleXPort::doInit(unsigned int phase)
  {
  }

}
}
