#include <sstmac/hardware/flex_lion/slingshot_port.h>
#include <sstmac/hardware/flex_lion/opt_switch.h>
#include <sstmac/hardware/network/network_message.h>
//#include <sstmac/common/event_callback.h>
#include <sstmac/hardware/nic/nic.h>

namespace sstmac {
namespace hw {

  SlingshotPort::SlingshotPort(uint32_t cid, SST::Params& params, OptSwitch* parent) :
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
    //std::string name = sprockit::sprintf("port_%d_%d", port_id_, wave_id_);
    std::string name = params.find<std::string>("real_port_name");
    //std::string eve_name = name+sprockit::sprintf("_%d_%d", port_id_, wave_id_);
    //std::string eve_name = parent_->switchName()+name;
  //std::cout << "Test 0 in SlingshotPort with " << name << " port_id " << port_id_ << " wave_id " << wave_id_ << std::endl;
  //fflush(stdout);
    loc_link_ = new EventLink(name, TimeDelta(), configureLink(name, SharedBaseComponent::timeConverter(), newLinkHandler(this, &SlingshotPort::inputEvent)));
  //std::cout << "Test 1 in SlingshotPort with " << eve_name << std::endl;
  fflush(stdout);
  }

  void SlingshotPort::inputEvent(Event* ev)
  {
    //std::cout << "Test 0 in inputEvent of flex_lion/slingshot_port.cc port_id: " << port_id_ << " wave_id: " << wave_id_ << std::endl;
    NicEvent* nev = dynamic_cast<NicEvent*>(ev);
    NetworkMessage* msg = nev->msg();
    delete nev;
    //std::cout << "Test 1 in inputEvent of flex_lion/slingshot_port.cc port_id: " << port_id_ << " wave_id: " << wave_id_ << " to: " << msg->toaddr() << " msg: " << msg << std::endl;
    fflush(stdout);
    parent_->input_port(port_id_, wave_id_, msg);
  }

  void SlingshotPort::locSend(NetworkMessage* msg)
  {
    //std::cout << "Test 0 in locSend of flex_lion/slingshot_port.cc port_id: " << port_id_ << " wave_id: " << wave_id_ << " to: " << msg->toaddr() << std::endl;
    fflush(stdout);
    loc_link_->send(now()-now(), new NicEvent(msg));
    //std::cout << "Test 1 in locSend of flex_lion/slingshot_port.cc port_id: " << port_id_ << " wave_id: " << wave_id_ << " to: " << msg->toaddr() << " msg: " << msg << std::endl;
    fflush(stdout);
  }

  void SlingshotPort::doInit(unsigned int phase)
  {
    //std::cout << "Test 0 in doInit of flex_port.cc phase: " << phase << std::endl;
    fflush(stdout);
    switch (phase) {
      case 0:
      {
        Event* ev = new NicEvent();
    //std::cout << "Test 1 in doInit of flex_port.cc phase: " << phase << std::endl;
    fflush(stdout);
        loc_link_->sendInitData(ev);
    //std::cout << "Test 2 in doInit of flex_port.cc phase: " << phase << std::endl;
    fflush(stdout);
        break;
      }
      case 1:
      {
    //std::cout << "Test 3 in doInit of flex_port.cc phase: " << phase << std::endl;
    fflush(stdout);
        Event* ev = loc_link_->recvInitData();
    //std::cout << "Test 4 in doInit of flex_port.cc phase: " << phase << std::endl;
    fflush(stdout);
        delete ev;
        break;
      }
      default:
      {
        break;
      }
    }
  }

}
}
