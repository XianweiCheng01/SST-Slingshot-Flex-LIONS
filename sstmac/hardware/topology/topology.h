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

#ifndef SSTMAC_HARDWARE_NETWORK_SWITCHES_SWITCHTOPOLOGY_H_INCLUDED
#define SSTMAC_HARDWARE_NETWORK_SWITCHES_SWITCHTOPOLOGY_H_INCLUDED

#include <sstmac/hardware/topology/coordinates.h>
#include <sstmac/hardware/topology/traffic/traffic.h>
#include <sstmac/hardware/router/router_fwd.h>
#include <sstmac/hardware/common/connection.h>
#include <sstmac/hardware/common/packet.h>
#include <sstmac/backends/common/sim_partition_fwd.h>
#include <sstmac/hardware/topology/topology_fwd.h>
#include <sstmac/common/sstmac_config.h>
#include <sstmac/common/stats/ftq_tag.h>
#include <sstmac/libraries/nlohmann/json.hpp>
#include <sprockit/sim_parameters_fwd.h>
#include <sprockit/debug.h>
#include <sprockit/factory.h>
#include <sprockit/errors.h>
#include <unordered_map>
#include <cmath>

//#include <sstmac/hardware/topology/get_topo.h>
/*
#if SSTMAC_INTEGRATED_SST_CORE
#include <sst/core/component.h>
#include <sst/core/event.h>
#include <sst/core/timeConverter.h>
#include <sst/core/clock.h>
using namespace SST;
#endif
*/
DeclareDebugSlot(topology)

#define top_debug(...) \
  debug_printf(sprockit::dbg::topology, __VA_ARGS__)

namespace sstmac {
namespace hw {


static const unsigned int INF(std::numeric_limits<int>::max());
//static const unsigned undefined = INF;
// K-shortest paths from here
//
//
 
class DijPath
{
public:
	std::vector<unsigned int> onePath;
	int cost;
 
	//bool operator <(const DijPath &n2);
	//bool operator ==(const DijPath &n2);
 
 
bool operator <(const DijPath &n2)
{
	return cost < n2.cost;
}

bool operator ==(const DijPath &n2)
{
	if (onePath.size() == n2.onePath.size())
	{
		for (unsigned int i = 0; i < onePath.size(); i++)
		{
			if (onePath[i] != n2.onePath[i])
				return false;
		}
 
		return true;
	}
 
	return false;
}

};
 
class K_Shortest_Path {
public:

/*	std::vector<std::vector<unsigned int>> run(
		const unsigned int kPath,					// K Path
		const std::vector<std::vector<unsigned int>>& NW,		// network
		const unsigned int src,						// source node
		const unsigned int dst);					// destination node
*/

DijPath dijkstra(
	const std::vector<std::vector<unsigned int>> &NW,
	const int src,
	const int dst
)
{
	
	unsigned int sizeNW = NW.size(); 

	std::vector<bool> visited(sizeNW); 
 
	std::vector<unsigned int> prev(sizeNW);    
 
	
	int minPos = 0; 
 
	std::vector<unsigned int> len(sizeNW);
 
	for (unsigned int i = 0; i < NW.size(); i++) 
	{
		visited[i] = false;  
		len[i] = NW[src][i];
		prev[i] = INF;
	}
 
	visited[src] = true;
 
	for (unsigned int i = 0; i < sizeNW; ++i)  
	{
		unsigned int min = INF;      
 
		for (unsigned int j = 0; j < sizeNW; ++j)
		{
			if (!visited[j] && min > len[j])
			{
				minPos = j;   
				min = len[j];
			}
		}
 
		visited[minPos] = true;
 
		for (unsigned int j = 0; j < sizeNW; ++j)
		{
			if (!visited[j] && len[j] > (len[minPos] + NW[minPos][j]))
			{
				prev[j] = minPos;
				len[j] = len[minPos] + NW[minPos][j];
			}
		}
	}
 
	unsigned int beforeVertex = dst;
	std::stack<unsigned int> st;
	while (prev[beforeVertex] != INF)
	{
		st.push(beforeVertex);
		beforeVertex = prev[beforeVertex];
	}
	st.push(beforeVertex);
 
	DijPath oneDijPath;
	oneDijPath.onePath.resize(st.size() + 1);
	oneDijPath.onePath[0] = src;
	for (unsigned int i = 1; !st.empty(); i++)
	{
		oneDijPath.onePath[i] = st.top();
		st.pop();
	}
	oneDijPath.cost = len[dst];  
	return oneDijPath;
}
 
std::vector<std::vector<unsigned int>> cutEdge(
	const std::vector<std::vector<unsigned int>>& NW,
	std::vector< DijPath> kSPCost,
	unsigned int root)
{
	std::vector<std::vector<unsigned int>>NWCopy = NW;
	for (unsigned int i = 0; i < kSPCost.size(); i++)
	{
		for (unsigned int j = 0; j < kSPCost[i].onePath.size(); j++)
		{
			if (kSPCost[i].onePath[j] == root)
			{
				unsigned int nextVertex = kSPCost[i].onePath[j + 1];
				if (j >= 1)
				{
					unsigned int beforeVertex = kSPCost[i].onePath[j - 1];
					NWCopy[root][beforeVertex] = INF;
				}
				NWCopy[root][nextVertex] = INF;  
				break;
			}
		}
	}
 
	return NWCopy;
 
}
 
//
//Yen_k-shortest-path
//
std::vector<std::vector<unsigned int>> run(
	const unsigned int kPath,					// K Path
	const std::vector<std::vector<unsigned int>>& NW,		// network
	const unsigned int src,						// source node
	const unsigned int dst)						// destination node
 
{
	std::vector<std::vector<unsigned int>>NWCopy = NW;
	std::vector< DijPath> kSPCost(1);  
	std::vector< DijPath>B;    
	DijPath newPath = dijkstra(NW, src, dst);  
	std::vector<std::vector<unsigned int>> kSP;   
	if (newPath.cost==INF)  
	{
		kSP.resize(0);
		return kSP;
	}
 
 
	kSPCost[0] = newPath; 
	std::vector<unsigned int>forwardPath;    
	int nowCost;           
 
	for (unsigned int k = 1; k < kPath; k++)  
	{
		nowCost = 0;
 
		bool flag = false;
		
		for (unsigned int i = 0; i < B.size() && kSPCost.size() < kPath&&kSPCost.size() >= k - 1; i++)
		{
 
			kSPCost.push_back(B[i]);
 
			flag = true;
 
		}
 
		if (flag) 
		{
			B.resize(0);
		}
 
 
		if (kSPCost.size() < k)
		{
			sort(kSPCost.begin(), kSPCost.end());
			for (unsigned int i = 0; i < kSPCost.size(); i++)
			{
				kSP.push_back(kSPCost[i].onePath);
			}
			return kSP;
		}
 
		forwardPath.resize(0);
 
		for (unsigned int i = 0; i < kSPCost[k - 1].onePath.size() - 1; i++) 
		{
			forwardPath.push_back(kSPCost[k - 1].onePath[i]);
			if (i != 0)
 
			{
				unsigned int forwardVertex = kSPCost[k - 1].onePath[i];
				unsigned int nextVertex = kSPCost[k - 1].onePath[i - 1];
				nowCost += NW[forwardVertex][nextVertex];
			}
 
			NWCopy = cutEdge(NW, kSPCost, kSPCost[k - 1].onePath[i]);
 
			DijPath secondPath = dijkstra(NWCopy, kSPCost[k - 1].onePath[i], dst);
			if (secondPath.cost > 100000)
			{
				continue;
			}
 
			newPath.onePath = forwardPath;
			for (unsigned int j = 1; j < secondPath.onePath.size(); j++)
			{
 
				newPath.onePath.push_back(secondPath.onePath[j]);
 
			}
 
			newPath.cost = secondPath.cost + nowCost;
 
			secondPath.onePath.resize(0);
 
			DijPath tmp;
 
			tmp.cost = newPath.cost;
 
			bool flag = true;
 
			for (unsigned int j = 0; j < kSPCost.size(); j++)
			{
 
				tmp.onePath = kSPCost[j].onePath;
				if (tmp == newPath)
				{
					flag = false;   
					break;
				}
 
			}
 
			if (flag)  
			{
				B.push_back(newPath);
			}
 
			if (kSPCost.size() >= kPath)
			{
				sort(kSPCost.begin(), kSPCost.end());
				for (unsigned int i = 0; i < kSPCost.size(); i++)
				{
					kSP.push_back(kSPCost[i].onePath);
				}
				return kSP;
			}
		}
 
	}
 
	sort(kSPCost.begin(), kSPCost.end());
	for (unsigned int i = 0; i < kSPCost.size(); i++)
	{
		kSP.push_back(kSPCost[i].onePath);
	}
	return kSP;
}


};
 
class Topology : public sprockit::printable
{
 public:
  SPKT_DECLARE_BASE(Topology)
  SPKT_DECLARE_CTOR(SST::Params&)

  struct Connection {
    SwitchId src;
    SwitchId dst;
    int src_outport;
    int dst_inport;
  };

  typedef enum {
    Ejection,
    IntraL1,
    IntraL2,
    IntraL3,
    InterL1L2,
    InterL2L3
  } port_type_t;

  typedef enum {
    plusXface = 0,
    plusYface = 1,
    plusZface = 2,
    minusXface = 3,
    minusYface = 4,
    minusZface = 5
  } vtk_face_t;

  struct InjectionPort {
    NodeId nid;
    int switch_port;
    int ep_port;
  };

  struct Rotation {
    double x[3];
    double y[3];
    double z[3];

    /**
     * @brief rotation Initialize as a 3D rotation
     * @param ux  The x component of the rotation axis
     * @param uy  The y component of the rotation axis
     * @param uz  The z component of the rotation axis
     * @param theta The angle of rotation
     */
    Rotation(double ux, double uy, double uz, double theta){
      double cosTh = cos(theta);
      double oneMinCosth = 1.0 - cosTh;
      double sinTh = sin(theta);
      x[0] = cosTh + ux*ux*oneMinCosth;
      x[1] = ux*uy*oneMinCosth - uz*sinTh;
      x[2] = ux*uz*oneMinCosth + uy*sinTh;

      y[0] = uy*ux*oneMinCosth + uz*sinTh;
      y[1] = cosTh + uy*uy*oneMinCosth;
      y[2] = uy*uz*oneMinCosth - ux*sinTh;

      z[0] = uz*ux*oneMinCosth - uy*sinTh;
      z[1] = uz*uy*oneMinCosth + ux*sinTh;
      z[2] = cosTh + uz*uz*oneMinCosth;
    }

    /**
     * @brief rotation Initialize as a 2D rotation around Z-axis
     * @param theta The angle of rotation
     */
    Rotation(double theta){
      double cosTh = cos(theta);
      double sinTh = sin(theta);
      x[0] = cosTh;
      x[1] = -sinTh;
      x[2] = 0.0;

      y[0] = sinTh;
      y[1] = cosTh;
      y[2] = 0.0;

      z[0] = 0.0;
      z[1] = 0.0;
      z[2] = 1.0;
    }

  };

  struct xyz {
    double x;
    double y;
    double z;

    xyz() : x(0), y(0), z(0) {}

    xyz(double X, double Y, double Z) :
      x(X), y(Y), z(Z){}

    double& operator[](int dim){
      switch(dim){
      case 0: return x;
      case 1: return y;
      case 2: return z;
      }
      return x;//never reached, keep compiler from warning
    }

    xyz operator+(const xyz& r) const {
      return xyz(x+r.x, y+r.y, z+r.z);
    }

    xyz rotate(const Rotation& r) const {
      xyz ret;
      ret.x += r.x[0]*x + r.x[1]*y + r.x[2]*z;
      ret.y += r.y[0]*x + r.y[1]*y + r.y[2]*z;
      ret.z += r.z[0]*x + r.z[1]*y + r.z[2]*z;
      return ret;
    }
  };

  struct VTKBoxGeometry {
    xyz size;
    xyz corner;
    Rotation rot;

    xyz vertex(int id) const {
      switch(id){
      case 0:
        return corner.rotate(rot);
      case 1:
        return xyz(corner.x,corner.y+size.y,corner.z).rotate(rot);
      case 2:
        return xyz(corner.x,corner.y,corner.z+size.z).rotate(rot);
      case 3:
        return xyz(corner.x,corner.y+size.y,corner.z+size.z).rotate(rot);
      case 4:
        return xyz(corner.x+size.x,corner.y,corner.z).rotate(rot);
      case 5:
        return xyz(corner.x+size.x,corner.y+size.y,corner.z).rotate(rot);
      case 6:
        return xyz(corner.x+size.x,corner.y,corner.z+size.z).rotate(rot);
      case 7:
        return xyz(corner.x+size.x,corner.y+size.y,corner.z+size.z).rotate(rot);
      }
      spkt_abort_printf("vertex number should be 0-7: got %d", id);
      return xyz();
    }

    VTKBoxGeometry(double xLength, double yLength, double zLength,
                 double xCorner, double yCorner, double zCorner,
                 double xAxis, double yAxis, double zAxis, double theta) :
      size(xLength,yLength,zLength),
      corner(xCorner, yCorner, zCorner),
      rot(xAxis, yAxis, zAxis, theta) {}

    VTKBoxGeometry(double xLength, double yLength, double zLength,
                 double xCorner, double yCorner, double zCorner,
                 double theta) :
      size(xLength,yLength,zLength),
      corner(xCorner, yCorner, zCorner),
      rot(theta) {}

    VTKBoxGeometry(double xLength, double yLength, double zLength,
                 double xCorner, double yCorner, double zCorner) :
      VTKBoxGeometry(xLength, yLength, zLength, xCorner, yCorner, zCorner, 0.0)
   {}

    VTKBoxGeometry(double xLength, double yLength, double zLength,
                     double xCorner, double yCorner, double zCorner,
                     const Rotation& rot) :
      size(xLength,yLength,zLength),
      corner(xCorner,yCorner,zCorner),
      rot(rot)
    {
    }

    VTKBoxGeometry get_sub_geometry(double x_start, double x_span,
                                      double y_start, double y_span,
                                      double z_start, double z_span) const {

      return VTKBoxGeometry(size.x * x_span, size.y * y_span, size.z * z_span,
                              corner.x + size.x * x_start,
                              corner.y + size.y * y_start,
                              corner.z + size.z * z_start, rot);
    }

    xyz plus_x_corner() const {
      xyz loc = corner;
      loc.x += size.x;
      return loc;
    }

    xyz plus_y_corner() const {
      xyz loc = corner;
      loc.y += size.y;
      return loc;
    }

    xyz plus_z_corner() const {
      xyz loc = corner;
      loc.z += size.z;
      return loc;
    }

    xyz center() const {
      double newX = (corner.x + size.x*0.5);
      double newY = (corner.y + size.y*0.5);
      double newZ = (corner.z + size.z*0.5);
      return xyz(newX, newY, newZ).rotate(rot);
    }

    xyz x_anchor() const {
      //return the center of the front face
      double newX = corner.x;
      double newY = corner.y + size.y*0.5;
      double newZ = corner.z + size.y*0.5;
      return xyz(newX, newY, newZ).rotate(rot);
    }

  };

  struct VTKSwitchGeometry {
    VTKBoxGeometry box;
    struct port_geometry {
      double x_size;
      double y_size;
      double z_size;
      double x_offset;
      double y_offset;
      double z_offset;
    };
    std::vector<port_geometry> ports;

    VTKBoxGeometry get_port_geometry(int port) const {
      auto& cfg = ports[port];
      return box.get_sub_geometry(cfg.x_offset, cfg.x_size,
                                  cfg.y_offset, cfg.y_size,
                                  cfg.z_offset, cfg.z_size);
    }

    VTKSwitchGeometry(double xLength, double yLength, double zLength,
                 double xCorner, double yCorner, double zCorner,
                 double theta, std::vector<port_geometry>&& ps) :
      box(xLength, yLength, zLength,xCorner,yCorner,zCorner,theta),
      ports(std::move(ps))
    {}

  };

 public:
  typedef std::unordered_map<SwitchId, Connectable*> internal_Connectable_map;
  typedef std::unordered_map<NodeId, Connectable*> end_point_Connectable_map;

 public:
  virtual ~Topology();

  virtual double portScaleFactor(uint32_t  /*addr*/, int  /*port*/) const {
    return 1.0;
  }

  /**
   * @brief connected_outports
   *        Given a 3D torus e.g., the connection vector would contain
   *        6 entries, a +/-1 for each of 3 dimensions.
   * @param src   Get the source switch in the connection
   * @param conns The set of output connections with dst SwitchId
   *              and the port numbers for each connection
   */
  virtual void connectedOutports(SwitchId src,
                     std::vector<Topology::Connection>& conns) const = 0;

  /**
     For indirect networks, this includes all switches -
     those connected directly to nodes and internal
     switches that are only a part of the network
     @return The total number of switches
  */
  virtual SwitchId numSwitches() const = 0;

  virtual SwitchId numLeafSwitches() const {
    return numSwitches();
  }

  /**
   * @brief maxSwitchId Depending on the node indexing scheme, the maximum switch id
   *  might be larger than the actual number of switches.
   * @return The max switch id
   */
  virtual SwitchId maxSwitchId() const = 0;

  virtual NodeId numNodes() const = 0;

  /**
   * @brief maxNodeId Depending on the node indexing scheme, the maximum node id
   *  might be larger than the actual number of nodes.
   * @return The max node id
   */
  virtual NodeId maxNodeId() const = 0;

  /**
   * @brief getVtkGeometry
   * @param sid
   * @return The geometry (box size, rotation, port-face mapping)
   */
  virtual VTKSwitchGeometry getVtkGeometry(SwitchId sid) const;

  virtual bool isCurvedVtkLink(SwitchId  /*sid*/, int  /*port*/) const {
    return false;
  }

  /**
   * @brief Return the maximum number of ports on any switch in the network
   * @return
   */
  virtual int maxNumPorts() const = 0;

  /**
    This gives the minimal distance counting the number of hops between switches.
    @param src. The source node.
    @param dest. The destination node.
    @return The number of hops to final destination
  */
  virtual int numHopsToNode(NodeId src, NodeId dst) const = 0;

  virtual SwitchId endpointToSwitch(NodeId) const = 0;

  /**
   * @brief outputGraphviz
   * Request to output graphviz. If file is given, output will be written there.
   * If no file is given, topology will use default path from input file.
   * If not default was given in input file, nothing will be output
   * @param file An optional file
   */
  void outputGraphviz(const std::string& file = "");

  /**
   * @brief outputXYZ
   * Request to output graphviz. If file is given, output will be written there.
   * If no file is given, topology will use default path from input file.
   * If not default was given in input file, nothing will be output
   * @param file An optional file
   */
  void outputXYZ(const std::string& file = "");

  static void outputBox(std::ostream& os,
                       const Topology::VTKBoxGeometry& box,
                       const std::string& color,
                       const std::string& alpha);

  static void outputBox(std::ostream& os,
                       const Topology::VTKBoxGeometry& box);

  void injectionPorts(NodeId nid, std::vector<InjectionPort>& ports);

  /**
     For a given input switch, return all nodes connected to it.
     This return vector might be empty if the
     switch is an internal switch not connected to any nodes
     @return The nodes connected to switch for injection
  */
  virtual void endpointsConnectedToInjectionSwitch(SwitchId swid,
                          std::vector<InjectionPort>& nodes) const = 0;

  /**
     For a given input switch, return all nodes connected to it.
     This return vector might be empty if the
     switch is an internal switch not connected to any nodes
     @return The nodes connected to switch for ejection
  */
  virtual void endpointsConnectedToEjectionSwitch(SwitchId swid,
                          std::vector<InjectionPort>& nodes) const = 0;
  /**** END PURE VIRTUAL INTERFACE *****/

  void finalizeInit(SST::Params& params){
    initHostnameMap(params);
  }

  virtual void createPartition(
    int* switch_to_lp,
    int* switch_to_thread,
    int me,
    int nproc,
    int nthread,
    int noccupied) const;

#if SSTMAC_INTEGRATED_SST_CORE
  SwitchId nodeToLogpSwitch(NodeId nid) const;

  static int nproc;
#endif


  static Topology* global() {
    return main_top_;
  }

  std::string label(uint32_t comp_id) const;

  virtual std::string switchLabel(SwitchId sid) const;

  virtual std::string nodeLabel(NodeId nid) const;

  static Topology* staticTopology(SST::Params& params);

  static void setStaticTopology(Topology* top){
    staticTopology_ = top;
  }

  //virtual void update_dataflow(int src, int dst, int size_bit);


std::pair<int, double> get_max_pos_int(int * array, int size){
    std::pair<int, double> pp;

    double max = array[0];
    int max_pos = 0;

    for (int i=1; i<size; i++){
        if(max < array[i]){
            max = array[i];
            max_pos = i;
        }
    }
    pp.first = max_pos;
    pp.second = max;
    
    return pp;
}

std::vector<std::pair<double, int>>sort_by_value(double * array, int size){
    std::vector<std::pair<double, int>> a;

    for (int i=0; i<size;i++){
        a.push_back(std::make_pair(array[i], i));
    }

    sort(a.begin(), a.end());
    return a;
}

std::pair<int, double> get_max_pos(double * array, int size){
    std::pair<int, double> pp;

    double max = array[0];
    int max_pos = 0;

    for (int i=1; i<size; i++){
        if(max < array[i]){
            max = array[i];
            max_pos = i;
        }
    }
    pp.first = max_pos;
    pp.second = max;
    
    return pp;
}

std::vector<int> find_nonzero_elements(std::vector<int> array){
    std::vector<int> nonzero_elements;

    for (int i=0; i<array.size(); i++){
        if (array[i] != 0){
            nonzero_elements.push_back(i);
        }
    }

    return nonzero_elements;
}

std::vector<int> find_zero_elements(std::vector<int> array){
    std::vector<int> zero_elements;

    for (int i=0; i<array.size(); i++){
        if (array[i] == 0){
            zero_elements.push_back(i);
        }
    }

    return zero_elements;
}

bool check_existing(int id, std::vector<int> id_queue) {
  for (int tmp: id_queue) {
    if (tmp == id) {
      return true;
    }
  }
  return false;
}

int sum_of_row(int * array, int size){
    int sum=0;
    for (int i=0; i<size; i++){
        sum += array[i];
    }
    return sum;
}

//#if 1
std::vector<int> get_topo_reconfig(std::vector<long long> traffic_matrix, int num_port, double wave_capacity){
   std::vector<int> returned_topo;
   int num_tors = sqrt(traffic_matrix.size());
   double tm[num_tors][num_tors];

   for (int i=0; i<num_tors; i++){
    for (int j=0; j<num_tors; j++){
        tm[i][j] = traffic_matrix[num_tors*i+j];
        //monitored_tm[i][j] = monitored_traffic_matrix[num_tors*i+j];
    }
   } 

   double weight_matrix[num_tors][num_tors];
   double weight_matrix_trans[num_tors][num_tors];
   int num_zero_weight_matrix[num_tors];
   int num_zero_weight_matrix_trans[num_tors];
   int topology[num_tors][num_tors];
   int topology_trans[num_tors][num_tors];

   for (int i=0; i<num_tors; i++){
    num_zero_weight_matrix[i] = 0;
    for (int j=0; j<num_tors; j++){
        weight_matrix[i][j] = tm[i][j];
        topology[i][j] = 0;
    }
   } 
   for (int i=0; i<num_tors; i++){
    num_zero_weight_matrix_trans[i] = 0;
    for (int j=0; j<num_tors; j++){
        weight_matrix_trans[j][i] = tm[i][j];
        topology_trans[i][j] = 0;
    }
   } 

   //std::cout << "num_zero_weight:";
   for (int i=0; i<num_tors; i++){
    for (int j=0; j<num_tors; j++){
        if (weight_matrix[i][j] == 0){
            num_zero_weight_matrix[i] += 1;
        }
        else{
            topology[i][j] += 1;
            topology_trans[j][i] += 1;
        }
       
    }
    //std::cout << " " << num_zero_weight_matrix[i];
   } 
   //std::cout << std::endl;

   //std::cout << "num_zero_weight_trans:";
   for (int i=0; i<num_tors; i++){
    for (int j=0; j<num_tors; j++){
        if (weight_matrix_trans[i][j] == 0){
            num_zero_weight_matrix_trans[i] += 1;
        }
    }
    //std::cout << " " << num_zero_weight_matrix_trans[i];
   } 
   //std::cout << std::endl;


   for (int i=0; i<num_tors; i++){
        std::vector<std::pair<double, int>> a;
        double combine_array[2*num_tors];
        
        for (int n=0; n<num_tors;n++){
            combine_array[n] = weight_matrix[i][n];
        }
        for (int m=0; m<num_tors;m++){
            combine_array[num_tors+m] = weight_matrix_trans[i][m];
        }


        a = sort_by_value(combine_array, 2*num_tors);

        for (int j=0; j<2*num_tors; j++){
            std::pair<double, int> w = a[2*num_tors-j-1];
            //pair<double, int> w2 = b[num_tors-j-1];
            int sum_row = sum_of_row(topology[i], num_tors);
            int sum_row_trans = sum_of_row(topology_trans[i], num_tors);
            std::pair<int, double> topo_max = get_max_pos_int(topology[i], num_tors);
            std::pair<int, double> topo_max_trans = get_max_pos_int(topology_trans[i], num_tors);


            if (w.second > num_tors-1){                          /// choose the col
                int pos = w.second-num_tors;
                int sum_row_w = sum_of_row(topology[pos], num_tors);
                int sum_row_w_trans = sum_of_row(topology_trans[pos], num_tors);
                std::pair<int, double> topo_max_w = get_max_pos_int(topology[pos], num_tors);
                std::pair<int, double> topo_max_trans_w = get_max_pos_int(topology_trans[pos], num_tors);
            /*std::cout << "update topo 1-0 with i: " << i << " j: " << j << " pos: " << pos << " value: " << topo_max.second << " " << topo_max_trans.second << " " << topo_max_w.second << " " << topo_max_trans_w.second << std::endl;
            for (int tmp_i=0; tmp_i<num_tors; tmp_i++) {
              for (int tmp_j=0; tmp_j<num_tors; tmp_j++) {
                std::cout << topology[tmp_i][tmp_j] << " ";
              }
              std::cout << std::endl;
            }*/
                if (topo_max.second == 1 && topo_max_trans.second == 1){
                    if (topo_max_w.second == 1 && topo_max_trans_w.second == 1){    
						int num_avail_link_before = num_zero_weight_matrix_trans[i];
						int num_avail_link_after = num_zero_weight_matrix_trans[pos];
                        //int num_avail_link = min(num_avail_link_before, num_avail_link_after);
                        int num_avail_link = (num_avail_link_before < num_avail_link_after)? num_avail_link_before : num_avail_link_after;
                        //int num_avail_link = num_zero_weight_matrix_trans[i];
                        if (sum_row_w + num_avail_link - 1 < num_tors && sum_row_w_trans + num_avail_link - 1 < num_tors && sum_row + num_avail_link - 1 < num_tors && sum_row_trans + num_avail_link - 1 < num_tors){
                            //if(num_avail_link > 16) {
                            //  std::cout << "There might something wrong! i: " << i << " pos: " << pos << " num_avail_link: " << num_avail_link << std::endl;
                            //}
                            topology[i][pos] += num_avail_link - 1;
                            topology[pos][i] += num_avail_link - 1;
                            topology_trans[i][pos] += num_avail_link - 1;
                            topology_trans[pos][i] += num_avail_link - 1;
            /*std::cout << "update topo 1-1 with i: " << i << " j: " << j << " pos: " << pos << " links: " << num_avail_link << std::endl;
            for (int tmp_i=0; tmp_i<num_tors; tmp_i++) {
              for (int tmp_j=0; tmp_j<num_tors; tmp_j++) {
                std::cout << topology[tmp_i][tmp_j] << " ";
              }
              std::cout << std::endl;
            }*/
                            break;
                        }
                    }
                }
            }
            else{
                int pos = w.second;
                int sum_row_w = sum_of_row(topology[pos], num_tors);
                int sum_row_w_trans = sum_of_row(topology_trans[pos], num_tors);
                std::pair<int, double> topo_max_w = get_max_pos_int(topology[pos], num_tors);
                std::pair<int, double> topo_max_trans_w = get_max_pos_int(topology_trans[pos], num_tors);
                if(topo_max.second == 1 && topo_max_trans.second ==1){
                    if (topo_max_w.second == 1 && topo_max_trans_w.second == 1){      ///choose the row
                        int num_avail_link_before = num_zero_weight_matrix[i];
			int num_avail_link_after = num_zero_weight_matrix[pos];
                        //int num_avail_link = min(num_avail_link_before, num_avail_link_after);
                        int num_avail_link = (num_avail_link_before < num_avail_link_after)?num_avail_link_before : num_avail_link_after;
                        //int num_avail_link = num_zero_weight_matrix[i];
                        if (sum_row_w + num_avail_link - 1 < num_tors && sum_row_w_trans + num_avail_link - 1 < num_tors && sum_row + num_avail_link - 1 < num_tors && sum_row_trans + num_avail_link - 1 < num_tors){
                            //if(num_avail_link > 16) {
                            //  std::cout << "There might something wrong! i: " << i << " pos: " << pos << " num_avail_link: " << num_avail_link << std::endl;
                            //}
                            topology[i][pos] += num_avail_link- 1;
                            topology[pos][i] += num_avail_link - 1;
                            topology_trans[i][pos] += num_avail_link - 1;
                            topology_trans[pos][i] += num_avail_link - 1;
                            break;
                        }
                    }
                }  
            }
            
            /**/
        }

    }

    for (int i=0; i<num_tors; i++){
        for (int j=0; j<num_tors; j++){
            returned_topo.push_back(topology[i][j]);
        }
    } 

   return returned_topo;
}

//#else

std::vector<int> get_topo_reconfig_fully(std::vector<long long> traffic_matrix, int num_port, double wave_capacity)
//std::vector<int> get_topo_reconfig(std::vector<long long> traffic_matrix, int num_port, double wave_capacity)
{
   std::vector<int> returned_topo;
   int num_tors = sqrt(traffic_matrix.size());
   double tm[num_tors][num_tors];
   //std::cout << "Get_topo_traffic_matrix: " << num_port << std::endl;
   for (int i=0; i<num_tors; i++){
    for (int j=0; j<num_tors; j++){
        tm[i][j] = ((double)traffic_matrix[num_tors*i+j])/1;
        //std::cout << tm[i][j] << ",";
    }
    //std::cout << std::endl;
   } 
   double margin = 0.05;        // spare capacity on a link should not less than the margin, which means when a channel is fully loaded, the average packet latency would be 1/margin ns.
   int num_inport[num_tors];
   int num_outport[num_tors];

   for (int i=0; i<num_tors; i++){
        num_inport[i] =  num_port*1;
        num_outport[i] = num_port*1;
    }

   double weight_matrix[num_tors][num_tors];
   int topology[num_tors][num_tors];
   int record_node[num_tors][num_tors];

   for (int i=0; i<num_tors; i++){
    for (int j=0; j<num_tors; j++){
        weight_matrix[i][j] = tm[i][j];
        topology[i][j] = 0;
        record_node[i][j] = 0;
    }
   } 

   for (int src_node=1; src_node<=num_tors; src_node++){
    for (int ii=1; ii<=num_port; ii++){
        if (ii==num_port && src_node==num_tors-1 && num_inport[num_tors-1]>0 && num_outport[src_node-1]>0){
            int dst_node = num_tors;
            if (record_node[src_node-1][dst_node-1] == 0){
                record_node[src_node-1][dst_node-1] = 1;
                record_node[dst_node-1][src_node-1] = 1;
            }
            num_inport[dst_node-1] = num_inport[dst_node-1] - 1;
            num_outport[dst_node-1] = num_outport[dst_node-1] - 1;
            num_outport[src_node-1] = num_outport[src_node-1] - 1;
            num_inport[src_node-1] = num_inport[src_node-1] - 1;
            topology[src_node-1][dst_node-1] = topology[src_node-1][dst_node-1] + 1;
            topology[dst_node-1][src_node-1] = topology[dst_node-1][src_node-1] + 1;
            weight_matrix[src_node-1][dst_node-1] = weight_matrix[src_node-1][dst_node-1] - (wave_capacity - margin);
            weight_matrix[dst_node-1][src_node-1] = weight_matrix[dst_node-1][src_node-1] - (wave_capacity - margin);
        }
        else{
            if (num_outport[src_node-1]>0 && num_inport[src_node-1]>0 && src_node != num_tors) {
                double w1[num_tors];
                double w2[num_tors];
            
                for (int i = 0; i < num_tors; i++) {
                    double num1 = weight_matrix[src_node - 1][i];
                    int num2 = num_inport[i];
                    //System.out.println(num1);
                    //System.out.println(num2);
                    w1[i] = num1 * num2;
                }
                for (int j = 0; j < num_tors; j++) {
                    double num1 = weight_matrix[j][src_node - 1];
                    int num2 = num_outport[j];
                    w2[j] = num1 * num2;
                }

                std::pair<int, double> w1_max = get_max_pos(w1, num_tors);
                std::pair<int, double> w2_max = get_max_pos(w2, num_tors);

                if (w1_max.second >= w2_max.second){
                    double w[num_tors];
                    std::copy(w1, w1+num_tors, w);
                    double inf = std::numeric_limits<double>::infinity();
                    double nega_inf = -1 * inf;

                    for (int k=0; k<num_tors; k++){
                        if (num_inport[k] == 0){
                            w[k] = nega_inf;
                        }    
                    }
                    w[src_node-1] = nega_inf;
                    std::pair<int, double> p = get_max_pos(w, num_tors);
                    int dst_node = p.first;
                    dst_node = dst_node + 1;
                    double v = p.second;
                    if (v<=0 && v != nega_inf){
                        double w[num_tors];
                        std::copy(weight_matrix[src_node-1], weight_matrix[src_node-1]+num_tors, w);
                        std::vector<int> num_inport_vector = {};
                        
                        for (int i=0; i<num_tors; i++){
                            num_inport_vector.push_back(num_inport[i]);
                        }
                        std::vector<int> nonzero_indexs = find_nonzero_elements(num_inport_vector);
                        std::vector<int> zero_indexs = find_zero_elements(num_inport_vector);

                        for (int i=0; i<nonzero_indexs.size(); i++){
                            int index_nonzero = nonzero_indexs[i];
                            w[index_nonzero] = w[index_nonzero] / num_inport[index_nonzero];
                        }

                        for (int i=0; i<zero_indexs.size(); i++){
                            int index_zero = zero_indexs[i];
                            w[index_zero] = nega_inf;
                        }
                        w[src_node-1] = nega_inf;
                        std::pair<int, double> q = get_max_pos(w, num_tors);
                        dst_node = q.first;
                        dst_node = dst_node + 1;

                        if (record_node[src_node-1][dst_node-1] == 0){
                            record_node[src_node-1][dst_node-1] = 1;
                            record_node[dst_node-1][src_node-1] = 1;
                        }
                    }

                    if (v == nega_inf){
                        continue;
                    }

                    if (record_node[src_node - 1][dst_node - 1] == 0) {
                            record_node[src_node - 1][dst_node - 1] = 1;
                            record_node[dst_node - 1][src_node - 1] = 1;
                    }
                            
                    num_inport[dst_node - 1] = num_inport[dst_node - 1] - 1;
                    num_outport[dst_node - 1] = num_outport[dst_node - 1] - 1;
                    num_outport[src_node - 1] = num_outport[src_node - 1] - 1;
                    num_inport[src_node - 1] = num_inport[src_node - 1] - 1;
                    topology[src_node - 1][dst_node - 1] = topology[src_node - 1][dst_node - 1] + 1;
                    topology[dst_node - 1][src_node - 1] = topology[dst_node - 1][src_node - 1] + 1;
                    weight_matrix[src_node - 1][dst_node - 1] = weight_matrix[src_node - 1][dst_node - 1] - (wave_capacity - margin);
                    weight_matrix[dst_node - 1][src_node - 1] = weight_matrix[dst_node - 1][src_node - 1] - (wave_capacity - margin);
                }
                else{
                    double w[num_tors];
                    std::copy(w2, w2+num_tors, w);
                    int dst_node = src_node;
                    double inf = std::numeric_limits<double>::infinity();
                    double nega_inf = -1 * inf;
                    w[dst_node-1] = nega_inf;

                    for (int k=0; k<num_tors; k++){
                        if (num_outport[k] == 0){
                            w[k] = nega_inf;
                        }    
                    }
                    
                    std::pair<int, double> p = get_max_pos(w, num_tors);
                    int temp = p.first;
                    temp = temp + 1;
                    double v = p.second;

                    if (v<=0 && v != nega_inf){
                        double w[num_tors];
                        for (int i=0; i<num_tors; i++){
                            for (int j=0; j<num_tors; j++){
                                if (j == dst_node-1){
                                    w[i] = weight_matrix[i][j];
                                }
                            }
                        }

                        std::vector<int> num_outport_vector = {};
                        for (int i=0; i<num_tors; i++){
                            num_outport_vector.push_back(num_outport[i]);
                        }
                        std::vector<int> nonzero_indexs = find_nonzero_elements(num_outport_vector);
                        std::vector<int> zero_indexs = find_zero_elements(num_outport_vector);

                        for (int i=0; i<nonzero_indexs.size(); i++){
                            int index_nonzero = nonzero_indexs[i];
                            w[index_nonzero] = w[index_nonzero] / num_outport[index_nonzero];
                        }

                        for (int i=0; i<zero_indexs.size(); i++){
                            int index_zero = zero_indexs[i];
                            w[index_zero] = nega_inf;
                        }
                        w[dst_node-1] = nega_inf;
                        std::pair<int, double> q = get_max_pos(w, num_tors);
                        temp = q.first;
                        temp = temp + 1;

                        if (record_node[temp-1][dst_node-1] == 0){
                            record_node[temp-1][dst_node-1] = 1;
                            record_node[dst_node-1][temp-1] = 1;
                        }
                    }

                    if (v == nega_inf) {
                        continue;
                    }
                    
                    if (record_node[temp - 1][dst_node - 1] == 0) {
                        record_node[temp - 1][dst_node - 1] = 1;
                        record_node[dst_node - 1][temp - 1] = 1;
                    }
                    
                    num_inport[dst_node - 1] = num_inport[dst_node - 1] - 1;
                    num_outport[dst_node - 1] = num_outport[dst_node - 1] - 1;
                    num_outport[temp - 1] = num_outport[temp - 1] - 1;
                    num_inport[temp - 1] = num_inport[temp - 1] - 1;
                    topology[temp - 1][dst_node - 1] = topology[temp - 1][dst_node - 1] + 1;
                    topology[dst_node - 1][temp - 1] = topology[dst_node - 1][temp - 1] + 1;
                    weight_matrix[temp - 1][dst_node - 1] = weight_matrix[temp - 1][dst_node - 1] - (wave_capacity - margin);
                    weight_matrix[dst_node - 1][temp - 1] = weight_matrix[dst_node - 1][temp - 1] - (wave_capacity - margin);
                }
            }
        }
   /**/
   std::cout << "topo_update: " << src_node << ", " << ii << std::endl;
   for (int i=0; i<num_tors; i++){
    for (int j=0; j<num_tors; j++){
        std::cout << topology[i][j] << " ";
    }
    std::cout << std::endl;
   }
   /**/ 
    }
   }

   //std::cout << "returned_topo: " << std::endl;
   for (int i=0; i<num_tors; i++){
    for (int j=0; j<num_tors; j++){
        //std::cout << topology[i][j] << " ";
        returned_topo.push_back(topology[i][j]);
    }
    //std::cout << std::endl;
   } 

   return returned_topo;
}
//#endif
  //virtual int current_topo();
  
  virtual void update_dataflow(int src, int dst, int size_bit)
  {
    //std::cout << "Test 0 in dataflow of topology/topology.h with src, dst, size_bit, total: " << src << ", " << dst << ", " << size_bit << ", " << data_flow_[src*number_nodes_ + dst] << std::endl;
    fflush(stdout);
    int src_pod_idx = src%number_pods_;
    int dst_pod_idx = dst%number_pods_;
    int src_col_idx = (src/number_pods_)%number_cols_;
    int dst_col_idx = (dst/number_pods_)%number_cols_;
    int src_row_idx = (src/(number_pods_*number_cols_))%number_rows_;
    int dst_row_idx = (dst/(number_pods_*number_cols_))%number_rows_;

    if (src_pod_idx != dst_pod_idx) {
      data_flow_[src_row_idx*number_cols_+src_col_idx][src_pod_idx*number_pods_ + dst_pod_idx] += size_bit;
      p_data_flow_[src_row_idx*number_cols_+src_col_idx][src_pod_idx*number_pods_ + dst_pod_idx]++;
    }
    if (src_col_idx != dst_col_idx) {
      data_flow_col_[src_row_idx*number_pods_+dst_pod_idx][src_col_idx*number_cols_ + dst_col_idx] += size_bit;
      p_data_flow_col_[src_row_idx*number_pods_+dst_pod_idx][src_col_idx*number_cols_ + dst_col_idx]++;
    }
    if (src_row_idx != dst_row_idx) {
      data_flow_row_[dst_col_idx*number_pods_+dst_pod_idx][src_row_idx*number_rows_ + dst_row_idx] += size_bit;
      p_data_flow_row_[dst_col_idx*number_pods_+dst_pod_idx][src_row_idx*number_rows_ + dst_row_idx]++;
    }
    //std::cout << "Test 1 in dataflow of topology/topology.h with total: " << data_flow_[src*number_nodes_ + dst] << std::endl;
  }

  void topo_route_update(std::vector<std::vector<int> >& new_topo, std::vector<std::vector<std::vector<int> > >& new_routing, std::vector<std::vector<long long> > traffic_flow, int num_ToRs) {
    int num_ports = num_ToRs-1;
    double wave_capacity = 10;   // Link capacity is 10G

    for (int idx=0; idx<traffic_flow.size(); idx++) {
      //std::vector<int> topo = get_topo_reconfig(traffic_flow[idx], num_ports, wave_capacity);
      std::vector<int> topo;
      if(fully_reconfig_) {
        topo = get_topo_reconfig_fully(traffic_flow[idx], num_ports, wave_capacity);
      } else {
        topo = get_topo_reconfig(traffic_flow[idx], num_ports, wave_capacity);
      }

      std::vector<std::vector<unsigned int>> topology(num_ToRs, std::vector<unsigned int>(num_ToRs, 1));

      std::vector<int> tmp_topo(num_ToRs*num_ToRs, -1);

  std::cout << "Network: " << std::endl;
  fflush(stdout);
      std::vector<int> max_bw_port(num_ToRs, -1);
      for (int i=0; i<num_ToRs; i++){
        std::vector<int> reassign_queue;
        std::vector<int> port_queue;
        int index = 0;
        int total_bw = 0;
        int last_assign = 0;
        int cur_max_bw = 0;
        for (int j=0; j<num_ToRs; j++){
            topology.at(i).at(j) = topo[i*num_ToRs+j];
  std::cout << " " << topo[i*num_ToRs+j];
  fflush(stdout);
  total_bw += topo[i*num_ToRs+j];
            //topology[i][j] = topo[i*num_ToRs+j];
            if (topology.at(i).at(j) > cur_max_bw)
            {
	      cur_max_bw = topology.at(i).at(j);
	      max_bw_port[i] = j;
            }
            if (topology.at(i).at(j) >= 1){
                topology.at(i).at(j) = 1;
            }
            if (i != j) {
              if (topo[i*num_ToRs+j] > 0) {
                port_queue.push_back(j);
                last_assign = j;
                //tmp_topo[i*num_ToRs+j] = j;
              } else {
                reassign_queue.push_back(j);
              }
            }
        }
  if (total_bw < num_ports) {
    //std::cout << "Warning!! the total bw is not enough! " << total_bw << " " << last_assign << std::endl;
    topo[i*num_ToRs+last_assign] = topo[i*num_ToRs+last_assign] + (num_ports - total_bw);
  }
  std::cout << std::endl;
  fflush(stdout);
        for(int port: port_queue) {
          tmp_topo[i*num_ToRs+port] = port;
          for (int k=1; k<topo[i*num_ToRs+port]; k++) {
            tmp_topo[i*num_ToRs+reassign_queue[index]] = port;
            index++;
          }
        }
      }
      new_topo[idx] = tmp_topo;

      for (unsigned int i(0); i < topology.size(); i++) {
		for (unsigned int j(0); j < topology.size(); j++) {
			if (topology[i][j] == 0) {
					topology[i][j] = INF;
			}
		}
      }

    /**
        std::cout << "Network: " << std::endl;

	for (unsigned int i(0); i < topology.size(); i++) {
		std::cout << "  > ";
		for (unsigned int j(0); j < topology[i].size(); j++) {
			if (topology[i][j] !=INF)
				std::cout << "   " << topology[i][j];
			else
				std::cout << "   " << 0;
		}
		std::cout << std::endl;
        }

    std::cout << "Network: " << std::endl;
    for (unsigned int i=0; i<tmp_topo.size(); i++) {
      std::cout << " " << tmp_topo[i];
      if (i%num_ToRs == num_ToRs-1) {
        std::cout << std::endl;
      }
    }
    /**/

      unsigned int kPath = 3;
  //std::cout << std::endl << "K-Shortest Path (" << kPath << ")" << std::endl;
      for (int i=0; i<num_ToRs; i++){
        for (int j=0; j<num_ToRs; j++){
	    new_routing[idx][i*num_ToRs+j].clear();
            if (i != j){
                //if(fully_reconfig_) {
	            std::vector<std::vector<unsigned int>> kSP = KSP_.run(kPath, topology, i, j);
  //std::cout << "  > Number of path: " << kSP.size() << std::endl;
                    for (int k=0; k<kSP.size(); k++) {
		            /**
                            std::cout << "    >> " << (k + 1) << ":";
		            for (unsigned int m(0); m < kSP[k].size(); m++) {
			            std::cout << "   " << kSP[k][m];
		            }
		            std::cout << std::endl;
                            /**/
			if (kSP[k].size() == 2) {
			  new_routing[idx][i*num_ToRs+j].push_back(kSP[k][1]);
                          break;
                        }
                        if (!check_existing(kSP[k][1], new_routing[idx][i*num_ToRs+j])) {

			    new_routing[idx][i*num_ToRs+j].push_back(kSP[k][1]);
			  /**
			  if (kSP[k][1] == max_bw_port[i]) {
			    //new_routing[idx][i*num_ToRs+j].clear();
			    new_routing[idx][i*num_ToRs+j].insert(new_routing[idx][i*num_ToRs+j].begin()+0, kSP[k][1]);
			    //break;
			  } else {
			    new_routing[idx][i*num_ToRs+j].push_back(kSP[k][1]);
			  }
			  /**/
                        }
                    }
                /**} else {
		    if(topo[i*num_ToRs+j] >= 1) {
                      new_routing[idx][i*num_ToRs+j].push_back(j);
                    }
                    if(j != max_bw_port[i]){
                      new_routing[idx][i*num_ToRs+j].push_back(max_bw_port[i]);
                    }
                }/**/
            }
        }
      }
      /**
      for (int i=0; i<new_routing[idx].size(); i++) {
        std::cout << "src: " << i/num_ToRs << " dst: " << i%num_ToRs;
        for (int j=0; j<new_routing[idx][i].size(); j++) {
          std::cout << " " << new_routing[idx][i][j];
        }
        std::cout << std::endl;
      }
      /**/
    }

  }

  virtual void updateRouting(int aid, std::string file_path, std::vector<std::vector<int> >& reconfig_info, std::vector<std::vector<int> >& config_info, std::vector<int>& remap_id, std::vector< std::vector<int> >& waves_group, int node_id) {
  //need to modify when multiple applications are running!!
  if (app_topo_done_.find(aid) == app_topo_done_.end())
  {
    app_topo_done_[aid] = true;
    std::ifstream dm_file;
    std::string file_data;
    dm_file.open(file_path);

    std::cout << "Test in updateRouting of topology.h with " << node_id << " " << aid << " " << file_path << std::endl;
  std::vector<std::vector<long long> > tmp_data_flow;//rows * cols * (nodes * nodes)
  //std::vector<std::vector<long long> > tmp_data_double;//rows * cols * (nodes * nodes)
  std::vector<std::vector<long long> > tmp_data_flow_col;//rows * nodes * (cols * cols)
  std::vector<std::vector<long long> > tmp_data_flow_row;//cols * nodes * (rows * rows)
    tmp_data_flow.resize(number_rows_*number_cols_, std::vector<long long>(number_pods_*number_pods_, 0));
    tmp_data_flow_col.resize(number_rows_*number_pods_, std::vector<long long>(number_cols_*number_cols_, 0));
    tmp_data_flow_row.resize(number_cols_*number_pods_, std::vector<long long>(number_rows_*number_rows_, 0));

    //get the data from file
    if (number_pods_ > 1) {
      for (int j=0; j<tmp_data_flow.size(); j++) {
        for (int i = 0; i < number_pods_*number_pods_; i++) {
          getline(dm_file, file_data, ' ');
          double tmp_double = std::stod(file_data);
          tmp_data_flow[j][i] = (long long) tmp_double;
          if (tmp_double > 0 && tmp_double < 1) {
            tmp_data_flow[j][i] = 1;
          }
          std::cout << "index: " << i << " value: " << tmp_data_flow[j][i] << " " << file_data << " " << std::stoll(file_data) << std::endl;
        }
      }
    }
    if (number_cols_ > 1) {
      for (int j=0; j<tmp_data_flow_col.size(); j++) {
        for (int i = 0; i < number_cols_*number_cols_; i++) {
          getline(dm_file, file_data, ' ');
          tmp_data_flow_col[j][i] = std::stoll(file_data);
        }
      }
    }
    if (number_rows_ > 1) {
      for (int j=0; j<tmp_data_flow_row.size(); j++) {
        for (int i = 0; i < number_rows_*number_rows_; i++) {
          getline(dm_file, file_data, ' ');
          tmp_data_flow_row[j][i] = std::stoll(file_data);
        }
      }
    }
    //using the algorithm to do the reconfig
    /**/
      if (number_pods_ > 1) {
        topo_route_update(pod_topo_, pod_routing_, tmp_data_flow, number_pods_);
      }
      if (number_cols_ > 1) {
        topo_route_update(col_topo_, col_routing_, tmp_data_flow_col, number_cols_);
      }
      if (number_rows_ > 1) {
        topo_route_update(row_topo_, row_routing_, tmp_data_flow_row, number_rows_);
      }
    /**/

    dm_file.close();
  }
  #if 1
    int src_pod_idx = node_id%number_pods_;
    int src_col_idx = (node_id/number_pods_)%number_cols_;
    int src_row_idx = (node_id/(number_pods_*number_cols_))%number_rows_;

    /**/
    for (int i=0; i<port_pods_; i++) {
      remap_id[i] = pod_topo_[src_row_idx*number_cols_+src_col_idx][src_pod_idx*number_pods_+i];
    }
    for (int i=0; i<port_cols_; i++) {
      //remap_id[port_pods_+i] = port_pods_+col_topo_[src_row_idx*number_pods_+src_pod_idx][src_col_idx*number_cols_+i];
      remap_id[port_pods_+i] = ((col_topo_[src_row_idx*number_pods_+src_pod_idx][src_col_idx*number_cols_+i]>=0)?port_pods_:0)+col_topo_[src_row_idx*number_pods_+src_pod_idx][src_col_idx*number_cols_+i];
    }
    for (int i=0; i<port_rows_; i++) {
      //remap_id[port_pods_+port_cols_+i] = port_pods_+port_cols_+row_topo_[src_col_idx*number_pods_+src_pod_idx][src_row_idx*number_rows_+i];
      remap_id[port_pods_+port_cols_+i] = ((row_topo_[src_col_idx*number_pods_+src_pod_idx][src_row_idx*number_rows_+i]>=0)?(port_pods_+port_cols_):0)+row_topo_[src_col_idx*number_pods_+src_pod_idx][src_row_idx*number_rows_+i];
    }

    for (int i=0; i<waves_group.size(); i++) {
      waves_group[i].clear();
    }

    //std::cout << "remap_id: " << node_id;
    for (int i=0;  i<remap_id.size(); i++) {
      //std::cout << " " << remap_id[i];
      if (remap_id[i] >= 0) {
        waves_group[remap_id[i]].push_back(i);
      }
    }
    //std::cout << std::endl;
    int nic_avail_port = 0;
    for (int i=0; i<waves_group.size(); i++) {
      if(waves_group[i].size() > 0) {
        nic_avail_port++;
      }
    }
    std::cout << "Statistic NIC " << node_id << " port: " << nic_avail_port << std::endl;

    /**/
    for (int i=0; i<port_pods_; i++) {
      reconfig_info[i] = pod_routing_[src_row_idx*number_cols_+src_col_idx][src_pod_idx*number_pods_+i];
    }
    for (int i=0; i<port_cols_; i++) {
      reconfig_info[port_pods_+i] = col_routing_[src_row_idx*number_pods_+src_pod_idx][src_col_idx*number_cols_+i];
      for (int j = 0; j < reconfig_info[port_pods_+i].size(); j++) {
        reconfig_info[port_pods_+i][j] += port_pods_;
      }
    }
    for (int i=0; i<port_rows_; i++) {
      reconfig_info[port_pods_+port_cols_+i] = row_routing_[src_col_idx*number_pods_+src_pod_idx][src_row_idx*number_rows_+i];
      for (int j = 0; j < reconfig_info[port_pods_+port_cols_+i].size(); j++) {
        reconfig_info[port_pods_+port_cols_+i][j] += port_cols_+port_pods_;
      }
    }
    for (int i=0; i<config_info.size(); i++) {
      config_info[i].clear();
    }

    //std::cout << "reconfig_info: " << node_id << std::endl;
    for (int i=0; i<reconfig_info.size(); i++) {
      //std::cout << "dst: " << i;
      for (int tmp : reconfig_info[i]) {
        //std::cout << " " << tmp;
        config_info[tmp].push_back(i);
      }
      //std::cout << std::endl;
    }
  #endif

  }


  virtual void current_topo(std::vector<std::vector<int> >& reconfig_info, std::vector<std::vector<int> >& config_info, std::vector<int>& remap_id, std::vector< std::vector<int> >& waves_group, int node_id)
  {
    int src_pod_idx = node_id%number_pods_;
    int src_col_idx = (node_id/number_pods_)%number_cols_;
    int src_row_idx = (node_id/(number_pods_*number_cols_))%number_rows_;

    if (!topo_update_) {

      /**/
     if (!only_observe_) {
      if (number_pods_ > 1) {
        topo_route_update(pod_topo_, pod_routing_, data_flow_, number_pods_);
      }
      if (number_cols_ > 1) {
        topo_route_update(col_topo_, col_routing_, data_flow_col_, number_cols_);
      }
      if (number_rows_ > 1) {
        topo_route_update(row_topo_, row_routing_, data_flow_row_, number_rows_);
      }
     }
      /**/

      /**/
     if (number_pods_ > 1) {
      for (int j=0; j<data_flow_.size(); j++) {
        std::vector<long long int> tmp_src;
        std::vector<long long int> tmp_dst;
        tmp_src.resize(number_pods_, 0);
        tmp_dst.resize(number_pods_, 0);
        std::cout << "Dataflow_pod_collect: " << j;
        for (int i = 0; i < number_pods_*number_pods_; i++) {
          std::cout << " " << data_flow_[j][i];
          tmp_src[i/number_pods_] += data_flow_[j][i];
          tmp_dst[i%number_pods_] += data_flow_[j][i];
          data_flow_[j][i] = 0;
        }
        std::cout << std::endl;
        std::cout << "Dataflow_pod_src: " << j;
        for (int i=0; i < number_pods_; i++)
        {
          std::cout << " " << tmp_src[i];
        }
        std::cout << std::endl;
        std::cout << "Dataflow_pod_dst:" << j;
        for (int i=0; i < number_pods_; i++)
        {
          std::cout << " " << tmp_dst[i];
        }
        std::cout << std::endl;

        tmp_src.clear();
        tmp_dst.clear();
        tmp_src.resize(number_pods_, 0);
        tmp_dst.resize(number_pods_, 0);
        std::cout << "Packet_pod_collect: " << j;
        for (int i = 0; i < number_pods_*number_pods_; i++) {
          std::cout << " " << p_data_flow_[j][i];
          tmp_src[i/number_pods_] += p_data_flow_[j][i];
          tmp_dst[i%number_pods_] += p_data_flow_[j][i];
          p_data_flow_[j][i] = 0;
        }
        std::cout << std::endl;
        std::cout << "Packet_pod_src: " << j;
        for (int i=0; i < number_pods_; i++)
        {
          std::cout << " " << tmp_src[i];
        }
        std::cout << std::endl;
        std::cout << "Packet_pod_dst:" << j;
        for (int i=0; i < number_pods_; i++)
        {
          std::cout << " " << tmp_dst[i];
        }
        std::cout << std::endl;
      }
     }

     if (number_cols_ > 1) {
      for (int j=0; j<data_flow_col_.size(); j++) {
        std::vector<long long int> tmp_src;
        std::vector<long long int> tmp_dst;
        tmp_src.resize(number_cols_, 0);
        tmp_dst.resize(number_cols_, 0);
        std::cout << "Dataflow_col_collect: " << j;
        for (int i = 0; i < number_cols_*number_cols_; i++) {
          std::cout << " " << data_flow_col_[j][i];
          tmp_src[i/number_cols_] += data_flow_col_[j][i];
          tmp_dst[i%number_cols_] += data_flow_col_[j][i];
          data_flow_col_[j][i] = 0;
        }
        std::cout << std::endl;
        std::cout << "Dataflow_col_src: " << j;
        for (int i=0; i < number_cols_; i++)
        {
          std::cout << " " << tmp_src[i];
        }
        std::cout << std::endl;
        std::cout << "Dataflow_col_dst:" << j;
        for (int i=0; i < number_cols_; i++)
        {
          std::cout << " " << tmp_dst[i];
        }
        std::cout << std::endl;

        tmp_src.clear();
        tmp_dst.clear();
        tmp_src.resize(number_cols_, 0);
        tmp_dst.resize(number_cols_, 0);
        std::cout << "Packet_col_collect: " << j;
        for (int i = 0; i < number_cols_*number_cols_; i++) {
          std::cout << " " << p_data_flow_col_[j][i];
          tmp_src[i/number_cols_] += p_data_flow_col_[j][i];
          tmp_dst[i%number_cols_] += p_data_flow_col_[j][i];
          p_data_flow_col_[j][i] = 0;
        }
        std::cout << std::endl;
        std::cout << "Packet_col_src: " << j;
        for (int i=0; i < number_cols_; i++)
        {
          std::cout << " " << tmp_src[i];
        }
        std::cout << std::endl;
        std::cout << "Packet_col_dst:" << j;
        for (int i=0; i < number_cols_; i++)
        {
          std::cout << " " << tmp_dst[i];
        }
        std::cout << std::endl;
      }
     }

     if (number_rows_ > 1) {
      for (int j=0; j<data_flow_row_.size(); j++) {
        std::vector<long long int> tmp_src;
        std::vector<long long int> tmp_dst;
        tmp_src.resize(number_rows_, 0);
        tmp_dst.resize(number_rows_, 0);
        std::cout << "Dataflow_row_collect: " << j;
        for (int i = 0; i < number_rows_*number_rows_; i++) {
          std::cout << " " << data_flow_row_[j][i];
          tmp_src[i/number_rows_] += data_flow_row_[j][i];
          tmp_dst[i%number_rows_] += data_flow_row_[j][i];
          data_flow_row_[j][i] = 0;
        }
        std::cout << std::endl;
        std::cout << "Dataflow_row_src: " << j;
        for (int i=0; i < number_rows_; i++)
        {
          std::cout << " " << tmp_src[i];
        }
        std::cout << std::endl;
        std::cout << "Dataflow_row_dst:" << j;
        for (int i=0; i < number_rows_; i++)
        {
          std::cout << " " << tmp_dst[i];
        }
        std::cout << std::endl;

        tmp_src.clear();
        tmp_dst.clear();
        tmp_src.resize(number_rows_, 0);
        tmp_dst.resize(number_rows_, 0);
        std::cout << "Packet_row_collect: " << j;
        for (int i = 0; i < number_rows_*number_rows_; i++) {
          std::cout << " " << p_data_flow_row_[j][i];
          tmp_src[i/number_rows_] += p_data_flow_row_[j][i];
          tmp_dst[i%number_rows_] += p_data_flow_row_[j][i];
          p_data_flow_row_[j][i] = 0;
        }
        std::cout << std::endl;
        std::cout << "Packet_row_src: " << j;
        for (int i=0; i < number_rows_; i++)
        {
          std::cout << " " << tmp_src[i];
        }
        std::cout << std::endl;
        std::cout << "Packet_row_dst:" << j;
        for (int i=0; i < number_rows_; i++)
        {
          std::cout << " " << tmp_dst[i];
        }
        std::cout << std::endl;

      }
     }
      /**/

      topo_update_ = true;
    }
    if (num_nic_update_ < number_nodes_ - 1) {
      num_nic_update_++;
    } else {
      num_nic_update_ = 0;
      topo_update_ = false;
    }

   if (!only_observe_) {
    /**/
    for (int i=0; i<port_pods_; i++) {
      remap_id[i] = pod_topo_[src_row_idx*number_cols_+src_col_idx][src_pod_idx*number_pods_+i];
    }
    for (int i=0; i<port_cols_; i++) {
      //remap_id[port_pods_+i] = port_pods_+col_topo_[src_row_idx*number_pods_+src_pod_idx][src_col_idx*number_cols_+i];
      remap_id[port_pods_+i] = ((col_topo_[src_row_idx*number_pods_+src_pod_idx][src_col_idx*number_cols_+i]>=0)?port_pods_:0)+col_topo_[src_row_idx*number_pods_+src_pod_idx][src_col_idx*number_cols_+i];
    }
    for (int i=0; i<port_rows_; i++) {
      //remap_id[port_pods_+port_cols_+i] = port_pods_+port_cols_+row_topo_[src_col_idx*number_pods_+src_pod_idx][src_row_idx*number_rows_+i];
      remap_id[port_pods_+port_cols_+i] = ((row_topo_[src_col_idx*number_pods_+src_pod_idx][src_row_idx*number_rows_+i]>=0)?(port_pods_+port_cols_):0)+row_topo_[src_col_idx*number_pods_+src_pod_idx][src_row_idx*number_rows_+i];
    }

    for (int i=0; i<waves_group.size(); i++) {
      waves_group[i].clear();
    }

    //std::cout << "remap_id: " << node_id;
    for (int i=0;  i<remap_id.size(); i++) {
      //std::cout << " " << remap_id[i];
      if (remap_id[i] >= 0) {
        waves_group[remap_id[i]].push_back(i);
      }
    }
    //std::cout << std::endl;
    int nic_avail_port = 0;
    for (int i=0; i<waves_group.size(); i++) {
      if(waves_group[i].size() > 0) {
        nic_avail_port++;
      }
    }
    std::cout << "Statistic NIC " << node_id << " port: " << nic_avail_port << std::endl;

    /**/
    for (int i=0; i<port_pods_; i++) {
      reconfig_info[i] = pod_routing_[src_row_idx*number_cols_+src_col_idx][src_pod_idx*number_pods_+i];
    }
    for (int i=0; i<port_cols_; i++) {
      reconfig_info[port_pods_+i] = col_routing_[src_row_idx*number_pods_+src_pod_idx][src_col_idx*number_cols_+i];
      for (int j = 0; j < reconfig_info[port_pods_+i].size(); j++) {
        reconfig_info[port_pods_+i][j] += port_pods_;
      }
    }
    for (int i=0; i<port_rows_; i++) {
      reconfig_info[port_pods_+port_cols_+i] = row_routing_[src_col_idx*number_pods_+src_pod_idx][src_row_idx*number_rows_+i];
      for (int j = 0; j < reconfig_info[port_pods_+port_cols_+i].size(); j++) {
        reconfig_info[port_pods_+port_cols_+i][j] += port_cols_+port_pods_;
      }
    }
    for (int i=0; i<config_info.size(); i++) {
      config_info[i].clear();
    }

    //std::cout << "reconfig_info: " << node_id << std::endl;
    for (int i=0; i<reconfig_info.size(); i++) {
      //std::cout << "dst: " << i;
      for (int tmp : reconfig_info[i]) {
        //std::cout << " " << tmp;
        config_info[tmp].push_back(i);
      }
      //std::cout << std::endl;
    }
   }
    //return num_nic_update_;
  }

  virtual CartesianTopology* cartTopology() const;

  NodeId nodeNameToId(const std::string& name) const;

  virtual SwitchId switchNameToId(std::string name) const {
    std::size_t pos = name.find("switch");
    if (pos != 0)
      throw sprockit::InputError("topology: switch name should be switch<n>");
    std::string number(name,6);
    SwitchId id;
    try {
      id = stoi(number);
    }
    catch(...) {
      throw sprockit::InputError("topology: switch name should be switch<n>");
    }
    return id;
  }

  virtual std::string nodeIdToName(NodeId id);

  virtual std::string switchIdToName(SwitchId id) const {
    return std::string("switch") + std::to_string(id);
  }

  virtual std::string portTypeName(SwitchId  /*sid*/, int  /*port*/) const {
    return "network";
  }

  void dumpPorts();

  static void clearStaticTopology(){
    if (staticTopology_) delete staticTopology_;
    staticTopology_ = nullptr;
  }

  static std::string getPortNamespace(int port);

  const nlohmann::json& getRoutingTable(SwitchId sid) const {
    std::string name = switchIdToName(sid);
    return routing_tables_.at(name).at("routes");
  }

 protected:
  Topology(SST::Params& params);

  virtual void initHostnameMap(SST::Params& params);

  virtual void portConfigDump(const std::string& dumpFile);

 protected:
  static Topology* main_top_;
  std::unordered_map<std::string,NodeId> idmap_;
  std::vector<std::string> hostmap_;

 private:
  static Topology* staticTopology_;
  bool topo_update_;
  bool only_observe_;
  bool fully_reconfig_;
  int num_nic_update_;

  std::string dot_file_;
  std::string xyz_file_;
  std::string dump_file_;
  nlohmann::json routing_tables_;

  std::map<int, bool> app_topo_done_;

  std::vector<int> geometry_;
  int number_nodes_;
  int number_pods_;
  int number_cols_;
  int number_rows_;
  int port_pods_;
  int port_cols_;
  int port_rows_;
  //std::vector<long long> data_flow_;
  std::vector<std::vector<long long> > data_flow_;//rows * cols * (nodes * nodes)
  std::vector<std::vector<long long> > data_flow_col_;//rows * nodes * (cols * cols)
  std::vector<std::vector<long long> > data_flow_row_;//cols * nodes * (rows * rows)

  std::vector<std::vector<long long> > p_data_flow_;//rows * cols * (nodes * nodes)
  std::vector<std::vector<long long> > p_data_flow_col_;//rows * nodes * (cols * cols)
  std::vector<std::vector<long long> > p_data_flow_row_;//cols * nodes * (rows * rows)

  std::vector<std::vector<int> > pod_topo_;
  std::vector<std::vector<int> > col_topo_;
  std::vector<std::vector<int> > row_topo_;

  std::vector<std::vector<std::vector<int> > > pod_routing_;
  std::vector<std::vector<std::vector<int> > > col_routing_;
  std::vector<std::vector<std::vector<int> > > row_routing_;

  K_Shortest_Path KSP_;

  //
  //SST::Clock::Handler<Topology>* clockHandler;
};

static inline std::ostream& operator<<(std::ostream& os, const Topology::xyz& v) {
  os << v.x << "," << v.y << "," << v.z;
  return os;
}

}
}



#endif
