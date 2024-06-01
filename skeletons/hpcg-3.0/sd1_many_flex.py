
import sst

verbose = 0

DEBUG_L1 = 0
DEBUG_MEM = 0
DEBUG_LEVEL = 10

class Params(dict):
    def __missing__(self, key):
        print("Please enter %s: "%key)
        val = input()
        self[key] = val
        return val
    def subset(self, keys, optKeys = []):
        ret = dict((k, self[k]) for k in keys)
        #ret.update(dict((k, self[k]) for k in (optKeys and self)))
        for k in optKeys:
            if k in self:
                ret[k] = self[k]
        return ret
    def subsetWithRename(self, keys):
        ret = dict()
        for k,nk in keys:
            if k in self:
                ret[nk] = self[k]
        return ret
    # Needed to avoid asking for input when a key isn't present
#    def optional_subset(self, keys):
#        return

_params = Params()
debug = 0

_params["num_pods"] = 16
_params["num_cols"] = 1
_params["num_rows"] = 1
_params["num_core"] = 4

_params["pod"] = 0
if _params["num_pods"] > 1:
    _params["pod"] = _params["num_pods"]

_params["col"] = 0
if _params["num_cols"] > 1:
    _params["col"] = _params["num_cols"]

_params["row"] = 0
if _params["num_rows"] > 1:
    _params["row"] = _params["num_rows"]

_params["num_ports"] = _params["pod"] + _params["col"] + _params["row"]

_params["link_lat"] = "20ns"

def macroNode(nID):
    node = sst.Component("node.%d"%nID, "macro.simple_node")
    node.addParam("id", nID)
    node.addParam("nic.id", nID)
    node.addParam("mem_capacity", 4)
    node.addParam("storage_size", 4)
    node.addParams({
      'name' : 'simple',

      #'app1.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app1.argv' : '-s 120 -i 96',
      #'app1.launch_cmd' : 'aprun -n 125000 -N 7813',
      #'app1.launch_cmd' : 'aprun -n 64000 -N 4000',
      #'app1.launch_cmd' : 'aprun -n 32768 -N 2048',
      #'app1.launch_cmd' : 'aprun -n 15625 -N 977',
      #'app1.launch_cmd' : 'aprun -n 8000 -N 500',
      #'app1.launch_cmd' : 'aprun -n 64 -N 4',

      'app1.exe' : './runhpcg-3.0',
      'app1.argv' : '32 32 32',
      #'app1.argv' : '64 64 64',
      #'app1.argv' : '128 128 128',
      #'app1.argv' : '256 128 128',
      #'app1.argv' : '256 256 256',
      #'app1.load_reconfig' : 'true',
      'app1.load_reconfig' : 'false',

      #'nic.double_wave' : 'true',
      'nic.double_wave' : 'false',

      #'app1.exe' : '../traffic/runtraffic',
      #'app1.argv' : '4 2 37 42',
      #'app1.argv' : '128 128 37 42',
      #'app1.argv' : '32 32 37 42',

      #'app1.launch_cmd' : 'aprun -n 2048 -N 128',
      #'app1.launch_cmd' : 'aprun -n 131072 -N 8192',
      #'app1.launch_cmd' : 'aprun -n 65536 -N 4096',
      #'app1.launch_cmd' : 'aprun -n 32768 -N 2048',
      #'app1.launch_cmd' : 'aprun -n 16384 -N 1024',
      #'app1.launch_cmd' : 'aprun -n 8192 -N 512',
      #'app1.launch_cmd' : 'aprun -n 256 -N 16',
      #'app1.launch_cmd' : 'aprun -n 128 -N 8',
      #'app1.launch_cmd' : 'aprun -n 64 -N 4',
      'app1.launch_cmd' : 'aprun -n 32 -N 2',
      #'app1.launch_cmd' : 'aprun -n 16 -N 2',
      #'app1.launch_cmd' : 'aprun -n 16 -N 1',
      'app1.allocation' : 'coordinate',
      'app1.mem_size' : '3',
      'app1.storage_size' : '3',
      'app1.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app2.exe' : '../traffic/runtraffic',
      #'app2.launch_cmd' : 'aprun -n 8 -N 2',
      #'app2.allocation' : 'coordinate',
      #'app2.mem_size' : '1',
      #'app2.storage_size' : '1',
      #'app2.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',
      #'app2.argv' : '4 2 37 42',
      #'app2.allocation' : 'first_available',
      #'app2.indexing' : 'coordinate',
      #'app2.coordinate_file' : './coord1.txt',

      'memory.mtu' : '1000GB',
      'memory.latency' : '0ns',
      'memory.name' : 'logp',
      'memory.bandwidth' : '100GB/s',

      'nic.name' : 'flex',
      'nic.num_ports' : '16',
      'nic.pod' : '16',
      'nic.col' : '0',
      'nic.row' : '0',
      #'nic.num_core' : '128',
      'nic.num_core' : '8192',
      #'nic.num_core' : '4096',
      #'nic.num_core' : '2048',
      #'nic.num_core' : '1024',
      'nic.injection.latency' : '0ns',
      'nic.injection.bandwidth' : '10GB/s',
      #'nic.injection.bandwidth' : '20GB/s',
      #'nic.injection.bandwidth' : '100GB/s',
      'nic.update_freq' : '10Hz',

      'topology.auto' : 'true',
      'topology.only_observe' : 'true',
      #'topology.geometry' : '[2,2,2]',
      'topology.geometry' : '[1,1,16]',
      'topology.name' : 'torus',

      'proc.frequency' : '2.1Ghz',
      #64 cores / 2 nodes / 64 blades
      #'proc.ncores' : '128',
      'proc.ncores' : '8192',
      #'proc.ncores' : '4096',
      #'proc.ncores' : '2048',
      #'proc.ncores' : '1024',
      #'interconnect.topology.geometry' : '[2,2,2]',
      'interconnect.topology.geometry' : '[1,1,16]',

      'interconnect.topology.auto' : 'true',
      'interconnect.topology.name' : 'torus',
      #'auto' : 'true',
      #'geometry' : '[2,2,2]',

      #'name' : 'torus'
    })
    return node

switches = dict()
def getSwitch(switchType, idxF, idxS):
    name = "switch.%s_%d_%d"%(switchType, idxF, idxS)
    if name not in switches:
        switches[name] = sst.Component(name, "macro.flex_switch")
        switches[name].addParam("num_waves", _params[switchType])
    return switches[name]

links = dict()
def getLink(switchType, idxF, idxS, idxP, idxW):
    name = "link.%s_%d_%d_%d_%d"%(switchType, idxF, idxS, idxP, idxW)
    if name not in links:
        links[name] = sst.Link(name)
    return links[name]

for i in range(_params["num_pods"]):
    for j in range(_params["num_cols"]):
        if _params["row"] > 0:
            switch = getSwitch("row", j, i)
            for p_j in range(_params["row"]):
                for w_j in range(_params["row"]):
                    switch.addLink(getLink("row", j, i, p_j, w_j), "port_%d_%d"%(p_j, w_j), _params["link_lat"])

    for k in range(_params["num_rows"]):
        if _params["col"] > 0:
            switch = getSwitch("col", k, i)
            for p_k in range(_params["col"]):
                for w_k in range(_params["col"]):
                    switch.addLink(getLink("col", k, i, p_k, w_k), "port_%d_%d"%(p_k, w_k), _params["link_lat"])

for i in range(_params["num_rows"]):
    for j in range(_params["num_cols"]):
        if _params["pod"] > 0:
            switch = getSwitch("pod", i, j)
            print("Test with %d %d"%(i,j))
            for p_k in range(_params["num_pods"]):
                print("Test with %d %d %d"%(i,j, p_k))
                ep = macroNode(i*_params["num_cols"]+j*_params["num_pods"]+p_k)
                for w_k in range(_params["num_pods"]):
                    switch.addLink(getLink("pod", i, j, p_k, w_k), "port_%d_%d"%(p_k, w_k), _params["link_lat"])
                    ep.addLink(getLink("pod", i, j, p_k, w_k), "pod_port_%d"%(w_k), _params["link_lat"])
                for w_k in range(_params["row"]):
                    ep.addLink(getLink("row", j, p_k, i, w_k), "row_port_%d"%(w_k), _params["link_lat"])
                for w_k in range(_params["col"]):
                    ep.addLink(getLink("col", i, p_k, j, w_k), "col_port_%d"%(w_k), _params["link_lat"])

