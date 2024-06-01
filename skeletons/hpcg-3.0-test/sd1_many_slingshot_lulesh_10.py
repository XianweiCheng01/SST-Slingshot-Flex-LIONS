
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
#_params["num_pods"] = 4
_params["num_cols"] = 1
_params["num_rows"] = 1
_params["ports_per_global_link"] = 4
#_params["ports_per_global_link"] = 2
#_params["ports_per_global_link"] = 8
#_params["ports_per_global_link"] = 1
_params["opt_link_latency"] = "200ns"
#_params["opt_link_bandwidth"] = "12.5GB/s"

_params["num_nodes_per_rack"] = 128
#_params["num_nodes_per_rack"] = 4
#_params["num_nodes_per_rack"] = 1
_params["ports_per_nodes"] = 4
_params["ele_link_latency"] = "150ns"
#_params["ele_link_bandwidth"] = "12.5GB/s"

#_params["switch_forward"] = "12.5GB/s"
_params["switch_forward"] = "5GB/s"
_params["num_switches_per_rack"] = 32
_params["inter_switch_latency"] = "150ns"
_params["switch_latency"] = "350ns"
#25GB/s 700ns --> 0.1GB/s 
#_params["routing_latency"] = "175000ns"
#_params["routing_latency"] = "8500ns"
_params["routing_latency"] = "0ns"
#_params["switch_bandwidth"] = "12.5GB/s"
_params["switch_bandwidth"] = "5GB/s"

_params["nic_latency"] = "100ns"
#_params["nic_bandwidth"] = "12.5GB/s"
_params["nic_bandwidth"] = "5GB/s"

_params["pod"] = 0
if _params["num_pods"] > 1:
    _params["pod"] = _params["num_pods"] - 1

_params["col"] = 0
if _params["num_cols"] > 1:
    _params["col"] = _params["num_cols"] - 1

_params["row"] = 0
if _params["num_rows"] > 1:
    _params["row"] = _params["num_rows"] - 1

_params["num_local_ports_per_switch"] = _params["num_nodes_per_rack"]/_params["num_switches_per_rack"]*_params["ports_per_nodes"]

_params["num_ports_per_switch"] = 64
_params["num_nodes_per_switch"] = _params["num_nodes_per_rack"]/_params["num_switches_per_rack"]

_params["num_ports_pod"] = (_params["num_pods"] + _params["num_switches_per_rack"] - 1)/_params["num_switches_per_rack"]
_params["num_ports_col"] = (_params["num_cols"] + _params["num_switches_per_rack"] - 1)/_params["num_switches_per_rack"]
_params["num_ports_row"] = (_params["num_rows"] + _params["num_switches_per_rack"] - 1)/_params["num_switches_per_rack"]

#_params["num_ports"] = _params["pod"] + _params["col"] + _params["row"]

#_params["link_lat"] = "0ns"
#_params["link_lat"] = "40ns"
#_params["link_lat"] = "80ns"
#_params["link_lat"] = "160ns"
#_params["link_lat"] = "400ns"

sst.setStatisticLoadLevel(3)
sst.setStatisticOutput("sst.statOutputCSV", {"filepath" : "./TestOutput_lulesh_fully_reconfig.csv","separator" : "," } )

def macroNode(nID):
    node = sst.Component("node.%d"%nID, "macro.simple_node")
    node.addParam("id", nID)
    node.addParam("nic.id", nID)
    #node.addParam("mem_capacity", 4)
    #node.addParam("storage_size", 4)
    #node.addParam("mem_capacity", 1023)
    #node.addParam("storage_size", 1023)
    node.addParam("mem_capacity", 1048575)
    node.addParam("storage_size", 1048575)
    node.enableStatistics([
      "num_fp_inst", "num_int_inst", "num_mem_access"], {
      "type":"sst.AccumulatorStatistic",
      "rate":"0 ns"})
    #node.enableStatistics([
    #  "num_int_inst"], {
    #  "type":"sst.AccumulatorStatistic",
    #  "rate":"0 ns"})
    #node.enableStatistics([
    #  "num_mem_access"], {
    #  "type":"sst.AccumulatorStatistic",
    #  "rate":"0 ns"})
    node.addParams({
      'name' : 'simple',
      #"debug" : ["nic"],
      #"debug" : ["os"],
      #"debug" : ["os", "nic", "sumi_collective", "parallel", "sumi", "mpi_queue", "mpi"],
      #"debug" : ["app_compute"],

      'app1.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app1.argv' : '-s 120 -i 96',
      #'app1.argv' : '-s 60 -i 96',
      #'app1.argv' : '-s 10 -i 2560',
      #'app1.argv' : '-s 15 -i 2560',
      #'app1.argv' : '-s 30 -i 128',
      'app1.argv' : '-s 2 -i 16',
      #'app1.argv' : '-s 2 -i 4',
      #'app1.argv' : '-s 2 -i 16',
      #'app1.argv' : '-s 256 -i 4',
      #'app1.argv' : '-s 64 -i 2',
      #'app1.argv' : '-s 30 -i 30',
      #'app1.argv' : '-s 120 -i 128',
      #'app1.argv' : '-s 120 -i 48',
      #'app1.argv' : '-s 60 -i 48',
      #'app1.launch_cmd' : 'aprun -n 125000 -N 7813',
      #'app1.launch_cmd' : 'aprun -n 64000 -N 4000',
      #'app1.launch_cmd' : 'aprun -n 46656 -N 2916',
      #'app1.launch_cmd' : 'aprun -n 32768 -N 2048',
      #'app1.launch_cmd' : 'aprun -n 15625 -N 977',
      #'app1.launch_cmd' : 'aprun -n 8000 -N 500',
      #'app1.launch_cmd' : 'aprun -n 4096 -N 256',
      #'app1.launch_cmd' : 'aprun -n 2048 -N 128',
      #'app1.launch_cmd' : 'aprun -n 1000 -N 63',
      #'app1.launch_cmd' : 'aprun -n 512 -N 32',
      #'app1.launch_cmd' : 'aprun -n 125 -N 8',
      #'app1.launch_cmd' : 'aprun -n 64 -N 4',
      #'app1.launch_cmd' : 'aprun -n 512 -N 4',
      #'app1.launch_cmd' : 'aprun -n 1000 -N 2',
      #'app1.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app1.launch_cmd' : 'aprun -n 4096 -N 4',
      #'app1.launch_cmd' : 'aprun -n 32768 -N 16',
      #'app1.launch_cmd' : 'aprun -n 29791 -N 15',
      'app1.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app1.launch_cmd' : 'aprun -n 21952 -N 11',
      #'app1.launch_cmd' : 'aprun -n 262144 -N 128',
      #'app1.launch_cmd' : 'aprun -n 21952 -N 44',

      #'app1.exe' : './runhpcg-3.0',
      #'app1.argv' : '32 32 32',
      #'app1.argv' : '64 64 64',
      #'app1.argv' : '128 128 128',
      #'app1.argv' : '256 128 128',
      #'app1.argv' : '256 256 256',
      #'app1.argv' : '512 256 256',
      #'app1.load_reconfig' : 'true',
      #'app1.load_reconfig' : 'false',

      #'nic.double_wave' : 'true',
      #'nic.double_wave' : 'false',

      #'nic.extra_hops' : '1',
      #'nic.extra_hops' : '0',

      #'app1.exe' : '../traffic/runtraffic',
      #'app1.argv' : '4 2 37 42',
      #'app1.argv' : '256 256 37 42',
      #'app1.argv' : '32 32 37 42',
      #'app1.argv' : '64 64 38 42',

      #'proc.parallelism' : '16',
      #'proc.parallelism' : '2',
      #'proc.parallelism' : '1',

      #'app1.env.OMP_NUM_THREADS' : '2',

      #'app1.launch_cmd' : 'aprun -n 2048 -N 128',
      #'app1.launch_cmd' : 'aprun -n 131072 -N 8192',
      #'app1.launch_cmd' : 'aprun -n 65536 -N 4096',
      #'app1.launch_cmd' : 'aprun -n 32768 -N 2048',
      #'app1.launch_cmd' : 'aprun -n 16384 -N 1024',
      #'app1.launch_cmd' : 'aprun -n 8192 -N 512',
      #'app1.launch_cmd' : 'aprun -n 131072 -N 8192',
      #'app1.launch_cmd' : 'aprun -n 65536 -N 8192',
      #'app1.launch_cmd' : 'aprun -n 32768 -N 8192',
      #'app1.launch_cmd' : 'aprun -n 16384 -N 8192',
      #'app1.launch_cmd' : 'aprun -n 8192 -N 8192',
      #'app1.launch_cmd' : 'aprun -n 2048 -N 2048',
      #'app1.launch_cmd' : 'aprun -n 1024 -N 1024',
      #'app1.launch_cmd' : 'aprun -n 65536 -N 4096',
      #'nic.used_node' : '16',
      #'app1.launch_cmd' : 'aprun -n 32768 -N 4096',
      #'nic.used_node' : '8',
      #'app1.launch_cmd' : 'aprun -n 16384 -N 4096',
      #'nic.used_node' : '4',
      #'app1.launch_cmd' : 'aprun -n 8192 -N 4096',
      #'app1.launch_cmd' : 'aprun -n 4096 -N 4096',
      #'app1.launch_cmd' : 'aprun -n 4096 -N 256',
      #'app1.launch_cmd' : 'aprun -n 2048 -N 256',
      #'app1.launch_cmd' : 'aprun -n 1024 -N 256',
      #'app1.launch_cmd' : 'aprun -n 512 -N 256',
      #'app1.launch_cmd' : 'aprun -n 256 -N 256',
      #'app1.launch_cmd' : 'aprun -n 512 -N 32',
      #'app1.launch_cmd' : 'aprun -n 256 -N 16',
      #'app1.launch_cmd' : 'aprun -n 128 -N 8',
      #'app1.launch_cmd' : 'aprun -n 64 -N 4',
      #'app1.launch_cmd' : 'aprun -n 32 -N 2',
      #'app1.launch_cmd' : 'aprun -n 16 -N 2',
      #'app1.launch_cmd' : 'aprun -n 32 -N 8',
      #'app1.launch_cmd' : 'aprun -n 16 -N 1',
      #'app1.launch_cmd' : 'aprun -n 32 -N 2',
      #'app1.launch_cmd' : 'aprun -n 2 -N 2',
      #'app1.launch_cmd' : 'aprun -n 1 -N 1',
      'app1.allocation' : 'coordinate',
      'app1.mem_size' : '524288',
      'app1.storage_size' : '0',
      'app1.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      'app2.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      'app2.argv' : '-s 2 -i 16',
      'app2.launch_cmd' : 'aprun -n 4096 -N 2',
      'app2.allocation' : 'coordinate',
      'app2.mem_size' : '262144',
      'app2.storage_size' : '0',
      'app2.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      'app3.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      'app3.argv' : '-s 2 -i 16',
      'app3.launch_cmd' : 'aprun -n 4096 -N 2',
      'app3.allocation' : 'coordinate',
      'app3.mem_size' : '131072',
      'app3.storage_size' : '0',
      'app3.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      'app4.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      'app4.argv' : '-s 2 -i 16',
      'app4.launch_cmd' : 'aprun -n 4096 -N 2',
      'app4.allocation' : 'coordinate',
      'app4.mem_size' : '65536',
      'app4.storage_size' : '0',
      'app4.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      'app5.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      'app5.argv' : '-s 2 -i 16',
      'app5.launch_cmd' : 'aprun -n 4096 -N 2',
      'app5.allocation' : 'coordinate',
      'app5.mem_size' : '32768',
      'app5.storage_size' : '0',
      'app5.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      'app6.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      'app6.argv' : '-s 2 -i 16',
      'app6.launch_cmd' : 'aprun -n 4096 -N 2',
      'app6.allocation' : 'coordinate',
      'app6.mem_size' : '16384',
      'app6.storage_size' : '0',
      'app6.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      'app7.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      'app7.argv' : '-s 2 -i 16',
      'app7.launch_cmd' : 'aprun -n 4096 -N 2',
      'app7.allocation' : 'coordinate',
      'app7.mem_size' : '8192',
      'app7.storage_size' : '0',
      'app7.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      'app8.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      'app8.argv' : '-s 2 -i 16',
      'app8.launch_cmd' : 'aprun -n 4096 -N 2',
      'app8.allocation' : 'coordinate',
      'app8.mem_size' : '4096',
      'app8.storage_size' : '0',
      'app8.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      'app9.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      'app9.argv' : '-s 2 -i 16',
      'app9.launch_cmd' : 'aprun -n 4096 -N 2',
      'app9.allocation' : 'coordinate',
      'app9.mem_size' : '2048',
      'app9.storage_size' : '0',
      'app9.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      'app10.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      'app10.argv' : '-s 2 -i 16',
      'app10.launch_cmd' : 'aprun -n 4096 -N 2',
      'app10.allocation' : 'coordinate',
      'app10.mem_size' : '1024',
      'app10.storage_size' : '0',
      'app10.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app11.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app11.argv' : '-s 2 -i 16',
      #'app11.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app11.allocation' : 'coordinate',
      #'app11.mem_size' : '0',
      #'app11.storage_size' : '524288',
      #'app11.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app12.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app12.argv' : '-s 2 -i 16',
      #'app12.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app12.allocation' : 'coordinate',
      #'app12.mem_size' : '0',
      #'app12.storage_size' : '262144',
      #'app12.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app13.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app13.argv' : '-s 2 -i 16',
      #'app13.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app13.allocation' : 'coordinate',
      #'app13.mem_size' : '0',
      #'app13.storage_size' : '131072',
      #'app13.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app14.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app14.argv' : '-s 2 -i 16',
      #'app14.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app14.allocation' : 'coordinate',
      #'app14.mem_size' : '0',
      #'app14.storage_size' : '65536',
      #'app14.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app15.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app15.argv' : '-s 2 -i 16',
      #'app15.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app15.allocation' : 'coordinate',
      #'app15.mem_size' : '0',
      #'app15.storage_size' : '32768',
      #'app15.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app16.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app16.argv' : '-s 2 -i 16',
      #'app16.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app16.allocation' : 'coordinate',
      #'app16.mem_size' : '0',
      #'app16.storage_size' : '16384',
      #'app16.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app17.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app17.argv' : '-s 2 -i 16',
      #'app17.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app17.allocation' : 'coordinate',
      #'app17.mem_size' : '0',
      #'app17.storage_size' : '8192',
      #'app17.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app18.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app18.argv' : '-s 2 -i 16',
      #'app18.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app18.allocation' : 'coordinate',
      #'app18.mem_size' : '0',
      #'app18.storage_size' : '4096',
      #'app18.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app19.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app19.argv' : '-s 2 -i 16',
      #'app19.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app19.allocation' : 'coordinate',
      #'app19.mem_size' : '0',
      #'app19.storage_size' : '2048',
      #'app19.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app20.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app20.argv' : '-s 2 -i 16',
      #'app20.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app20.allocation' : 'coordinate',
      #'app20.mem_size' : '0',
      #'app20.storage_size' : '1024',
      #'app20.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app21.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app21.argv' : '-s 2 -i 16',
      #'app21.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app21.allocation' : 'coordinate',
      #'app21.mem_size' : '512',
      #'app21.storage_size' : '0',
      #'app21.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app22.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app22.argv' : '-s 2 -i 16',
      #'app22.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app22.allocation' : 'coordinate',
      #'app22.mem_size' : '256',
      #'app22.storage_size' : '0',
      #'app22.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app23.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app23.argv' : '-s 2 -i 16',
      #'app23.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app23.allocation' : 'coordinate',
      #'app23.mem_size' : '128',
      #'app23.storage_size' : '0',
      #'app23.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app24.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app24.argv' : '-s 2 -i 16',
      #'app24.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app24.allocation' : 'coordinate',
      #'app24.mem_size' : '64',
      #'app24.storage_size' : '0',
      #'app24.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app25.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app25.argv' : '-s 2 -i 16',
      #'app25.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app25.allocation' : 'coordinate',
      #'app25.mem_size' : '32',
      #'app25.storage_size' : '0',
      #'app25.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app26.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app26.argv' : '-s 2 -i 16',
      #'app26.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app26.allocation' : 'coordinate',
      #'app26.mem_size' : '16',
      #'app26.storage_size' : '0',
      #'app26.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app27.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app27.argv' : '-s 2 -i 16',
      #'app27.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app27.allocation' : 'coordinate',
      #'app27.mem_size' : '8',
      #'app27.storage_size' : '0',
      #'app27.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app28.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app28.argv' : '-s 2 -i 16',
      #'app28.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app28.allocation' : 'coordinate',
      #'app28.mem_size' : '4',
      #'app28.storage_size' : '0',
      #'app28.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app29.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app29.argv' : '-s 2 -i 16',
      #'app29.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app29.allocation' : 'coordinate',
      #'app29.mem_size' : '2',
      #'app29.storage_size' : '0',
      #'app29.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app30.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app30.argv' : '-s 2 -i 16',
      #'app30.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app30.allocation' : 'coordinate',
      #'app30.mem_size' : '1',
      #'app30.storage_size' : '0',
      #'app30.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app31.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app31.argv' : '-s 2 -i 16',
      #'app31.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app31.allocation' : 'coordinate',
      #'app31.mem_size' : '0',
      #'app31.storage_size' : '512',
      #'app31.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app32.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app32.argv' : '-s 2 -i 16',
      #'app32.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app32.allocation' : 'coordinate',
      #'app32.mem_size' : '0',
      #'app32.storage_size' : '256',
      #'app32.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app33.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app33.argv' : '-s 2 -i 16',
      #'app33.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app33.allocation' : 'coordinate',
      #'app33.mem_size' : '0',
      #'app33.storage_size' : '128',
      #'app33.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app34.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app34.argv' : '-s 2 -i 16',
      #'app34.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app34.allocation' : 'coordinate',
      #'app34.mem_size' : '0',
      #'app34.storage_size' : '64',
      #'app34.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app35.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app35.argv' : '-s 2 -i 16',
      #'app35.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app35.allocation' : 'coordinate',
      #'app35.mem_size' : '0',
      #'app35.storage_size' : '32',
      #'app35.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app36.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app36.argv' : '-s 2 -i 16',
      #'app36.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app36.allocation' : 'coordinate',
      #'app36.mem_size' : '0',
      #'app36.storage_size' : '16',
      #'app36.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app37.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app37.argv' : '-s 2 -i 16',
      #'app37.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app37.allocation' : 'coordinate',
      #'app37.mem_size' : '0',
      #'app37.storage_size' : '8',
      #'app37.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app38.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app38.argv' : '-s 2 -i 16',
      #'app38.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app38.allocation' : 'coordinate',
      #'app38.mem_size' : '0',
      #'app38.storage_size' : '4',
      #'app38.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app39.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app39.argv' : '-s 2 -i 16',
      #'app39.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app39.allocation' : 'coordinate',
      #'app39.mem_size' : '0',
      #'app39.storage_size' : '2',
      #'app39.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

      #'app40.exe' : '../lulesh2.0.3/runlulesh2.0.3',
      #'app40.argv' : '-s 2 -i 16',
      #'app40.launch_cmd' : 'aprun -n 4096 -N 2',
      #'app40.allocation' : 'coordinate',
      #'app40.mem_size' : '0',
      #'app40.storage_size' : '1',
      #'app40.coordinate_file' : '/home/xianwei/new_source/sst-macro/skeletons/hpcg-3.0/coord1.txt',

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

      'memory.mtu' : '1000000GB',
      'memory.latency' : '0ns',
      'memory.name' : 'logp',
      #'memory.bandwidth' : '128GB/s',
      #'memory.bandwidth' : '256GB/s',
      #'memory.bandwidth' : '2048GB/s',
      #'memory.bandwidth' : '65536GB/s',
      #'memory.bandwidth' : '524GB/s',
      'memory.bandwidth' : '400000000GB/s',
      #'memory.bandwidth' : '65536GB/s',
      #'memory.bandwidth' : '32768GB/s',

      'nic.name' : 'slingshot',
      #'nic.num_ports' : '16',
      #'nic.pod' : '16',
      #'nic.col' : '0',
      #'nic.row' : '0',
      #'nic.num_core' : '256',
      #'nic.num_core' : '8192',
      #'nic.num_core' : '4096',
      #'nic.num_core' : '2048',
      #'nic.num_core' : '1024',
      'nic.injection.latency' : _params["nic_latency"],
      'nic.injection.bandwidth' : _params["nic_bandwidth"],
      'nic.num_links' : _params["ports_per_nodes"],
      #'nic.injection.bandwidth' : '1GB/s',
      #'nic.injection.bandwidth' : '2GB/s',
      #'nic.injection.bandwidth' : '10GB/s',
      #'nic.injection.bandwidth' : '20GB/s',
      #'nic.injection.bandwidth' : '100GB/s',
      #'nic.injection.bandwidth' : '200GB/s',
      #'nic.update_freq' : '100Hz',
      #'nic.update_freq' : '10Hz',
      #'nic.update_freq' : '1Hz',
      #'nic.update_freq' : '0.001Hz',

      'topology.auto' : 'true',
      'topology.only_observe' : 'true',
      'topology.fully_reconfig' : 'false',
      #'topology.fully_reconfig' : 'true',
      #'topology.geometry' : '[2,2,2]',
      'topology.geometry' : '[1,16,128]',
      'topology.name' : 'torus',

      'proc.frequency' : '1000000Ghz',
      #'proc.frequency' : '2.1Ghz',
      #64 cores / 2 nodes / 64 blades
      #'proc.ncores' : '128',
      #'proc.ncores' : '256',
      #'proc.ncores' : '8192',
      #'proc.ncores' : '64',
      'proc.ncores' : '128',
      #'proc.ncores' : '2048',
      #'proc.ncores' : '1024',
      #'interconnect.topology.geometry' : '[2,2,2]',
      'interconnect.topology.geometry' : '[1,16,128]',

      'interconnect.topology.auto' : 'true',
      'interconnect.topology.name' : 'torus',
      #'auto' : 'true',
      #'geometry' : '[2,2,2]',

      #'name' : 'torus'
    })
    return node

e_switches = dict()
def getESwitch(rackId, idxF, idxS, idxT):
    name = "e_switch.%d_%d_%d"%(idxF, idxS, idxT)
    if name not in e_switches:
        e_switches[name] = sst.Component(name, "macro.slingshot_switch")
        e_switches[name].addParam("rackId", rackId)
        e_switches[name].addParam("switch_name", name)
        e_switches[name].addParam("num_pod", _params["pod"])
        e_switches[name].addParam("num_col", _params["col"])
        e_switches[name].addParam("num_row", _params["row"])
        e_switches[name].addParam("num_pod_ports", _params["ports_per_global_link"])
        e_switches[name].addParam("num_switches", _params["num_switches_per_rack"])
        e_switches[name].addParam("num_links_per_node", _params["ports_per_nodes"])
        e_switches[name].addParam("num_nodes_per_switch", _params["num_nodes_per_switch"])
        e_switches[name].addParam("latency", _params["inter_switch_latency"])
        e_switches[name].addParam("forward_bandwidth", _params["switch_forward"])
        e_switches[name].addParam("adaptive_routing", "false")
        e_switches[name].addParam("bandwidth_steering", "true")
        e_switches[name].addParam("dynamic_steering", "false")
        #baseline and adaptive_routing
        #e_switches[name].addParam("wave_remap", "[-1,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,0,-1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,0,1,-1,3,4,5,6,7,8,9,10,11,12,13,14,15,0,1,2,-1,4,5,6,7,8,9,10,11,12,13,14,15,0,1,2,3,-1,5,6,7,8,9,10,11,12,13,14,15,0,1,2,3,4,-1,6,7,8,9,10,11,12,13,14,15,0,1,2,3,4,5,-1,7,8,9,10,11,12,13,14,15,0,1,2,3,4,5,6,-1,8,9,10,11,12,13,14,15,0,1,2,3,4,5,6,7,-1,9,10,11,12,13,14,15,0,1,2,3,4,5,6,7,8,-1,10,11,12,13,14,15,0,1,2,3,4,5,6,7,8,9,-1,11,12,13,14,15,0,1,2,3,4,5,6,7,8,9,10,-1,12,13,14,15,0,1,2,3,4,5,6,7,8,9,10,11,-1,13,14,15,0,1,2,3,4,5,6,7,8,9,10,11,12,-1,14,15,0,1,2,3,4,5,6,7,8,9,10,11,12,13,-1,15,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,-1]")
        #bandwidth_steering_0
        #e_switches[name].addParam("wave_remap", "[-1,1,2,3,1,5,6,1,1,1,1,1,1,1,1,1,0,-1,2,0,4,0,6,7,0,0,0,0,0,0,0,0,0,1,-1,3,4,5,3,3,8,9,3,3,3,3,3,3,0,1,2,-1,4,5,2,2,2,2,10,11,2,2,2,2,5,1,2,3,-1,5,5,5,5,5,5,5,12,13,5,5,0,4,2,3,4,-1,6,7,4,9,10,4,4,4,14,15,0,1,7,7,7,5,-1,7,8,9,10,11,12,7,7,7,6,1,6,6,6,5,6,-1,8,9,6,6,12,13,6,6,9,9,2,9,9,9,6,7,-1,9,9,9,9,13,14,9,8,8,2,8,8,5,6,7,8,-1,10,11,12,8,14,15,11,11,11,3,11,5,6,11,11,9,-1,11,12,13,11,11,10,10,10,3,10,10,6,10,10,9,10,-1,12,10,14,10,13,13,13,13,4,13,6,7,13,9,10,11,-1,13,14,15,12,12,12,12,4,12,12,7,12,12,10,12,12,-1,14,15,15,15,15,15,15,5,15,15,8,9,15,11,12,13,-1,15,14,14,14,14,14,5,14,14,14,9,14,14,12,13,14,-1]")
        #bandwidth_steering_1
        #e_switches[name].addParam("wave_remap", "[-1,1,2,1,4,1,1,1,8,1,1,1,12,1,14,15,0,-1,2,3,0,5,0,0,0,9,0,0,0,13,0,15,0,1,-1,3,4,3,6,3,3,3,10,3,3,3,14,3,2,1,2,-1,4,5,2,7,2,2,2,11,2,2,2,15,0,5,2,3,-1,5,6,5,8,5,5,5,12,5,5,5,4,1,4,3,4,-1,6,7,4,9,4,4,4,13,4,4,7,7,2,7,4,5,-1,7,8,7,10,7,7,7,14,7,6,6,6,3,6,5,6,-1,8,9,6,11,6,6,6,15,0,9,9,9,4,9,6,7,-1,9,10,9,12,9,9,9,8,1,8,8,8,5,8,7,8,-1,10,11,8,13,8,8,11,11,2,11,11,11,6,11,8,9,-1,11,12,11,14,11,10,10,10,3,10,10,10,7,10,9,10,-1,12,13,10,15,0,13,13,13,4,13,13,13,8,13,10,11,-1,13,14,13,12,1,12,12,12,5,12,12,12,9,12,11,12,-1,14,15,0,15,2,15,15,15,6,15,15,15,10,15,12,13,-1,15,0,1,14,3,14,14,14,7,14,14,14,11,14,13,14,-1]")
        #bandwidth_steering_2
        e_switches[name].addParam("wave_remap", "[-1,1,2,1,4,5,1,7,8,9,1,11,12,1,14,15,0,-1,2,3,0,5,6,0,8,9,10,0,12,13,0,15,0,1,-1,3,4,3,6,7,3,9,10,11,3,13,14,3,2,1,2,-1,4,5,2,7,8,2,10,11,12,2,14,15,0,5,2,3,-1,5,6,5,8,9,5,11,12,13,5,15,0,1,4,3,4,-1,6,7,4,9,10,4,12,13,14,4,7,1,2,7,4,5,-1,7,8,7,10,11,7,13,14,15,0,6,2,3,6,5,6,-1,8,9,6,11,12,6,14,15,0,1,9,3,4,9,6,7,-1,9,10,9,12,13,9,15,0,1,2,8,4,5,8,7,8,-1,10,11,8,13,14,8,11,1,2,3,11,5,6,11,8,9,-1,11,12,11,14,15,0,10,2,3,4,10,6,7,10,9,10,-1,12,13,10,15,0,1,13,3,4,5,13,7,8,13,10,11,-1,13,14,13,12,1,2,12,4,5,6,12,8,9,12,11,12,-1,14,15,0,15,2,3,15,5,6,7,15,9,10,15,12,13,-1,15,0,1,14,3,4,14,6,7,8,14,10,11,14,13,14,-1]")
        #dynamic_steering
        #e_switches[name].addParam("wave_remap", "[-1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,3,-1,3,3,3,3,3,3,3,3,3,3,3,3,3,2,2,2,-1,2,2,2,2,2,2,2,2,2,2,2,2,5,5,5,5,-1,5,5,5,5,5,5,5,5,5,5,5,4,4,4,4,4,-1,4,4,4,4,4,4,4,4,4,4,7,7,7,7,7,7,-1,7,7,7,7,7,7,7,7,7,6,6,6,6,6,6,6,-1,6,6,6,6,6,6,6,6,9,9,9,9,9,9,9,9,-1,9,9,9,9,9,9,9,8,8,8,8,8,8,8,8,8,-1,8,8,8,8,8,8,11,11,11,11,11,11,11,11,11,11,-1,11,11,11,11,11,10,10,10,10,10,10,10,10,10,10,10,-1,10,10,10,10,13,13,13,13,13,13,13,13,13,13,13,13,-1,13,13,13,12,12,12,12,12,12,12,12,12,12,12,12,12,-1,12,12,15,15,15,15,15,15,15,15,15,15,15,15,15,15,-1,15,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,-1]")
        #e_switches[name].addParam("shortest_routing", "[]")
        e_switches[name].addParam("thr_inter_rack", "0")
        e_switches[name].addParam("thr_intra_rack", "0")
        e_switches[name].addParams({'crossbar.num_ports' : _params["num_ports_per_switch"],
                                    'crossbar.bandwidth' : _params["switch_bandwidth"],
                                    'crossbar.routing' : _params["routing_latency"],
                                    'crossbar.latency' : _params["switch_latency"]})
    return e_switches[name]

switches = dict()
def getSwitch(switchType, idxL, idxF, idxS):
    name = "switch.%s_%d_%d_%d"%(switchType, idxL, idxF, idxS)
    if name not in switches:
        switches[name] = sst.Component(name, "macro.double_switch")
        switches[name].addParam("switch_name", name)
        switches[name].addParam("num_waves", _params[switchType]+1)
        switches[name].addParam("reconfig_map", "[1,0,3,2,5,4,7,6,9,8,11,10,13,12,15,14]")
    else:
      print(name+" is existing")
    return switches[name]

links = dict()
##loc_r_c_p_l
def getLink(switchType, idxL, idxF, idxS, idxP, idxW):
    name = "link.%s_%d_%d_%d_%d_%d"%(switchType, idxL, idxF, idxS, idxP, idxW)
    if name not in links:
        links[name] = sst.Link(name)
    return links[name]

for l in range(_params["ports_per_global_link"]):
    for i in range(_params["num_pods"]):
        for j in range(_params["num_cols"]):
            if _params["row"] > 0:
                #print("Test 0 row with %d %d %d"%(l,i,j))
                switch = getSwitch("row", l, j, i)
                for p_j in range(_params["row"]):
                    for w_j in range(_params["row"]-1):
                        #print("Test 1 row with %d %d"%(p_j,w_j))
                        switch.addLink(getLink("row", l, j, i, p_j, w_j), "port_%d_%d"%(p_j, w_j), _params["opt_link_latency"])
                        switch.addLink(getLink("row_r", l, j, i, p_j, w_j), "port_r_%d_%d"%(p_j, w_j), _params["opt_link_latency"])

        for k in range(_params["num_rows"]):
            if _params["col"] > 0:
                switch = getSwitch("col", l, k, i)
                #print("Test 0 col with %d %d %d"%(l,k,i))
                for p_k in range(_params["col"]):
                    for w_k in range(_params["col"]-1):
                        #print("Test 1 col with %d %d"%(p_k,w_k))
                        switch.addLink(getLink("col", l, k, i, p_k, w_k), "port_%d_%d"%(p_k, w_k), _params["opt_link_latency"])
                        switch.addLink(getLink("col_r", l, k, i, p_k, w_k), "port_r_%d_%d"%(p_k, w_k), _params["opt_link_latency"])

for l in range(_params["ports_per_global_link"]):
    for i in range(_params["num_rows"]):
        for j in range(_params["num_cols"]):
            if _params["pod"] > 0:
                switch = getSwitch("pod", l, i, j)
                #print("Test 0 pod with %d %d %d"%(l,i,j))
                for p_k in range(_params["num_pods"]):
                    #print("Test with %d %d %d"%(i,j, p_k))
                    for w_k in range(_params["num_pods"]-1):
                        #print("Test 1 pod with %d %d"%(p_k,w_k))
                        switch.addLink(getLink("pod", l, i, j, p_k, w_k), "port_%d_%d"%(p_k, w_k), _params["opt_link_latency"])
                        switch.addLink(getLink("pod_r", l, i, j, p_k, w_k), "port_r_%d_%d"%(p_k, w_k), _params["opt_link_latency"])

ToR_id = 0
for i in range(_params["num_rows"]):
    for j in range(_params["num_cols"]):
        if _params["pod"] > 0:
            for p_k in range(_params["num_pods"]):
                es = getESwitch(ToR_id, i, j, p_k)
                for n in range(_params["num_nodes_per_rack"]):
                    ep = macroNode(ToR_id*_params["num_nodes_per_rack"]+n)
                    for l in range(_params["ports_per_nodes"]):
                        ep.addLink(getLink("local", i, j, p_k, n, l), "port_link_%d"%(l), _params["ele_link_latency"])
                        es.addLink(getLink("local", i, j, p_k, n, l), "local_%d_%d"%(n, l), _params["ele_link_latency"])
                ToR_id = ToR_id + 1
                for l in range(_params["ports_per_global_link"]):
                    for w_k in range(_params["num_pods"]-1):
                        es.addLink(getLink("pod", l, i, j, p_k, w_k), "port_0_%d_%d"%(l, w_k), _params["opt_link_latency"])
                        es.addLink(getLink("pod_r", l, i, j, p_k, w_k), "port_r_0_%d_%d"%(l, w_k), _params["opt_link_latency"])
                    for w_k in range(_params["row"]-1):
                        es.addLink(getLink("row", l, j, p_k, i, w_k), "port_1_%d_%d"%(l, w_k), _params["opt_link_latency"])
                        es.addLink(getLink("row_r", l, j, p_k, i, w_k), "port_r_1_%d_%d"%(l, w_k), _params["opt_link_latency"])
                    for w_k in range(_params["col"]-1):
                        es.addLink(getLink("col", l, i, p_k, j, w_k), "port_2_%d_%d"%(l, w_k), _params["opt_link_latency"])
                        es.addLink(getLink("col_r", l, i, p_k, j, w_k), "port_r_2_%d_%d"%(l, w_k), _params["opt_link_latency"])
#for i in range(_params["num_rows"]):
#    for j in range(_params["num_cols"]):
#        if _params["pod"] > 0:
#            switch = getSwitch("pod", i, j)
            #print("Test with %d %d"%(i,j))
#            for p_k in range(_params["num_pods"]):
                #print("Test with %d %d %d"%(i,j, p_k))
#                ep = macroNode(i*_params["num_cols"]+j*_params["num_pods"]+p_k)
#                for w_k in range(_params["num_pods"]):
#                    switch.addLink(getLink("pod", i, j, p_k, w_k), "port_%d_%d"%(p_k, w_k), _params["link_lat"])
#                    ep.addLink(getLink("pod", i, j, p_k, w_k), "pod_port_%d"%(w_k), _params["link_lat"])
#                for w_k in range(_params["row"]):
#                    ep.addLink(getLink("row", j, p_k, i, w_k), "row_port_%d"%(w_k), _params["link_lat"])
#                for w_k in range(_params["col"]):
#                    ep.addLink(getLink("col", i, p_k, j, w_k), "col_port_%d"%(w_k), _params["link_lat"])

#sst.enableAllStatisticsForComponentType("macro.simple_node")
