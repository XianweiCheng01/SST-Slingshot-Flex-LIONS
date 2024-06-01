#include <iostream>
#include <vector>
#include <string>

using namespace std;

class nodes{
public:
    string node_name;
    int available_proc;
    int available_mem;
    int available_storage;
};

bool isSuitableForVm(nodes selected_node, std::vector<int> required_specs){
    if (selected_node.available_proc < required_specs[0]){
        std::cout << "Allocation of the VM to " + selected_node.node_name + " failed by the proc" << endl;
        return false;
    }
    if (selected_node.available_mem < required_specs[1]){
        std::cout << "Allocation of the VM to " + selected_node.node_name + " failed by the mem" << endl;
        return false;
    }
    if (selected_node.available_storage < required_specs[2]){
        std::cout << "Allocation of the VM to " + selected_node.node_name + " failed by the storage" << endl;
        return false;
    }

    return true;
}

std::vector<nodes> allocateForVm(std::vector<nodes> node_list, std::vector<int> required_specs){
    int num_nodes = node_list.size();
    std::vector<nodes> allocation;

    for (int nid=0; nid < num_nodes; ++nid){
        nodes select_node = node_list[nid];
        bool result = isSuitableForVm(select_node, required_specs);
        if (result){    // need to check if this is reasonable
            select_node.available_proc -= required_specs[0];
            select_node.available_mem -= required_specs[1];
            select_node.available_storage -= required_specs[2];
            std::cout << "The vm has been successfully allocated in the host " + select_node.node_name << endl;
            allocation.push_back(select_node);
            break;
        }
        else{
            if (nid == num_nodes - 1){
                // the vm request can not be fulfilled, need to rewrite the previous all vm decisions (in the same application) as "null" 
                // add this later...
                std::cout << "The vm request can not be fulfilled." << endl;
                break;
            }
            else{
                continue;
            }
        }
    }

    return allocation;
}


int main(){

    // number of the nodes in the network (here we assume only 5 nodes)
    std::vector<nodes> node_list(5);
    for (int i=0; i<5; ++i){
        node_list[i].node_name = "Node_" + std::to_string(i);
        node_list[i].available_proc = 24;
        node_list[i].available_mem = 128;
        node_list[i].available_storage = 3200;  
    }   

    // required resources for a vm request
    vector<int> required_specs;
    required_specs.push_back(4);
    required_specs.push_back(15);
    required_specs.push_back(80);

    std::vector<nodes> allocation_result = allocateForVm(node_list, required_specs);

    return 0;
}