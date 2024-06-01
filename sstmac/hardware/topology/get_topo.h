#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
//#include <filesystem>
#include <experimental/filesystem>
#include <cmath>
#include <limits>
#include <bits/stdc++.h>
#include<math.h>
#include<algorithm>
#include<stack>

//using namespace std;


static const unsigned int INF(std::numeric_limits<int>::max());
static const unsigned undefined = INF;

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

std::vector<int> get_topo_reconfig(std::vector<double> traffic_matrix, int num_port, double wave_capacity){
   std::vector<int> returned_topo;
   int num_tors = sqrt(traffic_matrix.size());
   double tm[num_tors][num_tors];
   for (int i=0; i<num_tors; i++){
    for (int j=0; j<num_tors; j++){
        tm[i][j] = traffic_matrix[num_tors*i+j];
    }
   } 
   double margin = 0.05;        // spare capacity on a link should not less than the margin, which means when a channel is fully loaded, the average packet latency would be 1/margin ns.
   int num_inport[num_tors];
   int num_outport[num_tors];

   for (int i=0; i<num_tors; i++){
        num_inport[i] =  num_port*1;
        num_outport[i] = num_port*1;
    }

   double weight_matrix[num_tors][num_tors];
   for (int i=0; i<num_tors; i++){
    for (int j=0; j<num_tors; j++){
        weight_matrix[i][j] = tm[i][j];
    }
   } 

   int topology[num_tors][num_tors];
   int record_node[num_tors][num_tors];

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
                    double w[num_tors] = {};
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
                        double w[num_tors] = {};
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
                    double w[num_tors] = {};
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
                        double w[num_tors] = {};
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
    }
   }

   std::cout << "returned_topo: " << std::endl;
   for (int i=0; i<num_tors; i++){
    for (int j=0; j<num_tors; j++){
        std::cout << topology[i][j] << " ";
        returned_topo.push_back(topology[i][j]);
    }
    std::cout << std::endl;
   } 

   return returned_topo;
}



std::vector<std::string> read_from_csv(std::string filepath){
    std::ifstream fin;
    fin.open(filepath);
    std::vector<std::string> row;

    while(fin.peek()!=EOF){
        std::string records;
        getline(fin, records, ',');
        std::cout << records<< std::endl;
        row.push_back(records);
    }

    return row;
}





// K-shortest paths from here
//
//
class K_Shortest_Path {
public:
	std::vector<std::vector<unsigned int>> run(
		const unsigned int kPath,					// K Path
		const std::vector<std::vector<unsigned int>>& NW,		// network
		const unsigned int src,						// source node
		const unsigned int dst);					// destination node
};
 
 
class DijPath
{
public:
	std::vector<unsigned int> onePath;
	int cost;
 
	bool operator <(const DijPath &n2);
	bool operator ==(const DijPath &n2);
};
 
 
bool DijPath::operator <(const DijPath &n2)
{
	return cost < n2.cost;
}

bool DijPath::operator ==(const DijPath &n2)
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
std::vector<std::vector<unsigned int>> K_Shortest_Path::run(
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


int main(){
    std::vector<std::string> tm_data = read_from_csv("/home/xianwei/test/routing_algorithm/cry16.csv");
    std::vector<double> traffic_matrix;
    int num_ToRs = 16;
    int num_ports = num_ToRs-1;
    double wave_capacity = 10;   // Link capacity is 10G

    for (int i=0; i<num_ToRs*num_ToRs; i++){
        traffic_matrix.push_back(std::stod(tm_data[i]));
    }
    std::vector<int> topo = get_topo_reconfig(traffic_matrix, num_ports, wave_capacity);
    std::vector<std::vector<unsigned int>> topology(num_ToRs, std::vector<unsigned int>(num_ToRs, 1));

    for (int i=0; i<num_ToRs; i++){
        for (int j=0; j<num_ToRs; j++){
            topology.at(i).at(j) = topo[i*num_ToRs+j];
            //topology[i][j] = topo[i*num_ToRs+j];
            if (topology.at(i).at(j) >= 1){
                topology.at(i).at(j) = 1;
            }
        }
    }

    for (unsigned int i(0); i < topology.size(); i++) {
		for (unsigned int j(0); j < topology.size(); j++) {
			if (topology[i][j] == 0) {
					topology[i][j] = INF;
			}
		}
	}

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

    // K-Shortest Path
	unsigned int kPath = 3;
 
	std::cout << std::endl << "K-Shortest Path (" << kPath << ")" << std::endl;
	K_Shortest_Path KSP;
    std::ofstream myfile;                            // write the results to txt file
    myfile.open("shortest_paths.txt");

    for (int i=0; i<num_ToRs; i++){
        for (int j=0; j<num_ToRs; j++){
            if (i != j){
	            std::vector<std::vector<unsigned int>> kSP = KSP.run(kPath, topology, i, j);
                myfile << "\n";
                myfile << "src: " << i << "   " << "dst: " << j;
                myfile << "\n";
 
	            std::cout << "  > Number of path: " << kSP.size() << std::endl;
	            for (unsigned int i(0); i < kSP.size(); i++) {
		            std::cout << "    >> " << (i + 1) << ":";
                    myfile << (i + 1) << ":";
		            for (unsigned int j(0); j < kSP[i].size(); j++) {
			            std::cout << "   " << kSP[i][j];
                        myfile << "   " << kSP[i][j];
		            }
		            std::cout << std::endl;
                    myfile << "\n";
	            }
            }
        }
    }
}
