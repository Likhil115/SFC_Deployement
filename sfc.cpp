#include "SFC.hpp"
#include <algorithm>
#include <unordered_set>
#include <queue>


using namespace std;

void SFC::classify(double latency_threshold) {
    priority = (expected_latency <= latency_threshold) ? "high" : "low";
}

void SFC::buildDependencyDAG() {
    const std::string IN = "INGRESS";
    const std::string OUT = "EGRESS";

    adj.clear();
    adj[IN] = {};

    std::unordered_set<std::string> all, hasIncoming;

    // collect all VNF names and build edges
    for (auto& v : vnfs) {
        all.insert(v.name);
        if (v.depends_on.empty()) {
            adj[IN].insert(v.name);
        }
        for (auto& d : v.depends_on) {
            adj[d].insert(v.name);
            hasIncoming.insert(v.name);
        }
    }
    // any VNF with no outgoing in adj -> EGRESS
    for (auto& name : all) {
        if (adj.find(name) == adj.end() || adj[name].empty()) {
            adj[name].insert(OUT);
        }
    }
}



// Merge DAGs from multiple SFCs
void SFC::mergeDAGs(std::unordered_map<std::string, std::vector<std::string>>& mergedAdj, const SFC& sfc) {
    for (const auto& kv : sfc.adj) {
        const std::string& src = kv.first;
        for (const std::string& dst : kv.second) {
            // Avoid duplicate edges
            if (std::find(mergedAdj[src].begin(), mergedAdj[src].end(), dst) == mergedAdj[src].end()) {
                mergedAdj[src].push_back(dst);
            }
        }
    }
}

std::vector<std::string> SFC::topologicalSort(const std::unordered_map<std::string, std::vector<std::string>>& adj) {
    std::unordered_map<std::string, int> indegree;
    std::queue<std::string> q;
    std::vector<std::string> result;


    //print adj
    cout << "\nAdjacency List here you go :\n";
    if (adj.empty()) {
        std::cout << "   (empty)\n";
        return result;
    }
    for(const auto& kv : adj) {
        std::cout << kv.first << " -> ";
        for (const std::string& dst : kv.second) {
            std::cout << dst << " ";
        }
        std::cout << "\n";
    }

    // Compute indegree
    for (const auto& kv : adj) {
        if (!indegree.count(kv.first)) indegree[kv.first] = 0;
        for (const std::string& dst : kv.second) {
            indegree[dst]++;
        }
    }

    // Initialize queue with nodes of indegree 0
    for (const auto& kv : indegree) {
        if (kv.second == 0 && kv.first != "EGRESS") { // Exclude EGRESS as a start node
            q.push(kv.first);
        }
    }
    

    // Process nodes
    while (!q.empty()) {
        std::string u = q.front();
        q.pop();
        result.push_back(u);

        if (adj.count(u)) {
            for (const std::string& v : adj.at(u)) {
                if (--indegree[v] == 0) {
                    q.push(v);
                }
            }
        }
    }
    

    // Handle remaining nodes (e.g., EGRESS)
    if (adj.count("EGRESS") && indegree["EGRESS"] == 0) {
        result.push_back("EGRESS");
    }
    //print result
    std::cout << "\nTopological Sort Result:\n";
    if (result.empty()) {
        std::cout << "   (empty)\n";
        return result;
    }   
    for (const std::string& node : result) {
        std::cout << node << " ";
    }

    return result;
}


// Main function to merge and sort low-priority SFCs
std::vector<std::string> SFC::mergeAndSortSFCs(const std::vector<SFC>& sfcs) {
    std::unordered_map<std::string, std::vector<std::string>> mergedAdj; 
    for (const SFC& sfc : sfcs) {
        if (sfc.priority == "low") {
            mergeDAGs(mergedAdj, sfc);
        }
    }
    if(hasCycle(mergedAdj)) {
        std::cerr << "Error: Cycle detected in the merged DAG. Cannot perform topological sort.\n";
        return {};
    }
    return topologicalSort(mergedAdj);
}



bool SFC::hasCycleDFS(
    const std::string& node,
    std::unordered_map<std::string, int>& visited,
    const std::unordered_map<std::string, std::vector<std::string>>& adj
) {
    visited[node] = 1; // Mark as being visited
    if (adj.count(node)) {
        for (const auto& neighbor : adj.at(node)) {
            if (visited[neighbor] == 1) return true; // Cycle detected
            if (visited[neighbor] == 0 && hasCycleDFS(neighbor, visited, adj)) {
                return true;
            }
        }
    }
    visited[node] = 2; // Mark as fully visited
    return false;
}

bool SFC::hasCycle(const std::unordered_map<std::string, std::vector<std::string>>& adj) {
    std::unordered_map<std::string, int> visited;
    for (const auto& kv : adj) {
        visited[kv.first] = 0; // Initialize all nodes as unvisited
        for (const auto& dst : kv.second) {
            visited[dst] = 0;
        }
    }
    for (const auto& kv : adj) {
        if (visited[kv.first] == 0 && hasCycleDFS(kv.first, visited, adj)) {
            return true;
        }
    }
    return false;
}


bool SFC::validateTopoOrder(
    const std::vector<std::string>& topoOrder,
    const std::unordered_map<std::string, std::vector<std::string>>& adj
) {
    std::unordered_map<std::string, int> index;
    for (int i = 0; i < topoOrder.size(); ++i) {
        index[topoOrder[i]] = i;
    }
    for (const auto& node : topoOrder) {
        if (adj.count(node)) {
            for (const auto& dep : adj.at(node)) {
                if (index[dep] <= index[node]) {
                    std::cerr << "Dependency violation: " << node << " -> " << dep << "\n";
                    return false;
                }
            }
        }
    }
    return true;
}

bool SFC::validateParallelStages(
    const std::vector<std::vector<std::string>>& stages,
    const std::unordered_map<std::string, std::vector<std::string>>& adj
) {
    std::unordered_map<std::string, int> stageIndex;
    for (int i = 0; i < stages.size(); ++i) {
        for (const auto& node : stages[i]) {
            stageIndex[node] = i;
        }
    }
    for (const auto& kv : adj) {
        for (const auto& dep : kv.second) {
            if (stageIndex[kv.first] >= stageIndex[dep]) {
                std::cerr << "Stage violation: " << kv.first << " (stage " 
                          << stageIndex[kv.first] << ") depends on " << dep 
                          << " (stage " << stageIndex[dep] << ")\n";
                return false;
            }
        }
    }
    return true;
}



std::unordered_map<std::string, std::unordered_set<std::string>> 
SFC::buildLowPriorityDependencyList(const std::vector<SFC>& low_priority_sfcs) 
{
    std::unordered_map<std::string, std::unordered_set<std::string>> dependency_list;
    std::unordered_set<std::string> all_nodes;

    for (const auto& sfc : low_priority_sfcs) {
        const_cast<SFC&>(sfc).buildDependencyDAG();  // Ensure DAG is built
        
        // First collect all nodes
        for (const auto& vnf : sfc.vnfs) {
            all_nodes.insert(vnf.name);
            for (const auto& dep : vnf.depends_on) {
                all_nodes.insert(dep);
            }
        }

        // Build dependency list (inverse of adjacency)
        for (const auto& vnf : sfc.vnfs) {
            for (const auto& dep : vnf.depends_on) {
                dependency_list[vnf.name].insert(dep);
            }
            // Handle nodes with no dependencies
            if (vnf.depends_on.empty()) {
                dependency_list[vnf.name] = {};
            }
        }
    }

    // Add virtual nodes
    dependency_list["INGRESS"] = {};
    dependency_list["EGRESS"] = {};
    for (const auto& node : all_nodes) {
        if (dependency_list.find(node) == dependency_list.end()) {
            dependency_list[node] = {};
        }
    }

    return dependency_list;
}


std::vector<std::vector<std::string>> SFC::convert_to_parallel(
    const std::vector<std::string>& topo_order,
    const std::unordered_map<std::string, std::unordered_set<std::string>>& dependencies,
    const std::unordered_map<std::string, std::string>& vnf_types) 
{
    std::vector<std::vector<std::string>> stages;
    std::unordered_map<std::string, int> node_to_level;
    int last_monitor_level = -1;

    // Validate input sizes
    if (topo_order.empty()) {
        std::cerr << "Warning: Empty topological order\n";
        return stages;
    }

    for (const auto& node : topo_order) {
        // Skip virtual nodes
        if (node == "INGRESS" || node == "EGRESS") {
            std::cout << "\n Skipping virtual node: " << node << "\n";
            continue;
        }

        // 1. Find highest stage among dependencies (with safety checks)
        int max_parent_level = -1;
        auto deps_it = dependencies.find(node);
        
        if (deps_it != dependencies.end()) {
            for (const auto& parent : deps_it->second) {
                auto parent_it = node_to_level.find(parent);
                if (parent_it != node_to_level.end()) {
                    max_parent_level = std::max(max_parent_level, parent_it->second);
                } else {
                    std::cerr << "Warning: Parent " << parent << " not found for node " 
                              << node << "\n";
                }
            }
        } else {
            std::cerr << "Warning: No dependencies entry for node " << node << "\n";
        }

        // 2. Determine VNF type (with safety check)
        bool is_monitor = false;
        auto type_it = vnf_types.find(node);
        if (type_it != vnf_types.end()) {
            is_monitor = (type_it->second == "monitor");
        } else {
            std::cerr << "Warning: VNF type not found for node " << node << "\n";
        }

        // 3. Calculate current level with bounds checking
        int current_level;
        if (is_monitor && last_monitor_level >= max_parent_level) {
            current_level = last_monitor_level;
        } else {
            current_level = max_parent_level + 1;
        }

        // Ensure level is non-negative
        if (current_level < 0) {
            std::cerr << "Error: Negative level (" << current_level 
                      << ") for node " << node << ". Resetting to 0.\n";
            current_level = 0;
        }

        // 4. Update tracking
        node_to_level[node] = current_level;
        if (is_monitor) {
            last_monitor_level = current_level;
        }

        // 5. Ensure stages vector is safe to access
        size_t required_size = static_cast<size_t>(current_level) + 1;
        if (stages.size() < required_size) {
            try {
                stages.resize(required_size);
            } catch (const std::bad_alloc& e) {
                std::cerr << "Fatal: Failed to resize stages to " << required_size 
                          << ": " << e.what() << "\n";
                return {};
            }
        }

        // 6. Safe insertion
        stages[current_level].push_back(node);
    }

    return stages;
}






std::vector<Stage> SFC::createParallelStages(const SFC& sfc) {
    std::vector<Stage> stages;
    std::unordered_map<std::string, int> vnf_indices;
    std::vector<int> in_degree(sfc.vnfs.size(), 0);
    std::vector<int> topo_level(sfc.vnfs.size(), 0);
    
    // Build dependency graph and calculate in-degrees
    for (int i = 0; i < sfc.vnfs.size(); ++i) {
        vnf_indices[sfc.vnfs[i].name] = i;
        for (const auto& dep : sfc.vnfs[i].depends_on) {
            auto it = vnf_indices.find(dep);
            if (it != vnf_indices.end()) {
                in_degree[i]++;
            }
        }
    }
    
    // Kahn's algorithm for topological sort with level tracking
    std::queue<int> ready_queue;
    for (int i = 0; i < in_degree.size(); ++i) {
        if (in_degree[i] == 0) {
            ready_queue.push(i);
            topo_level[i] = 0; // Starting level
        }
    }
    
    while (!ready_queue.empty()) {
        int vnf_idx = ready_queue.front();
        ready_queue.pop();
        const VNF& vnf = sfc.vnfs[vnf_idx];
        
        // Ensure we have enough stages
        if (stages.size() <= topo_level[vnf_idx]) {
            stages.resize(topo_level[vnf_idx] + 1);
        }
        
        // Add VNF to its appropriate stage
        Stage& current_stage = stages[topo_level[vnf_idx]];
        current_stage.vnfs.push_back(vnf);
        current_stage.stage_latency = std::max(current_stage.stage_latency, vnf.processing_latency);
        current_stage.total_cpu += vnf.cpu_requirement;
        current_stage.total_memory += vnf.memory_requirement;
        current_stage.types.insert(vnf.type);
        
        // Update dependencies for neighbors and set their levels
        for (const auto& adj_vnf : sfc.adj.at(vnf.name)) {
            int neighbor_idx = vnf_indices[adj_vnf];
            in_degree[neighbor_idx]--;
            if (in_degree[neighbor_idx] == 0) {
                ready_queue.push(neighbor_idx);
                topo_level[neighbor_idx] = topo_level[vnf_idx] + 1;
            }
        }
    }
    
    // Optimize stages by merging compatible consecutive stages
    if (!stages.empty()) {
        std::vector<Stage> optimized_stages;
        optimized_stages.push_back(stages[0]);
        
        for (size_t i = 1; i < stages.size(); ++i) {
            bool can_merge = true;
            
            // Check if we can merge with previous stage
            // (No dependencies between these stages and compatible types)
            for (const auto& vnf : stages[i].vnfs) {
                for (const auto& dep : vnf.depends_on) {
                    auto dep_it = vnf_indices.find(dep);
                    if (dep_it != vnf_indices.end() && 
                        topo_level[dep_it->second] == i-1) {
                        can_merge = false;
                        break;
                    }
                }
                if (!can_merge) break;
            }
            
            if (can_merge) {
                // Merge with previous stage
                Stage& prev = optimized_stages.back();
                prev.vnfs.insert(prev.vnfs.end(), stages[i].vnfs.begin(), stages[i].vnfs.end());
                prev.stage_latency = std::max(prev.stage_latency, stages[i].stage_latency);
                prev.total_cpu += stages[i].total_cpu;
                prev.total_memory += stages[i].total_memory;
                prev.types.insert(stages[i].types.begin(), stages[i].types.end());
            } else {
                // Start new stage
                optimized_stages.push_back(stages[i]);
            }
        }
        
        return optimized_stages;
    }
    
    return stages;
}