#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <functional>

#include <algorithm>
#include <stdexcept>
#include <iostream>



struct VNF {
    std::string name;
    std::string type;
    double processing_latency;
    int cpu_requirement;
    int memory_requirement;
    double bandwidth_requirement;
    std::vector<std::string> depends_on;
};
struct Stage {
    std::vector<VNF> vnfs;
    double stage_latency; // Max latency in this stage
    int total_cpu;
    int total_memory;
    std::unordered_set<std::string> types; // Types present in this stage
};


struct OptimizedSFC {
    int original_id;
    std::string priority;
    std::vector<std::vector<VNF>> stages; // For parallel deployment
    std::vector<VNF> merged_vnfs;         // For merged low-priority SFCs
    bool is_merged;
    int contained_sfcs;
};

class SFC {
public:
    // Metadata
    int request_id;
    double expected_latency;
    int ttl;
    double arrival_rate;
    std::string priority;
    std::vector<VNF> vnfs;
    std::unordered_map<std::string, std::unordered_set<std::string>> adj;



    

     // Core methods
    void classify(double latency_threshold);
    void buildDependencyDAG();



     static std::unordered_map<std::string, std::unordered_set<std::string>> 
    buildLowPriorityDependencyList(const std::vector<SFC>& low_priority_sfcs);

    static std::vector<std::vector<std::string>> 
    convert_to_parallel(
    const std::vector<std::string>& topo_order,
    const std::unordered_map<std::string, std::unordered_set<std::string>>& dependencies,
    const std::unordered_map<std::string, std::string>& vnf_types);

    static std::vector<std::string> mergeAndSortSFCs(const std::vector<SFC>& sfcs);

    static bool hasCycle(const std::unordered_map<std::string, std::vector<std::string>>& adj);

    static bool validateTopoOrder(
        const std::vector<std::string>& topoOrder,
        const std::unordered_map<std::string, std::vector<std::string>>& adj
    );

    bool validateParallelStages(
        const std::vector<std::vector<std::string>>& stages,
        const std::unordered_map<std::string, std::vector<std::string>>& adj
    );

    static std::vector<Stage> createParallelStages(const SFC& sfc);


private:
    static void mergeDAGs(std::unordered_map<std::string, std::vector<std::string>>& mergedAdj, const SFC& sfc);
    static std::vector<std::string> topologicalSort(const std::unordered_map<std::string, std::vector<std::string>>& adj);
    static bool hasCycleDFS(
        const std::string& node,
        std::unordered_map<std::string, int>& visited,
        const std::unordered_map<std::string, std::vector<std::string>>& adj
    );



};