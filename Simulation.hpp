// File: Simulation.hpp
#pragma once

#include "SFC.hpp"
#include <vector>
#include <map>
#include <unordered_map>
#include <random>
#include <chrono>
#include <fstream>
#include <stdexcept>

enum class NodeType {
    CORE, EDGE, ACCESS, SERVER, REGULAR
};

struct NodeResources {
    int total_cpu = 0;
    int available_cpu = 0;
    int total_memory = 0;
    int available_memory = 0;
    bool has_dpu = false;
    double dpu_capacity = 0.0;
    double dpu_available = 0.0;
};

struct NetworkLink {
    int source;
    int destination;
    double total_bandwidth;  // in Gbps
    double available_bandwidth; // in Gbps
    double latency;         // in ms
    
    bool canAccommodate(double required) const {
        return available_bandwidth >= required;
    }
};

struct VNFDeployment {
    int node_id = -1;
    double start_time = 0.0;
    double finish_time = 0.0;
    bool uses_dpu = false;
    double cost = 0.0;
    std::vector<int> communication_path;
    double bandwidth_reserved = 0.0;
};

struct SFCDeployment {
    int sfc_id = -1;
    std::vector<VNFDeployment> vnf_deployments;
    double arrival_time = 0.0;
    double completion_time = 0.0;
    bool accepted = false;
    bool completed = false;
    double total_latency = 0.0;
    double total_cost = 0.0;
    double processing_latency = 0.0;
    double network_latency = 0.0;
    double total_bandwidth = 0.0;
    std::vector<int> communication_path;
};

struct NodeUtilization {
    int node_id;
    double dpu_utilization;
    double cpu_utilization;
    double memory_utilization;
};

struct SimulationStats {
    int total_sfcs = 0;
    int accepted_sfcs = 0;
    int high_priority_sfcs = 0;
    int low_priority_sfcs = 0;
    
    double total_processing_latency = 0.0;
    double total_network_latency = 0.0;
    double avg_acceptance_rate = 0.0;
    double avg_processing_latency = 0.0;
    double avg_network_latency = 0.0;
    double avg_end_to_end_latency = 0.0;
    
    double resource_utilization = 0.0;
    double dpu_utilization = 0.0;
    
    int dpu_allocations = 0;
    int low_priority_dpu_usage = 0;
    int high_priority_dpu_usage = 0;
    double total_dpu_utilized = 0.0;
    double dpu_priority_ratio = 0.0;
    
    double total_deployment_cost = 0.0;
    double avg_deployment_cost = 0.0;

    double total_bandwidth_utilized = 0.0;
    double avg_bandwidth_utilization = 0.0;
    int bandwidth_constrained_rejections = 0;
    
    std::vector<NodeUtilization> dpu_nodes_utilization;
    std::vector<NodeUtilization> cpu_nodes_utilization;
};

class NetworkSimulator {
private:
    static constexpr double DPU_TIME_CONVERSION_FACTOR = 0.5;
    static constexpr double DPU_COST_PER_MS = 0.1;
    static constexpr double CPU_COST_PER_MS = 0.05;
    static constexpr double CPU_CORE_COST = 0.01;
    static constexpr double MEMORY_COST = 0.001;
    
    std::vector<NodeType> nodes;
    std::vector<NodeResources> node_resources;
    std::vector<NetworkLink> links;
    std::map<std::pair<int, int>, int> link_map;
    std::vector<SFCDeployment> sfc_deployments;
    std::default_random_engine generator;
    
    double current_time = 0.0;
    double simulation_duration = 0.0;
    double warmup_period = 0.0;
    
    SimulationStats stats;
    
    struct NodeScore {
        int id;
        double score;
        bool operator<(const NodeScore& other) const {
            return score > other.score;
        }
    };

    void initializeCoronetTopology();
    void initializeNodeResources();
    std::vector<int> shortestPath(int src, int dest, double required_bandwidth);
    double calculatePathLatency(const std::vector<int> &path);
    bool allocateBandwidth(const std::vector<int>& path, double bandwidth);
    void releaseBandwidth(const std::vector<int>& path, double bandwidth);
    bool allocateResources(int node_id, const VNF &vnf, bool use_dpu, const std::vector<int> &communication_path, double bandwidth);
    void releaseResources(int node_id, const VNF &vnf, bool used_dpu, const std::vector<int> &communication_path, double bandwidth);
    std::vector<int> shortestPath(int src, int dest);
    bool allocateResources(int node_id, const VNF& vnf, bool use_dpu);
    void releaseResources(int node_id, const VNF& vnf, bool used_dpu);
    bool validateDeployment(const SFCDeployment& deployment, const OptimizedSFC& sfc);
    std::vector<int> selectBestNodes(const VNF& vnf, bool use_dpu);
    std::vector<int> findAvailableDpuNodes(const VNF& vnf);
    std::vector<int> findAvailableCpuNodes(const VNF &vnf);
    double calculateDeploymentCost(const VNF &vnf, bool use_dpu);

public:
    NetworkSimulator(double duration = 36000.0, double warmup = 300.0);
    
    void initialize();
    bool deploySFC(const SFC& sfc, double arrival_time);
    bool deployOptimizedSFC(const OptimizedSFC& sfc, double arrival_time);
    bool deployHighPrioritySFC(const OptimizedSFC &sfc, SFCDeployment &deployment);
    bool deployLowPrioritySFC(const OptimizedSFC &sfc, SFCDeployment &deployment);
    void calculateDeploymentMetrics(SFCDeployment &deployment, const OptimizedSFC &sfc);
    void printDeploymentDetails(const SFCDeployment &deployment);
    void runDeployment(const std::vector<SFC> &sfcs, bool useOriginal = true);
    void finalizeStatistics();
    void calculateResourceUtilization();
    void printFinalStatistics();

    const SimulationStats& getStatistics() const { return stats; }
    const std::vector<SFCDeployment>& getDeployments() const { return sfc_deployments; }
    
    void saveDeploymentLog(const std::string& filename);
    void saveStatistics(const std::string& filename);
};