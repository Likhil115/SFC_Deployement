// File: Simulation.cpp
#include "Simulation.hpp"
#include <queue>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include "Common.hpp"
#include <iostream>

NetworkSimulator::NetworkSimulator(double duration, double warmup) 
    : simulation_duration(duration), warmup_period(warmup), current_time(0.0) {
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
}

void NetworkSimulator::initialize() {
    initializeCoronetTopology();
    initializeNodeResources();
    stats = SimulationStats{};
}

void NetworkSimulator::initializeCoronetTopology() {
    // Simplified CORONET topology with 75 nodes
    nodes.resize(75);
    for (int i = 0; i < 15; ++i) nodes[i] = NodeType::CORE;
    for (int i = 15; i < 45; ++i) nodes[i] = NodeType::EDGE;
    for (int i = 45; i < 60; ++i) nodes[i] = NodeType::ACCESS;
    for (int i = 60; i < 75; ++i) nodes[i] = NodeType::SERVER;
    
    // Create links with bandwidth
    for (int i = 0; i < 15; ++i) {
        for (int j = i+1; j < 15; ++j) {
            links.push_back({i, j, 100.0, 100.0, 1.0});
            links.push_back({j, i, 100.0, 100.0, 1.0});
            link_map[{i, j}] = links.size() - 2;
            link_map[{j, i}] = links.size() - 1;
        }
    }
    
    for (int i = 15; i < 45; ++i) {
        int connections = 2 + (i % 2);
        for (int j = 0; j < connections; ++j) {
            int core = (i + j) % 15;
            links.push_back({i, core, 40.0, 40.0, 2.0});
            links.push_back({core, i, 40.0, 40.0, 2.0});
            link_map[{i, core}] = links.size() - 2;
            link_map[{core, i}] = links.size() - 1;
        }
    }
    
    for (int i = 45; i < 60; ++i) {
        int edge = 15 + (i % 30);
        links.push_back({i, edge, 10.0, 10.0, 5.0});
        links.push_back({edge, i, 10.0, 10.0, 5.0});
        link_map[{i, edge}] = links.size() - 2;
        link_map[{edge, i}] = links.size() - 1;
    }
    
    for (int i = 60; i < 75; ++i) {
        int access = 45 + (i % 15);
        links.push_back({i, access, 10.0, 10.0, 1.0});
        links.push_back({access, i, 10.0, 10.0, 1.0});
        link_map[{i, access}] = links.size() - 2;
        link_map[{access, i}] = links.size() - 1;
    }
}

void NetworkSimulator::initializeNodeResources() {
    node_resources.resize(nodes.size());
    
    std::uniform_int_distribution<int> cpu_dist(8, 16);   
    std::uniform_int_distribution<int> mem_dist(1024, 4096);  
    
    for (size_t i = 0; i < nodes.size(); ++i) {
        node_resources[i].total_cpu = cpu_dist(generator);
        node_resources[i].available_cpu = node_resources[i].total_cpu;
        node_resources[i].total_memory = mem_dist(generator);
        node_resources[i].available_memory = node_resources[i].total_memory;
        
        if (nodes[i] == NodeType::SERVER) {
            node_resources[i].has_dpu = true;
            node_resources[i].dpu_capacity = 5.0; 
            node_resources[i].dpu_available = 5.0;
        } else {
            node_resources[i].has_dpu = false;
            node_resources[i].dpu_capacity = 0.0;
            node_resources[i].dpu_available = 0.0;
        }
    }
}

std::vector<int> NetworkSimulator::shortestPath(int src, int dest, double required_bandwidth) {
    const double INF = std::numeric_limits<double>::max();
    std::vector<double> dist(nodes.size(), INF);
    std::vector<int> prev(nodes.size(), -1);
    std::priority_queue<std::pair<double, int>, 
                        std::vector<std::pair<double, int>>,
                        std::greater<std::pair<double, int>>> pq;
    
    dist[src] = 0.0;
    pq.push({0.0, src});
    
    while (!pq.empty()) {
        auto [current_dist, u] = pq.top();
        pq.pop();
        
        if (u == dest) break;
        if (current_dist > dist[u]) continue;
        
        for (const auto& link : links) {
            if (link.source == u && link.available_bandwidth >= required_bandwidth) {
                int v = link.destination;
                double new_dist = dist[u] + link.latency;
                
                if (new_dist < dist[v]) {
                    dist[v] = new_dist;
                    prev[v] = u;
                    pq.push({new_dist, v});
                }
            }
        }
    }
    
    std::vector<int> path;
    if (dist[dest] == INF) return path;
    
    for (int at = dest; at != -1; at = prev[at]) {
        path.push_back(at);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<int> NetworkSimulator::shortestPath(int src, int dest) {
    return shortestPath(src, dest, 0.0);
}

bool NetworkSimulator::allocateBandwidth(const std::vector<int>& path, double bandwidth) {
    if (path.size() < 2) return true;
    
    for (size_t i = 0; i < path.size() - 1; ++i) {
        auto it = link_map.find({path[i], path[i+1]});
        if (it == link_map.end()) return false;
        
        NetworkLink& link = links[it->second];
        if (!link.canAccommodate(bandwidth)) {
            // Rollback any already allocated bandwidth
            for (size_t j = 0; j < i; ++j) {
                auto rollback_it = link_map.find({path[j], path[j+1]});
                links[rollback_it->second].available_bandwidth += bandwidth;
            }
            return false;
        }
        link.available_bandwidth -= bandwidth;
    }
    return true;
}

void NetworkSimulator::releaseBandwidth(const std::vector<int>& path, double bandwidth) {
    if (path.size() < 2) return;
    
    for (size_t i = 0; i < path.size() - 1; ++i) {
        auto it = link_map.find({path[i], path[i+1]});
        if (it != link_map.end()) {
            links[it->second].available_bandwidth += bandwidth;
        }
    }
}

double NetworkSimulator::calculatePathLatency(const std::vector<int>& path) {
    if (path.size() < 2) return 0.0;
    
    double total_latency = 0.0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        auto it = link_map.find({path[i], path[i+1]});
        if (it != link_map.end()) {
            total_latency += links[it->second].latency;
        }
    }
    return total_latency;
}

bool NetworkSimulator::allocateResources(int node_id, const VNF& vnf, bool use_dpu, 
                                       const std::vector<int>& communication_path, 
                                       double bandwidth) {
    if (node_id < 0 || node_id >= static_cast<int>(node_resources.size())) {
        return false;
    }
    
    auto& node = node_resources[node_id];
    
    // Check node resources
    if (node.available_cpu < vnf.cpu_requirement || 
        node.available_memory < vnf.memory_requirement) {
        return false;
    }
    
    // Check DPU if needed
    if (use_dpu) {
        if (!node.has_dpu || node.dpu_available < (vnf.processing_latency * DPU_TIME_CONVERSION_FACTOR)) {
            return false;
        }
    }
    
    // Check bandwidth along the path
    if (!communication_path.empty() && !allocateBandwidth(communication_path, bandwidth)) {
        return false;
    }
    
    // Allocate resources
    node.available_cpu -= vnf.cpu_requirement;
    node.available_memory -= vnf.memory_requirement;
    if (use_dpu) {
        node.dpu_available -= (vnf.processing_latency * DPU_TIME_CONVERSION_FACTOR);
    }
    
    return true;
}

void NetworkSimulator::releaseResources(int node_id, const VNF& vnf, bool used_dpu, 
                                      const std::vector<int>& communication_path, 
                                      double bandwidth) {
    if (node_id < 0 || node_id >= static_cast<int>(node_resources.size())) return;
    
    auto& node = node_resources[node_id];
    node.available_cpu += vnf.cpu_requirement;
    node.available_memory += vnf.memory_requirement;
    if (used_dpu) {
        node.dpu_available += (vnf.processing_latency * DPU_TIME_CONVERSION_FACTOR);
    }
    
    if (!communication_path.empty()) {
        releaseBandwidth(communication_path, bandwidth);
    }
}

bool NetworkSimulator::allocateResources(int node_id, const VNF& vnf, bool use_dpu) {
    return allocateResources(node_id, vnf, use_dpu, {}, 0.0);
}

void NetworkSimulator::releaseResources(int node_id, const VNF& vnf, bool used_dpu) {
    releaseResources(node_id, vnf, used_dpu, {}, 0.0);
}

double NetworkSimulator::calculateDeploymentCost(const VNF& vnf, bool use_dpu) {
    double cost = 0.0;
    cost += vnf.cpu_requirement * CPU_CORE_COST;
    cost += vnf.memory_requirement * MEMORY_COST;
    
    if (use_dpu) {
        cost += vnf.processing_latency * DPU_COST_PER_MS;
    } else {
        cost += vnf.processing_latency * CPU_COST_PER_MS;
    }
    
    // Add bandwidth cost (assuming $0.01 per Gbps per ms)
    cost += vnf.bandwidth_requirement * 0.01 * vnf.processing_latency;
    
    return cost;
}

std::vector<int> NetworkSimulator::findAvailableDpuNodes(const VNF& vnf) {
    std::vector<int> available_nodes;
    for (int i = 60; i <= 74; ++i) {
        if (node_resources[i].available_cpu >= vnf.cpu_requirement &&
            node_resources[i].available_memory >= vnf.memory_requirement &&
            node_resources[i].dpu_available >= (vnf.processing_latency * DPU_TIME_CONVERSION_FACTOR)) {
            available_nodes.push_back(i);
        }
    }
    return available_nodes;
}

std::vector<int> NetworkSimulator::findAvailableCpuNodes(const VNF& vnf) {
    std::vector<int> available_nodes;
    for (size_t i = 0; i < nodes.size(); ++i) {
        if (node_resources[i].available_cpu >= vnf.cpu_requirement &&
            node_resources[i].available_memory >= vnf.memory_requirement) {
            available_nodes.push_back(i);
        }
    }
    return available_nodes;
}

bool NetworkSimulator::deployOptimizedSFC(const OptimizedSFC& sfc, double arrival_time) {
    SFCDeployment deployment;
    deployment.sfc_id = sfc.original_id;
    deployment.arrival_time = arrival_time;
    deployment.accepted = false;
    deployment.completed = false;
    deployment.total_latency = 0.0;
    deployment.total_cost = 0.0;
    deployment.network_latency = 0.0;
    deployment.processing_latency = 0.0;
    deployment.total_bandwidth = 0.0;
    
    std::cout << "\n=== Deploying SFC " << sfc.original_id 
              << " (Priority: " << sfc.priority << ") ===\n";
    
    if (sfc.priority == "high") {
        if (deployHighPrioritySFC(sfc, deployment)) {
            deployment.accepted = true;
            std::cout << "✓ High-priority SFC deployed successfully\n";
        } else {
            std::cout << "✗ High-priority SFC deployment failed\n";
        }
    } else {
        if (deployLowPrioritySFC(sfc, deployment)) {
            deployment.accepted = true;
            std::cout << "✓ Low-priority SFC deployed successfully\n";
        } else {
            std::cout << "✗ Low-priority SFC deployment failed\n";
        }
    }
    
    if (deployment.accepted) {
        calculateDeploymentMetrics(deployment, sfc);
        printDeploymentDetails(deployment);
    }
    
    sfc_deployments.push_back(deployment);
    return deployment.accepted;
}

bool NetworkSimulator::deployHighPrioritySFC(const OptimizedSFC& sfc, SFCDeployment& deployment) {
    std::cout << "Attempting DPU deployment for high-priority SFC...\n";
    
    // Phase 1: Try DPU deployment with bandwidth reservation
    std::vector<VNFDeployment> dpu_deployments;
    bool dpu_success = true;
    
    int prev_node = -1;
    for (const auto& stage : sfc.stages) {
        for (const auto& vnf : stage) {
            auto dpu_nodes = findAvailableDpuNodes(vnf);
            bool vnf_deployed = false;
            
            for (int node_id : dpu_nodes) {
                std::vector<int> comm_path;
                double bandwidth = vnf.bandwidth_requirement;
                
                if (prev_node != -1 && prev_node != node_id) {
                    comm_path = shortestPath(prev_node, node_id, bandwidth);
                    if (comm_path.empty()) continue;
                }
                
                if (allocateResources(node_id, vnf, true, comm_path, bandwidth)) {
                    VNFDeployment vd;
                    vd.node_id = node_id;
                    vd.start_time = current_time;
                    vd.finish_time = current_time + (vnf.processing_latency * DPU_TIME_CONVERSION_FACTOR);
                    vd.uses_dpu = true;
                    vd.cost = calculateDeploymentCost(vnf, true);
                    vd.communication_path = comm_path;
                    vd.bandwidth_reserved = bandwidth;
                    dpu_deployments.push_back(vd);
                    vnf_deployed = true;
                    prev_node = node_id;
                    
                    std::cout << "  DPU: VNF " << vnf.name << " → Node " << node_id 
                              << " (Cost: $" << vd.cost << ", BW: " << bandwidth << "Gbps)\n";
                    break;
                }
            }
            
            if (!vnf_deployed) {
                dpu_success = false;
                break;
            }
        }
        if (!dpu_success) break;
    }
    
    if (dpu_success) {
        deployment.vnf_deployments = dpu_deployments;
        stats.high_priority_dpu_usage++;
        std::cout << "DPU deployment successful!\n";
        return true;
    } else {
        // Release allocated DPU resources
        for (size_t i = 0; i < dpu_deployments.size(); ++i) {
            releaseResources(dpu_deployments[i].node_id, 
                           sfc.stages[i / sfc.stages[0].size()][i % sfc.stages[0].size()], 
                           true,
                           dpu_deployments[i].communication_path,
                           dpu_deployments[i].bandwidth_reserved);
        }
        std::cout << "DPU deployment failed, trying CPU fallback...\n";
    }
    
    // Phase 2: CPU fallback with bandwidth reservation
    std::vector<VNFDeployment> cpu_deployments;
    bool cpu_success = true;
    prev_node = -1;
    
    for (const auto& stage : sfc.stages) {
        for (const auto& vnf : stage) {
            auto cpu_nodes = findAvailableCpuNodes(vnf);
            bool vnf_deployed = false;
            
            for (int node_id : cpu_nodes) {
                std::vector<int> comm_path;
                double bandwidth = vnf.bandwidth_requirement;
                
                if (prev_node != -1 && prev_node != node_id) {
                    comm_path = shortestPath(prev_node, node_id, bandwidth);
                    if (comm_path.empty()) continue;
                }
                
                if (allocateResources(node_id, vnf, false, comm_path, bandwidth)) {
                    VNFDeployment vd;
                    vd.node_id = node_id;
                    vd.start_time = current_time;
                    vd.finish_time = current_time + vnf.processing_latency;
                    vd.uses_dpu = false;
                    vd.cost = calculateDeploymentCost(vnf, false);
                    vd.communication_path = comm_path;
                    vd.bandwidth_reserved = bandwidth;
                    cpu_deployments.push_back(vd);
                    vnf_deployed = true;
                    prev_node = node_id;
                    
                    std::cout << "  CPU: VNF " << vnf.name << " → Node " << node_id 
                              << " (Cost: $" << vd.cost << ", BW: " << bandwidth << "Gbps)\n";
                    break;
                }
            }
            
            if (!vnf_deployed) {
                cpu_success = false;
                break;
            }
        }
        if (!cpu_success) break;
    }
    
    if (cpu_success) {
        deployment.vnf_deployments = cpu_deployments;
        std::cout << "CPU fallback deployment successful!\n";
        return true;
    } else {
        // Release allocated CPU resources
        for (size_t i = 0; i < cpu_deployments.size(); ++i) {
            releaseResources(cpu_deployments[i].node_id, 
                           sfc.stages[i / sfc.stages[0].size()][i % sfc.stages[0].size()], 
                           false,
                           cpu_deployments[i].communication_path,
                           cpu_deployments[i].bandwidth_reserved);
        }
        std::cout << "CPU fallback deployment failed!\n";
    }
    
    return false;
}

bool NetworkSimulator::deployLowPrioritySFC(const OptimizedSFC& sfc, SFCDeployment& deployment) {
    std::cout << "Attempting optimal CPU deployment for low-priority SFC...\n";
    
    std::vector<VNFDeployment> cpu_deployments;
    bool success = true;
    int prev_node = -1;
    
    for (const auto& stage : sfc.stages) {
        for (const auto& vnf : stage) {
            auto cpu_nodes = findAvailableCpuNodes(vnf);
            
            // Sort nodes by utilization (prefer less loaded nodes)
            std::sort(cpu_nodes.begin(), cpu_nodes.end(), [this](int a, int b) {
                double util_a = 1.0 - (double)node_resources[a].available_cpu / node_resources[a].total_cpu;
                double util_b = 1.0 - (double)node_resources[b].available_cpu / node_resources[b].total_cpu;
                return util_a < util_b;
            });
            
            bool vnf_deployed = false;
            for (int node_id : cpu_nodes) {
                std::vector<int> comm_path;
                double bandwidth = vnf.bandwidth_requirement;
                
                if (prev_node != -1 && prev_node != node_id) {
                    comm_path = shortestPath(prev_node, node_id, bandwidth);
                    if (comm_path.empty()) continue;
                }
                
                if (allocateResources(node_id, vnf, false, comm_path, bandwidth)) {
                    VNFDeployment vd;
                    vd.node_id = node_id;
                    vd.start_time = current_time;
                    vd.finish_time = current_time + vnf.processing_latency;
                    vd.uses_dpu = false;
                    vd.cost = calculateDeploymentCost(vnf, false);
                    vd.communication_path = comm_path;
                    vd.bandwidth_reserved = bandwidth;
                    cpu_deployments.push_back(vd);
                    vnf_deployed = true;
                    prev_node = node_id;
                    
                    std::cout << "  CPU: VNF " << vnf.name << " → Node " << node_id 
                              << " (Cost: $" << vd.cost << ", BW: " << bandwidth << "Gbps)\n";
                    break;
                }
            }
            
            if (!vnf_deployed) {
                success = false;
                break;
            }
        }
        if (!success) break;
    }
    
    if (success) {
        deployment.vnf_deployments = cpu_deployments;
        return true;
    } else {
        // Release allocated resources
        for (size_t i = 0; i < cpu_deployments.size(); ++i) {
            releaseResources(cpu_deployments[i].node_id, 
                           sfc.stages[i / sfc.stages[0].size()][i % sfc.stages[0].size()], 
                           false,
                           cpu_deployments[i].communication_path,
                           cpu_deployments[i].bandwidth_reserved);
        }
        return false;
    }
}

void NetworkSimulator::calculateDeploymentMetrics(SFCDeployment& deployment, const OptimizedSFC& sfc) {
    deployment.total_cost = 0.0;
    deployment.processing_latency = 0.0;
    deployment.network_latency = 0.0;
    deployment.total_bandwidth = 0.0;
    
    // Calculate processing latency and cost
    double max_finish_time = 0.0;
    for (const auto& vd : deployment.vnf_deployments) {
        deployment.total_cost += vd.cost;
        max_finish_time = std::max(max_finish_time, vd.finish_time);
        deployment.processing_latency += (vd.finish_time - vd.start_time);
        deployment.total_bandwidth += vd.bandwidth_reserved;
    }
    
    // Calculate network latency between consecutive VNFs
    if (deployment.vnf_deployments.size() > 1) {
        for (size_t i = 0; i < deployment.vnf_deployments.size() - 1; ++i) {
            int src = deployment.vnf_deployments[i].node_id;
            int dest = deployment.vnf_deployments[i+1].node_id;
            
            if (src != dest) {
                if (!deployment.vnf_deployments[i+1].communication_path.empty()) {
                    deployment.network_latency += calculatePathLatency(
                        deployment.vnf_deployments[i+1].communication_path);
                } else {
                    auto path = shortestPath(src, dest);
                    if (!path.empty()) {
                        deployment.network_latency += calculatePathLatency(path);
                    }
                }
            }
        }
    }
    
    deployment.total_latency = deployment.processing_latency + deployment.network_latency;
    deployment.completion_time = max_finish_time;
    deployment.completed = true;
    
    // Update bandwidth statistics
    stats.total_bandwidth_utilized += deployment.total_bandwidth;
}

void NetworkSimulator::printDeploymentDetails(const SFCDeployment& deployment) {
    std::cout << "--- Deployment Summary ---\n";
    std::cout << "Total Cost: $" << deployment.total_cost << "\n";
    std::cout << "Processing Latency: " << deployment.processing_latency << " ms\n";
    std::cout << "Network Latency: " << deployment.network_latency << " ms\n";
    std::cout << "Total Latency: " << deployment.total_latency << " ms\n";
    std::cout << "Total Bandwidth Reserved: " << deployment.total_bandwidth << " Gbps\n";
    std::cout << "Deployed on nodes: ";
    for (const auto& vd : deployment.vnf_deployments) {
        std::cout << vd.node_id << (vd.uses_dpu ? "(DPU)" : "(CPU)") << " ";
    }
    std::cout << "\n";
}

void NetworkSimulator::runDeployment(const std::vector<SFC>& sfcs, bool useOriginal) {
    std::cout << "=== Starting Network Simulation ===\n";
    std::cout << "Simulation duration: " << simulation_duration 
              << ", Warmup period: " << warmup_period << "\n";
    
    // Create optimized SFCs
    std::unordered_map<std::string, std::string> vnf_types;
    for (const auto& sfc : sfcs) {
        for (const auto& vnf : sfc.vnfs) {
            vnf_types[vnf.name] = vnf.type;
        }
    }
    
    auto optimized_sfcs = useOriginal ? 
        createOptimizedSFCs(sfcs, vnf_types) : 
        createUnifiedTopoSFCs(sfcs, vnf_types);
    
    // Simulate arrivals
    std::exponential_distribution<double> arrival_dist(1.0 / 15.0);
    size_t sfc_index = 0;
    
    while (current_time < simulation_duration && sfc_index < optimized_sfcs.size()) {
        const auto& sfc = optimized_sfcs[sfc_index++];
        double arrival_time = current_time;
        current_time += arrival_dist(generator);
        
        bool accepted = deployOptimizedSFC(sfc, arrival_time);
        
        if (current_time > warmup_period) {
            stats.total_sfcs += sfc.is_merged ? sfc.contained_sfcs : 1;
            
            if (accepted) {
                stats.accepted_sfcs += sfc.is_merged ? sfc.contained_sfcs : 1;
                const auto& deployment = sfc_deployments.back();
                
                if (sfc.priority == "high") {
                    stats.high_priority_sfcs++;
                    if (deployment.vnf_deployments[0].uses_dpu) {
                        stats.high_priority_dpu_usage++;
                    }
                } else {
                    stats.low_priority_sfcs += sfc.is_merged ? sfc.contained_sfcs : 1;
                }
                
                stats.total_processing_latency += deployment.processing_latency;
                stats.total_network_latency += deployment.network_latency;
                stats.total_deployment_cost += deployment.total_cost;
                stats.total_bandwidth_utilized += deployment.total_bandwidth;
            }
        }
    }
    
    finalizeStatistics();
    std::cout << "\n=== Simulation Complete ===\n";
}

void NetworkSimulator::finalizeStatistics() {
    if (stats.total_sfcs > 0) {
        stats.avg_acceptance_rate = static_cast<double>(stats.accepted_sfcs) / stats.total_sfcs;
    }

    if (stats.accepted_sfcs > 0) {
        stats.avg_processing_latency = stats.total_processing_latency / stats.accepted_sfcs;
        stats.avg_network_latency = stats.total_network_latency / stats.accepted_sfcs;
        stats.avg_end_to_end_latency = stats.avg_processing_latency + stats.avg_network_latency;
        stats.avg_deployment_cost = stats.total_deployment_cost / stats.accepted_sfcs;
        stats.avg_bandwidth_utilization = stats.total_bandwidth_utilized / stats.accepted_sfcs;
    }
    
    calculateResourceUtilization();
    printFinalStatistics();
}

void NetworkSimulator::calculateResourceUtilization() {
    double total_cpu = 0.0, used_cpu = 0.0;
    double total_dpu = 0.0, used_dpu = 0.0;
    double total_bandwidth = 0.0, used_bandwidth = 0.0;
    
    for (const auto& res : node_resources) {
        total_cpu += res.total_cpu;
        used_cpu += (res.total_cpu - res.available_cpu);
        
        if (res.has_dpu) {
            total_dpu += res.dpu_capacity;
            used_dpu += (res.dpu_capacity - res.dpu_available);
        }
    }
    
    for (const auto& link : links) {
        total_bandwidth += link.total_bandwidth;
        used_bandwidth += (link.total_bandwidth - link.available_bandwidth);
    }
    
    stats.resource_utilization = (total_cpu > 0) ? used_cpu / total_cpu : 0.0;
    stats.dpu_utilization = (total_dpu > 0) ? used_dpu / total_dpu : 0.0;
    
    if (stats.high_priority_dpu_usage + stats.low_priority_dpu_usage > 0) {
        stats.dpu_priority_ratio = static_cast<double>(stats.high_priority_dpu_usage) / 
                                   (stats.high_priority_dpu_usage + stats.low_priority_dpu_usage);
    }
    
    if (total_bandwidth > 0) {
        stats.avg_bandwidth_utilization = used_bandwidth / total_bandwidth;
    }
}

void NetworkSimulator::printFinalStatistics() {
    std::cout << "\n=== Final Statistics ===\n";
    std::cout << "Total SFCs: " << stats.total_sfcs << "\n";
    std::cout << "Accepted SFCs: " << stats.accepted_sfcs << "\n";
    std::cout << "Acceptance Rate: " << (stats.avg_acceptance_rate * 100.0) << "%\n";
    std::cout << "High Priority SFCs: " << stats.high_priority_sfcs << "\n";
    std::cout << "Low Priority SFCs: " << stats.low_priority_sfcs << "\n";
    std::cout << "Average Processing Latency: " << stats.avg_processing_latency << " ms\n";
    std::cout << "Average Network Latency: " << stats.avg_network_latency << " ms\n";
    std::cout << "Average End-to-End Latency: " << stats.avg_end_to_end_latency << " ms\n";
    std::cout << "Average Deployment Cost: $" << stats.avg_deployment_cost << "\n";
    std::cout << "CPU Utilization: " << (stats.resource_utilization * 100.0) << "%\n";
    std::cout << "DPU Utilization: " << (stats.dpu_utilization * 100.0) << "%\n";
    std::cout << "DPU Priority Ratio: " << (stats.dpu_priority_ratio * 100.0) << "%\n";
    std::cout << "Average Bandwidth Utilization: " << (stats.avg_bandwidth_utilization * 100.0) << "%\n";
}

void NetworkSimulator::saveDeploymentLog(const std::string& filename) {
    std::ofstream out(filename);
    if (!out) return;
    
    out << "SFC_ID,Priority,Accepted,CompletionTime,ProcessingLatency,NetworkLatency,TotalLatency,DeploymentCost,BandwidthUsed,NodeDeployments\n";
    for (const auto& dep : sfc_deployments) {
        out << dep.sfc_id << "," 
            << (dep.sfc_id > 0 ? "high" : "low") << ","
            << (dep.accepted ? "true" : "false") << ","
            << dep.completion_time << ","
            << dep.processing_latency << ","
            << dep.network_latency << ","
            << dep.total_latency << ","
            << dep.total_cost << ","
            << dep.total_bandwidth << ",\"";
        
        for (const auto& vd : dep.vnf_deployments) {
            out << "N" << vd.node_id << (vd.uses_dpu ? "(DPU)" : "(CPU)") << ":" << vd.cost;
            if (&vd != &dep.vnf_deployments.back()) out << " ";
        }
        out << "\"\n";
    }
}

void NetworkSimulator::saveStatistics(const std::string& filename) {
    std::ofstream out(filename);
    if (!out) return;
    
    out << "Metric,Value\n"
        << "Total_SFCs," << stats.total_sfcs << "\n"
        << "Accepted_SFCs," << stats.accepted_sfcs << "\n"
        << "High_Priority_SFCs," << stats.high_priority_sfcs << "\n"
        << "Low_Priority_SFCs," << stats.low_priority_sfcs << "\n"
        << "Acceptance_Rate," << stats.avg_acceptance_rate << "\n"
        << "Avg_Processing_Latency," << stats.avg_processing_latency << "\n"
        << "Avg_Network_Latency," << stats.avg_network_latency << "\n"
        << "Avg_End_to_End_Latency," << stats.avg_end_to_end_latency << "\n"
        << "Avg_Deployment_Cost," << stats.avg_deployment_cost << "\n"
        << "CPU_Utilization," << stats.resource_utilization << "\n"
        << "DPU_Utilization," << stats.dpu_utilization << "\n"
        << "DPU_Priority_Ratio," << stats.dpu_priority_ratio << "\n"
        << "DPU_Allocations," << stats.dpu_allocations << "\n"
        << "High_Priority_DPU_Usage," << stats.high_priority_dpu_usage << "\n"
        << "Low_Priority_DPU_Usage," << stats.low_priority_dpu_usage << "\n"
        << "Avg_Bandwidth_Utilization," << stats.avg_bandwidth_utilization << "\n"
        << "Total_Bandwidth_Utilized," << stats.total_bandwidth_utilized << "\n";
}