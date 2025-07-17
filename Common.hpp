// CommonFunctions.hpp
#pragma once
#include "sfc.hpp"
#include <unordered_set>
#include <algorithm>

namespace {
    // Configuration constants
    const size_t MAX_BATCH_SIZE = 10; // Optimal batch size for performance vs complexity
    const size_t MIN_BATCH_SIZE = 2;  // Minimum size to consider batching
    
    // Helper function for cycle detection
    bool hasCycleDFS(const std::string& node, 
                    const std::unordered_map<std::string, std::vector<std::string>>& graph,
                    std::unordered_map<std::string, int>& visited) {
        visited[node] = 1; // Mark as visiting
        
        for (const auto& neighbor : graph.at(node)) {
            if (visited[neighbor] == 1) return true;
            if (visited[neighbor] == 0 && hasCycleDFS(neighbor, graph, visited)) {
                return true;
            }
        }
        
        visited[node] = 2; // Mark as visited
        return false;
    }

    // Check if dependency graph has cycles
    bool hasCycles(const std::unordered_map<std::string, std::vector<std::string>>& depGraph) {
        std::unordered_map<std::string, int> visited;
        for (const auto& node : depGraph) {
            visited[node.first] = 0;
        }
        
        for (const auto& node : depGraph) {
            if (visited[node.first] == 0 && hasCycleDFS(node.first, depGraph, visited)) {
                return true;
            }
        }
        return false;
    }

    // Build combined dependency graph for a batch
    std::unordered_map<std::string, std::vector<std::string>> 
    buildCombinedDependencyGraph(const std::vector<SFC>& batch) {
        std::unordered_map<std::string, std::vector<std::string>> depGraph;
        
        for (const auto& sfc : batch) {
            for (const auto& vnf : sfc.vnfs) {
                // Ensure all VNFs are in the graph
                if (!depGraph.count(vnf.name)) {
                    depGraph[vnf.name] = {};
                }
                // Add dependencies
                for (const auto& dep : vnf.depends_on) {
                    depGraph[dep].push_back(vnf.name);
                }
            }
        }
        
        return depGraph;
    }

    // Smart batch grouping to prevent cycles with size limits
    std::vector<std::vector<SFC>> groupAcyclicBatches(const std::vector<SFC>& lowPriority) {
        std::vector<std::vector<SFC>> batches;
        std::unordered_map<std::string, std::vector<std::string>> globalDepGraph;
        
        for (const auto& sfc : lowPriority) {
            bool added = false;
            
            // Try adding to existing batches (respecting size limit)
            for (auto& batch : batches) {
                // Check if batch is already at maximum size
                if (batch.size() >= MAX_BATCH_SIZE) {
                    continue;
                }
                
                auto testBatch = batch;
                testBatch.push_back(sfc);
                auto testGraph = buildCombinedDependencyGraph(testBatch);
                
                if (!hasCycles(testGraph)) {
                    batch.push_back(sfc);
                    globalDepGraph = testGraph; // Update global graph
                    added = true;
                    break;
                }
            }
            
            // Create new batch if couldn't add to existing ones
            if (!added) {
                batches.push_back({sfc});
                globalDepGraph = buildCombinedDependencyGraph({sfc});
            }
        }
        
        return batches;
    }

    // Alternative: Size-first batching with cycle checking
    std::vector<std::vector<SFC>> groupSizeConstrainedBatches(const std::vector<SFC>& lowPriority) {
        std::vector<std::vector<SFC>> batches;
        
        // Create initial batches based on size limit
        for (size_t i = 0; i < lowPriority.size(); i += MAX_BATCH_SIZE) {
            size_t end = std::min(i + MAX_BATCH_SIZE, lowPriority.size());
            std::vector<SFC> batch(lowPriority.begin() + i, lowPriority.begin() + end);
            
            // Check if this batch creates cycles
            auto testGraph = buildCombinedDependencyGraph(batch);
            if (hasCycles(testGraph)) {
                // If cycles detected, break down into smaller batches
                std::cout << "Cycle detected in batch of size " << batch.size() 
                         << ", breaking down...\n";
                
                // Try splitting in half
                if (batch.size() >= 4) {
                    size_t mid = batch.size() / 2;
                    std::vector<SFC> batch1(batch.begin(), batch.begin() + mid);
                    std::vector<SFC> batch2(batch.begin() + mid, batch.end());
                    
                    // Recursively check smaller batches
                    auto subBatches1 = groupSizeConstrainedBatches(batch1);
                    auto subBatches2 = groupSizeConstrainedBatches(batch2);
                    
                    batches.insert(batches.end(), subBatches1.begin(), subBatches1.end());
                    batches.insert(batches.end(), subBatches2.begin(), subBatches2.end());
                } else {
                    // Process individually if small batch still has cycles
                    for (const auto& sfc : batch) {
                        batches.push_back({sfc});
                    }
                }
            } else {
                batches.push_back(batch);
            }
        }
        
        return batches;
    }
}

inline std::vector<OptimizedSFC> createOptimizedSFCs(
    const std::vector<SFC>& sfcs, 
    const std::unordered_map<std::string, std::string>& vnf_types) 
{
    std::vector<OptimizedSFC> optimized;
    const double LAT_THRESH = 10.0;
    
    // Separate high and low priority
    std::vector<SFC> lowPriority, highPriority;
    for (const auto& s : sfcs) {
        const_cast<SFC&>(s).classify(LAT_THRESH);
        (s.priority == "high" ? highPriority : lowPriority).push_back(s);
    }

    // Process high-priority SFCs (unchanged)
    for (const auto& s : highPriority) {
        OptimizedSFC opt;
        opt.original_id = s.request_id;
        opt.priority = "high";
        opt.is_merged = false;
        
        auto stages = SFC::createParallelStages(s);
        for (const auto& stage : stages) {
            opt.stages.push_back(stage.vnfs);
        }
        optimized.push_back(opt);
    }

    // Process low-priority SFCs with size-constrained cycle-aware merging
    if (!lowPriority.empty()) {
        if (lowPriority.size() == 1) {
            // Single low-priority SFC
            const auto& s = lowPriority[0];
            OptimizedSFC opt;
            opt.original_id = s.request_id;
            opt.priority = "low";
            opt.is_merged = false;
            
            auto stages = SFC::createParallelStages(s);
            for (const auto& stage : stages) {
                opt.stages.push_back(stage.vnfs);
            }
            optimized.push_back(opt);
        } else {
            std::cout << "Optimizing " << lowPriority.size() << " low-priority SFCs with batch size limit of " 
                     << MAX_BATCH_SIZE << "...\n";
            
            // Group into size-constrained acyclic batches
            // You can choose between two strategies:
            // 1. Cycle-first approach (prioritizes avoiding cycles)
            auto batches = groupAcyclicBatches(lowPriority);
            
            // 2. Size-first approach (prioritizes batch size, then checks cycles)
            // auto batches = groupSizeConstrainedBatches(lowPriority);
            
            std::cout << "Created " << batches.size() << " batches\n";
            
            for (size_t batch_idx = 0; batch_idx < batches.size(); ++batch_idx) {
                const auto& batch = batches[batch_idx];
                
                if (batch.size() == 1) {
                    // Process individually if couldn't merge
                    OptimizedSFC opt;
                    opt.original_id = batch[0].request_id;
                    opt.priority = "low";
                    opt.is_merged = false;
                    
                    auto stages = SFC::createParallelStages(batch[0]);
                    for (const auto& stage : stages) {
                        opt.stages.push_back(stage.vnfs);
                    }
                    optimized.push_back(opt);
                    continue;
                }

                std::cout << "Merging batch " << (batch_idx + 1) << " of " << batch.size() << " SFCs...\n";
                
                // Merge the batch
                auto mergedOrder = SFC::mergeAndSortSFCs(batch);
                auto deps = SFC::buildLowPriorityDependencyList(batch);
                
                std::cout << "Merged Order: ";
                for (const auto& vnf : mergedOrder) std::cout << vnf << " ";
                std::cout << "\n";

                OptimizedSFC opt;
                opt.original_id = -(optimized.size() + 1);
                opt.priority = "low";
                opt.is_merged = true;
                opt.contained_sfcs = batch.size();
                
                // Normalize VNFs
                std::unordered_map<std::string, VNF> normalized_vnfs;
                for (const auto& sfc : batch) {
                    for (const auto& vnf : sfc.vnfs) {
                        if (normalized_vnfs.count(vnf.name)) {
                            auto& existing = normalized_vnfs[vnf.name];
                            existing.cpu_requirement = std::max(existing.cpu_requirement, vnf.cpu_requirement);
                            existing.memory_requirement = std::max(existing.memory_requirement, vnf.memory_requirement);
                            existing.processing_latency = std::max(existing.processing_latency, vnf.processing_latency);
                            
                            existing.depends_on.insert(existing.depends_on.end(), 
                                                    vnf.depends_on.begin(), vnf.depends_on.end());
                            std::sort(existing.depends_on.begin(), existing.depends_on.end());
                            existing.depends_on.erase(std::unique(existing.depends_on.begin(), 
                                                             existing.depends_on.end()), 
                                                 existing.depends_on.end());
                        } else {
                            normalized_vnfs[vnf.name] = vnf;
                        }
                    }
                }

                // Build parallel stages
                auto stages = SFC::convert_to_parallel(mergedOrder, deps, vnf_types);
                for (const auto& stage_vnfs : stages) {
                    Stage stage;
                    double stage_latency = 0.0;
                    int stage_cpu = 0;
                    int stage_memory = 0;
                    std::unordered_set<std::string> stage_types;
                    
                    for (const auto& vnf_name : stage_vnfs) {
                        if (normalized_vnfs.count(vnf_name)) {
                            const VNF& vnf = normalized_vnfs.at(vnf_name);
                            stage.vnfs.push_back(vnf);
                            stage_latency = std::max(stage_latency, vnf.processing_latency);
                            stage_cpu += vnf.cpu_requirement;
                            stage_memory += vnf.memory_requirement;
                            stage_types.insert(vnf.type);
                        }
                    }
                    
                    stage.stage_latency = stage_latency;
                    stage.total_cpu = stage_cpu;
                    stage.total_memory = stage_memory;
                    stage.types = stage_types;
                    
                    opt.stages.push_back(stage.vnfs);
                }
                
                optimized.push_back(opt);
            }
        }
    }

    return optimized;
}

inline std::vector<OptimizedSFC> createUnifiedTopoSFCs(
    const std::vector<SFC>& sfcs, 
    const std::unordered_map<std::string, std::string>& vnf_types) 
{
    std::vector<OptimizedSFC> optimized;
    const double LAT_THRESH = 10.0;
    
    // Separate high and low priority
    std::vector<SFC> lowPriority, highPriority;
    for (const auto& s : sfcs) {
        const_cast<SFC&>(s).classify(LAT_THRESH);
        (s.priority == "high" ? highPriority : lowPriority).push_back(s);
    }

    // Process high-priority SFCs (unchanged)
    for (const auto& s : highPriority) {
        OptimizedSFC opt;
        opt.original_id = s.request_id;
        opt.priority = "high";
        opt.is_merged = false;
        
        auto stages = SFC::createParallelStages(s);
        for (const auto& stage : stages) {
            opt.stages.push_back(stage.vnfs);
        }
        optimized.push_back(opt);
    }

    // Process low-priority SFCs with size constraint
    if (!lowPriority.empty()) {
        if (lowPriority.size() == 1) {
            // Single low-priority SFC
            const auto& s = lowPriority[0];
            OptimizedSFC opt;
            opt.original_id = s.request_id;
            opt.priority = "low";
            opt.is_merged = false;
            
            auto stages = SFC::createParallelStages(s);
            for (const auto& stage : stages) {
                opt.stages.push_back(stage.vnfs);
            }
            optimized.push_back(opt);
        } 
        else {
            // Check if we should limit the unified approach
            if (lowPriority.size() <= MAX_BATCH_SIZE) {
                // Use unified approach for smaller sets
                auto mergedOrder = SFC::mergeAndSortSFCs(lowPriority);
                if(!mergedOrder.empty()) {
                    OptimizedSFC opt;
                    opt.original_id = -1; // Special ID for unified SFC
                    opt.priority = "low";
                    opt.is_merged = true;
                    opt.contained_sfcs = lowPriority.size();

                    // Create stages (one VNF per stage)
                    for (const auto& vnf_name : mergedOrder) {
                        if (vnf_name == "INGRESS" || vnf_name == "EGRESS") continue;
                        
                        // Find the VNF in any of the SFCs
                        for (const auto& sfc : lowPriority) {
                            for (const auto& vnf : sfc.vnfs) {
                                if (vnf.name == vnf_name) {
                                    opt.stages.push_back({vnf});
                                    break;
                                }
                            }
                        }
                    }
                    optimized.push_back(opt);
                }
                else {
                    std::cout << "Cycle detected in unified low-priority SFCs, ";
                    std::cout << "falling back to individual parallel processing\n";
                    
                    // Process each SFC individually with parallel stages
                    for (const auto& s : lowPriority) {
                        OptimizedSFC opt;
                        opt.original_id = s.request_id;
                        opt.priority = "low";
                        opt.is_merged = false;
                        
                        auto stages = SFC::createParallelStages(s);
                        for (const auto& stage : stages) {
                            opt.stages.push_back(stage.vnfs);
                        }
                        optimized.push_back(opt);
                    }
                }
            }
            else {
                // Use batch approach for larger sets
                std::cout << "Too many low-priority SFCs (" << lowPriority.size() 
                         << "), using batch approach with size limit " << MAX_BATCH_SIZE << "\n";
                
                auto batches = groupSizeConstrainedBatches(lowPriority);
                
                for (const auto& batch : batches) {
                    auto mergedOrder = SFC::mergeAndSortSFCs(batch);
                    if(!mergedOrder.empty()) {
                        OptimizedSFC opt;
                        opt.original_id = -(optimized.size() + 1);
                        opt.priority = "low";
                        opt.is_merged = true;
                        opt.contained_sfcs = batch.size();

                        // Create stages (one VNF per stage)
                        for (const auto& vnf_name : mergedOrder) {
                            if (vnf_name == "INGRESS" || vnf_name == "EGRESS") continue;
                            
                            // Find the VNF in any of the SFCs
                            for (const auto& sfc : batch) {
                                for (const auto& vnf : sfc.vnfs) {
                                    if (vnf.name == vnf_name) {
                                        opt.stages.push_back({vnf});
                                        break;
                                    }
                                }
                            }
                        }
                        optimized.push_back(opt);
                    }
                    else {
                        // Process individually if cycle detected
                        for (const auto& s : batch) {
                            OptimizedSFC opt;
                            opt.original_id = s.request_id;
                            opt.priority = "low";
                            opt.is_merged = false;
                            
                            auto stages = SFC::createParallelStages(s);
                            for (const auto& stage : stages) {
                                opt.stages.push_back(stage.vnfs);
                            }
                            optimized.push_back(opt);
                        }
                    }
                }
            }
        }
    }
    
    return optimized;
}