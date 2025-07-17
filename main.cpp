// File: main.cpp (updated with 20 iterations comparison)
#include "SFC.hpp"
#include "Simulation.hpp"
#include "json.hpp"
#include <fstream>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <vector>
#include <numeric>

using json = nlohmann::json;

struct ComparisonResults {
    double acceptance_rate = 0.0;
    double processing_latency = 0.0;
    double network_latency = 0.0;
    double end_to_end_latency = 0.0;
    double deployment_cost = 0.0;
    double cpu_utilization = 0.0;
    double dpu_utilization = 0.0;
    double bandwidth_utilization = 0.0;
};

ComparisonResults runSingleIteration(const std::vector<SFC>& sfcs, bool useOriginal = true) {
    NetworkSimulator simulator(3600.0, 0.0);
    simulator.initialize();
    simulator.runDeployment(sfcs, useOriginal);
    const auto& stats = simulator.getStatistics();
    
    ComparisonResults results;
    results.acceptance_rate = stats.avg_acceptance_rate * 100;
    results.processing_latency = stats.avg_processing_latency;
    results.network_latency = stats.avg_network_latency;
    results.end_to_end_latency = stats.avg_end_to_end_latency;
    results.deployment_cost = stats.avg_deployment_cost;
    results.cpu_utilization = stats.resource_utilization * 100;
    results.dpu_utilization = stats.dpu_utilization * 100;
    results.bandwidth_utilization = stats.avg_bandwidth_utilization * 100;
    
    return results;
}

ComparisonResults calculateAverage(const std::vector<ComparisonResults>& results) {
    ComparisonResults avg;
    int n = results.size();
    
    for (const auto& result : results) {
        avg.acceptance_rate += result.acceptance_rate;
        avg.processing_latency += result.processing_latency;
        avg.network_latency += result.network_latency;
        avg.end_to_end_latency += result.end_to_end_latency;
        avg.deployment_cost += result.deployment_cost;
        avg.cpu_utilization += result.cpu_utilization;
        avg.dpu_utilization += result.dpu_utilization;
        avg.bandwidth_utilization += result.bandwidth_utilization;
    }
    
    avg.acceptance_rate /= n;
    avg.processing_latency /= n;
    avg.network_latency /= n;
    avg.end_to_end_latency /= n;
    avg.deployment_cost /= n;
    avg.cpu_utilization /= n;
    avg.dpu_utilization /= n;
    avg.bandwidth_utilization /= n;
    
    return avg;
}



int main() {
    // Load SFCs from JSON
    std::ifstream in("sfc_requests_100.json");
    if (!in) { std::cerr << "Cannot open JSON\n"; return 1; }
    json arr; in >> arr;

    std::vector<SFC> sfcs;
    for (auto& j : arr) {
        SFC s;
        s.request_id       = j["request_id"];
        s.expected_latency = j["expected_latency"];
        s.ttl              = j["ttl"];
        s.arrival_rate     = j["arrival_rate"];

        for (auto& vj : j["vnfs"]) {
            VNF v;
            v.name               = vj["name"];
            v.type               = vj["type"];
            v.processing_latency = vj["processing_latency"];
            v.cpu_requirement    = vj["cpu_requirement"];
            v.memory_requirement = vj["memory_requirement"];
            v.depends_on         = vj.value("depends_on", std::vector<std::string>{});
            v.bandwidth_requirement = vj.value("bandwidth_requirement", 1.0);
            s.vnfs.push_back(v);
        }
        sfcs.push_back(std::move(s));
    }

    const double LAT_THRESH = 20.0;
    for (auto& s : sfcs) {
        s.classify(LAT_THRESH);
        s.buildDependencyDAG();
    }

    const int NUM_ITERATIONS = 20;
    std::vector<ComparisonResults> originalResults;
    std::vector<ComparisonResults> modifiedResults;

    std::cout << "\n=== Running " << NUM_ITERATIONS << " iterations of both algorithms ===\n";
    
    // Run iterations
    for (int i = 0; i < NUM_ITERATIONS; ++i) {
        std::cout << "Running iteration " << (i + 1) << "/" << NUM_ITERATIONS << "...\r" << std::flush;
        
        // Run original algorithm
        originalResults.push_back(runSingleIteration(sfcs, true));
        
        // Run modified algorithm
        modifiedResults.push_back(runSingleIteration(sfcs, false));
    }
    
    std::cout << "\nCompleted all iterations.\n";

    // Calculate averages
    ComparisonResults avgOriginal = calculateAverage(originalResults);
    ComparisonResults avgModified = calculateAverage(modifiedResults);

    // Create comparison table header
    std::cout << "\n=== Deployment Strategy Comparison (Average of " << NUM_ITERATIONS << " iterations) ===\n";
    std::cout << std::left << std::setw(25) << "Metric"
              << std::setw(20) << "Original Avg"
              << std::setw(20) << "Modified Avg"
              << std::setw(15) << "Difference"
              << std::setw(15) << "% Change\n";
    std::cout << std::string(95, '-') << "\n";

    auto printComparisonRow = [](const std::string& metric, double original, double modified, const std::string& unit = "") {
        double difference = modified - original;
        double percentChange = (original != 0) ? (difference / original) * 100 : 0;
        
        std::cout << std::left << std::setw(25) << metric
                  << std::setw(20) << std::fixed << std::setprecision(4) << original
                  << std::setw(20) << std::fixed << std::setprecision(4) << modified
                  << std::setw(15) << std::fixed << std::setprecision(4) << difference
                  << std::setw(15) << std::fixed << std::setprecision(2) << percentChange << "%\n";
    };

    // Print comparison results
    printComparisonRow("Acceptance Rate (%)", avgOriginal.acceptance_rate, avgModified.acceptance_rate);
    printComparisonRow("Avg Processing Latency", avgOriginal.processing_latency, avgModified.processing_latency);
    printComparisonRow("Avg Network Latency", avgOriginal.network_latency, avgModified.network_latency);
    printComparisonRow("Avg End-to-End Latency", avgOriginal.end_to_end_latency, avgModified.end_to_end_latency);
    printComparisonRow("Avg Deployment Cost", avgOriginal.deployment_cost, avgModified.deployment_cost);
    printComparisonRow("CPU Utilization (%)", avgOriginal.cpu_utilization, avgModified.cpu_utilization);
    printComparisonRow("DPU Utilization (%)", avgOriginal.dpu_utilization, avgModified.dpu_utilization);
    printComparisonRow("Bandwidth Util. (%)", avgOriginal.bandwidth_utilization, avgModified.bandwidth_utilization);

   

    // Summary
    std::cout << "\n=== Summary ===\n";
    std::cout << "Total iterations: " << NUM_ITERATIONS << "\n";
    std::cout << "Simulation time per iteration: 3600.0 seconds\n";
    
    // Determine which algorithm performed better
    int originalWins = 0, modifiedWins = 0;
    
    if (avgModified.acceptance_rate > avgOriginal.acceptance_rate) modifiedWins++; else originalWins++;
    if (avgModified.processing_latency < avgOriginal.processing_latency) modifiedWins++; else originalWins++;
    if (avgModified.network_latency < avgOriginal.network_latency) modifiedWins++; else originalWins++;
    if (avgModified.end_to_end_latency < avgOriginal.end_to_end_latency) modifiedWins++; else originalWins++;
    if (avgModified.deployment_cost < avgOriginal.deployment_cost) modifiedWins++; else originalWins++;
    
    std::cout << "Modified algorithm wins in " << modifiedWins << "/5 key performance metrics\n";
    std::cout << "Original algorithm wins in " << originalWins << "/5 key performance metrics\n";

    return 0;
}