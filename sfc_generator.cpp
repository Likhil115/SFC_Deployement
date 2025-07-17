#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <random>
#include <ctime>
#include <map>

using namespace std;

struct VNF {
    string name;
    string type;
    double processing_latency;
    int cpu_requirement;
    int memory_requirement;
    vector<string> depends_on;
};

struct SFC {
    int request_id;
    double expected_latency;
    int ttl;
    double arrival_rate;
    vector<VNF> vnfs;
    string priority;
};

// Common VNF types and names
const vector<string> MONITOR_VNFS = {
    // Core security monitors
    "Firewall",
    "IDS",                  // Intrusion Detection System
    "IPS",                  // Intrusion Prevention System
    "Logger",
    "TrafficAnalyzer",
    "MalwareDetector",
    "AnomalyDetector",
    "SecurityMonitor",
    "ThreatDetector",
    "AccessController"
};

const vector<string> SHAPER_VNFS = {
    // Core traffic shapers
    "DPI",                  // Deep Packet Inspection
    "NAT",                  // Network Address Translation
    "Shaper",
    "Encryptor",
    "Decryptor",
    "RateLimiter",
    "Compressor",
    "Decompressor",
    "LoadBalancer",
    "QoSManager"
};

// Random number generators
random_device rd;
mt19937 gen(rd());
uniform_int_distribution<> id_dist(1, 1000);
uniform_int_distribution<> latency_dist(1, 50);
uniform_int_distribution<> ttl_dist(30, 300);
uniform_real_distribution<> arrival_dist(5.0, 50.0);

string generateRandomName(const vector<string>& options) {
    uniform_int_distribution<> dist(0, options.size() - 1);
    return options[dist(gen)];
}

VNF generateVNF(const string& type, const vector<string>& dependencies = {}) {
    VNF vnf;
    vnf.type = type;
    vnf.depends_on = dependencies;
    
    if (type == "monitor") {
        vnf.name = generateRandomName(MONITOR_VNFS);
        vnf.processing_latency = 0.5 + (double)(gen() % 15) / 10.0; // 0.5-2.0
        vnf.cpu_requirement = 1 + gen() % 3; // 1-3
        vnf.memory_requirement = 64 + (gen() % 4) * 64; // 64,128,192,256
    } else { // shaper
        vnf.name = generateRandomName(SHAPER_VNFS);
        vnf.processing_latency = 1.0 + (double)(gen() % 40) / 10.0; // 1.0-5.0
        vnf.cpu_requirement = 1 + gen() % 5; // 1-5
        vnf.memory_requirement = 128 + (gen() % 8) * 128; // 128-1024
    }
    
    return vnf;
}

SFC generateSFC(int id,int length) {
    SFC sfc;
    sfc.request_id = id;
    sfc.expected_latency = latency_dist(gen);
    sfc.ttl = ttl_dist(gen);
    sfc.arrival_rate = arrival_dist(gen);
    sfc.priority = (sfc.expected_latency < 10) ? "high" : "low";
    
    // Generate VNF chain (4-8 VNFs) - Modified range
    // Allow user to input expected latency
    int vnf_count = length; 
    vector<VNF> vnf_chain;
    map<string, bool> used_names;
    
    for (int i = 0; i < vnf_count; i++) {
        string type = (gen() % 2) ? "monitor" : "shaper";
        vector<string> dependencies;
        
        // 40% chance to depend on previous VNFs
        if (i > 0 && (gen() % 100) < 40) {
            int dep_count = 1 + gen() % min(2, i); // 1-2 dependencies
            for (int d = 0; d < dep_count; d++) {
                int dep_index = gen() % i;
                if (used_names.count(vnf_chain[dep_index].name)) {
                    dependencies.push_back(vnf_chain[dep_index].name);
                }
            }
        }
        
        VNF vnf = generateVNF(type, dependencies);
        
        // Ensure unique VNF names in this SFC
        int attempts = 0;
        while (used_names.count(vnf.name) && attempts < 20) {
            vnf = generateVNF(type, dependencies);
            attempts++;
        }
        
        // If still not unique after 20 attempts, add a suffix
        if (used_names.count(vnf.name)) {
            vnf.name += "_" + to_string(i + 1);
        }
        
        used_names[vnf.name] = true;
        vnf_chain.push_back(vnf);
    }
    
    sfc.vnfs = vnf_chain;
    return sfc;
}

string toJson(const SFC& sfc) {
    string json = "  {\n";
    json += "    \"request_id\": " + to_string(sfc.request_id) + ",\n";
    json += "    \"expected_latency\": " + to_string(sfc.expected_latency) + ",\n";
    json += "    \"ttl\": " + to_string(sfc.ttl) + ",\n";
    json += "    \"arrival_rate\": " + to_string(sfc.arrival_rate) + ",\n";
    json += "    \"vnfs\": [\n";
    
    for (size_t i = 0; i < sfc.vnfs.size(); i++) {
        const VNF& vnf = sfc.vnfs[i];
        json += "      {\n";
        json += "        \"name\": \"" + vnf.name + "\",\n";
        json += "        \"type\": \"" + vnf.type + "\",\n";
        json += "        \"processing_latency\": " + to_string(vnf.processing_latency) + ",\n";
        json += "        \"cpu_requirement\": " + to_string(vnf.cpu_requirement) + ",\n";
        json += "        \"memory_requirement\": " + to_string(vnf.memory_requirement) + ",\n";
        json += "        \"depends_on\": [";
        
        for (size_t j = 0; j < vnf.depends_on.size(); j++) {
            json += "\"" + vnf.depends_on[j] + "\"";
            if (j != vnf.depends_on.size() - 1) json += ", ";
        }
        
        json += "]\n";
        json += "      }";
        if (i != sfc.vnfs.size() - 1) json += ",";
        json += "\n";
    }
    
    json += "    ]\n";
    json += "  }";
    return json;
}

int main() {
    cout << "SFC Generator (4-8 VNFs per SFC)\n";
    cout << "Enter number of SFC requests to generate: ";
    
    int count;
    cin >> count;
    
    if (count <= 0) {
        cout << "Invalid count. Exiting.\n";
        return 1;
    }
    int length;
    cout << "Enter number of VNFs per SFC (4-8): ";
    std::cin >> length; 
    
    string filename = "sfc_requests_" + to_string(count) + ".json";
    ofstream outfile(filename);
    
    if (!outfile.is_open()) {
        cout << "Error creating output file.\n";
        return 1;
    }
    
    outfile << "[\n";
    vector<SFC> sfcs;
    
    for (int i = 0; i < count; i++) {
        SFC sfc = generateSFC(id_dist(gen),length);
        sfcs.push_back(sfc);
        outfile << toJson(sfc);
        if (i != count - 1) outfile << ",";
        outfile << "\n";
    }
    
    outfile << "]\n";
    outfile.close();
    
    cout << "Generated " << count << " SFC requests in " << filename << "\n";
    
    // Print summary with VNF count distribution
    int high_priority = 0;
    int low_priority = 0;
    map<int, int> vnf_count_distribution;
    
    for (const auto& sfc : sfcs) {
        if (sfc.priority == "high") high_priority++;
        else low_priority++;
        
        int vnf_count = sfc.vnfs.size();
        vnf_count_distribution[vnf_count]++;
    }
    
    cout << "\nSummary:\n";
    cout << "  High priority (<10ms latency): " << high_priority << "\n";
    cout << "  Low priority (>=10ms latency): " << low_priority << "\n";
    cout << "\nVNF Count Distribution:\n";
    for (int i = 4; i <= 8; i++) {
        cout << "  " << i << " VNFs: " << vnf_count_distribution[i] << " SFCs\n";
    }
    
    return 0;
}