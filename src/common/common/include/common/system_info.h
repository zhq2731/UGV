#include <chrono>
#include <thread>
#include <vector>

// 存储单个 CPU（或核心）的时间片数据
struct CPUUsageData {
    long long user;    // 用户态时间（含 nice）
    long long system;  // 内核态时间（含 irq、softirq）
    long long idle;    // 空闲时间
    long long total;   // 总时间（user + system + idle）
};

std::vector<CPUUsageData> getCPUUsageData() {
    std::vector<CPUUsageData> data;
    std::ifstream file("/proc/stat");
    if (!file.is_open()) {
        throw std::runtime_error("无法打开 /proc/stat");
    }

    std::string line;
    while (std::getline(file, line)) {
        // 只处理以 "cpu" 开头的行（CPU 时间数据）
        if (line.substr(0, 3) != "cpu") {
            break;  // 后续行不是 CPU 数据，退出
        }

        std::istringstream iss(line);
        std::string cpu_label;  // 忽略 "cpu"、"cpu0" 等标签
        long long user, nice, system, idle, iowait, irq, softirq;

        // 解析时间字段（格式：cpu  user nice system idle iowait irq softirq ...）
        iss >> cpu_label >> user >> nice >> system >> idle >> iowait >> irq >> softirq;

        CPUUsageData entry;
        entry.user = user + nice;                  // 用户态总时间（含低优先级）
        entry.system = system + irq + softirq;     // 内核态总时间（含中断）
        entry.idle = idle + iowait;                // 空闲总时间（含 I/O 等待）
        entry.total = entry.user + entry.system + entry.idle;  // 总时间

        data.push_back(entry);
    }

    return data;
}

// 计算总 CPU 使用率（间隔 interval_ms 毫秒）
float getTotalCPUUsage(int interval_ms = 500) {
    auto start_data = getCPUUsageData();
    if (start_data.empty()) {
        throw std::runtime_error("未获取到 CPU 数据");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    auto end_data = getCPUUsageData();

    // 第0项是总 CPU 数据
    const auto& start = start_data[0];
    const auto& end = end_data[0];

    long long total_diff = end.total - start.total;
    long long idle_diff = end.idle - start.idle;

    if (total_diff <= 0) return 0.0f;
    return 100.0f * (1.0f - (float)idle_diff / total_diff);  // 使用率 = (总时间 - 空闲时间)/总时间
}

// 计算每个核心的使用率（返回值：index 0 对应 cpu0，以此类推）
std::vector<float> getPerCoreCPUUsage(int interval_ms = 1000) {
    auto start_data = getCPUUsageData();
    if (start_data.size() < 2) {  // 至少需要总CPU + 1个核心
        throw std::runtime_error("未获取到核心数据");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    auto end_data = getCPUUsageData();

    std::vector<float> usage;
    // 从 index 1 开始是单个核心（index 0 是总CPU）
    for (size_t i = 1; i < start_data.size(); ++i) {
        const auto& start = start_data[i];
        const auto& end = end_data[i];

        long long total_diff = end.total - start.total;
        long long idle_diff = end.idle - start.idle;

        if (total_diff <= 0) {
            usage.push_back(0.0f);
        } else {
            usage.push_back(100.0f * (1.0f - (float)idle_diff / total_diff));
        }
    }

    return usage;
}