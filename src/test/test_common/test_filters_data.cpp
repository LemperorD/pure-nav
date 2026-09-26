// test_filters_data.cpp —— 基于 test_data/ 中 CSV 的数据驱动测试
//
// 与 test_filters.cpp（手工构造的小数据）互补：本文件用真实规模的信号（正弦+噪声、
// 阶跃+噪声、带野值信号）驱动滤波器，并做两层校验：
//
//   1. golden 比对：CSV 的 expected 列由独立实现（Python 参考算法）算出，逐点比较
//      可以抓出 C++ 实现的算法性错误，而不是"自己验证自己"。
//   2. 质量指标：粗糙度（一阶差分标准差）下降幅度、阶跃稳态误差、野值抑制比。
//      注意不能用"与真值的 RMSE"判断平滑滤波器好坏——任何平滑滤波器都有相位滞后，
//      阶跃/正弦上 filtered 的 RMSE 反而可能大于 raw。
//
// 数据目录由 CMake 通过 PURE_NAV_TEST_DATA_DIR 编译定义传入（见 src/test/CMakeLists.txt），
// 因此无论从哪个目录运行 ctest 都能找到文件。

#include "filters.hpp"
#include "test_check.hpp"

#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#ifndef PURE_NAV_TEST_DATA_DIR
#error "test_filters_data.cpp 需要 PURE_NAV_TEST_DATA_DIR 编译定义，见 src/test/CMakeLists.txt"
#endif

namespace {

// ----------------------------------------------------------------------------
// 极简 CSV 读取
//
// 支持 `# key=value` 元数据行 + 一行列名 + 若干数据行，例如：
//     # filter=moving_average
//     # window=3
//     index,truth,raw,expected
//     0,0.0,0.123,0.123
// ----------------------------------------------------------------------------
struct DataTable {
    std::vector<std::string> headers;
    std::vector<std::vector<double> > columns; // 与 headers 一一对应
    std::map<std::string, std::string> meta;

    int index_of(const std::string& name) const {
        for (std::size_t i = 0; i < headers.size(); ++i) {
            if (headers[i] == name) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    bool has(const std::string& name) const { return index_of(name) >= 0; }

    std::size_t rows() const { return columns.empty() ? 0 : columns[0].size(); }

    const std::vector<double>& column(const std::string& name) const {
        static const std::vector<double> empty;
        const int index = index_of(name);
        if (index < 0) {
            return empty;
        }
        return columns[static_cast<std::size_t>(index)];
    }

    std::string meta_value(const std::string& key, const std::string& fallback) const {
        std::map<std::string, std::string>::const_iterator it = meta.find(key);
        return it == meta.end() ? fallback : it->second;
    }
};

std::string trim(const std::string& text) {
    const std::string::size_type begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return std::string();
    }
    const std::string::size_type end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

std::vector<std::string> split(const std::string& line, char delimiter) {
    std::vector<std::string> fields;
    std::string current;
    for (std::size_t i = 0; i < line.size(); ++i) {
        if (line[i] == delimiter) {
            fields.push_back(current);
            current.clear();
        } else {
            current.push_back(line[i]);
        }
    }
    fields.push_back(current);
    return fields;
}

bool load_table(const std::string& path, DataTable& table, std::string& error) {
    std::ifstream stream(path.c_str());
    if (!stream.is_open()) {
        error = "无法打开数据文件: " + path;
        return false;
    }

    std::string line;
    bool header_read = false;
    std::size_t line_number = 0;

    while (std::getline(stream, line)) {
        ++line_number;
        line = trim(line);
        if (line.empty()) {
            continue;
        }
        if (line[0] == '#') {
            const std::string content = line.substr(1);
            const std::string::size_type separator = content.find('=');
            if (separator != std::string::npos) {
                table.meta[trim(content.substr(0, separator))] = trim(content.substr(separator + 1));
            }
            continue;
        }

        const std::vector<std::string> fields = split(line, ',');
        if (!header_read) {
            for (std::size_t i = 0; i < fields.size(); ++i) {
                table.headers.push_back(trim(fields[i]));
                table.columns.push_back(std::vector<double>());
            }
            header_read = true;
            continue;
        }

        if (fields.size() != table.headers.size()) {
            std::ostringstream message;
            message << path << " 第 " << line_number << " 行有 " << fields.size() << " 列，期望 " << table.headers.size() << " 列";
            error = message.str();
            return false;
        }
        for (std::size_t i = 0; i < fields.size(); ++i) {
            table.columns[i].push_back(std::atof(fields[i].c_str()));
        }
    }

    if (!header_read || table.headers.empty()) {
        error = "数据文件缺少表头: " + path;
        return false;
    }
    return true;
}

std::string data_path(const std::string& file_name) {
    return std::string(PURE_NAV_TEST_DATA_DIR) + "/filters/" + file_name;
}

bool load_or_fail(const std::string& file_name, DataTable& table, const char* file, int line) {
    std::string error;
    if (!load_table(data_path(file_name), table, error)) {
        ::pure_nav_test::report(false, error, file, line);
        return false;
    }
    return true;
}

#define LOAD_OR_FAIL(file_name, table) load_or_fail((file_name), (table), __FILE__, __LINE__)

// ----------------------------------------------------------------------------
// 指标辅助
// ----------------------------------------------------------------------------
double max_absolute_error(const std::vector<double>& actual, const std::vector<double>& expected) {
    if (actual.size() != expected.size() || actual.empty()) {
        return 0.0;
    }
    double worst = 0.0;
    for (std::size_t i = 0; i < actual.size(); ++i) {
        const double difference = std::fabs(actual[i] - expected[i]);
        if (difference > worst) {
            worst = difference;
        }
    }
    return worst;
}

// 按 CSV 元数据里的 filter/参数构造滤波器并整段处理
std::vector<double> apply_named_filter(const DataTable& table, const std::vector<double>& raw) {
    const std::string name = table.meta_value("filter", "");

    if (name == "moving_average") {
        const std::size_t window = static_cast<std::size_t>(std::atoi(table.meta_value("window", "3").c_str()));
        pure::common::SlidingWindowFilter<double> filter(window);
        return filter.apply(raw);
    }
    if (name == "median") {
        const std::size_t window = static_cast<std::size_t>(std::atoi(table.meta_value("window", "3").c_str()));
        pure::common::MedianFilter<double> filter(window);
        return filter.apply(raw);
    }
    if (name == "lowpass") {
        const double alpha = std::atof(table.meta_value("alpha", "0.5").c_str());
        pure::common::FirstOrderLowPassFilter<double> filter(alpha);
        return filter.apply(raw);
    }
    if (name == "kalman") {
        const double q  = std::atof(table.meta_value("q", "1e-3").c_str());
        const double r  = std::atof(table.meta_value("r", "1e-2").c_str());
        const double x0 = std::atof(table.meta_value("x0", "0").c_str());
        const double p0 = std::atof(table.meta_value("p0", "1").c_str());
        pure::common::KalmanFilter1D<double> filter(q, r, x0, p0);
        return filter.apply(raw);
    }

    ::pure_nav_test::report(false, "CSV 元数据里的 filter 未知: " + name, __FILE__, __LINE__);
    return std::vector<double>();
}

// 与 Python 参考实现逐点比对
void report_golden(const DataTable& table, const std::vector<double>& actual, double tolerance, const char* file, int line) {
    if (!table.has("expected")) {
        ::pure_nav_test::report(false, "CSV 缺少 expected 列", file, line);
        return;
    }
    const std::vector<double>& expected = table.column("expected");
    if (actual.size() != expected.size()) {
        ::pure_nav_test::report(false, "滤波输出长度与 expected 不一致", file, line);
        return;
    }

    const double worst = max_absolute_error(actual, expected);
    std::cout << "        golden 比对: 样本数=" << actual.size()
              << "  最大误差=" << worst
              << "  RMSE=" << pure_nav_test::root_mean_square_error(actual, expected) << std::endl;
    ::pure_nav_test::report(worst <= tolerance, "golden 比对最大误差超过容差", file, line);
}

// 平滑效果：粗糙度至少下降 min_reduction_percent
void report_roughness(const std::vector<double>& raw, const std::vector<double>& filtered, double min_reduction_percent, const char* file, int line) {
    const double before = pure_nav_test::roughness(raw);
    const double after  = pure_nav_test::roughness(filtered);
    const double reduction = before > 0.0 ? (1.0 - after / before) * 100.0 : 0.0;

    std::cout << "        粗糙度(一阶差分std): raw=" << before
              << " -> filtered=" << after
              << "  降低 " << reduction << "%" << std::endl;
    ::pure_nav_test::report(after < before && reduction >= min_reduction_percent, "噪声抑制不足（粗糙度下降低于阈值）", file, line);
}

// 阶跃响应：末尾 tail_count 个点相对真值的平均绝对误差
void report_steady_state(const DataTable& table, const std::vector<double>& filtered, std::size_t tail_count, double tolerance, const char* file, int line) {
    if (!table.has("truth")) {
        ::pure_nav_test::report(false, "CSV 缺少 truth 列", file, line);
        return;
    }
    const std::vector<double>& truth = table.column("truth");
    const double mae = pure_nav_test::mean_absolute_error(pure_nav_test::last_n(filtered, tail_count), pure_nav_test::last_n(truth, tail_count));

    std::cout << "        末 " << tail_count << " 点稳态 MAE(滤波, 真值)=" << mae << std::endl;
    ::pure_nav_test::report(mae <= tolerance, "稳态误差超过阈值", file, line);
}

// ----------------------------------------------------------------------------
// 用例
// ----------------------------------------------------------------------------
void testDataFilesAvailable() {
    const char* files[] = {
        "moving_average_w3.csv",
        "median_w3.csv",
        "lowpass_alpha0.3.csv",
        "kalman_step.csv",
        "sine_noisy.csv",
    };

    for (std::size_t i = 0; i < sizeof(files) / sizeof(files[0]); ++i) {
        DataTable table;
        std::string error;
        const bool loaded = load_table(data_path(files[i]), table, error);
        if (!loaded) {
            std::cerr << "        " << error << std::endl;
        }
        ::pure_nav_test::report(loaded, std::string("数据文件可读取: ") + files[i], __FILE__, __LINE__);
        if (loaded) {
            std::cout << "        " << files[i] << ": " << table.rows() << " 行, 列 = [";
            for (std::size_t c = 0; c < table.headers.size(); ++c) {
                std::cout << (c == 0 ? "" : ", ") << table.headers[c];
            }
            std::cout << "]" << std::endl;
        }
    }
}

void testMovingAverageData() {
    DataTable table;
    if (!LOAD_OR_FAIL("moving_average_w3.csv", table)) {
        return;
    }

    const std::vector<double>& raw = table.column("raw");
    const std::vector<double> filtered = apply_named_filter(table, raw);

    std::cout << "        配置: window=" << table.meta_value("window", "?") << std::endl;
    report_golden(table, filtered, 1e-12, __FILE__, __LINE__);
    report_roughness(raw, filtered, 15.0, __FILE__, __LINE__);
}

void testMedianData() {
    DataTable table;
    if (!LOAD_OR_FAIL("median_w3.csv", table)) {
        return;
    }

    const std::vector<double>& raw   = table.column("raw");
    const std::vector<double>& truth = table.column("truth");
    const std::vector<double> filtered = apply_named_filter(table, raw);

    std::cout << "        配置: window=" << table.meta_value("window", "?")
              << "  注入野值=" << table.meta_value("spikes", "?") << std::endl;
    report_golden(table, filtered, 1e-12, __FILE__, __LINE__);
    report_roughness(raw, filtered, 15.0, __FILE__, __LINE__);

    // 三个已知野值点：滤波后的误差应远小于滤波前
    const std::size_t spike_indices[] = {8, 25, 33};
    for (std::size_t i = 0; i < sizeof(spike_indices) / sizeof(spike_indices[0]); ++i) {
        const std::size_t index = spike_indices[i];
        const double raw_error      = std::fabs(raw[index] - truth[index]);
        const double filtered_error = std::fabs(filtered[index] - truth[index]);
        std::cout << "        野值 index=" << index << ": 原始误差=" << raw_error
                  << " -> 滤波后误差=" << filtered_error << std::endl;
        ::pure_nav_test::report(filtered_error < 0.1 * raw_error, "中值滤波未抑制野值", __FILE__, __LINE__);
    }
}

void testLowPassData() {
    DataTable table;
    if (!LOAD_OR_FAIL("lowpass_alpha0.3.csv", table)) {
        return;
    }

    const std::vector<double>& raw = table.column("raw");
    const std::vector<double> filtered = apply_named_filter(table, raw);

    std::cout << "        配置: alpha=" << table.meta_value("alpha", "?") << std::endl;
    report_golden(table, filtered, 1e-12, __FILE__, __LINE__);
    report_roughness(raw, filtered, 15.0, __FILE__, __LINE__);
    report_steady_state(table, filtered, 10, 0.25, __FILE__, __LINE__);
}

void testKalmanData() {
    DataTable table;
    if (!LOAD_OR_FAIL("kalman_step.csv", table)) {
        return;
    }

    const std::vector<double>& raw = table.column("raw");
    const std::vector<double> filtered = apply_named_filter(table, raw);

    std::cout << "        配置: q=" << table.meta_value("q", "?")
              << " r=" << table.meta_value("r", "?")
              << " x0=" << table.meta_value("x0", "?")
              << " p0=" << table.meta_value("p0", "?") << std::endl;
    report_golden(table, filtered, 1e-12, __FILE__, __LINE__);
    report_roughness(raw, filtered, 15.0, __FILE__, __LINE__);
    report_steady_state(table, filtered, 10, 0.25, __FILE__, __LINE__);
}

void testNoiseSuppressionOnSine() {
    DataTable table;
    if (!LOAD_OR_FAIL("sine_noisy.csv", table)) {
        return;
    }

    const std::vector<double>& raw   = table.column("raw");
    const std::vector<double>& truth = table.column("truth");

    const double raw_rmse      = pure_nav_test::root_mean_square_error(raw, truth);
    const double raw_roughness = pure_nav_test::roughness(raw);
    std::cout << "        原始信号: RMSE(对真值)=" << raw_rmse << "  粗糙度=" << raw_roughness << std::endl;

    // 同一段真实信号过一遍全部四种滤波器
    std::vector<std::string> labels;
    std::vector<std::vector<double> > outputs;
    {
        pure::common::SlidingWindowFilter<double> filter(3);
        labels.push_back("SlidingWindowFilter(w=3)");
        outputs.push_back(filter.apply(raw));
    }
    {
        pure::common::MedianFilter<double> filter(3);
        labels.push_back("MedianFilter(w=3)");
        outputs.push_back(filter.apply(raw));
    }
    {
        pure::common::FirstOrderLowPassFilter<double> filter(0.3);
        labels.push_back("FirstOrderLowPass(alpha=0.3)");
        outputs.push_back(filter.apply(raw));
    }
    {
        pure::common::KalmanFilter1D<double> filter(1e-3, 1.0, 0.0, 1.0);
        labels.push_back("KalmanFilter1D(q=1e-3, r=1)");
        outputs.push_back(filter.apply(raw));
    }

    for (std::size_t i = 0; i < outputs.size(); ++i) {
        const double filtered_roughness = pure_nav_test::roughness(outputs[i]);
        const double reduction = (1.0 - filtered_roughness / raw_roughness) * 100.0;
        std::cout << "        " << labels[i] << ": 粗糙度=" << filtered_roughness
                  << "（降低 " << reduction << "%）  RMSE(对真值)="
                  << pure_nav_test::root_mean_square_error(outputs[i], truth) << std::endl;
        ::pure_nav_test::report(filtered_roughness < raw_roughness, std::string("未降低噪声: ") + labels[i], __FILE__, __LINE__);
    }
}

} // namespace

using pure_nav_test::run;

int main() {
    std::cout << "===== common_libs/filters 数据驱动测试（test_data/filters/*.csv）=====" << std::endl;
    std::cout << "数据目录: " << PURE_NAV_TEST_DATA_DIR << std::endl;

    run("数据文件可读取", testDataFilesAvailable);
    run("滑动窗口均值 golden + 降噪", testMovingAverageData);
    run("中值滤波 golden + 野值抑制", testMedianData);
    run("一阶低通 golden + 稳态", testLowPassData);
    run("一维卡尔曼 golden + 稳态", testKalmanData);
    run("正弦+噪声 四种滤波器降噪对比", testNoiseSuppressionOnSine);

    return pure_nav_test::summary();
}
