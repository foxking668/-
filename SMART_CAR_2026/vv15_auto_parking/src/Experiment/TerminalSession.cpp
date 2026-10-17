#include "TerminalSession.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include "SessionFiles.hpp"
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <vector>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

namespace fs = SessionFiles;
namespace {
std::string trim(const std::string &value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    return value.substr(first, value.find_last_not_of(" \t\r\n") - first + 1);
}

std::string readFile(const fs::path &path) {
    std::ifstream stream(path.string(), std::ios::binary);
    if (!stream) throw std::runtime_error("Cannot read: " + path.string());
    std::ostringstream data;
    data << stream.rdbuf();
    if (stream.bad()) throw std::runtime_error("Read failed: " + path.string());
    return data.str();
}

void writeFile(const fs::path &path, const std::string &data) {
    std::ofstream stream(path.string(), std::ios::binary | std::ios::trunc);
    if (!stream) throw std::runtime_error("Cannot write: " + path.string());
    stream << data;
    stream.flush();
    if (!stream) throw std::runtime_error("Write failed: " + path.string());
    stream.close();
    if (stream.fail()) throw std::runtime_error("Close failed: " + path.string());
}

std::map<std::string, std::string> parseConfig(const std::string &text) {
    std::map<std::string, std::string> result;
    std::istringstream lines(text);
    std::string line;
    while (std::getline(lines, line)) {
        if (line.compare(0, 3, "\xEF\xBB\xBF") == 0) line.erase(0, 3);
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        const auto equals = line.find('=');
        if (equals == std::string::npos) throw std::runtime_error("Invalid parameter line: " + line);
        const std::string key = trim(line.substr(0, equals));
        if (!result.emplace(key, trim(line.substr(equals + 1))).second)
            throw std::runtime_error("Duplicate parameter: " + key);
    }
    return result;
}

std::string updateConfig(const std::string &text, const std::map<std::string, std::string> &changes) {
    const auto old = parseConfig(text);
    for (const auto &change : changes)
        if (!old.count(change.first)) throw std::runtime_error("Missing parameter: " + change.first);
    std::istringstream lines(text);
    std::ostringstream result;
    result << "# Terminal selection below is authoritative; older example comments are reference only.\n";
    std::string line;
    while (std::getline(lines, line)) {
        if (line.compare(0, 3, "\xEF\xBB\xBF") == 0) line.erase(0, 3);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const std::string clean = trim(line);
        const auto equals = clean.find('=');
        if (!clean.empty() && clean[0] != '#' && equals != std::string::npos) {
            const std::string key = trim(clean.substr(0, equals));
            const auto replacement = changes.find(key);
            if (replacement != changes.end()) line = key + "=" + replacement->second;
        }
        if (line.find("# Terminal selection below is authoritative;") != 0) result << line << '\n';
    }
    return result.str();
}

bool parseNumber(const std::string &text, int &number, int maximum = 1000000) {
    if (text.empty() || !std::all_of(text.begin(), text.end(), [](unsigned char c) { return std::isdigit(c); })) return false;
    try {
        const auto value = std::stoull(text);
        if (value > static_cast<unsigned long long>(maximum)) return false;
        number = static_cast<int>(value);
        return true;
    } catch (...) { return false; }
}

bool askCount(std::istream &input, std::ostream &output, const std::string &prompt,
              int &value, int maximum = 1000000) {
    std::string line;
    while (true) {
        output << prompt << " (?=未记录): " << std::flush;
        if (!std::getline(input, line)) return false;
        line = trim(line);
        if (line == "?") { value = -1; return true; }
        if (parseNumber(line, value, maximum)) return true;
        output << "输入无效或为空，请重新填写；不清楚请明确输入 ?。\n";
    }
}

std::string countText(int count) { return count < 0 ? "UNRECORDED" : std::to_string(count); }

std::string runName(const Experiment::TerminalSelection &s, unsigned long long number) {
    std::ostringstream name;
    name << s.label << "_V" << std::setw(2) << std::setfill('0') << s.speed
         << "_R" << std::setw(2) << number;
    return name.str();
}

std::string quoted(std::string text) {
    std::string result = "\"";
    for (char c : text) {
        if (c == '\"') result += "\"\"";
        else if (c == '\r' || c == '\n') result += ' ';
        else result += c;
    }
    return result + "\"";
}

// Read CSV records with quoted commas/newlines and escaped quotes, including legacy files.
std::map<std::string, std::string> readOutcomeFields(const fs::path &path) {
    std::ifstream input(path.string(), std::ios::binary);
    if (!input) return {};
    std::vector<std::vector<std::string>> rows;
    std::vector<std::string> row;
    std::string field;
    bool inQuotes = false;
    char c;
    while (rows.size() < 2 && input.get(c)) {
        if (c == '"') {
            if (inQuotes && input.peek() == '"') { input.get(c); field += '"'; }
            else inQuotes = !inQuotes;
        } else if (!inQuotes && (c == ',' || c == '\n')) {
            row.push_back(field);
            field.clear();
            if (c == '\n') { rows.push_back(row); row.clear(); }
        } else if (c != '\r' || inQuotes) field += c;
    }
    if (inQuotes) return {};
    if (rows.size() < 2 && (!row.empty() || !field.empty())) {
        row.push_back(field); rows.push_back(row);
    }
    if (rows.size() != 2 || rows[0].size() != rows[1].size()) return {};
    std::map<std::string, std::string> result;
    for (std::size_t i = 0; i < rows[0].size(); ++i)
        if (!result.emplace(rows[0][i], rows[1][i]).second) return {};
    return result;
}

void printOutcomeFields(std::ostream &output, const fs::path &directory) {
    auto fields = readOutcomeFields(directory / "outcome_details.csv");
    if (!fields.empty()) {
        const auto value = [&](const std::string &key) {
            const auto found = fields.find(key);
            return found == fields.end() || found->second.empty() ? std::string("UNRECORDED") : found->second;
        };
        output << " 圈数=" << value("completed_laps") << " 出界=" << value("out_of_track_count")
               << " 压线=" << value("line_touch_count") << " 干预=" << value("manual_intervention")
               << " 扶车=" << value("reposition_count") << " 自主完成=" << value("autonomous_two_laps")
               << " 确认=" << value("outcome_confirmed");
    } else {
        output << " 旧格式或详细结果缺失，请核对outcome.csv（不推断压线/圈数）";
    }
}
}

namespace Experiment {
bool parseTerminalSelection(const std::string &label, int speed, TerminalSelection &selection) {
    if (speed != 6 && speed != 9 && speed != 12 && speed != 15) return false;
    std::string name = trim(label);
    for (char &c : name) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    if (name.size() == 1 && name[0] >= '1' && name[0] <= '4') name = "M" + name;
    if (name != "M1" && name != "M2" && name != "M3" && name != "M4" && name != "A3" && name != "A4") return false;
    selection.label = name;
    selection.speed = speed;
    selection.method = name[1] - '0';
    selection.group = name[0] == 'M' ? 1 : 0;
    return true;
}

TerminalHistory terminalHistory(const std::string &logRoot, const TerminalSelection &selection) {
    TerminalHistory result;
    unsigned long long maximum = 0;
    const std::regex pattern("^(?:[0-9]+_)?" + selection.label + "_V0?" + std::to_string(selection.speed) + "_R([0-9]+)$");
    for (const fs::path &root : {fs::path(logRoot), fs::path(logRoot) / ".session_numbers"}) {
        if (!fs::exists(root)) continue;
        for (const auto &entry : fs::directory_iterator(root)) {
            if (!fs::is_directory(entry.path())) continue;
            std::smatch match;
            const std::string name = entry.path().filename().string();
            if (!std::regex_match(name, match, pattern)) continue;
            const auto value = std::stoull(match[1].str());
            maximum = std::max(maximum, value);
            if (root == fs::path(logRoot)) ++result.logDirectories;
        }
    }
    if (maximum == std::numeric_limits<unsigned long long>::max()) throw std::runtime_error("Run numbers exhausted.");
    result.next = maximum + 1;
    return result;
}

std::array<int, 4> terminalMethodOrder(int speed, unsigned long long round) {
    if (round == 0) throw std::invalid_argument("Round must be positive.");
    const std::array<int, 4> speeds{{6, 9, 12, 15}};
    const auto found = std::find(speeds.begin(), speeds.end(), speed);
    if (found == speeds.end()) throw std::invalid_argument("Unknown speed.");
    const auto speedIndex = static_cast<unsigned long long>(found - speeds.begin());
    // Preserve the published schedule, including the two already-run V06 rounds.
    if (speed == 6 && round <= 2) return {{1, 2, 3, 4}};
    const std::array<std::array<int, 4>, 4> orders{{
        {{3, 1, 4, 2}}, {{1, 2, 3, 4}}, {{2, 4, 1, 3}}, {{4, 3, 2, 1}}
    }};
    const auto offset = round <= 2 ? round - 1 : round - 3;
    return orders[(offset % 4 + speedIndex) % 4];
}

TerminalRoundPlan terminalRoundPlan(const std::string &logRoot, int speed) {
    // Use the lowest next available number; a saved failure occupies its original slot too.
    terminalMethodOrder(speed, 1);
    TerminalRoundPlan plan;
    plan.round = std::numeric_limits<unsigned long long>::max();
    for (int method = 1; method <= 4; ++method) {
        TerminalSelection s;
        parseTerminalSelection(std::to_string(method), speed, s);
        plan.histories[method - 1] = terminalHistory(logRoot, s);
        plan.round = std::min(plan.round, plan.histories[method - 1].next);
    }
    plan.order = terminalMethodOrder(speed, plan.round);
    for (int method : plan.order) {
        if (plan.histories[method - 1].next == plan.round) {
            plan.nextMethod = method;
            break;
        }
    }
    return plan;
}

void printTerminalRoundGuide(std::ostream &output, const std::string &logRoot,
                             int speed, int selectedMethod) {
    const auto plan = terminalRoundPlan(logRoot, speed);
    const auto round = plan.round;
    const auto &order = plan.order;
    const auto &histories = plan.histories;
    const std::string roundLabel = "R" + (round < 10 ? std::string("0") : std::string{}) + std::to_string(round);
    output << "\n速度 " << speed << " | 当前待接续 " << roundLabel << " | 顺序：";
    for (std::size_t i = 0; i < order.size(); ++i) output << (i ? " -> M" : "M") << order[i];
    output << '\n';
    for (int method : order) {
        TerminalSelection s;
        parseTerminalSelection(std::to_string(method), speed, s);
        const std::regex pattern("^(?:[0-9]+_)?M" + std::to_string(method) + "_V0?" +
                                 std::to_string(speed) + "_R0*" + std::to_string(round) + "$");
        unsigned int matches = 0;
        bool hasOutcome = false;
        fs::path outcomeDirectory;
        if (fs::exists(logRoot)) {
            for (const auto &entry : fs::directory_iterator(logRoot)) {
                if (!fs::is_directory(entry.path()) || !std::regex_match(entry.path().filename().string(), pattern)) continue;
                ++matches;
                outcomeDirectory = entry.path();
                const auto resultFile = entry.path() / "outcome.csv";
                if (fs::exists(resultFile) && !readFile(resultFile).empty()) hasOutcome = true;
            }
        }
        const bool occupied = histories[method - 1].next > round;
        output << "  M" << method << " [";
        if (matches > 1) output << "同编号多份日志，需核对";
        else if (matches && hasOutcome) output << "已有日志和结果文件";
        else if (matches) output << "已有日志，结果待补录";
        else if (occupied) output << "编号已越过或已占用，本地无对应日志";
        else output << "待运行";
        output << "]";
        if (method == plan.nextMethod) output << " <- 自动安排下一次";
        if (selectedMethod == method) output << " [本次自动选择]";
        output << '\n';
        if (matches == 1) { printOutcomeFields(output, outcomeDirectory); output << '\n'; }
    }
    output << "下一次自动安排：M" << plan.nextMethod << "，速度 " << speed << "。\n";
    output << "本速度累计（每次读取当前日志）：\n";
    for (int method = 1; method <= 4; ++method) {
        unsigned long long logs = 0, confirmed = 0, autonomous = 0, failed = 0;
        const std::regex pattern("^(?:[0-9]+_)?M" + std::to_string(method) + "_V0?" +
                                 std::to_string(speed) + "_R[0-9]+$");
        if (fs::exists(logRoot)) for (const auto &entry : fs::directory_iterator(logRoot)) {
            if (!fs::is_directory(entry.path()) || !std::regex_match(entry.path().filename().string(), pattern)) continue;
            ++logs;
            const auto fields = readOutcomeFields(entry.path() / "outcome_details.csv");
            const auto confirmation = fields.find("outcome_confirmed");
            if (confirmation == fields.end() || confirmation->second != "1") continue;
            ++confirmed;
            const auto success = fields.find("autonomous_two_laps");
            if (success != fields.end() && success->second == "1") ++autonomous;
            if (success != fields.end() && success->second == "0") ++failed;
        }
        output << "  M" << method << " 日志=" << logs << " 新格式已确认=" << confirmed
               << " 自主完成=" << autonomous << " 未自主完成=" << failed
               << " 未判定/旧格式/待核对=" << (logs - autonomous - failed) << '\n';
    }
    output << "状态只依据本地日志/占号，不代表成功或有效样本；缺失与未确认结果需另行核对。\n";
}

bool prepareTerminalSession(std::istream &input, std::ostream &output,
                            const std::string &paramsPath, const std::string &logRoot) {
    // All interaction finishes before RuntimeConfig is first loaded or hardware is initialized.
    const std::string original = readFile(paramsPath);
    parseConfig(original);
    output << "\n=== 实车实验自动排期 / Experiment menu ===\n"
           << "1=M1质心法  2=M2扫描线  3=M3边界跟踪  4=M4你的完整方法\n"
           << "只选速度，M1-M4按本轮固定顺序自动接续；输入START才启动。\n"
           << "速度提示处也可输入A3/A4进入独立消融；Q退出。\n";
    for (const std::string label : {"M1", "M2", "M3", "M4"}) {
        output << label;
        unsigned long long total = 0;
        for (int speed : {6,9,12,15}) {
            TerminalSelection s;
            parseTerminalSelection(label, speed, s);
            const auto history = terminalHistory(logRoot, s);
            total += history.logDirectories;
            output << "  V" << speed << ":日志" << history.logDirectories << "/下次R" << history.next;
        }
        output << "  当前目录合计=" << total << '\n';
    }
    TerminalSelection selection;
    std::string line, label;
    while (true) {
        output << "选择速度档 (6/9/12/15，Q退出；A3/A4为独立消融): " << std::flush;
        if (!std::getline(input, line)) return false;
        line = trim(line);
        if (line == "q" || line == "Q") return false;
        if (line == "A3" || line == "a3" || line == "A4" || line == "a4") {
            label = line;
            output << "已进入独立消融 " << label << "，请输入速度。\n";
            continue;
        }
        int speed = 0;
        if (parseNumber(line, speed, 15) && (speed == 6 || speed == 9 || speed == 12 || speed == 15)) {
            const auto methodLabel = label.empty() ? "M" + std::to_string(terminalRoundPlan(logRoot, speed).nextMethod) : label;
            if (parseTerminalSelection(methodLabel, speed, selection)) break;
        }
        output << "只允许6、9、12、15。\n";
    }
    const auto history = terminalHistory(logRoot, selection);
    const auto runId = runName(selection, history.next);
    std::ostringstream orderSnapshot;
    if (selection.group == 1) {
        printTerminalRoundGuide(orderSnapshot, logRoot, selection.speed, selection.method);
        output << orderSnapshot.str();
        output << "本次实际编号：" << runId << "（独立接续，不改写已占编号）。\n";
    } else {
        orderSnapshot << "A3/A4消融组不适用M1-M4四方法排期。\n";
        output << orderSnapshot.str();
    }
    const std::map<std::string, std::string> changes = {
        {"experiment_enable","1"}, {"experiment_method_group",std::to_string(selection.group)},
        {"experiment_method",std::to_string(selection.method)}, {"motor_aim_speed",std::to_string(selection.speed)},
        {"experiment_run_id",runId}, {"experiment_log_enable","1"},
        {"experiment_video_enable","1"}, {"experiment_video_fps","15"}, {"experiment_task_features_enable","0"}
    };
    const auto selected = updateConfig(original, changes);
    output << "本次 " << runId << "；计划2圈；录像开启(15fps上限)；日志开启；任务干预关闭。\n"
           << "同组已有日志目录 " << history.logDirectories << " 个；编号不是成功次数。\n"
           << "输入 START 确认启动，其他输入取消: " << std::flush;
    if (!std::getline(input, line) || trim(line) != "START") return false;
    if (readFile(paramsPath) != original) throw std::runtime_error("Parameters changed during menu; restart selection.");

    const fs::path ledger = fs::path(logRoot) / ".session_numbers";
    fs::create_directories(ledger);
    // Import occupied IDs before reserving a new one; moving old logs later will not reuse their IDs.
    const std::regex historical("^[0-9]+_((?:M[1-4]|A[34])_V(?:06|09|12|15)_R[0-9]+)$");
    for (const auto &entry : fs::directory_iterator(logRoot)) {
        std::smatch match;
        const auto name = entry.path().filename().string();
        if (fs::is_directory(entry.path()) && std::regex_match(name, match, historical)) fs::create_directory(ledger / match[1].str());
    }
    const auto reservation = ledger / runId;
    if (!fs::create_directory(reservation)) throw std::runtime_error("Run ID was reserved concurrently; restart selection.");
    writeFile(reservation / "parameters_before.txt", original);
    writeFile(reservation / "parameters_selected.txt", selected);
    writeFile(reservation / "terminal_order_at_start.txt",
              "schedule_version=20260912_published_order\nrun_id=" + runId + "\n" + orderSnapshot.str());
    writeFile(reservation / "reservation.txt", "state=RESERVED\nrun_id=" + runId + "\nnot_a_completion_record=1\n");
    const std::string temporary = paramsPath + ".terminal.tmp";
    writeFile(temporary, selected);
#ifdef _WIN32
    if (!MoveFileExA(temporary.c_str(), paramsPath.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
        throw std::runtime_error("Cannot replace runtime parameters; startup cancelled.");
#else
    if (std::rename(temporary.c_str(), paramsPath.c_str()) != 0) throw std::runtime_error("Cannot replace runtime parameters; startup cancelled.");
#endif
    output << "参数已保存，本次编号 " << runId << "。准备初始化硬件。\n";
    return true;
}

DetailedOutcome readDetailedOutcome(std::istream &input, std::ostream &output) {
    while (true) {
        DetailedOutcome result;
        int intervention = -1;
        output << "\n=== 本次结果 / Session outcome ===\n";
        if (!askCount(input, output, "实际完成圈数 (0/1/2)", result.completedLaps, 2) ||
            !askCount(input, output, "真正出界次数（压线不计这里）", result.legacy.outOfTrackCount) ||
            !askCount(input, output, "压线次数", result.lineTouchCount) ||
            !askCount(input, output, "是否人工干预 (0=无,1=有；正常结束Ctrl+C不算)", intervention, 1) ||
            !askCount(input, output, "人工放回赛道次数", result.repositionCount)) {
            output << "输入中断，结果保持未确认，不能自动判为成功。\n";
            return DetailedOutcome{};
        }
        if (result.repositionCount > 0 && intervention != 1) {
            output << "扶车次数大于0，人工干预应填1，请重新填写。\n";
            continue;
        }
        result.legacy.manualIntervention = countText(intervention);
        if (intervention == 1 || (result.completedLaps >= 0 && result.completedLaps < 2)) result.legacy.success = "0";
        else if (intervention == 0 && result.completedLaps == 2) result.legacy.success = "1";
        output << "备注（可回车留空）: " << std::flush;
        std::string notes;
        if (!std::getline(input, notes)) return DetailedOutcome{};
        result.legacy.notes = "completed_laps=" + countText(result.completedLaps) +
            "; line_touch=" + countText(result.lineTouchCount) + "; reposition=" + countText(result.repositionCount) +
            "; outcome_confirmed=1" + (notes.empty() ? "" : "; " + notes);
        output << "确认：圈数=" << countText(result.completedLaps)
               << " 出界=" << countText(result.legacy.outOfTrackCount)
               << " 压线=" << countText(result.lineTouchCount)
               << " 人工干预=" << result.legacy.manualIntervention
               << " 扶车=" << countText(result.repositionCount)
               << " 自主完成=" << result.legacy.success << '\n'
               << "输入 Y 保存；其他输入重新填写: " << std::flush;
        std::string confirm;
        if (!std::getline(input, confirm)) return DetailedOutcome{};
        if (trim(confirm) == "y" || trim(confirm) == "Y") {
            result.confirmed = true;
            return result;
        }
    }
}

void saveDetailedOutcome(const std::string &runDirectory, const std::string &runId, const DetailedOutcome &result) {
    std::ostringstream csv;
    csv << "run_id,completed_laps,line_touch_count,out_of_track_count,manual_intervention,reposition_count,autonomous_two_laps,outcome_confirmed,notes\n"
        << quoted(runId) << ',' << countText(result.completedLaps) << ',' << countText(result.lineTouchCount) << ','
        << countText(result.legacy.outOfTrackCount) << ',' << result.legacy.manualIntervention << ','
        << countText(result.repositionCount) << ',' << result.legacy.success << ',' << (result.confirmed ? 1 : 0)
        << ',' << quoted(result.legacy.notes) << '\n';
    const auto path = fs::path(runDirectory) / "outcome_details.csv";
    const auto temporary = path.string() + ".tmp";
    writeFile(temporary, csv.str());
    if (std::rename(temporary.c_str(), path.string().c_str()) != 0) throw std::runtime_error("Cannot finalize outcome_details.csv");
}
}
