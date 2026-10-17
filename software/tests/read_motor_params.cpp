#include "config_loader.hpp"
#include "dm.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
namespace fs = std::filesystem;

constexpr uint32_t kBroadcastCanId = 0x7FF;
constexpr uint8_t kReadParameterOpcode = 0x33;
constexpr uint8_t kStatusOpcode = 0xCC;
constexpr uint8_t kResponseCommand = 0x11;
constexpr uint8_t kFrameEnd = 0x55;

struct Options {
    std::string config_path = "configs/default_right.yml";
    std::string output_path;
    int timeout_ms = 100;
    int retries = 2;
    bool include_legacy_registers = false;
    bool list_registers = false;
};

struct ParameterSpec {
    uint8_t rid;
    const char* name;
};

constexpr std::array<ParameterSpec, 56> kParameters{{
    {0, "UV_Value"}, {1, "KT_Value"}, {2, "OT_Value"}, {3, "OC_Value"},
    {4, "ACC"}, {5, "DEC"}, {6, "MAX_SPD"}, {7, "MST_ID"},
    {8, "ESC_ID"}, {9, "TIMEOUT"}, {10, "CTRL_MODE"}, {11, "Damp"},
    {12, "Inertia"}, {13, "hw_ver"}, {14, "sw_ver"}, {15, "SN"},
    {16, "NPP_pole_pairs"}, {17, "Rs"}, {18, "LS"}, {19, "Flux"}, {20, "Gr"},
    {21, "PMAX"}, {22, "VMAX"}, {23, "TMAX"}, {24, "I_BW"},
    {25, "KP_ASR"}, {26, "KI_ASR"}, {27, "KP_APR"}, {28, "KI_APR"},
    {29, "OV_Value"}, {30, "GREF"}, {31, "Deta"}, {32, "V_BW"},
    {33, "IQ_c1"}, {34, "VL_c1"}, {35, "can_br"}, {36, "sub_ver"},
    {37, "Boot_ver"}, {54, "x_off"}, {55, "dir"}, {56, "m_off"},
    {59, "IBase"}, {60, "VBus"}, {61, "TMOS"}, {62, "T_CEL"},
    {63, "I_U_OFF"}, {64, "I_V_OFF"}, {65, "I_W_OFF"}, {70, "isen.app_ver"},
    {71, "isen.boot_ver"}, {72, "osen.app_ver"}, {73, "osen.boot_ver"},
    {80, "p_m"}, {81, "xout"}, {82, "Xin_raw_d"}, {83, "Xout_raw_d"}
}};
constexpr std::array<ParameterSpec, 4> kLegacyParameters{{
    {50, "RID_50_legacy_u_off"}, {51, "RID_51_legacy_v_off"},
    {52, "RID_52_legacy_k1"}, {53, "RID_53_legacy_k2"}
}};

#pragma pack(push, 1)
struct ParsedFrame {
    uint8_t header = 0;
    uint8_t command = 0;
    uint8_t can_len_and_flags = 0;
    uint32_t can_id = 0;
    std::array<uint8_t, 8> data{};
    uint8_t end = 0;
};
#pragma pack(pop)
static_assert(sizeof(ParsedFrame) == 16, "Damiao response frame must be 16 bytes");

struct Response {
    bool received = false;
    std::string status = "timeout";
    ParsedFrame frame{};
    int64_t receive_monotonic_ns = 0;
};

DM_Motor::DM_Motor_Type parse_motor_type(const std::string& type) {
    if (type == "DM4310") return DM_Motor::DM4310;
    if (type == "DM4310_48V") return DM_Motor::DM4310_48V;
    if (type == "DM4340") return DM_Motor::DM4340;
    if (type == "DM4340_48V") return DM_Motor::DM4340_48V;
    if (type == "DM6006") return DM_Motor::DM6006;
    if (type == "DM8006") return DM_Motor::DM8006;
    if (type == "DM8009") return DM_Motor::DM8009;
    if (type == "DM10010L") return DM_Motor::DM10010L;
    if (type == "DM10010") return DM_Motor::DM10010;
    if (type == "DMH3510") return DM_Motor::DMH3510;
    if (type == "DMH6215") return DM_Motor::DMH6215;
    if (type == "DMG6220") return DM_Motor::DMG6220;
    throw std::runtime_error("Unsupported configured motor type: " + type);
}

speed_t parse_baud_rate(unsigned int baud_rate) {
    if (baud_rate == 921600) return B921600;
    if (baud_rate == 460800) return B460800;
    if (baud_rate == 230400) return B230400;
    if (baud_rate == 115200) return B115200;
    throw std::runtime_error("Unsupported baud rate: " + std::to_string(baud_rate));
}

void validate_config(const ArmConfig& config) {
    if (config.motors.empty()) throw std::runtime_error("Config contains no motors");
    std::set<uint32_t> ids;
    for (const auto& motor : config.motors) {
        parse_motor_type(motor.type);
        if (motor.slave_id == 0 || motor.slave_id >= kBroadcastCanId) {
            throw std::runtime_error("Slave CAN IDs must be in 1..0x7FE for " + motor.name);
        }
        if (!ids.insert(motor.slave_id).second) {
            throw std::runtime_error("Duplicate configured slave ID: " + std::to_string(motor.slave_id));
        }
        if (motor.master_id >= kBroadcastCanId) {
            throw std::runtime_error("Master CAN IDs must be in 0..0x7FE for " + motor.name);
        }
        if (motor.master_id != 0 && !ids.insert(motor.master_id).second) {
            throw std::runtime_error("Duplicate nonzero configured CAN ID: " + std::to_string(motor.master_id));
        }
    }
}

Options parse_options(int argc, char** argv) {
    Options options;
    bool positional_config = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto value_after = [&](const std::string& flag) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("Missing value after " + flag);
            return argv[++i];
        };
        if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0]
                      << " [config.yml] [--config config.yml] [--output directory]"
                         " [--timeout-ms 100] [--retries 2] [--legacy-registers]"
                         " [--list-registers]\n";
            std::exit(0);
        } else if (arg == "--config") {
            options.config_path = value_after(arg);
        } else if (arg == "--output") {
            options.output_path = value_after(arg);
        } else if (arg == "--timeout-ms") {
            options.timeout_ms = std::stoi(value_after(arg));
        } else if (arg == "--retries") {
            options.retries = std::stoi(value_after(arg));
        } else if (arg == "--legacy-registers") {
            options.include_legacy_registers = true;
        } else if (arg == "--list-registers") {
            options.list_registers = true;
        } else if (!arg.empty() && arg[0] != '-' && !positional_config) {
            options.config_path = arg;
            positional_config = true;
        } else {
            throw std::runtime_error("Unknown or duplicate argument: " + arg);
        }
    }
    if (options.timeout_ms < 1 || options.timeout_ms > 10000) {
        throw std::runtime_error("timeout-ms must be in 1..10000");
    }
    if (options.retries < 1 || options.retries > 10) {
        throw std::runtime_error("retries must be in 1..10");
    }
    return options;
}

std::string timestamp_now() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &time);
#else
    localtime_r(&time, &tm);
#endif
    std::ostringstream out;
    out << std::put_time(&tm, "%Y%m%dT%H%M%S");
    return out.str();
}

fs::path create_output_dir(const Options& options) {
    if (!options.output_path.empty()) {
        const fs::path requested(options.output_path);
        if (fs::exists(requested)) {
            throw std::runtime_error("Output path already exists: " + requested.string());
        }
        if (!fs::create_directories(requested)) {
            throw std::runtime_error("Could not create output path: " + requested.string());
        }
        return requested;
    }

    const fs::path root("motor_param_snapshots");
    fs::create_directories(root);
    const std::string base = "snapshot_" + timestamp_now();
    for (unsigned int suffix = 0; suffix < 1000; ++suffix) {
        const fs::path candidate = root / (suffix == 0 ? base : base + "_" + std::to_string(suffix));
        if (fs::create_directory(candidate)) return candidate;
    }
    throw std::runtime_error("Could not allocate a unique snapshot directory");
}

std::string hex_bytes(const uint8_t* bytes, size_t count) {
    std::ostringstream out;
    out << std::hex << std::uppercase << std::setfill('0');
    for (size_t i = 0; i < count; ++i) {
        if (i) out << ' ';
        out << std::setw(2) << static_cast<unsigned int>(bytes[i]);
    }
    return out.str();
}

std::string csv_escape(const std::string& value) {
    if (value.find_first_of(",\"\r\n") == std::string::npos) return value;
    std::string escaped = "\"";
    for (const char c : value) {
        if (c == '"') escaped += '"';
        escaped += c;
    }
    escaped += '"';
    return escaped;
}

bool is_integer_parameter(uint8_t rid) {
    return (rid >= 7 && rid <= 10) || (rid >= 13 && rid <= 16) ||
           (rid >= 35 && rid <= 37) ||
           (rid >= 70 && rid <= 73) || rid == 82 || rid == 83;
}

bool is_known_float_parameter(uint8_t rid) {
    return rid <= 6 || (rid >= 11 && rid <= 12) || (rid >= 17 && rid <= 34) ||
           (rid >= 50 && rid <= 56) || (rid >= 59 && rid <= 65) ||
           rid == 80 || rid == 81;
}

std::string parameter_value(const std::array<uint8_t, 8>& data, uint8_t rid,
                            std::string& value_kind, std::string& status) {
    const uint32_t raw32 = static_cast<uint32_t>(data[4]) |
                           (static_cast<uint32_t>(data[5]) << 8) |
                           (static_cast<uint32_t>(data[6]) << 16) |
                           (static_cast<uint32_t>(data[7]) << 24);
    if (is_integer_parameter(rid)) {
        value_kind = "uint32";
        return std::to_string(raw32);
    }
    if (!is_known_float_parameter(rid)) {
        value_kind = "raw32";
        return "";
    }
    float value = 0.0f;
    std::memcpy(&value, &raw32, sizeof(value));
    value_kind = "float32";
    if (!std::isfinite(value)) {
        status = "nonfinite";
        return "";
    }
    std::ostringstream out;
    out << std::setprecision(std::numeric_limits<float>::max_digits10) << value;
    return out.str();
}

std::optional<double> numeric_parameter_value(const std::array<uint8_t, 8>& data,
                                              uint8_t rid) {
    const uint32_t raw32 = static_cast<uint32_t>(data[4]) |
                           (static_cast<uint32_t>(data[5]) << 8) |
                           (static_cast<uint32_t>(data[6]) << 16) |
                           (static_cast<uint32_t>(data[7]) << 24);
    if (is_integer_parameter(rid)) return static_cast<double>(raw32);
    if (!is_known_float_parameter(rid)) return std::nullopt;
    float value = 0.0f;
    std::memcpy(&value, &raw32, sizeof(value));
    if (!std::isfinite(value)) return std::nullopt;
    return static_cast<double>(value);
}

std::vector<ParameterSpec> selected_parameters(bool include_legacy) {
    std::vector<ParameterSpec> result(kParameters.begin(), kParameters.end());
    if (include_legacy) result.insert(result.end(), kLegacyParameters.begin(), kLegacyParameters.end());
    return result;
}

void print_registers(bool include_legacy) {
    std::cout << "rid,name,interpretation_note\n";
    for (const auto& spec : selected_parameters(include_legacy)) {
        std::cout << static_cast<unsigned int>(spec.rid) << ',' << spec.name << ',';
        if (spec.rid == 16) std::cout << "current manual: pole-pair count";
        else if (spec.rid >= 50 && spec.rid <= 56)
            std::cout << "legacy or firmware dependent; preserve raw bytes";
        else std::cout << "name follows current SDK/manual table where identified";
        std::cout << '\n';
    }
}

class PassiveProtocol {
public:
    PassiveProtocol(SerialPort::SharedPtr serial, int timeout_ms)
        : serial_(std::move(serial)), timeout_ms_(timeout_ms) {}

    bool send_request(uint32_t slave_id, uint8_t opcode, uint8_t rid) {
        const bool recognized_rid = (rid <= 37) || (rid >= 50 && rid <= 56) ||
            (rid >= 59 && rid <= 65) || (rid >= 70 && rid <= 73) ||
            (rid >= 80 && rid <= 83);
        if (!((opcode == kReadParameterOpcode && recognized_rid) ||
              (opcode == kStatusOpcode && rid == 0))) {
            return false;
        }
        DM_Motor::can_send_frame packet;
        packet.modify(kBroadcastCanId, std::array<uint8_t, 8>{
            static_cast<uint8_t>(slave_id & 0xFF),
            static_cast<uint8_t>((slave_id >> 8) & 0xFF),
            opcode, rid, 0, 0, 0, 0}.data());
        return serial_->send(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet)) ==
               static_cast<ssize_t>(sizeof(packet));
    }

    Response receive_parameter(uint32_t slave_id, uint8_t rid) {
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(timeout_ms_);
        do {
            const auto now = std::chrono::steady_clock::now();
            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
            const int wait_ms = std::max(1, static_cast<int>(remaining.count()));
            serial_->set_timeout(wait_ms);
            ParsedFrame frame{};
            serial_->recv(reinterpret_cast<uint8_t*>(&frame), 0xAA, sizeof(frame));
            if (!valid_frame(frame)) continue;
            const uint32_t response_slave = static_cast<uint32_t>(frame.data[0]) |
                (static_cast<uint32_t>(frame.data[1]) << 8);
            if (response_slave != slave_id || frame.data[2] != kReadParameterOpcode ||
                frame.data[3] != rid) continue;
            return {true, "success", frame,
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count()};
        } while (std::chrono::steady_clock::now() < deadline);
        return {};
    }

    Response receive_status(uint32_t slave_id, uint32_t configured_master_id,
                            const std::optional<uint32_t>& actual_master_id) {
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(timeout_ms_);
        do {
            const auto now = std::chrono::steady_clock::now();
            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
            serial_->set_timeout(std::max(1, static_cast<int>(remaining.count())));
            ParsedFrame frame{};
            serial_->recv(reinterpret_cast<uint8_t*>(&frame), 0xAA, sizeof(frame));
            if (!valid_frame(frame)) continue;
            const uint32_t response_slave = static_cast<uint32_t>(frame.data[0]) |
                (static_cast<uint32_t>(frame.data[1]) << 8);
            if (response_slave == slave_id && frame.data[2] == kReadParameterOpcode) continue;
            const uint32_t expected_master = actual_master_id
                ? *actual_master_id : configured_master_id;
            const bool master_match = frame.can_id == expected_master;
            const bool payload_slave_match = slave_id <= 0x0F &&
                                             (frame.data[0] & 0x0F) == slave_id;
            if (!master_match || !payload_slave_match) continue;
            return {true, "success", frame,
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count()};
        } while (std::chrono::steady_clock::now() < deadline);
        return {};
    }

    static bool valid_frame(const ParsedFrame& frame) {
        return frame.header == 0xAA && frame.command == kResponseCommand &&
               frame.end == kFrameEnd && (frame.can_len_and_flags & 0x3F) == 8 &&
               (frame.can_len_and_flags & 0xC0) == 0 && frame.can_id <= 0x7FF;
    }

private:
    SerialPort::SharedPtr serial_;
    int timeout_ms_;
};

void write_manifest(const fs::path& out_dir, const Options& options,
                    const ArmConfig& config, const std::string& timestamp) {
    std::ofstream manifest(out_dir / "manifest.txt");
    if (!manifest) throw std::runtime_error("Could not create manifest.txt");
    manifest << "Read-only Damiao motor parameter snapshot\n"
             << "timestamp_local=" << timestamp << "\n"
             << "config_source=" << options.config_path << "\n"
             << "config_copy=config.yml\n"
             << "serial_port=" << config.can.id << "\n"
             << "baud_rate=" << config.can.baud_rate << "\n"
             << "protocol=OpenEAI software CAN serial wrapper; config-driven standard CAN IDs\n"
             << "parameter_table=RID 0..37, 54..56, 59..65, 70..73, 80..83\n"
             << "legacy_table_included=" << (options.include_legacy_registers ? "true" : "false") << "\n"
             << "parameter_read_opcode=0x33\n"
             << "status_read_opcode=0xCC\n"
             << "writes=none; enable/disable/zero/control/mode-change/save=none\n"
             << "timeout_ms=" << options.timeout_ms << "\n"
             << "retries=" << options.retries << "\n"
             << "motors=" << config.motors.size() << "\n"
             << "Note: timeout means no matching reply arrived; it does not prove unsupported RID.\n"
             << "RID54 note: current table labels it x_off; older SDKs labeled RID54 m_off. Raw bytes are preserved.\n"
             << "RID16 note: current manual identifies this as pole-pair count.\n"
             << "Protocol has no transaction sequence number; collection is sequential and matches read replies by slave ID and RID.\n"
             << "Official SDK reference: https://github.com/dmBots/motor-sdk/tree/main/Python%E4%BE%8B%E7%A8%8B/u2can\n";
    if (!manifest) throw std::runtime_error("Failed writing manifest.txt");
}

void write_csv_headers(std::ofstream& params, std::ofstream& status) {
    params << "index,name,type,slave_id,master_id,rid,parameter,status,value_kind,value,raw32_le_hex,raw_bytes_hex,attempts\n";
    status << "index,name,type,slave_id,master_id,mst_id_readback,master_id_matches_config,status,receive_monotonic_ns,can_id,can_flags_len,raw_data_hex,error_nibble,q_raw,dq_raw,tau_raw,temp_mos_raw,temp_cel_raw,decoded_q_rad,decoded_dq_rad_s,decoded_tau_nm,decode_limits_source\n";
}

void write_parameter_row(std::ofstream& output, size_t index,
                         const MotorConfig& motor, const ParameterSpec& spec,
                         const Response& response, int attempts) {
    output << index << ',' << csv_escape(motor.name) << ',' << motor.type << ",0x"
           << std::hex << motor.slave_id << ",0x" << motor.master_id << std::dec
           << ',' << static_cast<unsigned int>(spec.rid) << ',' << spec.name << ',';
    if (response.received) {
        const uint32_t raw32 = static_cast<uint32_t>(response.frame.data[4]) |
                               (static_cast<uint32_t>(response.frame.data[5]) << 8) |
                               (static_cast<uint32_t>(response.frame.data[6]) << 16) |
                               (static_cast<uint32_t>(response.frame.data[7]) << 24);
        std::string value_kind;
        std::string parsed_status = "success";
        const std::string value = parameter_value(response.frame.data, spec.rid,
                                                  value_kind, parsed_status);
        output << (parsed_status == "success" ? "success" : parsed_status) << ','
               << value_kind << ',' << value << ",0x" << std::hex << std::setw(8)
               << std::setfill('0') << raw32 << std::dec << ','
               << hex_bytes(response.frame.data.data(), response.frame.data.size())
               << ',' << attempts;
    } else {
        output << response.status << ",,,,," << attempts;
    }
    output << '\n';
    output.flush();
    if (!output) throw std::runtime_error("Failed writing parameters.csv");
}

bool write_status_row(std::ofstream& output, size_t index, const MotorConfig& motor,
                      const Response& response,
                      const DM_Motor::Limit_param& limits,
                      const std::string& limits_source,
                      const std::optional<uint32_t>& actual_master_id) {
    output << index << ',' << csv_escape(motor.name) << ',' << motor.type << ",0x"
           << std::hex << motor.slave_id << ",0x" << motor.master_id << std::dec << ',';
    if (actual_master_id) {
        output << "0x" << std::hex << *actual_master_id << std::dec << ','
               << (*actual_master_id == motor.master_id ? "true" : "false") << ',';
    } else {
        output << ",,";
    }
    output << response.status << ',';
    if (!response.received) {
        output << std::string(13, ',') << '\n';
        output.flush();
        if (!output) throw std::runtime_error("Failed writing status.csv");
        return false;
    }
    const auto& frame = response.frame;
    const uint8_t* d = frame.data.data();
    const uint16_t q = static_cast<uint16_t>((d[1] << 8) | d[2]);
    const uint16_t dq = static_cast<uint16_t>((d[3] << 4) | (d[4] >> 4));
    const uint16_t tau = static_cast<uint16_t>(((d[4] & 0x0F) << 8) | d[5]);
    const auto decode = [](uint16_t raw, float min, float max, int bits) {
        return static_cast<double>(raw) / ((1u << bits) - 1u) * (max - min) + min;
    };
    const bool limits_valid = std::isfinite(limits.Q_MAX) && limits.Q_MAX > 0 &&
                              std::isfinite(limits.DQ_MAX) && limits.DQ_MAX > 0 &&
                              std::isfinite(limits.TAU_MAX) && limits.TAU_MAX > 0;
    output << response.receive_monotonic_ns << ",0x" << std::hex << frame.can_id << std::dec << ",0x"
           << std::hex << static_cast<unsigned int>(frame.can_len_and_flags) << std::dec
           << ',' << hex_bytes(d, 8) << ',' << static_cast<unsigned int>((d[0] >> 4) & 0x0F)
           << ',' << q << ',' << dq << ',' << tau << ','
           << static_cast<unsigned int>(d[6]) << ','
           << static_cast<unsigned int>(d[7]) << ',';
    if (limits_valid) {
        output << std::setprecision(9)
               << decode(q, -limits.Q_MAX, limits.Q_MAX, 16) << ','
               << decode(dq, -limits.DQ_MAX, limits.DQ_MAX, 12) << ','
               << decode(tau, -limits.TAU_MAX, limits.TAU_MAX, 12) << ','
               << limits_source;
    } else {
        output << ",,," << limits_source;
    }
    output << '\n';
    output.flush();
    if (!output) throw std::runtime_error("Failed writing status.csv");
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parse_options(argc, argv);
        if (options.list_registers) {
            print_registers(options.include_legacy_registers);
            return 0;
        }
        const ArmConfig config = load_arm_config(options.config_path);
        validate_config(config);
        const std::vector<ParameterSpec> parameters =
            selected_parameters(options.include_legacy_registers);
        const fs::path output_dir = create_output_dir(options);
        std::error_code copy_error;
        fs::copy_file(options.config_path, output_dir / "config.yml",
                      fs::copy_options::none, copy_error);
        if (copy_error) throw std::runtime_error("Could not snapshot config: " + copy_error.message());

        const std::string timestamp = timestamp_now();
        write_manifest(output_dir, options, config, timestamp);
        std::ofstream params(output_dir / "parameters.csv");
        std::ofstream statuses(output_dir / "status.csv");
        if (!params || !statuses) throw std::runtime_error("Could not create output CSV files");
        write_csv_headers(params, statuses);

        auto serial = std::make_shared<SerialPort>(
            config.can.id, parse_baud_rate(config.can.baud_rate), options.timeout_ms);
        PassiveProtocol protocol(serial, options.timeout_ms);
        bool collection_ok = true;
        for (size_t motor_index = 0; motor_index < config.motors.size(); ++motor_index) {
            const MotorConfig& motor = config.motors[motor_index];
            bool motor_has_any_parameter = false;
            std::map<uint8_t, double> read_values;
            std::cout << "Reading configured motor " << motor_index << " ("
                      << motor.name << ", slave 0x" << std::hex << motor.slave_id
                      << std::dec << ")\n";
            for (const ParameterSpec& spec : parameters) {
                Response response;
                int attempts = 0;
                for (; attempts < options.retries; ++attempts) {
                    if (!protocol.send_request(motor.slave_id, kReadParameterOpcode, spec.rid)) {
                        response.status = "send_error";
                        ++attempts;
                        collection_ok = false;
                        break;
                    }
                    response = protocol.receive_parameter(motor.slave_id, spec.rid);
                    if (response.received) {
                        motor_has_any_parameter = true;
                        const auto numeric = numeric_parameter_value(response.frame.data, spec.rid);
                        if (numeric) read_values[spec.rid] = *numeric;
                        ++attempts;
                        break;
                    }
                }
                write_parameter_row(params, motor_index, motor, spec, response, attempts);
            }

            const auto master_it = read_values.find(7);
            std::optional<uint32_t> actual_master_id;
            if (master_it != read_values.end() && master_it->second >= 0.0 &&
                master_it->second < static_cast<double>(kBroadcastCanId)) {
                actual_master_id = static_cast<uint32_t>(master_it->second);
            }
            Response status_response;
            for (int attempt = 0; attempt < options.retries; ++attempt) {
                if (!protocol.send_request(motor.slave_id, kStatusOpcode, 0)) {
                    status_response.status = "send_error";
                    break;
                }
                status_response = protocol.receive_status(motor.slave_id,
                    motor.master_id, actual_master_id);
                if (status_response.received) break;
            }

            DM_Motor::Limit_param limits{
                std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::quiet_NaN()};
            const auto pmax = read_values.find(21);
            const auto vmax = read_values.find(22);
            const auto tmax = read_values.find(23);
            bool limits_available = pmax != read_values.end() &&
                vmax != read_values.end() && tmax != read_values.end() &&
                pmax->second > 0 && vmax->second > 0 && tmax->second > 0;
            if (limits_available) {
                limits = {static_cast<float>(pmax->second),
                          static_cast<float>(vmax->second),
                          static_cast<float>(tmax->second)};
            } else {
                collection_ok = false;
                std::cerr << "WARNING: " << motor.name
                          << " is missing a valid PMAX/VMAX/TMAX readback; status feedback will remain raw.\n";
            }
            if (!motor_has_any_parameter) {
                collection_ok = false;
                std::cerr << "WARNING: no parameter replies received for " << motor.name << ".\n";
            }
            if (!status_response.received) {
                collection_ok = false;
                std::cerr << "WARNING: no matching passive status reply received for " << motor.name << ".\n";
            }
            write_status_row(statuses, motor_index, motor, status_response,
                             limits, limits_available ? "RID_21_22_23_readback" :
                             "unavailable_readback_required", actual_master_id);
        }

        std::cout << "Snapshot written to " << output_dir.string() << '\n'
                  << "No motor enable, disable, zero, control, mode-change, parameter-write, or save commands were sent.\n";
        return collection_ok ? 0 : 2;
    } catch (const std::exception& error) {
        std::cerr << "read_motor_params: " << error.what() << '\n';
        return 1;
    }
}
