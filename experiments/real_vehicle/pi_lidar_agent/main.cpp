#include <arpa/inet.h>
#include <netdb.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

namespace {

constexpr std::uint8_t kProtocolVersion = 1;
constexpr std::uint8_t kMessageTypeScan = 1;
constexpr std::size_t kHeaderSize = 64;
constexpr std::size_t kChecksumOffset = 60;
constexpr std::uint16_t kFlagHealthOk = 1U << 0U;
constexpr std::uint16_t kFlagNormalized = 1U << 1U;
constexpr std::uint16_t kFlagAngleInverted = 1U << 2U;
constexpr double kPi = 3.14159265358979323846;

volatile sig_atomic_t g_stop_requested = 0;

void handle_signal(int) {
    g_stop_requested = 1;
}

struct Options {
    std::string device =
        "/dev/serial/by-id/"
        "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0";
    std::uint32_t baudrate = 115200;
    std::string host = "127.0.0.1";
    std::uint16_t port = 5010;
    std::uint16_t num_beams = 360;
    std::uint16_t beams_per_packet = 120;
    double range_min_m = 0.15;
    double range_max_m = 12.0;
    bool angle_inverted = false;
    double angle_offset_rad = 0.0;
    std::uint64_t max_scans = 0;
    std::string csv_path;
    bool synthetic = false;
    double synthetic_rate_hz = 5.5;
};

void print_usage(const char* program) {
    std::cout
        << "Usage: " << program << " [options]\n"
        << "  --device PATH              RPLIDAR serial device\n"
        << "  --baudrate N               Serial baudrate (default 115200)\n"
        << "  --host HOST                PC hostname or IPv4 address\n"
        << "  --port N                   PC UDP port (default 5010)\n"
        << "  --beams N                  Normalized beam count (default 360)\n"
        << "  --beams-per-packet N       UDP chunk size (default 120)\n"
        << "  --range-min M              Minimum valid range (default 0.15)\n"
        << "  --range-max M              Maximum valid range (default 12.0)\n"
        << "  --angle-inverted           Reverse raw angle direction\n"
        << "  --angle-offset-rad R       Add yaw offset before binning\n"
        << "  --max-scans N              Stop after N scans; 0 is unlimited\n"
        << "  --csv PATH                 Write per-scan statistics\n"
        << "  --synthetic                Send scans without opening a LiDAR\n"
        << "  --synthetic-rate-hz HZ     Synthetic send rate (default 5.5)\n"
        << "  --help                     Show this message\n";
}

template <typename T>
T parse_integer(const char* value, const char* option) {
    try {
        const unsigned long long parsed = std::stoull(value);
        if (parsed > static_cast<unsigned long long>(
                         std::numeric_limits<T>::max())) {
            throw std::out_of_range("integer overflow");
        }
        return static_cast<T>(parsed);
    } catch (const std::exception&) {
        throw std::runtime_error(std::string("invalid value for ") + option);
    }
}

double parse_double(const char* value, const char* option) {
    try {
        const double parsed = std::stod(value);
        if (!std::isfinite(parsed)) {
            throw std::out_of_range("non-finite value");
        }
        return parsed;
    } catch (const std::exception&) {
        throw std::runtime_error(std::string("invalid value for ") + option);
    }
}

Options parse_options(int argc, char** argv) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string argument(argv[index]);
        auto require_value = [&](const char* option) -> const char* {
            if (index + 1 >= argc) {
                throw std::runtime_error(std::string(option) + " requires a value");
            }
            return argv[++index];
        };
        if (argument == "--device") {
            options.device = require_value("--device");
        } else if (argument == "--baudrate") {
            options.baudrate = parse_integer<std::uint32_t>(
                require_value("--baudrate"), "--baudrate");
        } else if (argument == "--host") {
            options.host = require_value("--host");
        } else if (argument == "--port") {
            options.port = parse_integer<std::uint16_t>(
                require_value("--port"), "--port");
        } else if (argument == "--beams") {
            options.num_beams = parse_integer<std::uint16_t>(
                require_value("--beams"), "--beams");
        } else if (argument == "--beams-per-packet") {
            options.beams_per_packet = parse_integer<std::uint16_t>(
                require_value("--beams-per-packet"), "--beams-per-packet");
        } else if (argument == "--range-min") {
            options.range_min_m = parse_double(
                require_value("--range-min"), "--range-min");
        } else if (argument == "--range-max") {
            options.range_max_m = parse_double(
                require_value("--range-max"), "--range-max");
        } else if (argument == "--angle-inverted") {
            options.angle_inverted = true;
        } else if (argument == "--angle-offset-rad") {
            options.angle_offset_rad = parse_double(
                require_value("--angle-offset-rad"), "--angle-offset-rad");
        } else if (argument == "--max-scans") {
            options.max_scans = parse_integer<std::uint64_t>(
                require_value("--max-scans"), "--max-scans");
        } else if (argument == "--csv") {
            options.csv_path = require_value("--csv");
        } else if (argument == "--synthetic") {
            options.synthetic = true;
        } else if (argument == "--synthetic-rate-hz") {
            options.synthetic_rate_hz = parse_double(
                require_value("--synthetic-rate-hz"), "--synthetic-rate-hz");
        } else if (argument == "--help") {
            print_usage(argv[0]);
            std::exit(0);
        } else {
            throw std::runtime_error("unknown option: " + argument);
        }
    }
    if (options.num_beams < 1) {
        throw std::runtime_error("--beams must be positive");
    }
    if (options.beams_per_packet < 1 || options.beams_per_packet > 300) {
        throw std::runtime_error("--beams-per-packet must be in [1, 300]");
    }
    if (options.range_min_m < 0.0 ||
        options.range_min_m >= options.range_max_m) {
        throw std::runtime_error("invalid range limits");
    }
    if (options.synthetic_rate_hz <= 0.0) {
        throw std::runtime_error("--synthetic-rate-hz must be positive");
    }
    return options;
}

std::uint32_t crc32(const std::uint8_t* data, std::size_t size) {
    std::uint32_t crc = 0xFFFFFFFFU;
    for (std::size_t index = 0; index < size; ++index) {
        crc ^= data[index];
        for (int bit = 0; bit < 8; ++bit) {
            const std::uint32_t mask = -(crc & 1U);
            crc = (crc >> 1U) ^ (0xEDB88320U & mask);
        }
    }
    return ~crc;
}

void append_u8(std::vector<std::uint8_t>& data, std::uint8_t value) {
    data.push_back(value);
}

void append_u16(std::vector<std::uint8_t>& data, std::uint16_t value) {
    const std::uint16_t network = htons(value);
    const auto* bytes = reinterpret_cast<const std::uint8_t*>(&network);
    data.insert(data.end(), bytes, bytes + sizeof(network));
}

void append_u32(std::vector<std::uint8_t>& data, std::uint32_t value) {
    const std::uint32_t network = htonl(value);
    const auto* bytes = reinterpret_cast<const std::uint8_t*>(&network);
    data.insert(data.end(), bytes, bytes + sizeof(network));
}

void append_u64(std::vector<std::uint8_t>& data, std::uint64_t value) {
    append_u32(data, static_cast<std::uint32_t>(value >> 32U));
    append_u32(data, static_cast<std::uint32_t>(value & 0xFFFFFFFFU));
}

void append_float(std::vector<std::uint8_t>& data, float value) {
    static_assert(sizeof(float) == sizeof(std::uint32_t), "float32 required");
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    append_u32(data, bits);
}

void write_u32(std::vector<std::uint8_t>& data,
               std::size_t offset,
               std::uint32_t value) {
    const std::uint32_t network = htonl(value);
    std::memcpy(data.data() + offset, &network, sizeof(network));
}

class UdpSender {
public:
    UdpSender(const std::string& host, std::uint16_t port) {
        struct addrinfo hints {};
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_DGRAM;
        struct addrinfo* result = nullptr;
        const std::string service = std::to_string(port);
        const int status = getaddrinfo(host.c_str(), service.c_str(), &hints, &result);
        if (status != 0 || result == nullptr) {
            throw std::runtime_error(
                "cannot resolve UDP host " + host + ": " + gai_strerror(status));
        }
        socket_fd_ = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
        if (socket_fd_ < 0) {
            freeaddrinfo(result);
            throw std::runtime_error("cannot create UDP socket");
        }
        std::memcpy(&destination_, result->ai_addr, result->ai_addrlen);
        destination_size_ = static_cast<socklen_t>(result->ai_addrlen);
        freeaddrinfo(result);
    }

    ~UdpSender() {
        if (socket_fd_ >= 0) {
            close(socket_fd_);
        }
    }

    void send(const std::vector<std::uint8_t>& packet) {
        const ssize_t sent = sendto(
            socket_fd_,
            packet.data(),
            packet.size(),
            0,
            reinterpret_cast<const struct sockaddr*>(&destination_),
            destination_size_);
        if (sent < 0 || static_cast<std::size_t>(sent) != packet.size()) {
            throw std::runtime_error("UDP send failed");
        }
    }

private:
    int socket_fd_ = -1;
    struct sockaddr_storage destination_ {};
    socklen_t destination_size_ = 0;
};

std::uint64_t unix_time_ns() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
}

double wrap_angle(double angle) {
    angle = std::fmod(angle + kPi, 2.0 * kPi);
    if (angle < 0.0) {
        angle += 2.0 * kPi;
    }
    return angle - kPi;
}

struct NormalizedScan {
    std::vector<float> ranges;
    std::uint16_t valid_beams = 0;
};

NormalizedScan normalize_nodes(
    const sl_lidar_response_measurement_node_hq_t* nodes,
    std::size_t count,
    const Options& options) {
    NormalizedScan scan;
    scan.ranges.assign(
        options.num_beams,
        std::numeric_limits<float>::infinity());
    const double increment = 2.0 * kPi / options.num_beams;
    const double direction = options.angle_inverted ? -1.0 : 1.0;
    for (std::size_t index = 0; index < count; ++index) {
        const double distance_m = nodes[index].dist_mm_q2 / 4000.0;
        if (!std::isfinite(distance_m) || distance_m < options.range_min_m ||
            distance_m > options.range_max_m) {
            continue;
        }
        const double raw_degrees = nodes[index].angle_z_q14 * 90.0 / 16384.0;
        const double angle = wrap_angle(
            direction * raw_degrees * kPi / 180.0 + options.angle_offset_rad);
        const double position = (angle + kPi) / increment;
        const std::size_t beam = static_cast<std::size_t>(
            std::floor(position + 0.5)) % options.num_beams;
        scan.ranges[beam] = std::min(
            scan.ranges[beam], static_cast<float>(distance_m));
    }
    scan.valid_beams = static_cast<std::uint16_t>(std::count_if(
        scan.ranges.begin(), scan.ranges.end(), [](float value) {
            return std::isfinite(value);
        }));
    return scan;
}

NormalizedScan synthetic_scan(std::uint32_t scan_id, const Options& options) {
    NormalizedScan scan;
    scan.ranges.assign(
        options.num_beams,
        std::numeric_limits<float>::infinity());
    const std::size_t center = (options.num_beams / 2U + scan_id) % options.num_beams;
    for (int offset = -2; offset <= 2; ++offset) {
        const std::size_t index = static_cast<std::size_t>(
            (static_cast<long>(center) + offset + options.num_beams) %
            options.num_beams);
        scan.ranges[index] = 1.0F + 0.05F * std::abs(offset);
    }
    scan.valid_beams = 5;
    return scan;
}

std::vector<std::uint8_t> make_packet(
    const NormalizedScan& scan,
    std::uint32_t session_id,
    std::uint32_t scan_id,
    std::uint16_t chunk_index,
    std::uint16_t chunk_count,
    std::uint16_t beam_start,
    std::uint16_t beam_count,
    std::uint64_t capture_time_ns,
    std::uint32_t scan_period_us,
    std::uint16_t raw_points,
    std::uint16_t flags,
    const Options& options) {
    std::vector<std::uint8_t> packet;
    packet.reserve(kHeaderSize + beam_count * sizeof(float));
    packet.insert(packet.end(), {'R', 'V', 'L', 'D'});
    append_u8(packet, kProtocolVersion);
    append_u8(packet, kMessageTypeScan);
    append_u16(packet, static_cast<std::uint16_t>(kHeaderSize));
    append_u32(packet, session_id);
    append_u32(packet, scan_id);
    append_u16(packet, chunk_index);
    append_u16(packet, chunk_count);
    append_u16(packet, beam_start);
    append_u16(packet, beam_count);
    append_u16(packet, options.num_beams);
    append_u16(packet, flags);
    append_u64(packet, capture_time_ns);
    append_u32(packet, scan_period_us);
    append_u16(packet, scan.valid_beams);
    append_u16(packet, raw_points);
    append_float(packet, static_cast<float>(-kPi));
    append_float(packet, static_cast<float>(2.0 * kPi / options.num_beams));
    append_float(packet, static_cast<float>(options.range_min_m));
    append_float(packet, static_cast<float>(options.range_max_m));
    append_u32(packet, 0);
    if (packet.size() != kHeaderSize) {
        throw std::logic_error("internal LiDAR header size mismatch");
    }
    for (std::uint16_t offset = 0; offset < beam_count; ++offset) {
        append_float(packet, scan.ranges[beam_start + offset]);
    }
    const std::uint32_t checksum = crc32(packet.data(), packet.size());
    write_u32(packet, kChecksumOffset, checksum);
    return packet;
}

std::uint16_t send_scan(
    UdpSender& sender,
    const NormalizedScan& scan,
    std::uint32_t session_id,
    std::uint32_t scan_id,
    std::uint64_t capture_time_ns,
    std::uint32_t scan_period_us,
    std::uint16_t raw_points,
    bool health_ok,
    const Options& options) {
    const std::uint16_t chunk_count = static_cast<std::uint16_t>(
        (options.num_beams + options.beams_per_packet - 1U) /
        options.beams_per_packet);
    std::uint16_t flags = kFlagNormalized;
    if (health_ok) {
        flags |= kFlagHealthOk;
    }
    if (options.angle_inverted) {
        flags |= kFlagAngleInverted;
    }
    for (std::uint16_t chunk = 0; chunk < chunk_count; ++chunk) {
        const std::uint16_t start = chunk * options.beams_per_packet;
        const std::uint16_t count = std::min<std::uint16_t>(
            options.beams_per_packet, options.num_beams - start);
        sender.send(make_packet(
            scan,
            session_id,
            scan_id,
            chunk,
            chunk_count,
            start,
            count,
            capture_time_ns,
            scan_period_us,
            raw_points,
            flags,
            options));
    }
    return chunk_count;
}

class CsvLogger {
public:
    explicit CsvLogger(const std::string& path) {
        if (!path.empty()) {
            output_.open(path);
            if (!output_) {
                throw std::runtime_error("cannot open CSV path: " + path);
            }
            output_
                << "capture_time_ns,scan_id,raw_points,valid_beams,valid_ratio,"
                   "scan_period_us,packets_sent\n";
        }
    }

    void write(
        std::uint64_t capture_time_ns,
        std::uint32_t scan_id,
        std::uint16_t raw_points,
        const NormalizedScan& scan,
        std::uint32_t scan_period_us,
        std::uint16_t packets_sent) {
        if (!output_) {
            return;
        }
        output_ << capture_time_ns << ',' << scan_id << ',' << raw_points << ','
                << scan.valid_beams << ',' << std::fixed << std::setprecision(6)
                << static_cast<double>(scan.valid_beams) / scan.ranges.size() << ','
                << scan_period_us << ',' << packets_sent << '\n';
        output_.flush();
    }

private:
    std::ofstream output_;
};

std::uint32_t create_session_id() {
    std::random_device device;
    const auto now = static_cast<std::uint32_t>(unix_time_ns());
    return device() ^ now ^ static_cast<std::uint32_t>(getpid());
}

int run_synthetic(const Options& options) {
    UdpSender sender(options.host, options.port);
    CsvLogger logger(options.csv_path);
    const std::uint32_t session_id = create_session_id();
    const auto period = std::chrono::duration<double>(1.0 / options.synthetic_rate_hz);
    const std::uint32_t period_us = static_cast<std::uint32_t>(
        std::llround(1000000.0 / options.synthetic_rate_hz));
    std::cout << "synthetic LiDAR host=" << options.host
              << " port=" << options.port
              << " session=" << session_id << std::endl;
    std::uint32_t scan_id = 0;
    auto next = std::chrono::steady_clock::now();
    while (!g_stop_requested &&
           (options.max_scans == 0 || scan_id < options.max_scans)) {
        ++scan_id;
        const NormalizedScan scan = synthetic_scan(scan_id, options);
        const std::uint64_t capture = unix_time_ns();
        const std::uint16_t packets = send_scan(
            sender,
            scan,
            session_id,
            scan_id,
            capture,
            period_us,
            options.num_beams,
            true,
            options);
        logger.write(
            capture,
            scan_id,
            options.num_beams,
            scan,
            period_us,
            packets);
        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        std::this_thread::sleep_until(next);
    }
    return 0;
}

int run_lidar(const Options& options) {
    using namespace sl;
    IChannel* channel = nullptr;
    ILidarDriver* driver = nullptr;
    bool motor_started = false;
    bool scan_started = false;
    try {
        Result<IChannel*> channel_result = createSerialPortChannel(
            options.device, static_cast<int>(options.baudrate));
        if (!channel_result) {
            throw std::runtime_error("cannot create LiDAR serial channel");
        }
        channel = *channel_result;
        Result<ILidarDriver*> driver_result = createLidarDriver();
        if (!driver_result) {
            throw std::runtime_error("cannot create SLAMTEC LiDAR driver");
        }
        driver = *driver_result;
        if (SL_IS_FAIL(driver->connect(channel))) {
            throw std::runtime_error("cannot connect to LiDAR serial device");
        }

        sl_lidar_response_device_info_t info {};
        if (SL_IS_FAIL(driver->getDeviceInfo(info))) {
            throw std::runtime_error("cannot read LiDAR device information");
        }
        sl_lidar_response_device_health_t health {};
        if (SL_IS_FAIL(driver->getHealth(health))) {
            throw std::runtime_error("cannot read LiDAR health");
        }
        if (health.status == SL_LIDAR_STATUS_ERROR) {
            throw std::runtime_error("LiDAR reports an internal health error");
        }
        std::cout << "LiDAR model=" << static_cast<int>(info.model)
                  << " firmware=" << (info.firmware_version >> 8U) << '.'
                  << (info.firmware_version & 0xFFU)
                  << " hardware=" << static_cast<int>(info.hardware_version)
                  << " health=" << static_cast<int>(health.status) << std::endl;

        if (SL_IS_FAIL(driver->setMotorSpeed())) {
            throw std::runtime_error("cannot start LiDAR motor");
        }
        motor_started = true;
        LidarScanMode mode {};
        if (SL_IS_FAIL(driver->startScan(false, true, 0, &mode))) {
            throw std::runtime_error("cannot start typical LiDAR scan mode");
        }
        scan_started = true;
        std::cout << "scan_mode=" << mode.scan_mode
                  << " us_per_sample=" << mode.us_per_sample
                  << " max_distance_m=" << mode.max_distance << std::endl;

        UdpSender sender(options.host, options.port);
        CsvLogger logger(options.csv_path);
        const std::uint32_t session_id = create_session_id();
        std::cout << "UDP destination=" << options.host << ':' << options.port
                  << " session=" << session_id << std::endl;
        std::uint32_t scan_id = 0;
        bool have_previous = false;
        auto previous = std::chrono::steady_clock::now();
        while (!g_stop_requested &&
               (options.max_scans == 0 || scan_id < options.max_scans)) {
            sl_lidar_response_measurement_node_hq_t nodes[8192];
            std::size_t count = sizeof(nodes) / sizeof(nodes[0]);
            const sl_result result = driver->grabScanDataHq(nodes, count, 3000);
            if (result == SL_RESULT_OPERATION_TIMEOUT) {
                std::cerr << "LiDAR scan timeout" << std::endl;
                continue;
            }
            if (SL_IS_FAIL(result)) {
                throw std::runtime_error("LiDAR scan acquisition failed");
            }
            if (SL_IS_FAIL(driver->ascendScanData(nodes, count))) {
                throw std::runtime_error("cannot sort LiDAR scan by angle");
            }
            const auto now = std::chrono::steady_clock::now();
            std::uint32_t period_us = 0;
            if (have_previous) {
                const auto measured = std::chrono::duration_cast<
                    std::chrono::microseconds>(now - previous).count();
                period_us = static_cast<std::uint32_t>(std::max<std::int64_t>(
                    0, std::min<std::int64_t>(measured, 0xFFFFFFFFLL)));
            }
            previous = now;
            have_previous = true;
            ++scan_id;
            const NormalizedScan scan = normalize_nodes(nodes, count, options);
            const std::uint64_t capture = unix_time_ns();
            const std::uint16_t raw_points = static_cast<std::uint16_t>(
                std::min<std::size_t>(count, 0xFFFFU));
            const std::uint16_t packets = send_scan(
                sender,
                scan,
                session_id,
                scan_id,
                capture,
                period_us,
                raw_points,
                health.status == SL_LIDAR_STATUS_OK,
                options);
            logger.write(
                capture,
                scan_id,
                raw_points,
                scan,
                period_us,
                packets);
            if (scan_id == 1 || scan_id % 10 == 0) {
                const double rate_hz = period_us > 0
                    ? 1000000.0 / period_us
                    : 0.0;
                std::cout << "scan=" << scan_id
                          << " raw=" << count
                          << " valid=" << scan.valid_beams
                          << " rate_hz=" << std::fixed << std::setprecision(3)
                          << rate_hz << std::endl;
            }
        }
        if (scan_started) {
            driver->stop();
            scan_started = false;
        }
        if (motor_started) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            driver->setMotorSpeed(0);
            motor_started = false;
        }
        driver->disconnect();
        delete driver;
        delete channel;
        std::cout << "LiDAR agent stopped cleanly" << std::endl;
        return 0;
    } catch (...) {
        if (driver != nullptr) {
            if (scan_started) {
                driver->stop();
            }
            if (motor_started) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                driver->setMotorSpeed(0);
            }
            driver->disconnect();
            delete driver;
        }
        delete channel;
        throw;
    }
}

}  // namespace

int main(int argc, char** argv) {
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    try {
        const Options options = parse_options(argc, argv);
        return options.synthetic ? run_synthetic(options) : run_lidar(options);
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << std::endl;
        return 1;
    }
}
