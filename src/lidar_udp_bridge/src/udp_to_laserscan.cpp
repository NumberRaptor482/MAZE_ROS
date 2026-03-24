#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#pragma pack(push, 1)
struct PacketHeader {
  char magic[4];      // "LDR1"
  uint32_t seq;
  uint64_t t_us;
  uint16_t count;
};
struct PacketPoint {
  uint16_t angle_cdeg;
  uint16_t dist_mm;
  uint8_t quality;
};
#pragma pack(pop)

class UdpToLaserScan : public rclcpp::Node {
public:
  UdpToLaserScan() : Node("udp_to_laserscan") {
    port_ = declare_parameter<int>("port", 5005);
    frame_id_ = declare_parameter<std::string>("frame_id", "laser");
    topic_ = declare_parameter<std::string>("topic", "/scan");
    publish_hz_ = declare_parameter<double>("publish_hz", 10.0);
    range_min_ = declare_parameter<double>("range_min", 0.15);
    range_max_ = declare_parameter<double>("range_max", 6.0);

    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(topic_, 10);

    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) throw std::runtime_error("socket failed");

    int reuse = 1;
    setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock_, (sockaddr*)&addr, sizeof(addr)) < 0) {
      throw std::runtime_error("bind failed (port in use?)");
    }

    dist_mm_.fill(0);

    poll_timer_ = create_wall_timer(std::chrono::milliseconds(2),
                                    std::bind(&UdpToLaserScan::poll_socket, this));

    publish_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(0.1, publish_hz_)),
        std::bind(&UdpToLaserScan::publish_scan, this));

    RCLCPP_INFO(get_logger(), "Listening UDP :%d, publishing LaserScan %s frame=%s",
                port_, topic_.c_str(), frame_id_.c_str());
  }

  ~UdpToLaserScan() override {
    if (sock_ >= 0) close(sock_);
  }

private:
  void poll_socket() {
    uint8_t buf[2048];
    ssize_t n = recvfrom(sock_, buf, sizeof(buf), MSG_DONTWAIT, nullptr, nullptr);
    if (n <= 0) return;
    if ((size_t)n < sizeof(PacketHeader)) return;

    PacketHeader hdr{};
    std::memcpy(&hdr, buf, sizeof(hdr));
    if (std::memcmp(hdr.magic, "LDR1", 4) != 0) return;

    size_t expected = sizeof(PacketHeader) + hdr.count * sizeof(PacketPoint);
    if ((size_t)n < expected) return;

    auto* pts = reinterpret_cast<const PacketPoint*>(buf + sizeof(PacketHeader));

    for (size_t i = 0; i < hdr.count; i++) {
      int bin = (pts[i].angle_cdeg / 100) % 360;
      uint16_t d = pts[i].dist_mm;

      // keep nearest per bin
      if (d == 0) continue;
      if (dist_mm_[bin] == 0 || d < dist_mm_[bin]) dist_mm_[bin] = d;
    }
  }

  void publish_scan() {
    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    msg.angle_min = 0.0;
    msg.angle_max = 2.0 * M_PI;
    msg.angle_increment = (msg.angle_max - msg.angle_min) / 360.0;

    msg.time_increment = 0.0;
    msg.scan_time = 0.0;

    msg.range_min = range_min_;
    msg.range_max = range_max_;

    msg.ranges.resize(360);
    for (int i = 0; i < 360; i++) {
      if (dist_mm_[i] == 0) {
        msg.ranges[i] = std::numeric_limits<float>::infinity();
      } else {
        float r = dist_mm_[i] / 1000.0f;
        if (r < msg.range_min || r > msg.range_max) {
          msg.ranges[i] = std::numeric_limits<float>::infinity();
        } else {
          msg.ranges[i] = r;
        }
      }
      // reset bin after publishing so next publish reflects fresh data
      dist_mm_[i] = 0;
    }

    pub_->publish(msg);
  }

  int sock_{-1};
  int port_{5005};
  std::string frame_id_;
  std::string topic_;
  double publish_hz_{10.0};
  double range_min_{0.15};
  double range_max_{6.0};

  std::array<uint16_t, 360> dist_mm_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<UdpToLaserScan>());
  } catch (const std::exception& e) {
    std::cerr << "Fatal: " << e.what() << "\n";
  }
  rclcpp::shutdown();
  return 0;
}
