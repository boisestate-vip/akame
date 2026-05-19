
/* a ros2 driver for the bender regolith controller teensy.
 * polls sensor data over the binary serial protocol and
 * publishes it to ros2 topics.
 *
 * intended to be used on a computer connected to the
 * teensy via usb.                                       */

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <functional>

#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

#include "teensy_interface.h"

using std::placeholders::_1;

/* best effort baud constant lookup */
static speed_t get_baud_const(int baud) {
   static const int baud_nums[] = {
      0, 50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800, 2400,
      4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 500000,
      576000, 921600, 1000000, 1152000, 1500000, 2000000, 2500000,
      3000000, 3500000, 4000000,
   };
   static const speed_t baud_vals[] = {
      B0, B50, B75, B110, B134, B150, B200, B300, B600, B1200, B1800,
      B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400,
      B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000,
      B2000000, B2500000, B3000000, B3500000, B4000000,
   };
   static const int n = (int)(sizeof(baud_nums) / sizeof(baud_nums[0]));

   int idx = 0;
   while (idx < n - 1 && baud > baud_nums[idx])
      idx++;

   if (idx == 0 || idx == n - 1)
      return baud_vals[idx];
   if (baud == baud_nums[idx])
      return baud_vals[idx];

   int dist_low  = std::abs(baud_nums[idx - 1] - baud);
   int dist_high = std::abs(baud_nums[idx]     - baud);
   return (dist_low <= dist_high) ? baud_vals[idx - 1] : baud_vals[idx];
}

/* open and configure the serial device.
 * USB CDC ignores baud but we pass the real value for documentation. */
static int try_connect(const char *device, int baud) {
   int fd = open(device, O_RDWR);
   if (fd < 0) {
      perror("open");
      return -1;
   }

   struct termios t;
   if (tcgetattr(fd, &t) < 0) {
      perror("tcgetattr");
      close(fd);
      return -1;
   }

   cfmakeraw(&t);
   speed_t speed = get_baud_const(baud);
   if (cfsetispeed(&t, speed) < 0 || cfsetospeed(&t, speed) < 0) {
      perror("cfsetspeed");
      close(fd);
      return -1;
   }

   if (tcsetattr(fd, TCSANOW, &t) < 0) {
      perror("tcsetattr");
      close(fd);
      return -1;
   }

   return fd;
}

class RegolithDriver : public rclcpp::Node {
public:

   RegolithDriver() : Node("regolith_driver") {

      /* serial device the teensy is connected on. defaults to the udev
       * symlink installed by setup.sh — avoids collision with other
       * /dev/ttyACM* devices (e.g. the roboclaw). */
      this->declare_parameter("device", "/dev/teensy-regolith");

      /* sensor polling rate */
      this->declare_parameter("poll_hz", 10);

      /* topics to publish sensor readings on */
      this->declare_parameter("tof_topic",            "/regolith/tof");
      this->declare_parameter("power_topic",          "/regolith/power");
      this->declare_parameter("analog_topic",         "/regolith/analog");
      this->declare_parameter("mass_topic",           "/regolith/mass");
      this->declare_parameter("kin_calc_topic",       "/regolith/kin_calc");
      this->declare_parameter("estimated_load_topic", "/regolith/estimated_load");

      /* topic to receive actuator position for regolith mass calculation */
      this->declare_parameter("actuator_pos_in", "/regolith/actuator_pos");

      /* publishers */
      tof_pub      = this->create_publisher<std_msgs::msg::Float32>(
                         this->get_parameter("tof_topic").as_string(), 10);
      power_pub    = this->create_publisher<std_msgs::msg::Float32MultiArray>(
                         this->get_parameter("power_topic").as_string(), 10);
      analog_pub   = this->create_publisher<std_msgs::msg::Float32MultiArray>(
                         this->get_parameter("analog_topic").as_string(), 10);
      mass_pub     = this->create_publisher<std_msgs::msg::Float32>(
                         this->get_parameter("mass_topic").as_string(), 10);
      kin_calc_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(
                         this->get_parameter("kin_calc_topic").as_string(), 10);
      estimated_load_pub = this->create_publisher<std_msgs::msg::Float32>(
                               this->get_parameter("estimated_load_topic").as_string(), 10);

      /* actuator position subscriber — required to compute regolith mass */
      actuator_pos_sub = this->create_subscription<std_msgs::msg::Float32>(
                             this->get_parameter("actuator_pos_in").as_string(), 10,
                             std::bind(&RegolithDriver::collect_actuator_pos, this, _1));

      /* connect to device */
      std::string dev = this->get_parameter("device").as_string();
      fd = try_connect(dev.c_str(), 4500000);
      if (fd < 0) {
         RCLCPP_ERROR(this->get_logger(), "failed to connect to regolith controller on %s", dev.c_str());
         exit(1);
      }
      RCLCPP_INFO(this->get_logger(), "connected to regolith controller on %s", dev.c_str());

      /* the teensy's setup() emits text warnings to USB CDC if any sensor
       * fails to init (see controller's tof.cpp / power.cpp). those bytes
       * would sit ahead of our first binary response and permanently
       * desync the protocol. wait for setup() to finish, then drain. */
      RCLCPP_INFO(this->get_logger(), "waiting for teensy boot...");
      usleep(500000);
      tcflush(fd, TCIFLUSH);

      /* initial ping to verify comms */
      send_cmd(CMD_SYN, 0, nullptr);
      struct data res;
      if (read_all(&res, sizeof(res)) < 0 || res.type != DATA_ACK) {
         RCLCPP_WARN(this->get_logger(), "initial ping to teensy failed (type=%d)", res.type);
         tcflush(fd, TCIOFLUSH);
      }
      else {
         RCLCPP_INFO(this->get_logger(), "teensy ping OK");
      }

      /* polling timer */
      int hz = this->get_parameter("poll_hz").as_int();
      poll_timer = this->create_wall_timer(
                       std::chrono::milliseconds(1000 / hz),
                       std::bind(&RegolithDriver::poll, this));
   }

private:

   /* class variables */

   /* file descriptor for the serial connection */
   int fd;

   /* actuator position for mass and kinematic calculations */
   float actuator_pos = 0.0f;
   bool has_actuator_pos = false;

   /* strain gauge bridge voltage (analog channel 0, pin A13) for load estimation */
   float bridge_voltage = 0.0f;
   bool has_bridge_voltage = false;

   /* publishers */
   rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr tof_pub;
   rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr power_pub;
   rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr analog_pub;
   rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mass_pub;
   rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr kin_calc_pub;
   rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr estimated_load_pub;

   /* subscriptions */
   rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr actuator_pos_sub;

   /* poll timer */
   rclcpp::TimerBase::SharedPtr poll_timer;

   void collect_actuator_pos(const std_msgs::msg::Float32 & msg) {
      actuator_pos = msg.data;
      has_actuator_pos = true;
   }

   int read_all(void *buf, size_t n) {
      size_t total = 0;
      while (total < n) {
         ssize_t r = read(fd, (char *)buf + total, n - total);
         if (r <= 0) {
            if (r < 0)
               RCLCPP_WARN(this->get_logger(), "read error: %s", strerror(errno));
            return -1;
         }
         total += r;
      }
      return 0;
   }

   void send_cmd(int type, int argc, float *argv) {
      struct cmd c = {};
      c.magic_0 = CMD_MAGIC_0;
      c.magic_1 = CMD_MAGIC_1;
      c.type    = (char)type;
      c.argc    = (uint8_t)argc;
      for (int i = 0; i < argc && i < MAX_CMD_ARGS; i++)
         c.argv[i] = argv[i];
      if (write(fd, &c, sizeof(struct cmd)) < 0)
         RCLCPP_WARN(this->get_logger(), "write error: %s", strerror(errno));
   }

   /* poll all sensors each tick */
   void poll() {
      poll_tof();
      poll_power();
      poll_analog();
      if (has_actuator_pos) {
         poll_mass();
         poll_kin_calc();
         if (has_bridge_voltage)
            poll_estimate_load();
      }
   }

   /* tof distance in mm */
   void poll_tof() {
      send_cmd(CMD_TOF, 0, nullptr);
      struct data res;
      if (read_all(&res, sizeof(res)) < 0 || res.type != DATA_TOF) {
         RCLCPP_WARN(this->get_logger(), "tof read failed (type=%d)", res.type);
         tcflush(fd, TCIOFLUSH);
         return;
      }
      auto msg = std_msgs::msg::Float32();
      msg.data = res.as.tof.distance_mm;
      tof_pub->publish(msg);
   }

   /* power monitor readings.
    * layout: [voltage, current, power, est_voltage, est_current, est_power] per monitor.
    * zeros are published for uninitialized monitors to preserve indexing. */
   void poll_power() {
      send_cmd(CMD_POWER, 0, nullptr);
      struct data res;
      if (read_all(&res, sizeof(res)) < 0 || res.type != DATA_POWER) {
         RCLCPP_WARN(this->get_logger(), "power read failed (type=%d)", res.type);
         tcflush(fd, TCIOFLUSH);
         return;
      }
      auto msg = std_msgs::msg::Float32MultiArray();
      int count = res.as.power.count;
      if (count > MAX_POWER_MONITORS) count = MAX_POWER_MONITORS;
      for (int i = 0; i < count; i++) {
         auto &pm = res.as.power.values[i];
         if (!pm.initialized) {
            for (int j = 0; j < 6; j++) msg.data.push_back(0.0f);
            continue;
         }
         msg.data.push_back(pm.voltage);
         msg.data.push_back(pm.current);
         msg.data.push_back(pm.power);
         msg.data.push_back(pm.estimated_voltage);
         msg.data.push_back(pm.estimated_current);
         msg.data.push_back(pm.estimated_power);
      }
      power_pub->publish(msg);
   }

   /* analog channel voltages — [bridge_voltage, temperature_voltage]
    * channel 0 (A13) is the strain gauge bridge voltage; cache it for poll_estimate_load(). */
   void poll_analog() {
      send_cmd(CMD_ANALOG, 0, nullptr);
      struct data res;
      if (read_all(&res, sizeof(res)) < 0 || res.type != DATA_ANALOG) {
         RCLCPP_WARN(this->get_logger(), "analog read failed (type=%d)", res.type);
         tcflush(fd, TCIOFLUSH);
         return;
      }
      auto msg = std_msgs::msg::Float32MultiArray();
      int count = res.as.analog.count;
      if (count > MAX_ANALOG_CHANNELS) count = MAX_ANALOG_CHANNELS;
      for (int i = 0; i < count; i++)
         msg.data.push_back(res.as.analog.values[i].voltage);
      analog_pub->publish(msg);
      if (count > 0) {
         bridge_voltage = res.as.analog.values[0].voltage;
         has_bridge_voltage = true;
      }
   }

   /* regolith mass estimate — requires actuator position from actuator_pos_in topic */
   void poll_mass() {
      float args[1] = { actuator_pos };
      send_cmd(CMD_REG_MASS, 1, args);
      struct data res;
      if (read_all(&res, sizeof(res)) < 0 || res.type != DATA_REG_MASS) {
         RCLCPP_WARN(this->get_logger(), "mass read failed (type=%d)", res.type);
         tcflush(fd, TCIOFLUSH);
         return;
      }
      auto msg = std_msgs::msg::Float32();
      msg.data = res.as.reg_mass.mass;
      mass_pub->publish(msg);
   }

   /* kinematic calculation — requires actuator position from actuator_pos_in topic.
    * publishes [h, h_a, h_delta, target_L] in mm. */
   void poll_kin_calc() {
      float args[1] = { actuator_pos };
      send_cmd(CMD_KIN_CALC, 1, args);
      struct data res;
      if (read_all(&res, sizeof(res)) < 0 || res.type != DATA_KIN_CALC) {
         RCLCPP_WARN(this->get_logger(), "kin_calc read failed (type=%d)", res.type);
         tcflush(fd, TCIOFLUSH);
         return;
      }
      auto msg = std_msgs::msg::Float32MultiArray();
      msg.data.push_back(res.as.kin_calc.h);
      msg.data.push_back(res.as.kin_calc.h_a);
      msg.data.push_back(res.as.kin_calc.h_delta);
      msg.data.push_back(res.as.kin_calc.target_L);
      kin_calc_pub->publish(msg);
   }

   /* bucket load estimate — requires actuator_pos and bridge voltage (A13, from poll_analog). */
   void poll_estimate_load() {
      float args[2] = { actuator_pos, bridge_voltage };
      send_cmd(CMD_ESTIMATE_LOAD, 2, args);
      struct data res;
      if (read_all(&res, sizeof(res)) < 0 || res.type != DATA_ESTIMATE_LOAD) {
         RCLCPP_WARN(this->get_logger(), "estimate_load read failed (type=%d)", res.type);
         tcflush(fd, TCIOFLUSH);
         return;
      }
      auto msg = std_msgs::msg::Float32();
      msg.data = res.as.estimate_load.estimated_load;
      estimated_load_pub->publish(msg);
   }

};

int main(int argc, char **argv) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<RegolithDriver>());
   rclcpp::shutdown();
   return 0;
}
