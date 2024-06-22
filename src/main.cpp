#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include <gciSensors.hpp>
#include <squaternion.hpp>
#include <quaternion_filters.hpp>

using namespace std::chrono_literals;  // cpp_std s,ms,etc

// constexpr int SENSOR_AGM_RATE_HZ = 1;
// constexpr int SENSOR_PT_RATE_HZ = 1;
constexpr std::string FRAME_ID = "imu";

using namespace LIS3MDL;
using namespace LSM6DSOX;
using namespace BMP390;
using namespace gci::sensors;


namespace rtf_imu {

class rtfImu : public rclcpp::Node {
public:
  rtfImu(): Node("rtf_imu"), qcf(0.1) {
    RCLCPP_INFO(this->get_logger(), "rtf_imu: CTRL+C to exit!");

    // Setup sensors --------------------------------------
    int err = mag.init(RANGE_4GAUSS,ODR_155HZ);
    if (err == 0) mag_ok = true;
    else RCLCPP_ERROR(this->get_logger(), "LIS3MDL failed");

    err = imu.init(ACCEL_RANGE_4_G, GYRO_RANGE_2000_DPS, RATE_208_HZ);
    if (err == 0) imu_ok = true;
    else RCLCPP_ERROR(this->get_logger(), "LSM6DSOX failed");

    err = bmp.init(ODR_100_HZ, IIR_FILTER_COEFF_127);
    if (err == 0) bmp_ok = true;
    else RCLCPP_ERROR(this->get_logger(), "BMP390 failed");

    // Setup publishers -----------------------------------
    pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    pub_mag = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", 10);
    pub_press = this->create_publisher<sensor_msgs::msg::FluidPressure>("press", 10);
    pub_temp = this->create_publisher<sensor_msgs::msg::Temperature>("temp", 10);

    // Setup timers for publishers ------------------------
    auto interval = std::chrono::milliseconds(1000/pub_rate_agm);
    timer_agm = this->create_wall_timer(interval, std::bind(&rtfImu::cb_agm, this));
    interval = std::chrono::milliseconds(1000/pub_rate_pt);
    timer_pt = this->create_wall_timer(interval, std::bind(&rtfImu::cb_pt, this));
  }

  ~rtfImu() {
    RCLCPP_INFO(this->get_logger(), "Bye rtf_imu!");
  }

  // void error_init(int err, std::string s) {
  //   if (err == 0)
  //   else RCLCPP_ERROR(this->get_logger(), "LSM6DSOX failed");
  // }

  void cb_pt() {
    auto press_msg = sensor_msgs::msg::FluidPressure();
    press_msg.header.stamp = this->now();
    press_msg.header.frame_id = FRAME_ID;

    auto temp_msg = sensor_msgs::msg::Temperature();
    temp_msg.header.stamp = press_msg.header.stamp;
    temp_msg.header.frame_id = FRAME_ID;

    if (bmp_ok) {
      bmp390_t pt = bmp.read();
      if (pt.ok == false) bmp_ok = false;
      else {
        // Absolute pressure reading in Pascals
        press_msg.fluid_pressure = pt.press;
        press_msg.variance = 0;

        // Temperature in Degrees Celsius
        temp_msg.temperature = pt.temp;
        temp_msg.variance = 0;

        // RCLCPP_INFO(this->get_logger(), "Publishing temperature");
        pub_temp->publish(temp_msg);
        // RCLCPP_INFO(this->get_logger(), "Publishing pressure");
        pub_press->publish(press_msg);
      }
    }

  }

  void cb_agm() {
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = FRAME_ID;

    auto mag_msg = sensor_msgs::msg::MagneticField();
    mag_msg.header.stamp = imu_msg.header.stamp;
    mag_msg.header.frame_id = FRAME_ID;

    if (mag_ok) {
      const lis3mdl_t m = mag.read_cal();
      if (m.ok == false) mag_ok = false;
      else {
        // field vector in Tesla
        mag_msg.magnetic_field.x = m.x;
        mag_msg.magnetic_field.y = m.y;
        mag_msg.magnetic_field.z = m.z;
        mag_msg.magnetic_field_covariance[0] = -1;

        // RCLCPP_INFO(this->get_logger(), "Publishing mag");
        pub_mag->publish(mag_msg);
      }
    }

    if (imu_ok) {
      lsm6dsox_t m = imu.read_cal();
      if (m.ok == false) imu_ok = false;
      else {
        // filter data and calculate quaternion
        vect_t<double> a;
        a.x = m.a.x;
        a.y = m.a.y;
        a.z = m.a.z;
        vect_t<double> w;
        w.x = m.g.x;
        w.y = m.g.y;
        w.z = m.g.z;
        qcf.update(a,w,0.01);
        q = qcf.q;

        // generate message and publish
        imu_msg.orientation.w = q.w;
        imu_msg.orientation.x = q.x;
        imu_msg.orientation.y = q.y;
        imu_msg.orientation.z = q.z;
        imu_msg.orientation_covariance[0] = -1;

        imu_msg.linear_acceleration.x = m.a.x;
        imu_msg.linear_acceleration.y = m.a.y;
        imu_msg.linear_acceleration.z = m.a.z;
        imu_msg.linear_acceleration_covariance[0] = -1;

        imu_msg.angular_velocity.x = m.g.x;
        imu_msg.angular_velocity.y = m.g.y;
        imu_msg.angular_velocity.z = m.g.z;
        imu_msg.angular_velocity_covariance[0] = -1;

        // RCLCPP_INFO(this->get_logger(), "Publishing accel/gyro");
        pub_imu->publish(imu_msg);
      }
    }
  }

  void calc_quaternion() {

  }

  // sensors
  gciLSM6DSOX imu;
  gciLIS3MDL mag;
  gciBMP390 bmp;

  // sensor status
  bool imu_ok{false}, mag_ok{false}, bmp_ok{false};

  // callback timers
  int pub_rate_agm{100}; // Hz
  int pub_rate_pt{10};  // Hz
  rclcpp::TimerBase::SharedPtr timer_agm; // 100Hz
  rclcpp::TimerBase::SharedPtr timer_pt;  // 25Hz

  // publishers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pub_press;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temp;

  // squaternions
  QuaternionT<double> q;
  QCF<double> qcf;
};

} // namespace rtfImu


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rtf_imu::rtfImu>());
  rclcpp::shutdown();
  return 0;
}