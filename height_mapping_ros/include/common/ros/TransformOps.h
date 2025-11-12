#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class TransformOps {
public:
  // Crea un nodo interno exclusivo para TF y lo spinea en un hilo con un executor multihilo.
  explicit TransformOps(const std::string & node_name = "transform_ops")
  : running_(false)
  {
    // Nodo interno exclusivo para TF (no compartido con tu nodo principal)
    tf_node_ = std::make_shared<rclcpp::Node>(node_name);

    // Buffer + listener (sin hilo interno de tf2; nosotros proveemos el hilo del executor)
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(tf_node_->get_clock());
    // No es estrictamente necesario con executor propio, pero es inocuo:
    tf_buffer_->setUsingDedicatedThread(true);

    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, tf_node_, /*spin_thread=*/false);

    // Executor multihilo y spin en hilo aparte
    exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    exec_->add_node(tf_node_);
    running_.store(true);

    spin_thread_ = std::thread([this]() {
      // Spin hasta que cancelemos el executor
      while (rclcpp::ok() && running_.load()) {
        exec_->spin_some();                     // evita bloquear al salir
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    });
  }

  ~TransformOps() {
    stop();
  }

  // No copiable
  TransformOps(const TransformOps&) = delete;
  TransformOps& operator=(const TransformOps&) = delete;

  // Movible
  TransformOps(TransformOps&& other) noexcept {
    move_from(std::move(other));
  }
  TransformOps& operator=(TransformOps&& other) noexcept {
    if (this != &other) {
      stop();
      move_from(std::move(other));
    }
    return *this;
  }

  /**
   * @brief Intenta obtener la transformación entre dos frames.
   * @param target_frame Frame destino (p.ej., "base_link").
   * @param source_frame Frame origen  (p.ej., "front_cam_link").
   * @param out Salida.
   * @param time Momento (por defecto: último disponible).
   * @param timeout Tiempo máx de espera (por defecto: 0.1 s).
   * @return true si tuvo éxito; false si falló.
   */
  bool lookupTransform(const std::string& target_frame,
                       const std::string& source_frame,
                       geometry_msgs::msg::TransformStamped& out,
                       const rclcpp::Time& time = rclcpp::Time(0, 0, RCL_SYSTEM_TIME),
                       const rclcpp::Duration& timeout = rclcpp::Duration::from_seconds(0.1)) const
  {
    try {
      out = tf_buffer_->lookupTransform(target_frame, source_frame, time, timeout);
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(
        tf_node_->get_logger(), *tf_node_->get_clock(), 1000,
        "TF lookup failed (%s -> %s): %s",
        source_frame.c_str(), target_frame.c_str(), ex.what());
      return false;
    }
  }

  // Multiplica transformaciones (t1 * t2) manteniendo convención clásica de cabeceras.
  static geometry_msgs::msg::TransformStamped
  multiplyTransforms(const geometry_msgs::msg::TransformStamped& t1_st,
                     const geometry_msgs::msg::TransformStamped& t2_st)
  {
    tf2::Transform t1, t2;
    tf2::fromMsg(t1_st.transform, t1);
    tf2::fromMsg(t2_st.transform, t2);

    tf2::Transform t_mult = t1 * t2;

    geometry_msgs::msg::TransformStamped out;
    out.transform     = tf2::toMsg(t_mult);
    out.header.frame_id = t2_st.header.frame_id;   // convención ROS1/ROS2 habitual
    out.header.stamp    = t2_st.header.stamp;
    out.child_frame_id  = t1_st.child_frame_id;
    return out;
  }

  // Acceso al logger del nodo TF (útil para debug)
  rclcpp::Logger get_logger() const { return tf_node_->get_logger(); }

  // Parar explícitamente (también se llama en el destructor)
  void stop() {
    if (running_.exchange(false)) {
      if (exec_) {
        exec_->cancel();
      }
      if (spin_thread_.joinable()) {
        spin_thread_.join();
      }
      if (exec_ && tf_node_) {
        exec_->remove_node(tf_node_);
      }
      exec_.reset();
      tf_listener_.reset();
      tf_buffer_.reset();
      tf_node_.reset();
    }
  }

private:
  void move_from(TransformOps&& other) {
    tf_node_     = std::move(other.tf_node_);
    tf_buffer_   = std::move(other.tf_buffer_);
    tf_listener_ = std::move(other.tf_listener_);
    exec_        = std::move(other.exec_);
    running_.store(other.running_.load());
    other.running_.store(false);
    spin_thread_ = std::move(other.spin_thread_);
  }

  // Nodo y TF
  rclcpp::Node::SharedPtr tf_node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // Ejecutor/hilo
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
  std::thread spin_thread_;
  std::atomic_bool running_;
};
