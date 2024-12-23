// Copyright (c) 2024 ibis-ssl
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>
#include <chrono>
#include <consai_visualizer_msgs/conversions.hpp>
#include <consai_visualizer_msgs/msg/objects_array.hpp>
#include <memory>
#include <proto2ros/conversions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>

class WebSocketClient
{
public:
  explicit WebSocketClient(const std::string & uri) : uri_(uri), stop_flag_(false)
  {
    parseUri(uri);
    startConnection();
    sender_thread_ = std::thread([this]() { processQueue(); });
  }

  ~WebSocketClient()
  {
    stop_flag_ = true;
    queue_cond_var_.notify_all();

    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
    if (sender_thread_.joinable()) {
      sender_thread_.join();
    }
  }

  void send(const std::string & message)
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    message_queue_.push(message);
    queue_cond_var_.notify_one();
  }

  bool isConnected() const
  {
    std::lock_guard<std::mutex> lock(connection_mutex_);
    return is_connected_;
  }

private:
  void parseUri(const std::string & uri)
  {
    auto pos = uri.find("://");
    if (pos == std::string::npos) {
      throw std::invalid_argument("Invalid URI format");
    }
    protocol_ = uri.substr(0, pos);
    auto host_and_port = uri.substr(pos + 3);
    auto port_pos = host_and_port.find(':');
    if (port_pos == std::string::npos) {
      host_ = host_and_port;
      port_ = "80";  // Default port
    } else {
      host_ = host_and_port.substr(0, port_pos);
      port_ = host_and_port.substr(port_pos + 1);
    }
    std::cout << "Parsed URI: " << protocol_ << "://" << host_ << ":" << port_ << std::endl;
  }

  void createNewConnection()
  {
    std::cout << "Creating new WebSocket connection to " << host_ << ":" << port_ << std::endl;

    auto ioc = std::make_shared<boost::asio::io_context>();
    auto resolver = boost::asio::ip::tcp::resolver(*ioc);
    auto ws = std::make_unique<boost::beast::websocket::stream<boost::asio::ip::tcp::socket>>(*ioc);

    try {
      // Set these options before the handshake
      ws->set_option(boost::beast::websocket::stream_base::decorator(
        [](boost::beast::websocket::request_type & req) {
          req.set(
            boost::beast::http::field::user_agent,
            std::string(BOOST_BEAST_VERSION_STRING) + " websocket-client-coro");
        }));

      std::cout << "Resolving host..." << std::endl;
      auto const results = resolver.resolve(host_, port_);

      std::cout << "Connecting to endpoint..." << std::endl;
      boost::asio::connect(ws->next_layer(), results.begin(), results.end());

      std::cout << "Performing WebSocket handshake..." << std::endl;
      // Enable binary data
      ws->binary(true);
      // Disable compression
      ws->set_option(
        boost::beast::websocket::stream_base::timeout::suggested(boost::beast::role_type::client));

      ws->handshake(host_, "/");

      std::cout << "WebSocket connection established successfully" << std::endl;

      {
        std::lock_guard<std::mutex> lock(connection_mutex_);
        io_context_ = std::move(ioc);
        socket_ = std::move(ws);
        is_connected_ = true;
        connection_error_.clear();
      }
    } catch (const std::exception & e) {
      std::cout << "Connection failed with error: " << e.what() << std::endl;
      std::lock_guard<std::mutex> lock(connection_mutex_);
      is_connected_ = false;
      connection_error_ = e.what();
      throw;
    }
  }

  void startConnection()
  {
    worker_thread_ = std::thread([this]() {
      while (!stop_flag_) {
        try {
          createNewConnection();
          io_context_->run();
          break;  // 正常に接続された場合はループを抜ける
        } catch (const std::exception & e) {
          std::cout << "Connection failed: " << e.what() << std::endl;
          // 接続に失敗した場合、3秒待って再試行
          std::this_thread::sleep_for(std::chrono::seconds(3));
        }
      }
    });
  }

  void processQueue()
  {
    while (!stop_flag_) {
      std::string message;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cond_var_.wait(lock, [this]() { return stop_flag_ || !message_queue_.empty(); });

        if (stop_flag_) break;

        message = std::move(message_queue_.front());
        message_queue_.pop();
        std::cout << "Processing message of size: " << message.size() << " bytes" << std::endl;
      }

      bool sent = false;
      while (!stop_flag_ && !sent) {
        std::lock_guard<std::mutex> lock(connection_mutex_);
        if (is_connected_ && socket_) {
          try {
            std::cout << "Attempting to send message..." << std::endl;
            socket_->write(boost::asio::buffer(message));
            std::cout << "Message sent successfully" << std::endl;
            sent = true;
          } catch (const std::exception & e) {
            std::cout << "Send failed with error: " << e.what() << std::endl;
            connection_error_ = e.what();
            is_connected_ = false;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            try {
              std::cout << "Attempting to reconnect..." << std::endl;
              createNewConnection();
            } catch (const std::exception & e) {
              std::cout << "Reconnection failed: " << e.what() << std::endl;
            }
          }
        } else {
          std::cout << "Not connected, waiting..." << std::endl;
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      }
    }
  }

  std::string uri_;
  std::string host_;
  std::string port_;
  std::string protocol_;

  std::shared_ptr<boost::asio::io_context> io_context_;
  std::unique_ptr<boost::beast::websocket::stream<boost::asio::ip::tcp::socket>> socket_;

  bool is_connected_ = false;
  bool stop_flag_ = false;
  std::string connection_error_;

  mutable std::mutex connection_mutex_;

  std::thread worker_thread_;
  std::thread sender_thread_;

  std::queue<std::string> message_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cond_var_;
};

class Bridge : public rclcpp::Node
{
public:
  Bridge() : Node("bridge")
  {
    std::string websocket_url =
      this->declare_parameter<std::string>("websocket_url", "ws://localhost:9000");
    websocket_client = std::make_shared<WebSocketClient>(websocket_url);

    sub_objects_array = create_subscription<consai_visualizer_msgs::msg::ObjectsArray>(
      "/visualizer_objects", rclcpp::SensorDataQoS(),
      [&](const consai_visualizer_msgs::msg::ObjectsArray & ros_msg) {
        // std::cout << "Received ObjectsArray: " << static_cast<int>(ros_msg.objects.size())
        //           << std::endl;
        visualizer::ObjectsArray proto_msg;
        consai_visualizer_msgs::conversions::Convert(ros_msg, &proto_msg);
        std::string buffer;
        proto_msg.SerializeToString(&buffer);
        websocket_client->send(buffer);
      });
  }

private:
  std::shared_ptr<WebSocketClient> websocket_client;
  rclcpp::Subscription<consai_visualizer_msgs::msg::ObjectsArray>::SharedPtr sub_objects_array;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Bridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
