#include "crsf_receiver.h"
#include "utils.h"

#include "baudrate_helper.h"


CrsfReceiverNode::CrsfReceiverNode(): Node("crsf_reader_node")
{
    this->declare_parameter("device", "/dev/ttyUSB0");
    this->declare_parameter("baudrate", 460800);
    this->declare_parameter("link_stats", true);
    this->declare_parameter("receiver_rate", 100);

    channels_publisher = this->create_publisher<crsf_receiver_msg::msg::CRSFChannels16>(
        "rc/channels", 
        rclcpp::QoS(1).best_effort().durability_volatile()
    );

    link_publisher = this->create_publisher<crsf_receiver_msg::msg::CRSFLinkInfo>(
        "rc/link", 
        rclcpp::QoS(1).best_effort().durability_volatile()
    );

    device = this->get_parameter("device").as_string();
    int baudrate = this->get_parameter("baudrate").as_int();
    int rate = this->get_parameter("receiver_rate").as_int();
    int period = 1000 / rate;

    RCLCPP_INFO(this->get_logger(), "Receiver rate is %dhz (period %dms)", rate, period);
    RCLCPP_INFO(this->get_logger(), "Target serial device is: %s", device.c_str());
    RCLCPP_INFO(this->get_logger(), "Selected baudrate: %d", baudrate);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period), 
        std::bind(&CrsfReceiverNode::main_timer_callback, this)
    );

    serial_io_context = std::make_unique<IoContext>(2);
    serial_driver = std::make_unique<SerialDriver>(*serial_io_context);

    SerialPortConfig config(115200, FlowControl::NONE, Parity::NONE, StopBits::ONE); // Config with default ASIO baudrate


    try {
        serial_driver->init_port(device, config);
        serial_driver->port()->open();

        // Redefine to target baudrate
        if (set_custom_baudrate(device.c_str(), baudrate) == false) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set custom baudrate %d", baudrate);
        }

        serial_driver->port()->async_receive(
            std::bind(&CrsfReceiverNode::receive_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open serial port %s: %s", device.c_str(), e.what());
    }

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period),
        std::bind(&CrsfReceiverNode::main_timer_callback, this)
    );
}

void CrsfReceiverNode::receive_callback(
    const std::vector<uint8_t> & buffer,
    const size_t & bytes_transferred)
{
    std::copy_n(buffer.begin(), bytes_transferred, back_inserter(parser.rx_buffer));
    parser.parse_incoming_bytes();
}

void CrsfReceiverNode::main_timer_callback()
{
    if (parser.is_channels_actual()) {
        CRSFChannels16 message = convert_to_channels_message(parser.get_channels_values());
        channels_publisher->publish(message);
    }

    if (parser.is_link_statistics_actual()) {
        CRSFLinkInfo message = convert_to_link_info(parser.get_link_info());
        link_publisher->publish(message);
    }
}

CrsfReceiverNode::~CrsfReceiverNode() {
    RCLCPP_INFO(this->get_logger(), "Node destroyed");
}
