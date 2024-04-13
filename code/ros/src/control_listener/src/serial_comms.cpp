//SOURCE: https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include "rclcpp/rclcpp.hpp"
#include "orca_ctrl_msgs/msg/ctrl.hpp"


using std::placeholders::_1;

class SerialCommunicator : public rclcpp::Node
{
  public:
  SerialCommunicator() : Node("serial_communicator")
  {
    subscription_ = this->create_subscription<orca_ctrl_msgs::msg::Ctrl>(
      "orca_ctrl", 10, std::bind(&SerialCommunicator::topic_callback, this, _1)
    );
    //this->init_serial();
    RCLCPP_INFO(this->get_logger(), "Serial init returned: '%i'", this->init_serial());
  }

  private:


  void topic_callback(const orca_ctrl_msgs::msg::Ctrl &msg) //const
  {
    //RCLCPP_INFO(this->get_logger(), "Got message: '%s'", msg.msgtype.c_str());
    this->lastMessage = msg;
    this->construct_msg();

  }

  int init_serial()
  {
    this->serial_port = open("/dev/ttyACM0", O_RDWR);
    if(tcgetattr(serial_port, &(this->tty)) != 0)
    {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return 1;
    }

    this->tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    this->tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    this->tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    this->tty.c_cflag |= CS8; // 8 bits per byte (most common)
    this->tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    this->tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    this->tty.c_lflag &= ~ICANON;
    this->tty.c_lflag &= ~ECHO; // Disable echo
    this->tty.c_lflag &= ~ECHOE; // Disable erasure
    this->tty.c_lflag &= ~ECHONL; // Disable new-line echo
    this->tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    this->tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    this->tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    this->tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    this->tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    this->tty.c_cc[VTIME] = 10;
    this->tty.c_cc[VMIN] = 0;

    cfsetispeed(&(this->tty), B115200);
    cfsetospeed(&(this->tty), B115200);

    if(tcsetattr(this->serial_port, TCSANOW, &(this->tty)) != 0)
    {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return 1;
    }

    char msg[32] = { 'h', 'o', 'm', 'e', '\r', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    write(this->serial_port, msg, sizeof(msg));
    tcflush(this->serial_port, TCIFLUSH);

    return 0;
  }

  void construct_msg()
  {
    if(this->lastMessage.msgtype == "num")
    {
      char serialized[32];
      int headerOffset = 3; //length of header, const so as to not upset the compiler
      char header[headerOffset] = {'n','u','m'};
      memcpy(&serialized[0], header, headerOffset);
      this->float_to_bytes(&serialized[headerOffset + 0], &(this->lastMessage.pos_rail));
      this->float_to_bytes(&serialized[headerOffset + 4], &(this->lastMessage.pos_shoulder));
      this->float_to_bytes(&serialized[headerOffset + 8], &(this->lastMessage.pos_elbow));
      this->float_to_bytes(&serialized[headerOffset + 12], &(this->lastMessage.pos_wrist));
      this->float_to_bytes(&serialized[headerOffset + 16], &(this->lastMessage.pos_twist));

      float test;
      memcpy(&test, &serialized[headerOffset], 4);
      RCLCPP_INFO(this->get_logger(), "Got message: '%fl'", test);
      // RCLCPP_INFO(this->get_logger(), "Header: '%s'", header);

      write(this->serial_port, serialized, 32);
      tcflush(this->serial_port, TCIFLUSH);
    }

  }

  void float_to_bytes(char* target, float* input)
  {
    memcpy(target, input, 4);
  }

  void send_serial()
  {

  }

  int serial_port;
  struct termios tty;
  orca_ctrl_msgs::msg::Ctrl lastMessage;
  rclcpp::Subscription<orca_ctrl_msgs::msg::Ctrl>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
  // int serial_port = open("/dev/ttyACM0", O_RDWR);

  // struct termios tty;

  // if(tcgetattr(serial_port, &tty) != 0)
  // {
  //   printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  //   return 1;
  // }

  // tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  // tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  // tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
  // tty.c_cflag |= CS8; // 8 bits per byte (most common)
  // tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  // tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  // tty.c_lflag &= ~ICANON;
  // tty.c_lflag &= ~ECHO; // Disable echo
  // tty.c_lflag &= ~ECHOE; // Disable erasure
  // tty.c_lflag &= ~ECHONL; // Disable new-line echo
  // tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  // tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  // tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  // tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  // tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

  // tty.c_cc[VTIME] = 10;
  // tty.c_cc[VMIN] = 0;

  // cfsetispeed(&tty, B115200);
  // cfsetospeed(&tty, B115200);

  // if(tcsetattr(serial_port, TCSANOW, &tty) != 0)
  // {
  //   printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  //   return 1;
  // }

  // char msg[32] = { 'h', 'o', 'm', 'e', '\r', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  // write(serial_port, msg, sizeof(msg));

  // char read_buf[256];

  // memset(&read_buf, '\0', sizeof(read_buf));


  // int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

  // if(num_bytes < 0)
  // {
  //   printf("Error reading: %s\n", strerror(errno));
  //   return 1;
  // }


  // printf("Read %i bytes. Received message: %s\n\r", num_bytes, read_buf);
  // close(serial_port);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialCommunicator>());
  rclcpp::shutdown();

  return 0;

}