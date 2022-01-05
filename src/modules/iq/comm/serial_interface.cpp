#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>

#include "serial_interface.hpp"
#include "crc_helper.h"
#include "string.h" // for memcpy
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
// #include <algorithm>

int min(int x, int y)
{
  return (x < y) ? x : y;
}

SerialInterface::SerialInterface(const char* dev)
{
  InitBQ(&index_queue, pf_index_data, SERIAL_PF_INDEX_DATA_SIZE);
  InitPacketFinder(&pf, &index_queue);
  tx_bipbuf = BipBuffer(tx_buffer, SERIAL_TX_BUFFER_SIZE);

  // creates a usb flag
  dev_fd_ = open(dev, O_RDWR | O_NOCTTY | O_SYNC);

  //if can't open the port
  if (dev_fd_ == -1)
  {
    perror("open_port: Unable to open port");
    return;
  }
  else
  {
    //set file status flag
    fcntl(dev_fd_, F_SETFL, 0);
  }

  //Configure Port
  struct termios tty;
  memset (&tty, 0, sizeof tty);

  if ( tcgetattr ( dev_fd_, &tty ) != 0 )
  {
    perror("from tcgetattr");
  }

  //Set Baud Rate
  // cfsetospeed (&tty, B115200);
  cfsetispeed (&tty, B115200);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK;         // disable break processing
  tty.c_lflag = 0;                // no signaling chars, no echo,
  tty.c_iflag &= ~INLCR;
  tty.c_iflag &= ~ICRNL;
  tty.c_oflag &= ~OCRNL;
  tty.c_oflag &= ~ONLCR;
  // no canonical processing
  tty.c_oflag = 0;                // no remapping, no delays
  tty.c_cc[VMIN]  = 0;            // read doesn't block
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag |= 0;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  cfmakeraw(&tty);

  if ( tcsetattr ( dev_fd_, TCSANOW, &tty ) != 0)
  {
     perror("from tcsetattr \n");
  }
}

// int8_t SerialInterface::GetBytes()
// {
//   int num_bytes_read_queue, result;

//   uint8_t read_buffer[SERIAL_TX_BUFFER_SIZE];
//   // PX4_INFO("before read");
//   result = read(dev_fd_, read_buffer, SERIAL_TX_BUFFER_SIZE);
//   // PX4_INFO("after read");

//   //debugging error message if read() fails
//   if (result==-1)
//   {
//     perror("Serial read error");
//   }

//   // Puts the recently read bytes into com's receive queue
//   SerialInterface::SetRxBytes(read_buffer, result);

//   return 0;
// }

int8_t SerialInterface::GetBytes()
{
  // int num_bytes_read_queue, result;
  int result;

  fcntl(dev_fd_, F_SETFL, FNDELAY);

  //creates a buffer of the rigth size to read the incoming message
  uint8_t read_buffer[SERIAL_TX_BUFFER_SIZE];

  // reads the bytes comming in from the serial port and stores them in read_buffer
  // result = read(dev_fd_, read_buffer,min(num_bytes_read_queue,SERIAL_TX_BUFFER_SIZE));
  result = read(dev_fd_, read_buffer,SERIAL_TX_BUFFER_SIZE);

  // Puts the recently read bytes into com's receive queue
  SerialInterface::SetRxBytes(read_buffer, result);
  fcntl(dev_fd_, F_SETFL, 0);

  return 0;
}

int8_t SerialInterface::SetRxBytes(uint8_t* data_in, uint16_t length_in)
{
  if(data_in == nullptr)
    return -1;

  if(length_in)
  {
    //copy it over
    PutBytes(&pf, data_in, length_in);
    return 1;
  }
  else
    return 0;
}

int8_t SerialInterface::PeekPacket(uint8_t **packet, uint8_t *length)
{
  return(::PeekPacket(&pf, packet, length));
}

int8_t SerialInterface::DropPacket()
{
  return(::DropPacket(&pf));
}

int8_t SerialInterface::SendPacket(uint8_t msg_type, uint8_t *data, uint16_t length)
{
  // This function must not be interrupted by another call to SendBytes or
  // SendPacket, or else the packet it builds will be spliced/corrupted.

  uint8_t header[3];
  header[0] = kStartByte;                   // const defined by packet_finder.c
  header[1] = length;
  header[2] = msg_type;
  SendBytes(header, 3);

  SendBytes(data, length);

  uint8_t footer[2];
  uint16_t crc;
  crc = MakeCrc(&(header[1]), 2);
  crc = ArrayUpdateCrc(crc, data, length);
  footer[0] = crc & 0x00FF;
  footer[1] = crc >> 8;
  SendBytes(footer, 2);

  return(1);
}

int8_t SerialInterface::SendBytes(uint8_t *bytes, uint16_t length)
{
  uint16_t length_temp = 0;
  uint8_t* location_temp;
  int8_t ret = 0;

  // Reserve space in the buffer
  location_temp = tx_bipbuf.Reserve(length, length_temp);

  // If there's room, do the copy
  if(length == length_temp)
  {
    memcpy(location_temp, bytes, length_temp);   // do copy
    tx_bipbuf.Commit(length_temp);
    ret = 1;
  }
  else
  {
    tx_bipbuf.Commit(0); // Call the restaurant, cancel the reservations
  }

  return ret;
}

int8_t SerialInterface::GetTxBytes(uint8_t* data_out, uint8_t& length_out)
{
  uint16_t length_temp;
  uint8_t* location_temp;

  location_temp = tx_bipbuf.GetContiguousBlock(length_temp);
  if(length_temp)
  {
    memcpy(data_out, location_temp, length_temp);
    length_out = length_temp;
    tx_bipbuf.DecommitBlock(length_temp);

    location_temp = tx_bipbuf.GetContiguousBlock(length_temp);
    memcpy(&data_out[length_out], location_temp, length_temp);
    length_out = length_out + length_temp;
    tx_bipbuf.DecommitBlock(length_temp);
    return 1;
  }
  return 0;
}

int SerialInterface::SendNow()
{
    //local variables
    uint8_t write_communication_buffer[256];
    uint8_t write_communication_length;

    // Grab outbound messages in the com queue, store into buffer
    if(SerialInterface::GetTxBytes(write_communication_buffer,write_communication_length))
    {
        //converts write_communication_length to an integer
        int write_communication_length_int = static_cast<int>(write_communication_length);

        //sends buffer over serial port
        uint8_t written = write(dev_fd_, write_communication_buffer, write_communication_length_int);
        if(written != write_communication_length_int)
            return -1;
        return 0;
    }
    return 2;
}

int SerialInterface::get_fd()
{
    return dev_fd_;
}

// void ReceiveMessages()
// {
  // // This buffer is for passing around messages.
  // uint8_t communication_buffer[64];
  // // Stores length of message to send or receive
  // uint8_t communication_length;

  // // Reads however many bytes are currently available
  // communication_length = Serial1.readBytes(communication_buffer, Serial1.available());

  // // Puts the recently read bytes into coms receive queue
  // com.SetRxBytes(communication_buffer,communication_length);

  // uint8_t *rx_data; // temporary pointer to received type+data bytes
  // uint8_t rx_length; // number of received type+data bytes

  // // while we have message packets to parse
  // while(com.PeekPacket(&rx_data,&rx_length))
  // {
    // // Share that packet with all client objects
    // mot.ReadMsg(com,rx_data,rx_length);
    // // Once were done with the message packet, drop it
    // com.DropPacket();
  // }
// }


void SerialInterface::ReadMsg(CommunicationInterface& com, uint8_t* data, uint8_t length)
{
  // I currently don't support being talked to

}

void SerialInterface::Flush()
{
  uint8_t *rx_data;// temporary pointer to received type+data bytes
  uint8_t rx_length = 1; // number of received type+data bytes

  while(rx_length > 0)
  {
    SerialInterface::GetBytes();
    rx_length = 0;
    while(PeekPacket(&rx_data,&rx_length))
    {
      DropPacket();
    }
  }
}

SerialInterface::~SerialInterface()
{
  close(dev_fd_);
}

