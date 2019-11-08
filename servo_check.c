/******************************************************************************
 * Copyright 2017 Geoffrey Biggs
 *
 * Simple program to test reading from and writing to a Dynamixel servo. Useful
 * to confirm that the servo is working.
 *
 * Based on read_write.c example from ROBOTIS CO., LTD.
 * https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/
 *   249cda48571ea62435c132ab20bc9dd1f1fc93a8/c/example/protocol1.0/read_write/read_write.c
 *****************************************************************************/

#include "dynamixel_sdk.h"

#include <stdlib.h>

#define DEVICENAME "/dev/ttyUSB0"
#define BAUDRATE 1000000
#define PROTOCOL_VERSION 1.0
#define ADDR_MX_TORQUE_ENABLE 24
#define ADDR_MX_GOAL_POSITION 30
#define ADDR_MX_PRESENT_POSITION 36
#define DXL_MOVING_STATUS_THRESHOLD 10
#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0

int main(int argc, char **argv) {
  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;
  uint16_t dxl_present_position = 0;

  int port_num = portHandler(DEVICENAME);

  packetHandler();

  if (argc < 3) {
    fprintf(stderr, "Usage: %s <servo number> <goal_posirion>", argv[0]);
    return 1;
  }
  int dxl_id = strtol(argv[1], 0, 10);
  int dxl_goal_position = strtol(argv[2], 0, 10);
  fprintf(stderr, "servo id:%d, goal position:%d\n", dxl_id, dxl_goal_position);

  if (openPort(port_num)) {
    fprintf(stderr, "Opened port\n");
  } else {
    fprintf(stderr, "Failed to open the port\n");
    return 1;
  }

  if (setBaudRate(port_num, BAUDRATE)) {
    fprintf(stderr, "Changed buadrate\n");
  } else {
    fprintf(stderr, "Failed to change the baudrate\n");
    return 1;
  }

  // Enable torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS) {
    fprintf(stderr, "%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
  } else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0) {
    fprintf(stderr, "%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
  } else {
    fprintf(stderr, "Dynamixel has been successfully connected\n");
  }

  // Write goal position
  write2ByteTxRx(port_num, PROTOCOL_VERSION, dxl_id, ADDR_MX_GOAL_POSITION, dxl_goal_position);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS) {
    fprintf(stderr, "%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
  } else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0) {
    fprintf(stderr, "%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
  }

  do
  {
    // Read present position
    dxl_present_position = read2ByteTxRx(port_num, PROTOCOL_VERSION, dxl_id, ADDR_MX_PRESENT_POSITION);
    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS) {
      fprintf(stderr, "%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    } else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0) {
      fprintf(stderr, "%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }

    printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", dxl_id, dxl_goal_position, dxl_present_position);

  } while ((abs(dxl_goal_position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

  // Disable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    fprintf(stderr, "%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    fprintf(stderr, "%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
  }

  // Close port
  closePort(port_num);

  return 0;
}
