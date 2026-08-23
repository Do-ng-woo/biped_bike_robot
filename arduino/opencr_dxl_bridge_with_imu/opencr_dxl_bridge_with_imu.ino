#include <Arduino.h>
#include <IMU.h>

// OpenCR USB-to-Dynamixel TTL bridge with a virtual OpenCR IMU sensor.
//
// Normal packets are forwarded:
//   PC USB Serial <-> OpenCR Serial3 <-> Dynamixel TTL bus
//
// Read/Ping packets addressed to VIRTUAL_IMU_ID are answered by OpenCR itself.
// This lets the PC read the built-in IMU through the same Dynamixel SDK port
// while the robot is moving.

#define USB_PORT Serial
#define DXL_PORT Serial3

#define DEFAULT_DXL_BAUD 1000000
#define BUFFER_LENGTH 1024
#define USB_PACKET_BUFFER_LENGTH 512

#define VIRTUAL_IMU_ID 200
#define VIRTUAL_TABLE_SIZE 180
#define ADDR_IMU_BLOCK 100
#define LEN_IMU_BLOCK 68

#define INST_PING 0x01
#define INST_READ 0x02
#define INST_STATUS 0x55

#define ERR_NONE 0x00
#define ERR_INSTRUCTION 0x02
#define ERR_DATA_RANGE 0x04

#define LED_USB_TO_DXL BDPIN_LED_USER_1
#define LED_DXL_TO_USB BDPIN_LED_USER_2

static uint8_t bridge_buffer[BUFFER_LENGTH];
static uint8_t usb_packet_buffer[USB_PACKET_BUFFER_LENGTH];
static size_t usb_packet_length = 0;
static uint8_t virtual_table[VIRTUAL_TABLE_SIZE];
static uint32_t current_dxl_baud = DEFAULT_DXL_BAUD;

static uint8_t usb_to_dxl_led_count = 0;
static uint8_t dxl_to_usb_led_count = 0;
static uint32_t last_usb_to_dxl_led_ms = 0;
static uint32_t last_dxl_to_usb_led_ms = 0;

static cIMU imu;

static void setDxlPower(bool enabled)
{
  digitalWrite(BDPIN_DXL_PWR_EN, enabled ? HIGH : LOW);
}

static void setDxlTx(bool enabled)
{
  drv_dxl_tx_enable(enabled ? 1 : 0);
}

static uint16_t updateCrc(uint16_t crc_accum, const uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
  for (uint16_t j = 0; j < data_blk_size; ++j) {
    crc_accum ^= static_cast<uint16_t>(data_blk_ptr[j]) << 8;
    for (uint8_t i = 0; i < 8; ++i) {
      if (crc_accum & 0x8000) {
        crc_accum = (crc_accum << 1) ^ 0x8005;
      } else {
        crc_accum <<= 1;
      }
    }
  }
  return crc_accum;
}

static bool hasValidHeader(const uint8_t *packet)
{
  return packet[0] == 0xFF && packet[1] == 0xFF && packet[2] == 0xFD && packet[3] == 0x00;
}

static bool hasValidCrc(const uint8_t *packet, size_t packet_length)
{
  if (packet_length < 10) {
    return false;
  }

  uint16_t expected = updateCrc(0, packet, packet_length - 2);
  uint16_t actual = packet[packet_length - 2] | (static_cast<uint16_t>(packet[packet_length - 1]) << 8);
  return expected == actual;
}

static void putU16(uint16_t address, uint16_t value)
{
  if (address + 1 >= VIRTUAL_TABLE_SIZE) {
    return;
  }
  virtual_table[address] = value & 0xFF;
  virtual_table[address + 1] = (value >> 8) & 0xFF;
}

static void putU32(uint16_t address, uint32_t value)
{
  if (address + 3 >= VIRTUAL_TABLE_SIZE) {
    return;
  }
  virtual_table[address] = value & 0xFF;
  virtual_table[address + 1] = (value >> 8) & 0xFF;
  virtual_table[address + 2] = (value >> 16) & 0xFF;
  virtual_table[address + 3] = (value >> 24) & 0xFF;
}

static void putI16(uint16_t address, int16_t value)
{
  putU16(address, static_cast<uint16_t>(value));
}

static void putFloat(uint16_t address, float value)
{
  if (address + 3 >= VIRTUAL_TABLE_SIZE) {
    return;
  }
  memcpy(&virtual_table[address], &value, sizeof(float));
}

static void updateVirtualTable()
{
  memset(virtual_table, 0, sizeof(virtual_table));

  putU16(0, 2000);
  virtual_table[6] = 1;
  virtual_table[7] = VIRTUAL_IMU_ID;
  virtual_table[8] = 3;
  virtual_table[13] = 2;
  virtual_table[68] = 2;

  putU32(ADDR_IMU_BLOCK + 0, millis());
  putFloat(ADDR_IMU_BLOCK + 4, imu.quat[0]);
  putFloat(ADDR_IMU_BLOCK + 8, imu.quat[1]);
  putFloat(ADDR_IMU_BLOCK + 12, imu.quat[2]);
  putFloat(ADDR_IMU_BLOCK + 16, imu.quat[3]);

  putFloat(ADDR_IMU_BLOCK + 20, imu.rpy[0]);
  putFloat(ADDR_IMU_BLOCK + 24, imu.rpy[1]);
  putFloat(ADDR_IMU_BLOCK + 28, imu.rpy[2]);

  putFloat(ADDR_IMU_BLOCK + 32, imu.gx);
  putFloat(ADDR_IMU_BLOCK + 36, imu.gy);
  putFloat(ADDR_IMU_BLOCK + 40, imu.gz);

  putFloat(ADDR_IMU_BLOCK + 44, imu.ax);
  putFloat(ADDR_IMU_BLOCK + 48, imu.ay);
  putFloat(ADDR_IMU_BLOCK + 52, imu.az);

  putI16(ADDR_IMU_BLOCK + 56, imu.gyroData[0]);
  putI16(ADDR_IMU_BLOCK + 58, imu.gyroData[1]);
  putI16(ADDR_IMU_BLOCK + 60, imu.gyroData[2]);
  putI16(ADDR_IMU_BLOCK + 62, imu.accData[0]);
  putI16(ADDR_IMU_BLOCK + 64, imu.accData[1]);
  putI16(ADDR_IMU_BLOCK + 66, imu.accData[2]);
}

static void sendStatusPacket(uint8_t error, const uint8_t *params, uint16_t param_length)
{
  uint16_t status_length = param_length + 4;
  uint16_t total_length = param_length + 11;

  if (total_length > BUFFER_LENGTH) {
    return;
  }

  bridge_buffer[0] = 0xFF;
  bridge_buffer[1] = 0xFF;
  bridge_buffer[2] = 0xFD;
  bridge_buffer[3] = 0x00;
  bridge_buffer[4] = VIRTUAL_IMU_ID;
  bridge_buffer[5] = status_length & 0xFF;
  bridge_buffer[6] = (status_length >> 8) & 0xFF;
  bridge_buffer[7] = INST_STATUS;
  bridge_buffer[8] = error;

  if (param_length > 0 && params != nullptr) {
    memcpy(&bridge_buffer[9], params, param_length);
  }

  uint16_t crc = updateCrc(0, bridge_buffer, total_length - 2);
  bridge_buffer[total_length - 2] = crc & 0xFF;
  bridge_buffer[total_length - 1] = (crc >> 8) & 0xFF;

  USB_PORT.write(bridge_buffer, total_length);
  USB_PORT.flush();
  dxl_to_usb_led_count = 4;
}

static void handleVirtualPacket(const uint8_t *packet, size_t packet_length)
{
  if (!hasValidCrc(packet, packet_length)) {
    return;
  }

  uint8_t instruction = packet[7];

  if (instruction == INST_PING) {
    uint8_t params[3] = {
      static_cast<uint8_t>(virtual_table[0]),
      static_cast<uint8_t>(virtual_table[1]),
      static_cast<uint8_t>(virtual_table[6]),
    };
    sendStatusPacket(ERR_NONE, params, sizeof(params));
    return;
  }

  if (instruction != INST_READ) {
    sendStatusPacket(ERR_INSTRUCTION, nullptr, 0);
    return;
  }

  uint16_t address = packet[8] | (static_cast<uint16_t>(packet[9]) << 8);
  uint16_t read_length = packet[10] | (static_cast<uint16_t>(packet[11]) << 8);
  if (address + read_length > VIRTUAL_TABLE_SIZE || read_length > 200) {
    sendStatusPacket(ERR_DATA_RANGE, nullptr, 0);
    return;
  }

  updateVirtualTable();
  sendStatusPacket(ERR_NONE, &virtual_table[address], read_length);
}

static void forwardPacketToDxl(const uint8_t *packet, size_t packet_length)
{
  setDxlTx(true);
  DXL_PORT.write(packet, packet_length);
  DXL_PORT.flush();
  setDxlTx(false);
  usb_to_dxl_led_count = 4;
}

static void processUsbPacketBuffer()
{
  while (usb_packet_length >= 7) {
    if (!hasValidHeader(usb_packet_buffer)) {
      memmove(usb_packet_buffer, usb_packet_buffer + 1, usb_packet_length - 1);
      --usb_packet_length;
      continue;
    }

    uint16_t packet_length_field = usb_packet_buffer[5] | (static_cast<uint16_t>(usb_packet_buffer[6]) << 8);
    size_t full_packet_length = static_cast<size_t>(packet_length_field) + 7;
    if (full_packet_length > USB_PACKET_BUFFER_LENGTH) {
      usb_packet_length = 0;
      return;
    }
    if (usb_packet_length < full_packet_length) {
      return;
    }

    if (usb_packet_buffer[4] == VIRTUAL_IMU_ID) {
      handleVirtualPacket(usb_packet_buffer, full_packet_length);
    } else {
      forwardPacketToDxl(usb_packet_buffer, full_packet_length);
    }

    size_t remaining = usb_packet_length - full_packet_length;
    if (remaining > 0) {
      memmove(usb_packet_buffer, usb_packet_buffer + full_packet_length, remaining);
    }
    usb_packet_length = remaining;
  }
}

static void updateDxlBridge()
{
  while (USB_PORT.available() > 0) {
    int value = USB_PORT.read();
    if (value < 0) {
      break;
    }
    if (usb_packet_length < USB_PACKET_BUFFER_LENGTH) {
      usb_packet_buffer[usb_packet_length++] = static_cast<uint8_t>(value);
    } else {
      usb_packet_length = 0;
    }
  }
  processUsbPacketBuffer();

  int length = DXL_PORT.available();
  if (length > BUFFER_LENGTH) {
    length = BUFFER_LENGTH;
  }
  if (length > 0) {
    for (int i = 0; i < length; ++i) {
      int value = DXL_PORT.read();
      if (value < 0) {
        length = i;
        break;
      }
      bridge_buffer[i] = static_cast<uint8_t>(value);
    }

    if (length > 0) {
      USB_PORT.write(bridge_buffer, length);
      USB_PORT.flush();
      dxl_to_usb_led_count = 4;
    }
  }
}

static void syncDxlBaudWithUsb()
{
  uint32_t usb_baud = USB_PORT.getBaudRate();
  if (usb_baud == 0) {
    usb_baud = DEFAULT_DXL_BAUD;
  }

  if (usb_baud != current_dxl_baud) {
    DXL_PORT.begin(usb_baud);
    current_dxl_baud = usb_baud;
  }
}

static void updateLed()
{
  uint32_t now = millis();

  if (now - last_usb_to_dxl_led_ms > 40) {
    last_usb_to_dxl_led_ms = now;
    if (usb_to_dxl_led_count > 0) {
      digitalWrite(LED_USB_TO_DXL, !digitalRead(LED_USB_TO_DXL));
      --usb_to_dxl_led_count;
    } else {
      digitalWrite(LED_USB_TO_DXL, HIGH);
    }
  }

  if (now - last_dxl_to_usb_led_ms > 40) {
    last_dxl_to_usb_led_ms = now;
    if (dxl_to_usb_led_count > 0) {
      digitalWrite(LED_DXL_TO_USB, !digitalRead(LED_DXL_TO_USB));
      --dxl_to_usb_led_count;
    } else {
      digitalWrite(LED_DXL_TO_USB, HIGH);
    }
  }
}

void setup()
{
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  pinMode(LED_USB_TO_DXL, OUTPUT);
  pinMode(LED_DXL_TO_USB, OUTPUT);

  digitalWrite(LED_USB_TO_DXL, HIGH);
  digitalWrite(LED_DXL_TO_USB, HIGH);

  USB_PORT.begin(DEFAULT_DXL_BAUD);
  DXL_PORT.begin(DEFAULT_DXL_BAUD);
  current_dxl_baud = DEFAULT_DXL_BAUD;

  setDxlTx(false);
  setDxlPower(true);

  imu.begin();
  updateVirtualTable();
}

void loop()
{
  imu.update();
  syncDxlBaudWithUsb();
  updateDxlBridge();
  updateLed();
}
