#include <Arduino.h>

// OpenCR USB-to-Dynamixel TTL passthrough.
//
// This sketch makes the OpenCR behave like a simple U2D2-style adapter:
//   PC USB Serial <-> OpenCR Serial3 <-> Dynamixel TTL bus
//
// Do not print debug text to Serial. The PC-side Dynamixel SDK expects raw
// Dynamixel packets only, and any extra bytes on USB will corrupt packets.

#define USB_PORT Serial
#define DXL_PORT Serial3

#define DEFAULT_DXL_BAUD 1000000
#define BUFFER_LENGTH 1024

#define LED_USB_TO_DXL BDPIN_LED_USER_1
#define LED_DXL_TO_USB BDPIN_LED_USER_2

static uint8_t buffer[BUFFER_LENGTH];
static uint32_t current_dxl_baud = DEFAULT_DXL_BAUD;

static uint8_t usb_to_dxl_led_count = 0;
static uint8_t dxl_to_usb_led_count = 0;
static uint32_t last_usb_to_dxl_led_ms = 0;
static uint32_t last_dxl_to_usb_led_ms = 0;

static void setDxlPower(bool enabled)
{
  digitalWrite(BDPIN_DXL_PWR_EN, enabled ? HIGH : LOW);
}

static void setDxlTx(bool enabled)
{
  drv_dxl_tx_enable(enabled ? 1 : 0);
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

static int readAvailable(HardwareSerial &port, uint8_t *dst, int max_len)
{
  int length = port.available();

  if (length > max_len) {
    length = max_len;
  }

  for (int i = 0; i < length; ++i) {
    int value = port.read();
    if (value < 0) {
      return i;
    }
    dst[i] = static_cast<uint8_t>(value);
  }

  return length;
}

static void updateDxlBridge()
{
  int length = USB_PORT.available();
  if (length > 0) {
    if (length > BUFFER_LENGTH) {
      length = BUFFER_LENGTH;
    }

    for (int i = 0; i < length; ++i) {
      int value = USB_PORT.read();
      if (value < 0) {
        length = i;
        break;
      }
      buffer[i] = static_cast<uint8_t>(value);
    }

    if (length > 0) {
      setDxlTx(true);
      DXL_PORT.write(buffer, length);
      DXL_PORT.flush();
      setDxlTx(false);

      usb_to_dxl_led_count = 4;
    }
  }

  length = readAvailable(DXL_PORT, buffer, BUFFER_LENGTH);
  if (length > 0) {
    USB_PORT.write(buffer, length);
    USB_PORT.flush();

    dxl_to_usb_led_count = 4;
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
}

void loop()
{
  syncDxlBaudWithUsb();
  updateDxlBridge();
  updateLed();
}
