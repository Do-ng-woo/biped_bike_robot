#include <Arduino.h>
#include <IMU.h>

// Test-only OpenCR IMU stream.
//
// Upload this sketch only when you want to inspect the built-in OpenCR IMU.
// It does not act as a Dynamixel USB bridge. Re-upload
// arduino/opencr_usb_to_dxl/opencr_usb_to_dxl.ino before running the robot.

#define USB_PORT Serial
#define STREAM_BAUD 115200
#define STREAM_PERIOD_MS 20

cIMU imu;

static uint32_t last_stream_ms = 0;

static void printHeader()
{
  USB_PORT.println(
    "time_ms,"
    "qw,qx,qy,qz,"
    "roll_deg,pitch_deg,yaw_deg,"
    "gyro_x_dps,gyro_y_dps,gyro_z_dps,"
    "acc_x_g,acc_y_g,acc_z_g,"
    "gyro_x_adc,gyro_y_adc,gyro_z_adc,"
    "acc_x_adc,acc_y_adc,acc_z_adc"
  );
}

static void printFloat(float value)
{
  USB_PORT.print(value, 6);
}

static void printComma()
{
  USB_PORT.print(',');
}

static void printImuRow()
{
  USB_PORT.print(millis());
  printComma();

  printFloat(imu.quat[0]);
  printComma();
  printFloat(imu.quat[1]);
  printComma();
  printFloat(imu.quat[2]);
  printComma();
  printFloat(imu.quat[3]);
  printComma();

  printFloat(imu.rpy[0]);
  printComma();
  printFloat(imu.rpy[1]);
  printComma();
  printFloat(imu.rpy[2]);
  printComma();

  printFloat(imu.gx);
  printComma();
  printFloat(imu.gy);
  printComma();
  printFloat(imu.gz);
  printComma();

  printFloat(imu.ax);
  printComma();
  printFloat(imu.ay);
  printComma();
  printFloat(imu.az);
  printComma();

  USB_PORT.print(imu.gyroData[0]);
  printComma();
  USB_PORT.print(imu.gyroData[1]);
  printComma();
  USB_PORT.print(imu.gyroData[2]);
  printComma();

  USB_PORT.print(imu.accData[0]);
  printComma();
  USB_PORT.print(imu.accData[1]);
  printComma();
  USB_PORT.println(imu.accData[2]);
}

void setup()
{
  USB_PORT.begin(STREAM_BAUD);
  while (!USB_PORT && millis() < 3000) {
  }

  uint8_t err = imu.begin();
  if (err != IMU_OK || !imu.bConnected) {
    USB_PORT.println("error,opencr_imu_not_connected");
    return;
  }

  printHeader();
}

void loop()
{
  imu.update();

  uint32_t now = millis();
  if (now - last_stream_ms >= STREAM_PERIOD_MS) {
    last_stream_ms = now;
    printImuRow();
  }
}
