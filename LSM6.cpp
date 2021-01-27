#include <LSM6.h>
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define DS33_SA0_HIGH_ADDRESS 0b1101011
#define DS33_SA0_LOW_ADDRESS  0b1101010

#define TEST_REG_ERROR -1

#define DS33_WHO_ID    0x69

// Constructors ////////////////////////////////////////////////////////////////

LSM6::LSM6(void)
{
  _wire = &Wire;
  _device = device_auto;

  io_timeout = 0;  // 0 = no timeout
  did_timeout = false;
}

LSM6::LSM6(TwoWire *wire)
{
  _wire = wire;
  _device = device_auto;

  io_timeout = 0;  // 0 = no timeout
  did_timeout = false;
}

// Public Methods //////////////////////////////////////////////////////////////

// Did a timeout occur in readAcc(), readGyro(), or read() since the last call to timeoutOccurred()?
bool LSM6::timeoutOccurred()
{
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

void LSM6::setTimeout(uint16_t timeout)
{
  io_timeout = timeout;
}

uint16_t LSM6::getTimeout()
{
  return io_timeout;
}

bool LSM6::init(deviceType device, sa0State sa0)
{
  // perform auto-detection unless device type and SA0 state were both specified
  if (device == device_auto || sa0 == sa0_auto)
  {
    // check for LSM6DS33 if device is unidentified or was specified to be this type
    if (device == device_auto || device == device_DS33)
    {
      // check SA0 high address unless SA0 was specified to be low
      if (sa0 != sa0_low && testReg(DS33_SA0_HIGH_ADDRESS, WHO_AM_I) == DS33_WHO_ID)
      {
        sa0 = sa0_high;
        if (device == device_auto) { device = device_DS33; }
      }
      // check SA0 low address unless SA0 was specified to be high
      else if (sa0 != sa0_high && testReg(DS33_SA0_LOW_ADDRESS, WHO_AM_I) == DS33_WHO_ID)
      {
        sa0 = sa0_low;
        if (device == device_auto) { device = device_DS33; }
      }
    }

    // make sure device and SA0 were successfully detected; otherwise, indicate failure
    if (device == device_auto || sa0 == sa0_auto)
    {
      return false;
    }
  }

  _device = device;

  switch (device)
  {
    case device_DS33:
    address = (sa0 == sa0_high) ? DS33_SA0_HIGH_ADDRESS : DS33_SA0_LOW_ADDRESS;
    break;
  }

  return true;
}

/*
Enables the LSM6's accelerometer and gyro. Also:
- Sets sensor full scales (gain) to default power-on values, which are
+/- 2 g for accelerometer and 245 dps for gyro
- Selects 1.66 kHz (high performance) ODR (output data rate) for accelerometer
and 1.66 kHz (high performance) ODR for gyro. (These are the ODR settings for
which the electrical characteristics are specified in the datasheet.)
- Enables automatic increment of register address during multiple byte access
Note that this function will also reset other settings controlled by
the registers it writes to.
*/
void LSM6::enableDefault(void)
{
  if (_device == device_DS33)
  {
    // Accelerometer

    // 0x80 = 0b10000000
    // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (+/-2 g full scale)
    writeReg(CTRL1_XL, 0x80);

    // Gyro

    // 0x80 = 0b010000000
    // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (245 dps)
    writeReg(CTRL2_G, 0x80);

    // Common

    // 0x04 = 0b00000100
    // IF_INC = 1 (automatically increment register address)
    writeReg(CTRL3_C, 0x04);
  }
}

void LSM6::writeReg(uint8_t reg, uint8_t value)
{
  _wire -> beginTransmission(address);
  _wire -> write(reg);
  _wire -> write(value);
  last_status = _wire -> endTransmission();
}

uint8_t LSM6::readReg(uint8_t reg)
{
  uint8_t value;

  _wire -> beginTransmission(address);
  _wire -> write(reg);
  last_status = _wire -> endTransmission();
  _wire -> requestFrom(address, (uint8_t)1);
  value = _wire -> read();
  _wire -> endTransmission();

  return value;
}

// Reads the 3 accelerometer channels and stores them in vector a
void LSM6::readAcc(void)
{
  _wire -> beginTransmission(address);
  // automatic increment of register address is enabled by default (IF_INC in CTRL3_C)
  _wire -> write(OUTX_L_XL);
  _wire -> endTransmission();
  _wire -> requestFrom(address, (uint8_t)6);

  uint16_t millis_start = millis();
  while (_wire -> available() < 6) {
    if (io_timeout > 0 && ((uint16_t)millis() - millis_start) > io_timeout)
    {
      did_timeout = true;
      return;
    }
  }

  uint8_t xla = _wire -> read();
  uint8_t xha = _wire -> read();
  uint8_t yla = _wire -> read();
  uint8_t yha = _wire -> read();
  uint8_t zla = _wire -> read();
  uint8_t zha = _wire -> read();

  // combine high and low bytes
  a.x = (int16_t)(xha << 8 | xla);
  a.y = (int16_t)(yha << 8 | yla);
  a.z = (int16_t)(zha << 8 | zla);
}

// Reads the 3 gyro channels and stores them in vector g
void LSM6::readGyro(void)
{
  _wire -> beginTransmission(address);
  // automatic increment of register address is enabled by default (IF_INC in CTRL3_C)
  _wire -> write(OUTX_L_G);
  _wire -> endTransmission();
  _wire -> requestFrom(address, (uint8_t)6);

  uint16_t millis_start = millis();
  while (_wire -> available() < 6) {
    if (io_timeout > 0 && ((uint16_t)millis() - millis_start) > io_timeout)
    {
      did_timeout = true;
      return;
    }
  }

  uint8_t xlg = _wire -> read();
  uint8_t xhg = _wire -> read();
  uint8_t ylg = _wire -> read();
  uint8_t yhg = _wire -> read();
  uint8_t zlg = _wire -> read();
  uint8_t zhg = _wire -> read();

  // combine high and low bytes
  g.x = (int16_t)(xhg << 8 | xlg);
  g.y = (int16_t)(yhg << 8 | ylg);
  g.z = (int16_t)(zhg << 8 | zlg);
}

// Reads all 6 channels of the LSM6 and stores them in the object variables
void LSM6::read(void)
{
  readAcc();
  readGyro();
}

void LSM6::vector_normalize(vector<float> *a)
{
  float mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

// Private Methods //////////////////////////////////////////////////////////////

int16_t LSM6::testReg(uint8_t address, regAddr reg)
{
  _wire -> beginTransmission(address);
  _wire -> write((uint8_t)reg);
  if (_wire -> endTransmission() != 0)
  {
    return TEST_REG_ERROR;
  }

  _wire -> requestFrom(address, (uint8_t)1);
  if (_wire -> available())
  {
    return _wire -> read();
  }
  else
  {
    return TEST_REG_ERROR;
  }
}
