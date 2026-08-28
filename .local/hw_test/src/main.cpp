#include <Arduino.h>
#include <Wire.h>

// AK09911C magnetometer on RP2040, wired to GPIO6 (SDA) / GPIO7 (SCL).
// This core's global Wire has no setSDA/setSCL remap, so bind a dedicated
// TwoWire instance to these pins directly.
static const uint8_t AK_SDA_PIN = 2; //6;
static const uint8_t AK_SCL_PIN = 3; //7;
static const uint8_t AK09911_ADDR = 0x0C;

static TwoWire i2c(AK_SDA_PIN, AK_SCL_PIN);

// AK09911 register map
enum : uint8_t {
	REG_WIA1  = 0x00, // company ID, expect 0x48
	REG_WIA2  = 0x01, // device ID, expect 0x05
	REG_ST1   = 0x10,
	REG_HXL   = 0x11,
	REG_ST2   = 0x18,
	REG_CNTL2 = 0x31,
	REG_CNTL3 = 0x32,
};

enum : uint8_t {
	MODE_POWER_DOWN     = 0x00,
	MODE_CONT_100HZ     = 0x08,
	SOFT_RESET          = 0x01,
};

static bool writeReg(uint8_t reg, uint8_t val)
{
	i2c.beginTransmission(AK09911_ADDR);
	i2c.write(reg);
	i2c.write(val);
	return i2c.endTransmission() == 0;
}

static bool readRegs(uint8_t reg, uint8_t *buf, size_t len)
{
	i2c.beginTransmission(AK09911_ADDR);
	i2c.write(reg);

	if (i2c.endTransmission(false) != 0) {
		return false;
	}

	if (i2c.requestFrom(AK09911_ADDR, len) != len) {
		return false;
	}

	for (size_t i = 0; i < len; i++) {
		buf[i] = i2c.read();
	}

	return true;
}

static void scanBus()
{
	Serial.println("I2C scan: starting...");
	int found = 0;

	for (uint8_t addr = 1; addr < 127; addr++) {
		i2c.beginTransmission(addr);

		if (i2c.endTransmission() == 0) {
			Serial.print("I2C scan: device found at 0x");
			Serial.println(addr, HEX);
			found++;
		}
	}

	Serial.print("I2C scan: done, ");
	Serial.print(found);
	Serial.println(" device(s) found");
}

static bool probeAndInit()
{
	uint8_t wia[2] = {};

	if (!readRegs(REG_WIA1, wia, sizeof(wia))) {
		Serial.println("AK09911: no ACK on I2C, device not responding");
		return false;
	}

	Serial.print("AK09911: WIA1=0x");
	Serial.print(wia[0], HEX);
	Serial.print(" WIA2=0x");
	Serial.print(wia[1], HEX);
	Serial.println(" (expect 0x48 0x05)");

	if (wia[0] != 0x48 || wia[1] != 0x05) {
		Serial.println("AK09911: unexpected WIA, wrong device or bad wiring");
		return false;
	}

	writeReg(REG_CNTL3, SOFT_RESET);
	delay(2);

	if (!writeReg(REG_CNTL2, MODE_CONT_100HZ)) {
		Serial.println("AK09911: failed to set continuous measurement mode");
		return false;
	}

	Serial.println("AK09911: initialized, continuous mode 100Hz");
	return true;
}

void setup()
{
	Serial.begin(115200);
	uint32_t start = millis();

	while (!Serial && millis() - start < 3000) {
	}

	i2c.begin();

	delay(5000);
	scanBus();

	while (!probeAndInit()) {
		delay(1000);
	}
}

void loop()
{
	uint8_t st1 = 0;

	if (readRegs(REG_ST1, &st1, 1) && (st1 & 0x01)) {
		uint8_t data[7] = {}; // HXL..HZH + ST2

		if (readRegs(REG_HXL, data, sizeof(data))) {
			int16_t x = (int16_t)((data[1] << 8) | data[0]);
			int16_t y = (int16_t)((data[3] << 8) | data[2]);
			int16_t z = (int16_t)((data[5] << 8) | data[4]);
			uint8_t st2 = data[6]; // reading ST2 latches the next sample

			if (st2 & 0x08) {
				Serial.println("AK09911: magnetic sensor overflow (HOFL)");

			} else {
				Serial.print("AK09911: x=");
				Serial.print(x);
				Serial.print(" y=");
				Serial.print(y);
				Serial.print(" z=");
				Serial.print(z);
				Serial.println(" (raw counts)");
			}
		}
	}

	delay(200);
}
