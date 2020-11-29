#include <atmel_start.h>
#include <adc_basic.h>
#include <i2c_master.h>
#include <atomic.h>

static volatile uint8_t adc_measurement;
static volatile bool adc_done;

static const uint8_t ADC_LOOKUP[] = {
	35, 35, 35, 36, 36, 37, 37, 38, 38, 38, 39,
	39, 40, 40, 41, 41, 41, 42, 42, 43, 43, 44,
	44, 44, 45, 45, 46, 46, 47, 47, 47, 48, 48,
	49, 49, 50, 50, 50, 51, 51, 52, 52, 53, 53,
	53, 54, 54, 55, 55, 56, 56, 56, 57, 57, 58,
	58, 59, 59, 59, 60, 60, 61, 61, 62, 62, 62,
	63, 63, 64, 64, 65, 65, 65, 66, 66, 67, 67,
	68, 68, 68, 69, 69, 70, 70, 71, 71, 71, 72,
	72, 73, 73, 74, 74, 74, 75, 75, 76, 76, 77,
	77, 77, 78, 78, 79, 79, 80, 80, 80, 81, 81,
	82, 82, 83, 83, 83, 84, 84, 85, 85, 86, 86,
	86, 87, 87, 88, 88, 89, 89, 90, 90, 90, 91,
	91, 92, 92, 93, 93, 93, 94, 94, 95, 95, 96,
	96, 96, 97, 97, 98, 98, 99, 99, 99, 100, 100,
	101, 101, 102, 102, 102, 103, 103, 104, 104,
	105, 105, 105, 106, 106, 107, 107, 108, 108,
	108, 109, 109, 110, 110, 111, 111, 111, 112,
	112, 113, 113, 114, 114, 114, 115, 115, 116,
	116, 117, 117, 117, 118, 118, 119, 119, 120,
	120, 120, 121, 121, 122, 122, 123, 123, 123,
	124, 124, 125, 125, 126, 126, 126, 127, 127,
	128, 128, 129, 129, 129, 130, 130, 131, 131,
	132, 132, 132, 133, 133, 134, 134, 135, 135,
	135, 136, 136, 137, 137, 138, 138, 138, 139,
	139, 140, 140, 141, 141, 141, 142, 142, 143,
	143, 144, 144
};

static const uint8_t SEVEN_SEG_LOOKUP[] = {
	0b1111110,
	0b0110000,
	0b1101101,
	0b1111001,
	0b0110011,
	0b1011011,
	0b1011111,
	0b1110000,
	0b1111111,
	0b1111011
};

static const i2c_address_t I2C_ADDR_1 = 0x24;
static const i2c_address_t I2C_ADDR_2 = 0x23;

static const uint8_t I2C_CONFIG_1 = 0x06;
static const uint8_t I2C_CONFIG_2 = 0x07;
static const uint8_t I2C_OUTPUT_PORT_1 = 0x02;
static const uint8_t I2C_OUTPUT_PORT_2 = 0x03;

static const uint16_t I2C_TIMEOUT = 10000;

static i2c_error_t lastError;

uint8_t scale_adc(uint16_t adc_val) {
	return ADC_LOOKUP[adc_val & 0xff];
}

void adc_callback(void)
{
	adc_measurement = scale_adc(ADC_0_get_conversion_result());
	adc_done = true;
}

uint8_t modulo(uint8_t val, uint8_t mod) {
	while (val > mod) {
		val -= mod;
	}
	return val;
}

uint8_t divide(uint8_t val, uint8_t div) {
	uint8_t ret = 0;
	while (val > div) {
		val -= div;
		ret++;
	}
	return ret;
}

typedef struct {
	uint8_t	tens;
	uint8_t ones;
	uint8_t decimal;
} seven_seg_t;

typedef struct {
	uint8_t *data;
	uint8_t  size;
} transfer_descriptor_t;

i2c_error_t i2c_send(i2c_address_t address, void *data, size_t len)
{
	/* timeout is used to get out of twim_release, when there is no device connected to the bus*/
	uint16_t timeout = I2C_TIMEOUT;

	while (I2C_BUSY == I2C_0_open(address) && --timeout); // sit here until we get the bus..
	if (!timeout)
	return I2C_BUSY;
	I2C_0_set_buffer(data, len);
	I2C_0_set_address_nack_callback(i2c_cb_restart_write, NULL); // NACK polling?
	I2C_0_master_write();
	timeout = I2C_TIMEOUT;
	while (I2C_BUSY == I2C_0_close() && --timeout); // sit here until finished.
	if (!timeout)
	return I2C_FAIL;

	return I2C_NOERR;
}

void i2c_send_data(uint8_t addr, uint8_t reg, uint8_t data) {
	uint8_t i2c_buf[2] = {reg, data};
	i2c_error_t i2cRet = i2c_send(addr, i2c_buf, 2);
	if (i2cRet != I2C_NOERR) {
		lastError = i2cRet;
	}
}

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

	ADC_0_register_callback(adc_callback);

	i2c_send_data(I2C_ADDR_1, I2C_CONFIG_1, 0x0);
	i2c_send_data(I2C_ADDR_1, I2C_CONFIG_2, 0x0);
	i2c_send_data(I2C_ADDR_2, I2C_CONFIG_1, 0x0);
	i2c_send_data(I2C_ADDR_2, I2C_CONFIG_2, 0x0);

	i2c_send_data(I2C_ADDR_1, I2C_OUTPUT_PORT_1, 0x55);
	i2c_send_data(I2C_ADDR_1, I2C_OUTPUT_PORT_2, 0xAA);
	i2c_send_data(I2C_ADDR_2, I2C_OUTPUT_PORT_1, 0x55);
	
	/* Replace with your application code */
	//while (1) {
	//
	//ENABLE_INTERRUPTS();
	//adc_done = false;
	//ADC_0_start_conversion(0);
	//while (!adc_done);
	//DISABLE_INTERRUPTS();
	//
	//// do something with adc_measurement
	//seven_seg_t display_data;
	//display_data.tens = SEVEN_SEG_LOOKUP[divide(adc_measurement, 100)];
	//display_data.ones = SEVEN_SEG_LOOKUP[divide(modulo(adc_measurement, 100), 10)];
	//display_data.decimal = SEVEN_SEG_LOOKUP[modulo(adc_measurement, 10)];
	//
	//i2c_send_data(I2C_ADDR_1, I2C_OUTPUT_PORT_1, display_data.tens);
	//i2c_send_data(I2C_ADDR_1, I2C_OUTPUT_PORT_2, display_data.ones);
	//i2c_send_data(I2C_ADDR_2, I2C_OUTPUT_PORT_1, display_data.decimal);
	//}
}
