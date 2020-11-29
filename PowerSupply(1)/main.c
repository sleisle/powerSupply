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

static const uint8_t I2C_ADDR_1 = 0x01;
static const uint8_t I2C_ADDR_2 = 0x02;

typedef struct {
	uint8_t *data;
	uint8_t  size;
} transfer_descriptor_t;

/** This callback is called when the initial write of the pointer register has finished.
    This callback controls the second phase of the I2C transaction, the read of the
    targeted register after a REPEATED START.
*/
i2c_operations_t i2c_write_callback(void *d)
{
	transfer_descriptor_t *desc = (transfer_descriptor_t *)d;
	I2C_0_set_buffer((void *)desc->data, desc->size);
	// Set callback to terminate transfer and send STOP after read is complete
	I2C_0_set_data_complete_callback(i2c_cb_return_stop, NULL);
	return i2c_restart_read; // Send REPEATED START before read
}

i2c_error_t i2c_send(
	uint8_t i2c_addr,
	uint8_t *data,
	uint8_t size
) {
	/* timeout is used to get out of twim_release, when there is no device connected to the bus*/
	uint16_t              timeout = 1000;
	transfer_descriptor_t d       = {data, size};

	while (I2C_BUSY == I2C_0_open(i2c_addr) && --timeout); // sit here until we get the bus..
	if (!timeout)
	return I2C_BUSY;

	// This callback specifies what to do after the first write operation has completed
	// The parameters to the callback are bundled together in the aggregate data type d.
	I2C_0_set_data_complete_callback(i2c_write_callback, &d);
	// Transmit specified number of bytes
	I2C_0_set_buffer(d.data, d.size);
	// Start a Write operation
	I2C_0_master_operation(false);
	timeout = 1000;
	while (I2C_BUSY == I2C_0_close() && --timeout); // sit here until finished.
	if (!timeout)
	return I2C_FAIL;

	return I2C_NOERR;
}

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

	ADC_0_register_callback(adc_callback);

	uint8_t i2c_buf[2];
	
	i2c_buf[0] = 0x06;
	i2c_buf[1] = 0x00;
	i2c_send(I2C_ADDR_1, i2c_buf, 2);
	i2c_buf[0] = 0x07;
	i2c_buf[1] = 0x00;
	i2c_send(I2C_ADDR_1, i2c_buf, 2);
	
	i2c_buf[0] = 0x06;
	i2c_buf[1] = 0x00;
	i2c_send(I2C_ADDR_2, i2c_buf, 2);
	i2c_buf[0] = 0x07;
	i2c_buf[1] = 0x00;
	i2c_send(I2C_ADDR_2, i2c_buf, 2);
	
	/* Replace with your application code */
	while (1) {
		
		ENABLE_INTERRUPTS();
		adc_done = false;
		ADC_0_start_conversion(0);
		while (!adc_done);
		DISABLE_INTERRUPTS();
		
		// do something with adc_measurement
		seven_seg_t display_data;
		display_data.tens = SEVEN_SEG_LOOKUP[divide(adc_measurement, 100)];
		display_data.ones = SEVEN_SEG_LOOKUP[divide(modulo(adc_measurement, 100), 10)];
		display_data.decimal = SEVEN_SEG_LOOKUP[modulo(adc_measurement, 10)];
		
		i2c_buf[0] = 0x02;
		i2c_buf[1] = display_data.tens;
		i2c_send(I2C_ADDR_1, i2c_buf, 2);
	
		i2c_buf[0] = 0x03;
		i2c_buf[1] = display_data.ones;
		i2c_send(I2C_ADDR_1, i2c_buf, 2);
		
		i2c_buf[0] = 0x02;
		i2c_buf[1] = display_data.decimal;
		i2c_send(I2C_ADDR_2, i2c_buf, 2);
	}
}
