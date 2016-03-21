#ifndef SUPPORT_H_
#define SUPPORT_H_

#define YES				1
#define	NO				0
#define	FAIL			-1
#define	PASS			0

#define BMP180_PROM_START__ADDR		(0xAA)
#define BMP180_PROM_DATA__LEN		(22)

#define BMP180_CHIP_ID_REG			(0xD0)
#define BMP180_VERSION_REG			(0xD1)

#define BMP180_CTRL_MEAS_REG		(0xF4)
#define BMP180_ADC_OUT_MSB_REG		(0xF6)
#define BMP180_ADC_OUT_LSB_REG		(0xF7)

#define BMP180_SOFT_RESET_REG		(0xE0)

/* temperature measurement */
#define BMP180_T_MEASURE			(0x2E)
/* pressure measurement*/
#define BMP180_P_MEASURE			(0x34)

/* Slave Address for I2C Slave */
#define SLAVE_ADDRESS       0x77

#define TIMER_PERIOD    12000u //  = 1ms @ 12 MHz

/* Timer_A UpMode Configuration Parameter */
typedef union {
  struct {
    uint8_t hh;
    uint8_t hl;
    uint8_t th;
    uint8_t tl;
    uint8_t crc;
  } val;
  uint8_t bytes[5];
} dht22data;

dht22data dht_data;
uint8_t dht_data_byte, dht_data_bit;

enum {
  DHT_IDLE = 0,
  DHT_TRIGGERING,
  DHT_WAITING_ACK,
  DHT_ACK_LOW,
  DHT_ACK_HIGH,
  DHT_IN_BIT_LOW,
  DHT_IN_BIT_HIGH,
} dht_current_state;

void dht_start_read();
int dht_get_temp();
int dht_get_rh();
int RF_Init(void);
void GetLightValue(uint16_t *luxValue, int8_t *lightIndex);
int ADC_InitFunction(void);
int GetBaroTemp(int16_t calData[11], int32_t *temp, int32_t *pressure);
int I2C_InitFunc(int16_t calData[11]);
int InitOneWire(void);

#endif /* SUPPORT_H_ */
