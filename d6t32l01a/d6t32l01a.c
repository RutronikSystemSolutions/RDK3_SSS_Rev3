/*
 * d6t32l01a.c
 *
 *  Created on: 2023-11-07
 *      Author: Gintaras Drukteinis
 */


#include "d6t32l01a.h"
#include "cyhal.h"
#include "FreeRTOS.h"
#include "task.h"

/*RDK3 I2C Read/Write Implementation*/
extern cyhal_i2c_t I2C_scb1;
static uint32_t d6t32_read(uint8_t devAddr, uint8_t regAddr, uint8_t *data, int length)
{
	cy_rslt_t result;

	result = cyhal_i2c_master_write( &I2C_scb1, (uint16_t)devAddr, &regAddr, 1, D6T_I2C_TIMEOUT, false );
    if (result != CY_RSLT_SUCCESS)
    {
    	return D6T_READ_FAIL;
    }

	result = (int8_t)cyhal_i2c_master_read(&I2C_scb1,(uint16_t)devAddr, data, (uint16_t)length, D6T_I2C_TIMEOUT, true);
	if (result != CY_RSLT_SUCCESS)
	{
		 return D6T_READ_FAIL;
	}

	return D6T_OK;
}

static uint32_t d6t32_write(uint8_t devAddr, uint8_t *data, int length)
{
	cy_rslt_t result;

	result = cyhal_i2c_master_write( &I2C_scb1, (uint16_t)devAddr, data, length, D6T_I2C_TIMEOUT, true);
    if (result != CY_RSLT_SUCCESS)
    {
    	return D6T_WRITE_FAIL;
    }

    return D6T_OK;
}

/*RDK3 Delay Implementation*/
static void d6t32_delay_ms(uint32_t msec)
{
	vTaskDelay(pdMS_TO_TICKS(msec));
}

static unsigned char calc_crc(unsigned char data)
{
	int index;
	unsigned char temp;
	for(index=0; index<8; index++)
	{
		temp = data;
		data <<= 1;
		if(temp & 0x80) data ^= 0x07;
	}
	return data;
}

static int16_t conv8us_s16_le(uint8_t *buf, int n)
{
    uint16_t ret;
    ret = (uint16_t)buf[n];
    ret += ((uint16_t)buf[n + 1]) << 8;
    return (int16_t)ret;   // and convert negative.
}

static int D6T_checkPEC(uint8_t *buf, int pPEC)
{
	unsigned char crc;
	int i;
	crc = calc_crc( 0x15 );
	for(i=0; i<pPEC; i++)
	{
		crc = calc_crc(buf[i] ^ crc );
	}
	return (crc == buf[pPEC]);
}

uint32_t d6t32_init(void)
{
	d6t32_delay_ms(20);
	uint8_t dat1[] = {D6T_SET_ADD, (((uint8_t)D6T_IIR << 4)&&0xF0) | (0x0F && (uint8_t)D6T_AVERAGE)};
	return d6t32_write(D6T_ADDR, dat1, sizeof(dat1));
}

int D6T_getvalue(uint8_t *buf, float *ptat, float *pix_data)
{
	int i;
	uint32_t ret;
	int16_t itemp;

	memset(buf, 0, N_READ);
	for (i = 0; i < 10; i++)
	{
		ret = d6t32_read(D6T_ADDR, D6T_CMD, buf, N_READ);
		if (ret == 0)
		{
			break;
		}
		else if (ret == 23)
		{  /* write error */
			d6t32_delay_ms(60);
		}
		else if (ret == 24)
		{  /* read error */
			d6t32_delay_ms(60);
		}
	}
	if(!D6T_checkPEC(buf, N_READ - 1))
	{
		return D6T_PEC_FAIL;
	}

    /* Convert to temperature data (degC) */
	*ptat = (float)conv8us_s16_le(buf, 0) / 10.0;
	for (i = 0; i < N_PIXEL; i++)
	{
		itemp = conv8us_s16_le(buf, 2 + 2*i);
		pix_data[i] = (float)itemp / 10.0;
	}

	return D6T_OK;
}
