


#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "DSP310_I2C_PSoC6.h"



float Calculate_temperature(uint8_t *temperatura, uint32_t kT_personal, int standard, Coefficients_struct *p, float *T_raw){

	float volatile kT_used;

	if(standard)
		kT_used = kT;
	else
		kT_used = kT_personal;



	float volatile T_comp=0;
	int32_t temperatura_signed=0;

	temperatura_signed = ((temperatura[0]<<16)+(temperatura[1]<<8)+(temperatura[2]));

	getTowsComplement(&temperatura_signed, 24);

	*T_raw = temperatura_signed/kT_used;
	T_comp = (p->c0*0.5) + p->c1*(*T_raw);

	return T_comp;
}


float Calculate_pressure(uint8_t *presion, uint32_t kP_personal, int standard, Coefficients_struct *p, float T_raw){

	float kP_used;

	if(standard)
		kP_used = kP;
	else
		kP_used = kP_personal;


	float volatile P_raw=0;
	float volatile P_comp=0;
	int32_t presion_signed=0;

	presion_signed = ((presion[0]<<16)+(presion[1]<<8)+(presion[2]));

	getTowsComplement(&presion_signed, 24);

	P_raw = presion_signed/kP_used;
	P_comp = p->c00 + P_raw*(p->c10 + P_raw*(p->c20 + P_raw*p->c30)) + T_raw*p->c01 + T_raw*P_raw*(p->c11+P_raw*p->c21);

	return P_comp;
}


void read_temperature_parameters(cyhal_i2c_t *mI2C, uint8_t *temperatura){

	uint8_t meas_temp = meas_temp_byte;
			cyhal_i2c_master_mem_write(mI2C, DSP_SLAVE_ADDR, reg_meas, size_meas, &meas_temp, 1, 0);
	    	cyhal_system_delay_ms(100);
	    	cyhal_i2c_master_mem_read(mI2C, DSP_SLAVE_ADDR, reg_temperatura, size_temperatura, temperatura, SIZE_B_temperatura, 0);
	    	cyhal_system_delay_ms(100);

}


void read_pressure_parameters(cyhal_i2c_t *mI2C, uint8_t *presion){

	uint8_t meas_prss = meas_prss_byte;
				cyhal_i2c_master_mem_write(mI2C, DSP_SLAVE_ADDR, reg_meas, size_meas, &meas_prss, 1, 0);
		    	cyhal_system_delay_ms(100);
		    	cyhal_i2c_master_mem_read(mI2C, DSP_SLAVE_ADDR, reg_presion, size_presion, presion, SIZE_B_presion, 0);
		    	cyhal_system_delay_ms(100);

}

void init_some_pre_conf(cyhal_i2c_t *mI2C, int external_sensor){

	uint8_t temp_sensor_cfg;
	if (external_sensor)
		temp_sensor_cfg = 0b10000000;
	else
		temp_sensor_cfg = 0b00000000;


	cyhal_i2c_master_mem_write(mI2C, DSP_SLAVE_ADDR, 0x28UL , 1UL , &temp_sensor_cfg, 1, 0);

	cyhal_system_delay_ms(100);
}


void configure_I2Cm(cyhal_i2c_t *mI2C, cyhal_i2c_cfg_t *mI2C_cfg){

	(*mI2C_cfg).is_slave = false;
	(*mI2C_cfg).address = 0;
	(*mI2C_cfg).frequencyhal_hz = I2C_FREQ;

	cyhal_i2c_init( mI2C, mI2C_SDA, mI2C_SCL, NULL);
	cyhal_i2c_configure( mI2C, mI2C_cfg);



}

void write_configuration_reg(cyhal_i2c_t *mI2C, uint8_t pressure_conf, uint8_t temperature_conf, int standard, uint8_t FyI_conf){

	uint8_t press, temp, FyI;
	if (standard){
		press = prss_byte;
		temp = temp_byte;
		FyI = FyI_byte;
	}
	else{
		press = pressure_conf;
		temp = temperature_conf;
		FyI = FyI_conf;

	}


 	/*Escribimos en el registro 6 (PRS_CFG)*/
 	cyhal_i2c_master_mem_write(mI2C, DSP_SLAVE_ADDR, reg_prss_cfg, size_prss_cfg, &press, 1, 0);

 	 cyhal_system_delay_ms(100);

 	/*Escribimos en el registro 7 (TEMP_CFG)*/
 	cyhal_i2c_master_mem_write(mI2C, DSP_SLAVE_ADDR, reg_temp_cfg, size_temp_cfg, &temp, 1, 0);

 	 cyhal_system_delay_ms(100);

 	/*Escribimos en el registro 9 (FIFO E INT CFG)*/
 	cyhal_i2c_master_mem_write(mI2C, DSP_SLAVE_ADDR, reg_FyI, size_FyI, &FyI, 1, 0);

 	 cyhal_system_delay_ms(100);
}


void get_coefficients(Coefficients_struct *p, uint8_t *coeficientes, cyhal_i2c_t *mI2C){


	cyhal_i2c_master_mem_read(mI2C, DSP_SLAVE_ADDR, reg_coeff, size_coeff, coeficientes, SIZE_B_COEFF, 0);

	cyhal_system_delay_ms(100);

 	calculate_coefficients(p, coeficientes);

}


void calculate_coefficients(Coefficients_struct *p, uint8_t *coeficientes){

		p->c0 = (coeficientes[0] << 4) + ((coeficientes[1]>>4)&0x0F);
	 	getTowsComplement(&(p->c0), 12);

	 	p->c1 = ((coeficientes[1] & 0x0F)<<8) | (coeficientes[2]);
	 	getTowsComplement(&(p->c1), 12);

	 	p->c00 = (coeficientes[3]<<12) + (coeficientes[4]<<4) + ((coeficientes[5]>>4)&0x0F);
	 	getTowsComplement(&(p->c00), 20);

	 	p->c10 = ((coeficientes[5]&0x0F)<<16) + (coeficientes[6]<<8) + (coeficientes[7]);
	 	getTowsComplement(&(p->c10), 20);

	 	p->c20 = (coeficientes[12]<<8) + (coeficientes[13]);
	 	getTowsComplement(&(p->c20), 16);

	 	p->c30 = (coeficientes[16]<<8) + (coeficientes[17]);
	 	getTowsComplement(&(p->c30), 16);

	 	p->c01 = (coeficientes[8]<<8) + (coeficientes[9]);
	 	getTowsComplement(&(p->c01), 16);

	 	p->c11 = (coeficientes[10]<<8) + (coeficientes[11]);
	 	getTowsComplement(&(p->c11), 16);

	 	p->c21 = (coeficientes[14]<<8) + (coeficientes[15]);
	 	getTowsComplement(&(p->c21), 16);


}



void getTowsComplement(int32_t *raw, uint8_t length){
	if(*raw &((uint32_t)1<<(length-1))){
		*raw-=(uint32_t)1 << length;
	}
}
