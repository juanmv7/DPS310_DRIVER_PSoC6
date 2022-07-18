//#include "cy_retarget_io.h"

#include "DSP310_I2C_PSoC6.h"

static Coefficients_struct Coefs_struct;



int main(void){

	uint8_t coeficientes[SIZE_B_COEFF];
	uint8_t temperatura[SIZE_B_temperatura];
	uint8_t presion[SIZE_B_presion];




	float volatile T_raw=0;
	float volatile T_comp = 0, P_comp = 0;


	/* Set up the device based on configurator selections */
	cybsp_init();

	 	cyhal_i2c_t mI2C;
	 	cyhal_i2c_cfg_t mI2C_cfg;


	 	configure_I2Cm(&mI2C, &mI2C_cfg);


	 	init_some_pre_conf(&mI2C, 1);


	 	get_coefficients(&Coefs_struct, coeficientes, &mI2C);


	 	write_configuration_reg(&mI2C, 0, 0, 1, 0);

	 	while(1){

	 		read_temperature_parameters(&mI2C, temperatura);

	 		T_comp = Calculate_temperature(temperatura, 0, 1, &Coefs_struct, &T_raw);

	 		read_pressure_parameters(&mI2C, presion);

	 		P_comp = Calculate_pressure(presion, 0, 1, &Coefs_struct, T_raw);

	 	}

}
