
/***************************************************************************//**
* \file DPS310_I2C_PSoC6.h
*
* \brief
* Provides a high level interface for interacting with the Infineon sensor DPS310.
* This interface abstracts out the chip specific details. If any chip specific
* functionality is necessary, or performance is critical the low level functions
* can be used directly.
*/


#ifndef DPS310_I2C_PSoC6
#define DPS310_I2C_PSoC6





/***************************************
*            Constants
****************************************/
#define mI2C_SCL                (P6_0)    /*Ports of the PSoC 6 BLE - CY8CKIT-062-BLE*/
#define mI2C_SDA                (P6_1)



#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

/*In the next section I include general constants that can be used in a standard program.
 * Mainly I include all the registers where you have to write the configuration parameters and also
 * the standard pre-configured bytes you need to send to the sensor. IF you want to see more types of parameters,
 * see /ref <https://www.infineon.com/dgdl/Infineon-DPS310-DS-v01_00-EN.pdf?fileId=5546d462576f34750157750826c42242>*/

/** PRESSURE CONFIGURATION **/
#define prss_byte                  (0b00010100UL)  /*This is the standard configuration:
                                                    16 times(oversampling rate)    &&   2 measurements per second*/
#define reg_prss_cfg                 (0x06UL)
#define size_prss_cfg                  1UL

/*Temperature CONFIGURATION*/
#define temp_byte                 (0b10010000UL)  /*External sensor, 2 measurements per second, single (DEFAULT) */
#define reg_temp_cfg                (0x07UL)
#define size_temp_cfg                 (1UL)
//#define temp_sensor_cfg            (0b10000000UL)   /*Coefficients are based on external sensor*/
#define kT  524288.0
#define kP  253952.0


/*FIFO and Interrupt CONFIGURATION*/
#define FyI_byte               (0b10000100ul)    /*Pressure shift because we are using 16 times (oversampling rate)
                                                   No need of temperature shift -> Single                 */
#define reg_FyI                   (0x09UL)
#define size_FyI                   (1UL)

/*Measurements CONFIGURATION*/
#define meas_prss_byte        (0b00000001UL)     /* Pressure measurements*/
#define meas_temp_byte        (0b00000010UL)     /* Temperature measurements*/   /*We are following the tree in the datasheet*/
#define reg_meas                (0x08UL)
#define size_meas                 (1UL)


/*Registers of the coefficients (0x10 to 0x20)*/
#define reg_coeff                (0x10UL)
#define size_coeff                 (1UL)


/* I2C slave address to communicate with */
#define DSP_SLAVE_ADDR        (0x77UL)

/* I2C bus frequency */
#define I2C_FREQ                (400000UL)


/* Buffer and packet size */
#define SIZE_B_COEFF         18UL    /*There are 18 registers for the coefficients*/
#define SIZE_B_temperatura   3UL     /*There are 3 registers for the temperature measurements*/
#define SIZE_B_presion       3UL     /*There are 3 registers for the pressure measurements*/


// Temperature registers
#define reg_temperatura               (0x03UL)
#define size_temperatura               (1UL)

// Pressure registers
#define reg_presion                   (0x00UL)
#define size_presion                   (1UL)


typedef struct  {

	int16_t volatile c0;
	int16_t volatile c1;
	int32_t volatile c10;
	int16_t volatile c20;
	int16_t volatile c30;
	int16_t volatile c11;
	int16_t volatile c01;
	int16_t volatile c21;
	int32_t volatile c00;
}Coefficients_struct;



/**
 * Calculate_temperature
 *
 * @brief 		This function calculate the final value for the temperature measured by the sensor
 *
 *
 * @param[in]   temperatura             Array that will store the temperature parameters
 * @param[out]  T_comp             		The final temperature
 * @param[in]   kT_personal             constant kT that depend on the oversampling rate (0 or NULL if you want to use standard)
 * @param[in]   standard                (True) if you want to use standard kT or (false) if you want to use another
 * @param[in]   p                       Struct with coefficients
 * @param[out]  T_raw					Needed to calculate Pressure
 *
 *
 * */
float Calculate_temperature(uint8_t *temperatura, uint32_t kT_personal, int standard, Coefficients_struct *p, float *T_raw);



/**
 * Calculate_pressure
 *
 * @brief 		This function calculate the final value for the pressure measured by the sensor.
 *
 *
 * @param[in]   presion	                Array that will store the pressure parameters
 * @param[out]  P_comp             		The final pressure
 * @param[in]   kP_personal             constant kP that depend on the oversampling rate (0 or NULL if you want to use standard)
 * @param[in]   standard                (True) if you want to use standard kP or (false) if you want to use another
 * @param[in]   p                       Struct with coefficients
 * @param[in]   T_raw					Needed to calculate Pressure
 *
 * */
float Calculate_pressure(uint8_t *presion, uint32_t kP_personal, int standard, Coefficients_struct *p, float T_raw);


/**
 * read_temperature_parameters
 *
 * @brief 		This function read the parameters measured by the sensor for the temperature.
 *
 * @param[in]   mI2C                    The I2C object
 * @param[in]   temperatura             Array that will store the temperature parameters
 *
 */
void read_temperature_parameters(cyhal_i2c_t *mI2C, uint8_t *temperatura);



/**
 * read_pressure_parameters
 *
 * @brief 		This function read the parameters measured by the sensor for the pressure.
 *
 * @param[in]   mI2C                    The I2C object
 * @param[in]   presion                 Array that will store the pressure parameters
 *
 */
void read_pressure_parameters(cyhal_i2c_t *mI2C, uint8_t *presion);



/**
 * init_some_pre_conf
 *
 * @brief 		This function initialize in the sensor in the register 28 if we use internal or external sensor for the temperature.
 * 				It has to match with the option that we write in the byte for the temperature configuration.
 *
 * @param[in]   mI2C                    The I2C object
 * @param[in]   external_sensor         Type bool that specify if the sensor is external (true) or internal (false)
 *
 */
void init_some_pre_conf(cyhal_i2c_t *mI2C, int external_sensor);


/**
 * configure_I2Cm
 *
 * @brief 		This function make possible the configuration of the I2C master
 *
 * @param[in]   mI2C                    The I2C object
 * @param[in]   mI2C_cfg                Configuration settings to apply
 *
 *
 */
void configure_I2Cm(cyhal_i2c_t *mI2C, cyhal_i2c_cfg_t *mI2C_cfg);


/**
 * write_configuration_reg
 *
 * @brief
 *
 * @param[in]   mI2C                    The I2C object
 * @param[in]   pressure_conf           byte with the parameters to configure in the sensor for the pressure (0 or NULL if you want to use standard)
 * @param[in]   temperature_conf        byte with the parameters to configure in the sensor for the temperature (0 or NULL if you want to use standard)
 * @param[in]   FyI_conf                byte with the parameters to configure in the sensor for the FIFO & Interrupts (0 or NULL if you want to use standard)
 * @param[in]   standard                boolean for using the standard configuration (true) or another configuration (false)
 *
 */
void write_configuration_reg(cyhal_i2c_t *mI2C, uint8_t pressure_conf, uint8_t temperature_conf, int standard, uint8_t FyI_conf);



/**
 * Get_coefficients
 *
 * @brief This function will read the coefficients from the registers and then it will calculate all the values.
 *
 * @param[in]   p                       Struct with coefficients
 * @param[in]   coeficientes            array where we store all the coefficients
 * @param[in]   mI2C                    The I2C object
 *
 */
void get_coefficients(Coefficients_struct *p, uint8_t *coeficientes, cyhal_i2c_t *mI2C);




/**
 * calculate_coefficients
 *
 * @brief This function calculates each coefficient from the array given.
 *
 * @param[in]   p                       Struct with coefficients
 * @param[in]   coeficientes            array where we store all the coefficients
 *
 *
 */
void calculate_coefficients(Coefficients_struct *p, uint8_t *coeficientes);





/**
 * getTowsComplement
 *
 * @brief This function gets the Complement 2 of a binary number given.
 *
 * @param[in]   raw                     Number which we want to calculate the complement 2
 * @param[in]   length                  Lenght of the number (in bits)
 *
 *
 */
void getTowsComplement(int32_t *raw, uint8_t length);


#endif
