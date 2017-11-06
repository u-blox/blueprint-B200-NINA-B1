/*
 * Copyright (C) u-blox 
 * 
 * u-blox reserves all rights in this deliverable (documentation, software, etc.,
 * hereafter “Deliverable”). 
 * 
 * u-blox grants you the right to use, copy, modify and distribute the
 * Deliverable provided hereunder for any purpose without fee.
 * 
 * THIS DELIVERABLE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTY. IN PARTICULAR, NEITHER THE AUTHOR NOR U-BLOX MAKES ANY
 * REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY OF THIS
 * DELIVERABLE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
 * 
 * In case you provide us a feedback or make a contribution in the form of a
 * further development of the Deliverable (“Contribution”), u-blox will have the
 * same rights as granted to you, namely to use, copy, modify and distribute the
 * Contribution provided to us for any purpose without fee.
 */
 
/**
 * BMI160 REG BITS
 */
 
 #ifndef BMI160_REG_BITS_H__
 #define BMI160_REG_BITS_H__

//////////////////////////////////////////////////////////////////////
/// @name Includes
//@{


//@} End of Includes


//////////////////////////////////////////////////////////////////////
/// @name Defines
//@{


//@} End of Defines


//////////////////////////////////////////////////////////////////////
/// @name Typedefs
//@{
//@} End of Typedefs


//////////////////////////////////////////////////////////////////////
/// @name Data
//@{

/**@brief BMI160 Register Map
 *
 * 	 	+----------------------+------------------+ 										
 *   	| Register Name 	   | Register Address |
 *    	+----------------------+------------------+ 											
 */
 
 /**@brief Init SPI
 *
 * @details Need to do a single SPI read before using SPI commands. This will make the bmi160 to switch to SPI instead of I2C.
 */
#define INIT_SPI				0x7F		//Do a single SPI read before using SPI commands. This will make the bmi160 to switch to SPI instead of I2C.

/**@brief Command register
 *
 * @details Command register triggers operations like softreset, NVM programming, etc.
 *
 *				Status reports different modes:
 *				0b00 = Suspend
 *				0b01 = Normal
 *				0b10 = Low Power
 *				0b11 = Fast Start-Up
 *
 *			Set PMU mode of accelerometer to normal or low power:						0x11-0x12
 *			Set PMU mode of gyroscope to normal or fast start-up from suspend mode:		0x15;0x17
 *			Set PMU mode of magnetometer interface to suspend, normal or low-power:		0x18-0x1B	
 *
 * @cmd:
 *			Start fast offset calibration for the accel and gyro as configured in register (0x69)FOC_CONF
 *			and stores the result into the register (0x71-0x77)OFFSET register.
 * 			start_foc:	0x03
 *
 *			Sets the PMU mode for the acceleromter. The conding for 'nn' is identical to acc_pmu_status in
 *			register (0x03)PMU_STATUS.
 *			acc_set_pmu_mode:	0b0001 00nn
 *
 *			Sets the PMU mode for the gyroscope. The encoding for 'nn' is identical to gyr_pmu_status in
 *			register (0x03)PMU_STATUS.
 *			gyr_set_pmu_mode:	0b0001 01nn
 *
 *			Sets the PMU mode for the mag interface. The encoding for 'nn' is identical to mag_pmu_status in
 *			register (0x03)PMU_STATUS.
 *			mag_set_pmu_mode:	0b0001 10nn
 *
 *			Writes the NVM backed registers into NVM.
 *			prog_nvm:			0xA0
 *
 *			Clears all data in the FIFO, does not change the register(0x46-0x47)FIFO_CONFIG and register 
 *			(0x45)FIFO_DOWNS registers.
 *			fifo_flush:			0xB0
 *
 *			Resets the interrupt engine, the register(0x1C-0x1F)INT_STATUS and the interrupt pin.
 *			int_reset:			0xB1
 *
 *			Triggers a rest including a reboot. Other values are ignored. Following a delay, all user configuration
 *			settings are overwritten with their default state or the setting stored in the NVM, wherever applicable.
 *			This register is functional in all operation modes.
 *			softreset:			0xB6
 *
 *			Triggers a reset of the step counter. This register is functional in all operation modes.
 *			step_cnt_clr:		0xB2
 */
#define	CMD						0x7E

#define CMD_GYR_PMU_SUSPEND		0x14
#define CMD_GYR_PMU_NORMAL		0x15
#define	CMD_GYR_PMU_FAST_SU		0x17	
#define CMD_ACC_PMU_SUSPEND		0x10
#define CMD_ACC_PMU_NORMAL		0x11
#define CMD_ACC_PMU_LOW_PWR		0x12
#define CMD_SOFTRESET			0xB1

#define STEP_CONF_1				0x7B
#define	STEP_CONF_0				0x7A
#define STEP_CNT_1				0x79
#define	STEP_CNT_0				0x78

#define	OFFSET_6				0x77
#define	OFFSET_5				0x76
#define	OFFSET_4				0x75
#define	OFFSET_3				0x74
#define	OFFSET_2				0x73
#define	OFFSET_1				0x72
#define	OFFSET_0				0x71

#define	NV_CONF					0x70

#define	SELT_TEST				0x6D
#define	PMU_TRIGGER				0x6C
#define	IF_CONF					0x6B
#define	CONF					0x6A
#define	FOC_CONF				0x69

#define	INT_FLAT_1				0x68
#define	INT_FLAT_0				0x67

#define	INT_ORIENT_1			0x66
#define	INT_ORIENT_0			0x65

#define	INT_TAP_1				0x64
#define	INT_TAP_0				0x63

#define	INT_MOTION_3			0x62
#define	INT_MOTION_2			0x61
#define	INT_MOTION_1			0x60
#define	INT_MOTION_0			0x5F

#define	INT_LOWHIGH_4			0x5E
#define	INT_LOWHIGH_3			0x5D
#define	INT_LOWHIGH_2			0x5C
#define	INT_LOWHIGH_1			0x5B
#define	INT_LOWHIGH_0			0x5A

#define	INT_DATA_1				0x59
#define	INT_DATA_0				0x58

/**@brief Controls which interrupt signals are mapped to INT1 and INT2 pin.
 *
 * @details Default value: 0b0000-0000. 
 *			'1' means mapping is active. '0' means mapping is inactive.
 *
 * @bit[7]		flat				Flat
 * @bit[6]		Orientation			Orientation
 * @bit[5]		Single tap			Single tap
 * @bit[4]		Double tap			Double tap
 * @bit[3]		No motion			No motion
 * @bit[2]		Anymotion Z			Anymotion Z
 * @bit[1]		Anymotion Y			Anymotion Y
 * @bit[0]		Anymotion X			Anymotion X
 */
#define	INT_MAP_2				0x57

#define INT_MAP2_FLAT			0x80
#define	INT_MAP2_ORIENT			0x40
#define	INT_MAP2_SINGLE_TAP		0x20
#define	INT_MAP2_DOUBLE_TAP		0x10
#define	INT_MAP2_NO_MOTION		0x08
#define INT_MAP2_ANY_MOTION		0x04
#define	INT_MAP2_HIGH_G			0x02
#define	INT_MAP2_LOW_G			0x01

#define	INT_MAP_1				0x56
#define	INT_MAP_0				0x55
#define	INT_LATCH				0x54

/**@brief Contains the behavioral configuration (electrical definition of the interrupt pins).
 *
 * @details Default value: 0b0000-0000. 
 *			'1' means mapping is active. '0' means mapping is inactive.
 *
 * @bit[7]		int2_output_en		Output enable for INT2 pin, select '0'->output disabled, or '1'->output enabled
 * @bit[6]		int2_od				select '0'->push-pull, or '1'-> open drain behavior for INT2 pin. Only valid if int2_output_en = 1.
 * @bit[5]		int2_lv				'0'->active low, or '1'->active high level for INT2 pin. If int2_output_en = 1 this applies for interrupt
 *									outputs, if int2_output_en = 0 this applies to trigger PMU configured in Register (0x6C) PMU_TRIGGER.
 * @bit[4]		int2_edge_crtl		'1'('0') is edge (level) triggered for INT2 pin.
 * @bit[3]		int1_output_en		Output enable for INT1 pin, select '0'->output disabled, or '1'->output enabled.
 * @bit[2]		int1_od				select '0'->push-pull, or '1'-> open drain behavior for INT1 pin. Only valid if int1_output_en = 1.
 * @bit[1]		int1_lvl			'0'->active low, or '1'->active high level for INT1 pin. If int2_output_en = 1 this applies for interrupt
 *									outputs, if int1_output_en = 0 this applies to trigger PMU configured in Register (0x6C) PMU_TRIGGER.
 * @bit[0]		int1_edge_crtl		'1'('0') is edge (level) triggered for INT1 pin.
 */
#define	INT_OUT_CTRL			0x53

#define INT2_OUTPUT_EN			0x80
#define INT2_OD					0x40
#define	INT2_LVL				0x20
#define	INT2_EDGE_CRTL			0x10
#define INT1_OUTPUT_EN			0x08
#define INT1_OD					0x04
#define	INT1_LVL				0x02
#define	INT1_EDGE_CRTL			0x01

#define	INT_EN_2				0x52
#define	INT_EN_1				0x51

/**@brief Controls which interrupt engines are enabled.
 *
 * @details Default value: 0b0000-0000
 *
 * @bit[7]		int_flat_en			Flat interrupt
 * @bit[6]		int_orient_en		Orientation interrupt
 * @bit[5]		int_s_tap_en		Single tap interrupt
 * @bit[4]		int_d_tap_en		Double tap interrupt
 * @bit[3]		reserved
 * @bit[2]		int_anymo_z_en		Anymotion Z
 * @bit[1]		int_anymo_y_en		Anymotion Y
 * @bit[0]		int_anymo_x_en		Anymotion X
 */
#define	INT_EN_0				0x50

#define INT_FLAT_EN				0x80
#define	INT_ORIENT_EN			0x40
#define INT_S_TAP_EN			0x20
#define INT_D_TAP_EN			0x10
#define INT_ANYMO_Z_EN			0x04
#define INT_ANYMO_Y_EN			0x02
#define	INT_ANYMO_X_EN			0x01

/**@brief Register for indirect addressing of the magnetometer connected to the magnetometer interface.
 *
 * @MAG_IF[0]
 * @bit[7:1]	I2C device address
 *
 * @MAG_IF[1]
 * @bit[7]		mag_manual_en		Enable magnetometer register access on MAG_IF[2] OR MAG_IF[3].
 * @bit[6]		reserved
 * @bit[5:2]	mag_offset			Trigger-readout offset in units 2.5 ms. If set to 0, the offset is maximum.
 * @bit[1:0]	mag_rd_burst 		Data length of read burst operation, which reads out data from magnetometer.
 *
 * @MAG_IF[2]	Address to read
 * @MAG_IF[3]	Address to write
 * @MAG_IF[4]	Data to write
 */
#define	MAG_IF_4				0x4F
#define	MAG_IF_3				0x4E
#define	MAG_IF_2				0x4D

//MAG_IF[1]
#define	MAG_IF_1				0x4C

#define MAG_MANUAL_EN_MASK		0x80
#define MAG_OFFSET_MASK			0x3C
#define	MAG_RD_BURST_MASK		0x03

//MAG_IF[0]
#define	MAG_IF_0				0x4B

/**@brief The register (0x46 - 0x47) FIFO_CONFIG is a read/write register and can be used
 *		  for reading or setting the current FIFO watermark level. FIFO_CONFIG[1]
 *
 * @details	This register can also be used for setting the different modes of operation of the FIFO,
 *			e.g. which data is going to be stored in it and which format is going to be used (header or headerless mode).
 *
 * @bit[7]		fifo_gyr_en				'0' no gyro data are stored in FIFO. '1' gyro data are stored in FIFO.	
 * @bit[6]		fifo_acc_en				'0' no acc data are stored in FIFO. '1' acc data are stored in FIFO.
 * @bit[5]		fifo_header_en			'0' the frame format will be headerless. '1' each fram contains a header.	
 * @bit[4]		fifo_tag_int1_en		'0' disables FIFO tag (interrupt). '1' enables FIFO tag (interrupt)
 * @bit[3]		fifo_tag_int2_en		'0' disables FIFO tag (interrupt). '1' enables FIFO tag (interrupt)
 * @bit[2]		fifo_time_en			'0' does not return a sensortime frame. '1' returns a sensortime frame.
 */
#define	FIFO_CONFIG_1			0x47

#define FIFO_GYR_EN_MASK		0x80
#define	FIFO_ACC_EN_MASK		0x40
#define	FIFO_MAG_EN_MASK		0x20
#define	FIFO_HEADER_EN_MASK		0x10
#define	FIFO_TAG_INT1_EN_MASK	0x08
#define	FIFO_TAG_INT2_EN_MASK	0x04
#define	FIFO_TIME_EN_MASK		0x02

/**@brief The register (0x46 - 0x47) FIFO_CONFIG is a read/write register and can be used
 *		  for reading or setting the current FIFO watermark level. FIFO_CONFIG[0]
 *
 * @details	This register can also be used for setting the different modes of operation of the FIFO,
 *			e.g. which data is going to be stored in it and which format is going to be used (header or headerless mode).
 *
 * @bit[7:0]	fifo_water_mark		fifo_water_mark defines the FIFO watermark level. An interrupt will be generated, when the 
 *									number of entries in the FIFO exceeds fifo_water_mark. The unit of fifo_water_mark are 4 bytes.
 */
#define	FIFO_CONFIG_0			0x46

#define FIFO_WATER_MARK_MASK	0xFF

/**@brief Used to configure the down sampling ratios of the accel and gyro data for FIFO.
 *
 * @bit[7]		acc_fifo_filt_data
 * @bit[6:4]	acc_fifo_downs
 * @bit[3]		gyr_fifo_filt_data
 * @bit[2:0]	gyr_fifo_downs
 */
#define	FIFO_DOWNS				0x45

#define ACC_FIFO_FILT_DATA_MASK	0x80
#define ACC_FIFO_DOWN_MASK		0x70
#define	GYR_FIFO_FILT_DATA_MASK	0x08
#define	GYR_FIFO_DOWNS_MASK		0x07

/**@brief Sets the output data rate of the magnetometer interface in the sensor.
 *
 * @bit[3:0]	mag_odr			Define the poll rate for the magnetometer attached to the magnetometer interface.
 */
#define	MAG_CONF				0x44

#define	MAG_ODR_MASK			0x0F	/**< mag_odr */

/**@brief Defines the BMI160 angular rate measurement range
 *
 * @bit[2:0]	gyr_range			Angular rate range and resolution
 */
#define	GYR_RANGE				0x43

#define	GYR_RANGE_MASK			0x07	/**< gyr_range */

/**@brief Sets the output data rate, the bandwidth, and the read mode of the gyroscope in the sensor.
 *
 * @bit[5:4]	gyr_bwp				The gyroscope bandwidth coefficient defines the 3 dB cutoff frequency of the low pass filter
 *									for the sensor data.
 * @bit[3:0]	gyr_odr				Defines the output data rate of the gyro in the sensor.
 */
#define	GYR_CONF				0x42

#define	GYR_BWP_MASK			0x30	/**< gyr_bwp */
#define	GYR_ODR_MASK			0x0F	/**< gyr_odr */

/**@brief The register allows the selection of the accelerometer g-range.
 *
 * @details Changing the range of the accelerometer does not clear the data ready bit in the Register (0x1B) STATUS.
 *			It is recommended to read the Register (0x04-0x17) DATA after the range change to remove a stall data ready
 *			bit from before the range change.
 *
 * @bit[3:0]	acc_range
 */
#define	ACC_RANGE				0x41

#define ACC_RANGE_MASK			0x0F

/**@brief Sets the output data rate, the bandwidth, and the read mode of the acceleration sensor.
 *
 * @details 	acc_odr				Output data rate in Hz
 *				0b0000				Reserved
 *				0b0001				25/32
 *				...					
 *				0b1000				100
 *				0b1011				800
 *				0b1100				1600
 *				0b1101-0b1111		Reserved
 *
 * @bit[7]		acc_us				Undersampling parameter. The undersampling parameter is typically used in low power mode.
 * @bit[6:4]	acc_bwp				Bandwidth parameter determines filter configuration (acc_us = 0) and averaging for
 *									undersampling mode (acc_us = 1).
 * @bit[3:0]	acc_odr				Define the output data rate in Hz is given by 100/2^8-val(acc_odr). The output data rate
 * 									is independent of the power mode setting for the sensor.
 */
#define	ACC_CONF				0x40

#define	ACC_US_MASK				0x80	/**< acc_us */
#define ACC_BWP_MASK			0x70	/**< acc_bwp */
#define	ACC_ODR_MASK			0x08	/**< acc_odr */

/**@brief FIFO data readout register.
 */
#define	FIFO_DATA				0x24	/**< FIFO[7:0] */

/**@brief FIFO data readout register.
 *
 * @details The register contains FIFO status flags.
 */
#define	FIFO_LENGTH_1			0x23	
#define	FIFO_LENGTH_0			0x22 	

#define FIFO_LENGTH_1_MASK		0x07	/**< FIFO_byte_counter[10:8] */
#define FIFO_LENGTH_0_MASK		0xFF	/**< FIFO_byte_counter[7:0] */

/**@brief Contains the temperature of the sensor
 *
 * @details The temperature is disabled when all sensors are in supsend mode. The output word of the
 *			16-bit temperature sensor is valid if the gyroscope is in normal mode, i.e. gyr_pmu_status = 0b01.
 *			The resolution is typically 1/2^9 K/LSB. The absolute accuracy of the temperature is in order of:
 *			
 *			Value				Temperature
 *			0x7FFF				87 - 1/2^9 C
 *			...					...
 *			0x0000				23 C
 *			...					...
 *			0x8001				-41 + 1/2^9 C
 *			0x8000				Invalid
 */
#define	TEMPERATURE_1			0x21	/**< Temperature[15:8] */
#define	TEMPERATURE_0			0x20	/**< Temperature[7:0] */

/**@brief The register contains interrupt status flags, INT_STATUS_3.
 *
 * @bit[7]		flat					Device is in '1' = flat, or '0' = not flat position.
 *										Only valid if (0x16) int_flat_en = '1'
 *										'1' = negative.
 * @bit[6]		orient<2>				Orientation value of z-axis: '0' = upward looking, or
 *										'1' = downward looking. The flag always reflect the current orientation status,
 *										independent of the setting of [3:0]. The flas is not updated as long as an
 *										orientation blocking condition is active.
 * @bit[5:4]	orient<1:0>				Orientation value of xx-y-plane_:
 *										'00' = portrait upright,
 *										'01' = portrait upside down,
 *										'10' = landscape left,
 *										'11' = landscape right.
 *										The flag always reflect the current orientation status, independent of the setting
 *										of <3:0>. The flag is not updated as long as an orientation blocking condition is active.
 * @bit[3]		high_sign				Sign of acceleration signal that triggered high-g interrupt was
 *										'0' = positive,
 *										'1' = negative.
 * @bit[2]		high_first_z			High-g interrupt:
 *										'0' not triggered by, or
 *										'1' triggered by z-axis.
 * @bit[1]		high_first_y			High-g interrupt:
 *										'0' not triggered by, or
 *										'1' triggered by y-axis.
 * @bit[0]		high_first_x			High-g interrupt:
 *										'0' not triggered by, or
 *										'1' triggered by x-axis.
 */
#define	INT_STATUS_3			0x1F

#define FLAT_MASK				0x80	/**< flat */
#define ORIENT_Z_MASK			0x40	/**< orient<2> */
#define	ORIENT_X_Y_MASK			0x30	/**< orient<1:0> */
#define	HIGH_SIGN_MASK			0x08	/**< high_sign */
#define	HIGH_FIRST_Z_MASK		0x04	/**< high_first_z */
#define	HIGH_FIRST_Y_MASK		0x02	/**< high_first_y */
#define	HIGH_FIRST_X_MASK		0x01	/**< high_first_x */

/**@brief The register contains interrupt status flags, INT_STATUS_2.
 *
 * @bit[7]		tap_sign				sign of single/double tap triggering signal was
 *										'0' = positive,
 *										'1' = negative.
 * @bit[6]		tap_first_z				single/double tap interrupt:
 *										'0' not triggered by or,
 *										'1' triggered by z-axis.
 * @bit[5]		tap_frist_y				single/double tap interrupt:
 *										'0' not triggered by, or
 *										'1' triggered by y-axis.
 * @bit[4]		tap_first_x				single/double tap interrupt:
 *										'0' not triggered by, or
 *										'1' triggered by x-axis.
 * @bit[3]		anym_sign				slope sign of slope tap triggering signal was
 *										'0' = positive,
 *										'1' = negative.
 * @bit[2]		anym_first_z			Anymotion interrupt:
 *										'0' not triggered by, or
 *										'1' triggered by z-axis.
 * @bit[1]		anym_first_y			Anymotion interrupt:
 *										'0' not triggered by, or
 *										'1' triggered by y-axis.
 * @bit[0]		anym_first_x			Anymotion interrupt:
 *										'0' not triggered by, or
 *										'1' triggered by x-axis.
 */
#define	INT_STATUS_2			0x1E

#define TAP_SIGN_MASK			0x80	/**< tap_sign */
#define TAP_FIRST_Z_MASK		0x40	/**< tap_first_z */
#define TAP_FIRST_Y_MASK		0x20	/**< tap_frist_y */
#define TAP_FIRST_X_MASK		0x10	/**< tap_first_x */
#define ANYM_SIGN_MASK			0x08	/**< anym_sign */
#define	ANYM_FIRST_Z_MASK		0x04	/**< anym_first_z */
#define	ANYM_FIRST_Y_MASK		0x02	/**< anym_first_y */
#define	ANYM_FIRST_X_MASK		0x01	/**< anym_first_x */

/**@brief The register contains interrupt status flags, INT_STATUS_1.
 *
 * @details	'0' = interrupt inactive
 *			'1' = interrupt active
 *
 * @bit[7]		nomo_int				Nomotion
 * @bit[6]		fwm_int					FIFO watermark
 * @bit[5]		ffull_int				FIFO full
 * @bit[4]		drdy_int				Data Ready
 * @bit[3]		lowg_int				Low g
 * @bit[2]		highg_z_int				High g
 */
#define	INT_STATUS_1			0x1D

#define NOMO_INT_MASK			0x80	/**< nomo_int */
#define	FWM_INT_MASK			0x40	/**< fwm_int */
#define	FFULL_INT_MASK			0x20	/**< ffull_int */
#define	DRDY_INT_MASK			0x10	/**< drdy_int */
#define	LOWG_INT_MASK			0x08	/**< lowg_int */
#define	HIGHG_Z_INT_MASK		0x04	/**< highg_z_int */

/**@brief The register contains interrupt status flags, INT_STATUS_0.
 *
 * @details	'0' = interrupt inactive
 *			'1' = interrupt active
 *
 * @bit[7]		flat_int				Flat interrupt
 * @bit[6]		orient_int				Orientation interrupt
 * @bit[5]		s_tap_int				Single tap interrupt
 * @bit[4]		d_tap_int				Double tap interrupt
 * @bit[3]		pmu_trigger_int			pmu_trigger_interrupt
 * @bit[2]		anym_int				Anymotion
 * @bit[1]		sigmot_int				Signification motion interrupt
 * @bit[0]		step_int				Step detector interrupt
 */
#define	INT_STATUS_0			0x1C

#define FLAT_INT_MASK			0x80	/**< flat_int */
#define ORIENT_INT_MASK			0x40	/**< orient_int */
#define	S_TAP_INT_MASK			0x20	/**< s_tap_int */
#define	D_TAP_INT_MASK			0x10	/**< d_tap_int */
#define	PMU_TRIGGER_INT_MASK	0x08	/**< pmu_trigger_int */
#define	ANYM_INT_MASK			0x04	/**< anym_int */
#define	SIGMOT_INT_MASK			0x02	/**< sigmot_int */
#define	STEP_INT_MASK			0x01	/**< step_int */

/**@brief Reports sensor status flags
 *
 * @bit[7]		drdy_acc				Data ready (DRDY) for accelerometer in register.
 * @bit[6]		drdy_gyr				Data ready (DRDY) for gyroscope in register.
 * @bit[5]		drdy_mag				Data ready (DRDY) for magnetometer in register.
 * @bit[4]		nvm_rdy					NVM controller status.
 * @bit[3]		foc_rdy					FOC completed.
 * @bit[2]		mag_man_op				'0' indicates no manual magnetometer interface operation.
 *										'1' indicates a manual magnetmoeter interface operation triggered via MAG_IF[2] OR MAG_IF[3].
 * @bit[1]		gyr_self_test_ok		'0' when gyroscope self-test is running or failed.
 *										'1' when gyroscope self-test completed successfully.
 */
#define	REG_STATUS				0x1B	

#define	DRDY_ACC_MASK			0x80	/**< drdy_acc */
#define DRDY_GYR_MASK			0x40	/**< drdy_gyr */
#define	DRDY_MAG_MASK			0x20	/**< drdy_mag */
#define NVM_RDY_MASK			0x10	/**< nvm_rdy */
#define FOC_RDY_MASK			0x08	/**< foc_rdy */
#define MAG_MAN_OP_MASK			0x04	/**< mag_man_op */
#define GYR_SELF_TEST_OK_MASK	0x02	/**< gyr_self_test_ok */

/**@brief Sensortime is a 24 bit counter available in suspend, low power, and normal mode.
 */
 
/** @defgroup Accelerometer data
 * @{ */
#define	SENSORTIME_2			0x1A	/**< Sensor_time[23:16] */
#define	SENSORTIME_1			0x19	/**< Sensor_time[15:8] */
#define	SENSORTIME_0			0x18	/**< Sensor_time[7:0] */
/** @} */

/**@brief Register for accelerometer, gyroscope and magnetometer data.
 *
 * @details DATA[0-19] contains the latest data for the x,y, and z axis of magnemoeter, gyroscope and accelerometer.
 */
 
/** @defgroup Accelerometer data
 * @{ */
#define	DATA_19					0x17	/**< ACC_Z[15:8](MSB) */
#define	DATA_18					0x16	/**< ACC_Z[7:0] (LSB) */
#define	DATA_17					0x15	/**< ACC_Y[15:8](MSB) */
#define	DATA_16					0x14	/**< ACC_Y[7:0] (LSB) */
#define	DATA_15					0x13	/**< ACC_X[15:8](MSB) */
#define	DATA_14					0x12	/**< ACC_X[7:0] (LSB) */
/** @} */

/** @defgroup Gyroscope data
 * @{ */
#define	DATA_13					0x11	/**< GYR_Z[15:8](MSB) */
#define	DATA_12					0x10	/**< GYR_Z[7:0] (LSB) */
#define	DATA_11					0x0F	/**< GYR_Y[15:8](MSB) */
#define	DATA_10					0x0E	/**< GYR_Y[7:0] (LSB) */
#define	DATA_9					0x0D	/**< GYR_X[15:8](MSB) */
#define	DATA_8					0x0C 	/**< GYR_X[7:0] (LSB) */
/** @} */

/** @defgroup Hall data
 * @{ */
#define	DATA_7					0x0B	/**< RHALL[15:8](MSB) */
#define	DATA_6					0x0A	/**< RHALL[7:0] (LSB) */
/** @} */

/** @defgroup Magnetmoeter data
 * @{ */
#define	DATA_5					0x09	/**< MAG_Z[15:8](MSB) */
#define	DATA_4					0x08	/**< MAG_Z[0:8] (LSB) */
#define	DATA_3					0x07	/**< MAG_Y[15:8](MSB) */
#define	DATA_2					0x06	/**< MAG_Y[7:0] (LSB) */
#define	DATA_1					0x05	/**< MAG_X[15:8](MSB) */
#define	DATA_0					0x04	/**< MAG_X[7:0] (LSB) */
/** @} */

/**@brief Show the current power of the sensor
 *
 * @details Status reports different modes:
 *				0b00 = Suspend
 *				0b01 = Normal
 *				0b10 = Low Power
 *				0b11 = Fast Start-Up
 *
 * @bit[5:4]	acc_pmu_status			Accelerometer status.
 * @bit[3:2]	gyr_pmu_status			Gyro status.
 * @bit[1:0]	mag_pmu_status			Magnet Mode.
 */
#define PMU_STATUS				0x03

#define	ACC_PMU_STATUS_MASK		0x30	/**< acc_pmu_status */
#define GYR_PMU_STATUS_MASK		0x0C	/**< gyr_pmu_status */
#define MAG_PMU_STATUS_MASK		0x03	/**< mag_pmu_status */

/**@brief Reports sensor error flags. Flags are reset when read.
 *
 * @bit[6]		drop_cmd_err			Dropped command to register.
 * @bit[4:1]	error_code				@ref bmi160 datasheet
 * @bit[0]		fatal_err				Chip not operable
 */
#define	ERR_REG					0x02

#define DROP_CMD_ERR_MASK		0x40	/**< drop_cmd_err */
#define ERR_CODE_MASK			0x1E	/**< error_code */
#define	FATAL_ERR_MASK			0x01	/**< fatal_err */

/**@brief The register contains the chip identification code.
 *
 * @bit[7:0]	chip_id					Chip identification code.
 */
#define	CHIP_ID					0x00

//@} End of Data


//////////////////////////////////////////////////////////////////////
/// @name Private Methods
//@{
//@} End of Private Methods


//////////////////////////////////////////////////////////////////////
/// @name Public Methods
//@{
//@} End of Public Methods

#endif //BMI160_REG_BITS_H__

