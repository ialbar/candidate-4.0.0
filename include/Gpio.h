/*!
 * \file  Gpio.h
 * \brief Header file of the functions related to General Purpose Inputs/Outputs
 * ****************************************************************************/

 /* Functions to initilialize GPIOs */
extern void InitGpio(void);

/* GPIO macros */
#define YELLOW_STATUS_LED GpioDataRegs.GPBDAT.bit.GPIO57    /*!< Status yellow LED */
#define RED_STATUS_LED    GpioDataRegs.GPBDAT.bit.GPIO58    /*!< Status red LED */
#define YELLOW_CAN_LED    GpioDataRegs.GPBDAT.bit.GPIO59    /*!< CAN yellow LED */
#define RED_CAN_LED       GpioDataRegs.GPBDAT.bit.GPIO60    /*!< CAN red LED */
#define GP_LED_1          GpioDataRegs.GPBDAT.bit.GPIO61    /*!< General purpose LED 1 */
#define GP_LED_2          GpioDataRegs.GPBDAT.bit.GPIO62    /*!< General purpose LED 2 */

#define SET_YELLOW_STATUS_LED GpioDataRegs.GPBSET.bit.GPIO57 = 1
#define SET_RED_STATUS_LED    GpioDataRegs.GPBSET.bit.GPIO58 = 1
#define SET_YELLOW_CAN_LED    GpioDataRegs.GPBSET.bit.GPIO59 = 1
#define SET_RED_CAN_LED       GpioDataRegs.GPBSET.bit.GPIO60 = 1
#define SET_GP_LED_1          GpioDataRegs.GPBSET.bit.GPIO61 = 1
#define SET_GP_LED_2          GpioDataRegs.GPBSET.bit.GPIO62 = 1
#define CLEAR_YELLOW_STATUS_LED GpioDataRegs.GPBCLEAR.bit.GPIO57 = 1
#define CLEAR_RED_STATUS_LED    GpioDataRegs.GPBCLEAR.bit.GPIO58 = 1
#define CLEAR_YELLOW_CAN_LED    GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1
#define CLEAR_RED_CAN_LED       GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1
#define CLEAR_GP_LED_1          GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1
#define CLEAR_GP_LED_2          GpioDataRegs.GPBCLEAR.bit.GPIO62 = 1
#define TOGGLE_YELLOW_STATUS_LED GpioDataRegs.GPBTOGGLE.bit.GPIO57 = 1
#define TOGGLE_RED_STATUS_LED    GpioDataRegs.GPBTOGGLE.bit.GPIO58 = 1
#define TOGGLE_YELLOW_CAN_LED    GpioDataRegs.GPBTOGGLE.bit.GPIO59 = 1
#define TOGGLE_RED_CAN_LED       GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1
#define TOGGLE_GP_LED_1          GpioDataRegs.GPBTOGGLE.bit.GPIO61 = 1
#define TOGGLE_GP_LED_2          GpioDataRegs.GPBTOGGLE.bit.GPIO62 = 1

#define ID_SWITCH_CLOCK   GpioDataRegs.GPBDAT.bit.GPIO54    /*!< nodeID shift register CLOCK signal */
#define ID_SWITCH_DATA    GpioDataRegs.GPBDAT.bit.GPIO56    /*!< nodeID shift register DATA signal */
#define ID_SWITCH_SH_LDn  GpioDataRegs.GPBDAT.bit.GPIO55    /*!< nodeID shift register SHIFT/LOADn signal */

#define SET_ID_SWITCH_CLOCK  GpioDataRegs.GPBSET.bit.GPIO54 = 1
#define SET_ID_SWITCH_DATA   GpioDataRegs.GPBSET.bit.GPIO56 = 1
#define SET_ID_SWITCH_SH_LDn GpioDataRegs.GPBSET.bit.GPIO55 = 1
#define CLEAR_ID_SWITCH_CLOCK  GpioDataRegs.GPBCLEAR.bit.GPIO54 = 1
#define CLEAR_ID_SWITCH_DATA   GpioDataRegs.GPBCLEAR.bit.GPIO56 = 1
#define CLEAR_ID_SWITCH_SH_LDn GpioDataRegs.GPBCLEAR.bit.GPIO55 = 1
#define TOGGLE_ID_SWITCH_CLOCK  GpioDataRegs.GPBTOGGLE.bit.GPIO54 = 1
#define TOGGLE_ID_SWITCH_DATA   GpioDataRegs.GPBTOGGLE.bit.GPIO56 = 1
#define TOGGLE_ID_SWITCH_SH_LDn GpioDataRegs.GPBTOGGLE.bit.GPIO55 = 1

#define GPI1 (!GpioDataRegs.GPADAT.bit.GPIO22)
#define GPI2 (!GpioDataRegs.GPADAT.bit.GPIO27)
#define GPI3 (!GpioDataRegs.GPBDAT.bit.GPIO35)
#define GPI4 (!GpioDataRegs.GPADAT.bit.GPIO30)

#define GPO1 GpioDataRegs.GPBDAT.bit.GPIO34
#define GPO2 GpioDataRegs.GPADAT.bit.GPIO13
#define GPO3 GpioDataRegs.GPADAT.bit.GPIO14
#define GPO4 GpioDataRegs.GPADAT.bit.GPIO15

#define SET_GPO1 GpioDataRegs.GPBSET.bit.GPIO34 = 1
#define SET_GPO2 GpioDataRegs.GPASET.bit.GPIO13 = 1
#define SET_GPO3 GpioDataRegs.GPASET.bit.GPIO14 = 1
#define SET_GPO4 GpioDataRegs.GPASET.bit.GPIO15 = 1
#define CLEAR_GPO1 GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1
#define CLEAR_GPO2 GpioDataRegs.GPACLEAR.bit.GPIO13 = 1
#define CLEAR_GPO3 GpioDataRegs.GPACLEAR.bit.GPIO14 = 1
#define CLEAR_GPO4 GpioDataRegs.GPACLEAR.bit.GPIO15 = 1
#define TOGGLE_GPO1 GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1
#define TOGGLE_GPO2 GpioDataRegs.GPATOGGLE.bit.GPIO13 = 1
#define TOGGLE_GPO3 GpioDataRegs.GPATOGGLE.bit.GPIO14 = 1
#define TOGGLE_GPO4 GpioDataRegs.GPATOGGLE.bit.GPIO15 = 1

#define GPIO_I1 (!GpioDataRegs.GPADAT.bit.GPIO9)
#define GPIO_I2 (!GpioDataRegs.GPADAT.bit.GPIO16)
#define GPIO_O1 GpioDataRegs.GPADAT.bit.GPIO8
#define GPIO_O2 GpioDataRegs.GPADAT.bit.GPIO17

#define GPIO_I1_word GpioDataRegs.GPADAT.all
#define GPIO_I1_bit  9
#define GPIO_I2_word GpioDataRegs.GPADAT.all
#define GPIO_I2_bit 16

#define SET_GPIO_O1 GpioDataRegs.GPASET.bit.GPIO8  = 1
#define SET_GPIO_O2 GpioDataRegs.GPASET.bit.GPIO17 = 1
#define CLEAR_GPIO_O1 GpioDataRegs.GPACLEAR.bit.GPIO8  = 1
#define CLEAR_GPIO_O2 GpioDataRegs.GPACLEAR.bit.GPIO17 = 1
#define TOGGLE_GPIO_O1 GpioDataRegs.GPATOGGLE.bit.GPIO8  = 1
#define TOGGLE_GPIO_O2 GpioDataRegs.GPATOGGLE.bit.GPIO17 = 1

#define HALL1 GpioDataRegs.GPBDAT.bit.GPIO50		/*!< Hall effect sensor 1 */
#define HALL2 GpioDataRegs.GPBDAT.bit.GPIO51		/*!< Hall effect sensor 2 */
#define HALL3 GpioDataRegs.GPBDAT.bit.GPIO52		/*!< Hall effect sensor 3 */

#define SAFETY 1//GpioDataRegs.GPBDAT.bit.GPIO53		/*!< Safety signal */
#define SAFETY_ALLOW 1
#define SAFETY_DENY  0

#define OVERCURRENT GpioDataRegs.GPADAT.bit.GPIO12		/*!< Overcurrent signal */
#define OVERCURRENT_OK     1
#define OVERCURRENT_ACTIVE 0

//#define BRAKE2_INHIBIT_SAFETY        GpioDataRegs.GPADAT.bit.GPIO30
//#define SET_BRAKE2_INHIBIT_SAFETY    GpioDataRegs.GPASET.bit.GPIO30 = 1
//#define CLEAR_BRAKE2_INHIBIT_SAFETY  GpioDataRegs.GPACLEAR.bit.GPIO30 = 1
//#define TOGGLE_BRAKE2_INHIBIT_SAFETY GpioDataRegs.GPATOGGLE.bit.GPIO30 = 1

#define RESOLVER_PWM_PIN GpioDataRegs.GPADAT.bit.GPIO6  /*!< output pin of the PWM that generates the resolver excitation */
