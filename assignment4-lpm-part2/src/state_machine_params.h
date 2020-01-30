

typedef enum state{
	SENSOR_POWEROFF,
	SENSOR_WAITFORPOWERUP,
	I2C_WAITFORWRITECOMPLETE,
	I2C_WAITFORREADCOMPLETE,
}STATE_T;

// state machine flag
volatile STATE_T next_state;
volatile STATE_T current_state;


#define NO_EVENT					0
#define LETIMER0_UF_FLAG			(1<<0)
#define LETIMER0_COMP1_FLAG			(1<<1)
#define I2C_TRANSFER_COMPLETE		(1<<2)
#define	I2C_TRANSFER_ERROR			(1<<3)
// event flag
volatile uint8_t event;
