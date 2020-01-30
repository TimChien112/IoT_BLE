#include <stdint.h>

typedef enum state{
	SENSOR_POWEROFF,
	SENSOR_WAITFORPOWERUP,
	I2C_WAITFORWRITECOMPLETE,
	I2C_WAITFORREADCOMPLETE,
	STATE_STOP,
}STATE_T;

// state machine flag
volatile STATE_T next_state;
volatile STATE_T current_state;
volatile uint8_t connection_flag;



#define NO_EVENT					0
#define LETIMER0_UF_FLAG			(1<<0)
#define LETIMER0_COMP1_FLAG			(1<<1)
#define I2C_TRANSFER_COMPLETE		(1<<2)
#define	I2C_TRANSFER_ERROR			(1<<3)
// event flag
volatile uint8_t event;
int state_stop_flag;
#define TEMPREAD_FLAG				(1<<3)
#define	BLE_CONNECT					(1<<1)
#define	BLE_DISCONNECT				(1<<0)

void state_machine();
