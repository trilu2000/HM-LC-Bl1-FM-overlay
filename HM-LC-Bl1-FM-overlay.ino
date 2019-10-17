//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2017-12-14 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------

// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER
#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <AskSinPP.h>
#include <LowPower.h>
#include <Blind.h>


// we use a Pro Mini, Arduino pin for the LED is D4 == PIN 4 on Pro Mini
#define LED_PIN 4
// Arduino pin for the config button D3 == PIN 3 on Pro Mini
#define BTN_CONFIG_PIN 3

#define BTN_HMUP___PIN 8
#define BTN_HMDOWN_PIN 9

#define KEY_START__PIN 7
#define KEY_OPENED_PIN 15
#define KEY_CLOSED_PIN 16
#define KEY_LEARN__PIN 17

#define PIN_RESET_PIN 14
#define PIN_DIR_R1_PIN 5
#define PIN_DIR_R2_PIN 6

// number of available peers per channel
#define PEERS_PER_CHANNEL 12


// all library classes are placed in the namespace 'as'
using namespace as;

// define all device properties
const struct DeviceInfo PROGMEM devinfo = {
	{0x93, 0xA5, 0xDC},				// Device ID
	"HB81267740",					// Device Serial
	{0x00,0x05},					// Device Model
	0x24,							// Firmware Version
	as::DeviceType::BlindActuator,	// Device Type
	{0x01,0x00}						// Info Bytes
};


/**
 * Configure the used hardware
 */
typedef AvrSPI<10, 11, 12, 13> RadioSPI;
typedef AskSin<StatusLed<LED_PIN>, NoBattery, Radio<RadioSPI, 2> > Hal;


/**
* Keybutton template to monitor and bebounce input pins,
* can also set port pins accordinly
*/
template <uint8_t OFFSTATE = HIGH, WiringPinMode WPMODE = INPUT_PULLUP>
class KeyButton : public Alarm {
#define DEBOUNCETIME millis2ticks(50)

protected:
	uint8_t  keymode;														// keybutton mode: 0 input, 1 output
	uint8_t  pinstate;														// holds the pin status
	uint8_t  pin;															// holds the pin number

public:
	KeyButton() : Alarm(0), keymode(0), pinstate(OFFSTATE), pin(0) {}
	~KeyButton() {}

	void irq() {															// called on pin interrupt
		if (keymode) return;												// only if we are in input mode
		sysclock.cancel(*this);												// use alarm to run code outside of interrupt
		tick = DEBOUNCETIME;												// set the debounce time
		sysclock.add(*this);												// calls check function without waiting time
	}

	void trigger(AlarmClock& clock) {										// debounce timer started by check is done
		uint8_t ps = digitalRead(pin);										// reads the pinstatus
		if (pinstate == ps) return;											// nothing to do while pin status is the same
		pinstate = ps;														// remember for next time
		keybutton_collector(pin, state());									// jump to external function
	}

	void init(uint8_t pin) {												// init function and handover of the pin to monitor
		this->pin = pin;													// remember on the pin
		set_input();														// set the pin to input
		sysclock.add(*this);												// run the timer to set the initial state
	}
	
	uint8_t state() {														// returns the current pin state
		uint8_t ps = (pinstate == OFFSTATE) ? 0 : 1;						// take care of the OFFSTATE flag
		return ps;
	}
	void write(uint8_t value) {												// stop the irq debounce and set the pin output
		set_pinmode(OUTPUT);												// set the pinmode to output												
		digitalWrite(this->pin, value);										// and write the byte
	}
	void set_input() {														// set the initial mode of the keybutton function														
		set_pinmode(WPMODE);													// set the pinmode to input
	}
	void set_pinmode(uint8_t pinmode) {										// set pinMode and remember on it
		if (keymode == pinmode) return;										// already done, leave
		keymode = pinmode;													// remember on the mode
		pinMode(this->pin, keymode);										// set the mode accordingly
	}
};

// declare keybutton instances 
KeyButton<HIGH, INPUT_PULLUP> key_opened;											
KeyButton<HIGH, INPUT_PULLUP> key_closed;
KeyButton<HIGH, INPUT_PULLUP> key_start;
KeyButton<HIGH, INPUT_PULLUP> key_learn;

KeyButton<LOW, INPUT_PULLUP> pin_dir_r1;
KeyButton<LOW, INPUT_PULLUP> pin_dir_r2;

/**
* interface and state machine to drive the motor up and down
*/
class Door_Move : public Alarm {
protected:
	uint8_t status;															// holds the status of the function, enabled or disabled
public:
	uint8_t direction;														// direction flag, 0 stop, 1 up, 2 down
	uint8_t sm_pos = 0;														// state machine flag, 0 none, 1 restart, 2 boot, 3 toogle
	uint8_t const timetable[4] = { 1, 5, 5, 0 };							// time table for the state machine steps

	Door_Move() : Alarm(0), status(false) {									// constructor
		async(true);
	}
	~Door_Move() {}

	uint8_t enabled() {														// returns if the function is enabled or not
		return status;
	}

	/* Start the state machine: 0 stop/reset, 1 up, 2 down	*/
	void start(uint8_t mode) {												// start the statemachine, 0 stop, 1 up, 2 down
		this->direction = mode;												// remind the mode
		this->status = true;												// remember the status
		sm_pos = 0;															// set statemachine to start while enabled
		sysclock.add(*this);												// add it to the clock
	}

	void trigger(__attribute__((unused)) AlarmClock& clock) {
		if (status == false) return;										// only needed if the function is enabled

		DPRINT(F("move ")); DPRINT(sm_pos); DPRINT(F(", "));				// some debug

		switch (sm_pos) {													// state machine position
		case 0:
			DPRINT(F("restart"));											// some debug
			if (direction == 1) key_closed.write(LOW);						// set direction key on base of the state machine
			else if (direction == 2) key_opened.write(LOW);
			digitalWrite(PIN_RESET_PIN, LOW);								// start the reset
			break;
		case 1:
			DPRINT(F("boot"));												// some debug 
			digitalWrite(PIN_RESET_PIN, HIGH);								// reset done, set it to working mode
			break;
		case 2:
			DPRINT(F("toogle"));											// some debug
			key_start.write(LOW);											// simulate a press of the start key 
			break;
		case 3:
			DPRINT(F("finish"));											// some debug
			digitalWrite(PIN_RESET_PIN, HIGH);								// in case we missed step 1
			key_opened.set_input();											// all key button functions back to default
			key_closed.set_input();
			key_start.set_input();
			break;
		}
		DPRINT(F("... - ")); DPRINT(millis()); 

		if (sm_pos >= 3) {													// check if up down activity is done
			DPRINT(F("\n"));												// debug show disabled
			status = false;													// nothing to do any more, set inactive
			return;															// leave the function
		}
		if ((direction == 0) && (sm_pos == 1)) sm_pos = 2;					// check if we are in reset only and shorten the remaining steps

		tick = decis2ticks(timetable[sm_pos++]);							// still active, get timeing
		clock.add(*this);													// start the timer again
		DPRINT(F(", ")); DPRINTLN(tick);									// debug print timetable		
	}

	void stop() {															// stop the function from outside
		sysclock.cancel(*this);												// in case a clock is running
		sm_pos = 3;															// position at cleanup
		trigger(sysclock);
	}
};

// declare door move, stop instance
Door_Move door_move;

/**
* State machine for the learn function, we measure the time for opening and closing the door
*/
class Door_Learn : public Alarm {
# define TIMEOUT 120														// timeout in seconds
protected:
	uint8_t status;															// remembers the status: enabled or disabled
	uint8_t sm_pos = 0;														// state machine position: 0 started, closed 1st, opened 1st, closed 2nd, opened 2nd
public:
	Door_Learn() : Alarm(0), status(0) {
		async(true);
	}
	~Door_Learn() {}

	uint8_t enabled() {														// returns if true or false for the learn function status
		return status;
	}

	/* triggers the state machine, input is to start the function
	* or the changed status of the door,
	* 0 start button pressed, 1 for door opened or 2 for door closed */
	void statemachine(uint8_t position) {

		DPRINT(F("learn, "));												// some debug

		switch (position) {													// process the input
		case 0:	// learn key was pushed
			DPRINT(F("started"));											// some debug
			sm_pos = 0;														// position the state machine
			status = true;													// set as active
			break;
		case 1:	// opened key pushed
			DPRINT(F("open "));												// some debug
			if ((sm_pos == 0) || (sm_pos == 1)) sm_pos = 2;					// check where we are and set the position accordingly
			else if (sm_pos == 3) sm_pos = 4;
			else sm_pos = 255;												// something went wrong, indicate error
			break;
		case 2:	// closed key pushed
			DPRINT(F("close "));											// some debug
			if (sm_pos == 0) sm_pos = 1;
			else if (sm_pos == 2) sm_pos = 3;
			else sm_pos = 255;
			break;
		}
		if (status == false) return;										// not active, nothing to do

		switch (sm_pos) {													// position the state machine
		case 1: // close 1st
			DPRINT(1);														// some debug only
			break;
		case 2: // open 1st 
			DPRINT(1);
			static uint32_t timeTopBottom = millis();						// start measurement for closing time
			break;
		case 3: // close 2nd
			DPRINT(2);
			timeTopBottom = (millis() - timeTopBottom) / 100;				// finish measurement for closing time
			static uint32_t timeBottomTop = millis();						// start measurement for opening time
			break;
		case 4: // open 2nd
			DPRINT(2);
			timeBottomTop = (millis() - timeBottomTop) / 100;				// finish measurement for opening time 
			learn_save_time(timeBottomTop, timeTopBottom);					// call external function

		case 255:
			status = false;
			break;
		}
		DPRINT(F(" - ")); DPRINTLN(millis());
		sysclock.cancel(*this);												// cancel the current timeout 
		tick = seconds2ticks(TIMEOUT);										// set a new timeout time
		sysclock.add(*this);												// and start it
	}

	void stop() {
		DPRINTLN(F("learn stop"));
		status = false;														// flag it as inactive
		sysclock.cancel(*this);												// stop timer
	}

	void trigger(__attribute__((unused)) AlarmClock& clock) {				// function used for timeout only
		DPRINTLN(F("learn timeout"));
		status = false;
		motor_stop();														// to stop the motor, because outtimed
	}
};

// declare door learn instance 
Door_Learn door_learn;


DEFREGISTER(BlindReg0,MASTERID_REGS,DREG_INTKEY,DREG_CONFBUTTONTIME,DREG_LOCALRESETDISABLE)

class BlindList0 : public RegList0<BlindReg0> {
public:
	BlindList0 (uint16_t addr) : RegList0<BlindReg0>(addr) {}
	void defaults () {
		clear();
		// intKeyVisible(false);
		confButtonTime(0xff);
		// localResetDisable(false);
	}
};

class BlChannel : public ActorChannel<Hal,BlindList1,BlindList3,PEERS_PER_CHANNEL,BlindList0,BlindStateMachine> {
public:
	typedef ActorChannel<Hal,BlindList1,BlindList3,PEERS_PER_CHANNEL,BlindList0,BlindStateMachine> BaseChannel;

	BlChannel ()  {}
	virtual ~BlChannel () {}

	virtual void switchState(uint8_t oldstate,uint8_t newstate, uint32_t stateDelay) {
		BaseChannel::switchState(oldstate, newstate, stateDelay);
		/*switch (newstate) {
		case AS_CM_JT_NONE:		DPRINTLN(F("AS_CM_JT_NONE")); break;
		case AS_CM_JT_ONDELAY:	DPRINTLN(F("AS_CM_JT_ONDELAY")); break;
		case AS_CM_JT_REFON:	DPRINTLN(F("AS_CM_JT_REFON")); break;
		case AS_CM_JT_ON:		DPRINTLN(F("AS_CM_JT_ON")); break;
		case AS_CM_JT_OFFDELAY:	DPRINTLN(F("AS_CM_JT_OFFDELAY")); break;
		case AS_CM_JT_REFOFF:	DPRINTLN(F("AS_CM_JT_REFOFF")); break;
		case AS_CM_JT_OFF:		DPRINTLN(F("AS_CM_JT_OFF")); break;
		case AS_CM_JT_RAMPON:	DPRINTLN(F("AS_CM_JT_RAMPON")); break;
		case AS_CM_JT_RAMPOFF:	DPRINTLN(F("AS_CM_JT_RAMPOFF")); break;
		}*/

		if( newstate == AS_CM_JT_RAMPON && stateDelay > 0 ) {
			motor_up();
		} else if( newstate == AS_CM_JT_RAMPOFF && stateDelay > 0 ) {
			motor_down();
		} else {
			motor_stop();
		}
	}	

	void init() {
		motor_stop();
		BaseChannel::init();
	}
};

// setup the device with channel type and number of channels
typedef MultiChannelDevice<Hal,BlChannel,1,BlindList0> BlindType;

Hal hal;
BlindType sdev(devinfo,0x20);
ConfigButton<BlindType>   key_config(sdev);
InternalButton<BlindType> key_hmup__(sdev, 1);
InternalButton<BlindType> key_hmdown(sdev,2);

enum enum_motor { stopped = 0, opening = 1, closing = 2 };					// enum for motor status
enum_motor motor;															// holds the status of the motor

void initPeerings (bool first) {
	// create internal peerings - CCU2 needs this
	if( first == true ) {
		sdev.channel(1).peer(key_hmup__.peer(), key_hmdown.peer());
	}
}

void setup () {
	DINIT(57600,ASKSIN_PLUS_PLUS_IDENTIFIER);
	//storage().setByte(0,0);
	bool first = sdev.init(hal);
	sdev.channel(1).init();

	pinMode(PIN_RESET_PIN, OUTPUT);
	digitalWrite(PIN_RESET_PIN, HIGH);

	buttonISR(key_config, BTN_CONFIG_PIN);									// irq bridge for the button templates
	buttonISR(key_hmup__, BTN_HMUP___PIN);
	buttonISR(key_hmdown, BTN_HMDOWN_PIN);

	buttonISR(key_opened, KEY_OPENED_PIN);
	buttonISR(key_closed, KEY_CLOSED_PIN);
	buttonISR(key_start,  KEY_START__PIN);
	buttonISR(key_learn,  KEY_LEARN__PIN);

	buttonISR(pin_dir_r1, PIN_DIR_R1_PIN);
	buttonISR(pin_dir_r2, PIN_DIR_R2_PIN);

	initPeerings(first);
	sdev.initDone();
}

void loop() {
	bool worked = hal.runready();
	bool poll = sdev.pollRadio();
	if (worked == false && poll == false) {
			//hal.activity.savePower<Idle<> >(hal);
	}
}


// -- user defined functions --------------------------------------------------------------------------
void keybutton_collector(uint8_t pin, uint8_t status) {
	if (pin == PIN_DIR_R1_PIN || pin == PIN_DIR_R2_PIN) {
		uint8_t t_r1 = digitalRead(PIN_DIR_R1_PIN);							// get the relais status
		uint8_t t_r2 = digitalRead(PIN_DIR_R2_PIN);
		motor = enum_motor::stopped;										// per default motor is stopped
		if ((t_r1 == 0) && (t_r2 == 1)) motor = enum_motor::opening;		// set motor accordingly
		if ((t_r1 == 1) && (t_r2 == 0)) motor = enum_motor::closing;

		DPRINT(F("motor, "));												// some debug print
		switch (motor) {
		case enum_motor::stopped:
			DPRINT(F("stopped")); break;
		case enum_motor::opening:
			DPRINT(F("opening")); break;
		case enum_motor::closing:
			DPRINT(F("closing")); break;
		}
		DPRINT(F(", ")); DPRINTLN(millis());													
	}

	// status from keys is only needed while pressed
	if (status != 1) return;
	switch (pin) {
	case KEY_OPENED_PIN:
		DPRINT(F("KEY_OPENED, ")); DPRINTLN(millis());
		door_learn.statemachine(1);
		sdev.channel(1).stop();
		sdev.channel(1).updateLevel(200);
		break;
	case KEY_CLOSED_PIN:
		DPRINT(F("KEY_CLOSED, ")); DPRINTLN(millis());
		door_learn.statemachine(2);
		sdev.channel(1).stop();
		sdev.channel(1).updateLevel(0);
		break;
	case KEY_START__PIN:
		DPRINT(F("KEY_START__PIN, ")); DPRINTLN(millis());
		break;
	case KEY_LEARN__PIN:
		DPRINT(F("KEY_LEARN__PIN, ")); DPRINTLN(millis());
		if (door_learn.enabled()) door_learn.stop();
		else door_learn.statemachine(0);
		break;
	}
}

void motor_up() {															// motor up function, triggered by the channel class
	DPRINT(F("motor_up - ")); DPRINTLN(millis());							// some debug
	if (key_opened.state() == 2) return;									// is open, nothing to do any more
	door_move.start(1);
}

void motor_down() {															// motor down function, triggered by the channel class
	DPRINT(F("motor_down - ")); DPRINTLN(millis());							// some debug
	if (key_closed.state() == 2) return;									// is closed, nothing to do any more
	door_move.start(2);
}

void motor_stop() {															// motor stop function, triggered by the channel class
	DPRINT(F("motor_stop - ")); DPRINTLN(millis());							// some debug
	door_move.stop();														// in case door_move class is still active
	if (!motor) return;														// only needed if motor is working
	door_move.start(0);														// stop the motor by a reset of the main board
}

void learn_save_time(uint32_t timeBottomTop, uint32_t timeTopBottom) {
	DPRINT(F(", timeTopBottom ")); DPRINT(timeTopBottom); DPRINT(F(", timeBottomTop ")); DPRINT(timeBottomTop);
	sdev.channel(1).getList1().refRunningTimeBottomTop(timeBottomTop);
	sdev.channel(1).getList1().refRunningTimeTopBottom(timeTopBottom);
}