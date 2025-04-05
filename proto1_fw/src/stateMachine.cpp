#include "stateMachine.h"
#include "monitor.h"
#include "rollerMotor.h"
#include "distanceSensor.h"

#define TASK_PERIOD_MS 200U
#define STATE_BUTTON_PIN PA5
#define BUTTON_DEBOUNCE_MS (250U) // ms
#define EXTEND_TIMEOUT_MS (8000U)
#define RETRACT_TIMEOUT_MS (5000U)
#define FAN_SPIN_TIME_MS (5000U) // ms
#define NUMBER_OF_DISTANCE_INTERVALS (3U)

#define RAISED_HEIGHT_THRESHOLD_MM (40U)

#define SOLENOID_PIN PC9
#define FAN_EN_PIN PC0

static State_E state = OFF_STATE;
static State_E next_state = OFF_STATE;
static State_E prev_state = OFF_STATE;
static bool prevStateButton = true; // active low
static uint32_t state_last_millis = 0U;
static uint32_t fan_start_time_ms = 0U;
static uint32_t extend_start_time_ms = 0U;
static uint32_t retract_start_time_ms = 0U;
static const float picking_dist_intervals[NUMBER_OF_DISTANCE_INTERVALS] = {60, 80, 90}; // in mm
static uint8_t curr_dist_limit_idx = 0U;

typedef struct {
    void (*enterState)(State_E prev_state);
    State_E (*runState)(void);
    void (*exitState)(State_E next_state);
} stateFunction_t;

// State functions, every state must have a run function
void enterOffState(State_E prev_state);
State_E runOffState(void);
void enterRetractedIdleState(State_E prev_state);
State_E runRetractedIdleState(void);
void enterExtendingRollingState(State_E prev_state);
State_E runExtendingRollingState(void);
void enterRetractingNoRollingState(State_E prev_state);
State_E runRetractingNoRollingState(void);
void enterRetractedRollingState(State_E prev_state);
State_E runRetractedRollingState(void);
void enterFaultState(State_E prev_state);
State_E runFaultState(void);

// Ensure all states get added to stateFunction
stateFunction_t stateFunction[STATE_COUNT] =
{
    [OFF_STATE] = {.enterState = &enterOffState, .runState = &runOffState, .exitState = NULL},
    [RETRACTED_IDLE_STATE] = {.enterState = &enterRetractedIdleState, .runState = &runRetractedIdleState, .exitState = NULL},
    [EXTENDING_ROLLING_STATE] = {.enterState = &enterExtendingRollingState, .runState = &runExtendingRollingState, .exitState = NULL},
    [RETRACTING_NO_ROLLING_STATE] = {.enterState = &enterRetractingNoRollingState, .runState = &runRetractingNoRollingState, .exitState = NULL},
    [RETRACTED_ROLLING_STATE] = {.enterState = &enterRetractedRollingState, .runState = &runRetractedRollingState, .exitState = NULL},
    [FAULT_STATE] = {.enterState = &enterFaultState, .runState = &runFaultState, .exitState = NULL},
};

uint8_t buttonCount = 0U;
bool wasStateButtonPressed(){
    bool pressed = false;
    // handle overflow case
    if (millis() < state_last_millis)
    {
        state_last_millis = millis();
    }
    bool currButton = digitalRead(STATE_BUTTON_PIN);

    // More robust button press detection to prevent false triggers
    // Minimum button hold time of 3*200ms before releasing to trigger a press
    if(currButton == false && buttonCount < 250U)
    {
        buttonCount++;
    }
    else if (currButton == true && buttonCount >= 3U)
    {
        pressed = true;
        buttonCount = 0U;
    }
    else
    {
        buttonCount = 0U;
    }

    // if(currButton == false && prevStateButton == true && (millis() - state_last_millis) > BUTTON_DEBOUNCE_MS)
    // {
    //     pressed = true;
    //     state_last_millis = millis();
    // }
    // prevStateButton = currButton;
    return pressed;
}

void dropRoller(void)
{
    digitalWrite(SOLENOID_PIN, LOW);
}

void raiseRoller(void)
{
    digitalWrite(SOLENOID_PIN, HIGH);
}

void spinFans(void)
{
    digitalWrite(FAN_EN_PIN, HIGH);
}

void stopFans(void)
{
    digitalWrite(FAN_EN_PIN, LOW);
}

void enterOffState(State_E prev_state)
{
    setRollerMotorEnable(false);
    dropRoller();
}

State_E runOffState(void)
{
    State_E next = OFF_STATE;
    if(getMonitorTripped())
    {
        next = FAULT_STATE;
    }
    else if(wasStateButtonPressed())
    {
        next = RETRACTED_IDLE_STATE;
    }
    return next;
}

void enterRetractedIdleState(State_E prev_state)
{
    setRollerMotorEnable(false);
    raiseRoller();
    curr_dist_limit_idx = 0U; // reset auto-drop distances
}

State_E runRetractedIdleState(void)
{
    State_E next = RETRACTED_IDLE_STATE;
    if(getMonitorTripped())
    {
        next = FAULT_STATE;
    }
    else if(wasStateButtonPressed())
    {
        next = EXTENDING_ROLLING_STATE;
    }
    return next;
}

void enterExtendingRollingState(State_E prev_state)
{
    setRollerMotorEnable(true);
    dropRoller();
    extend_start_time_ms = millis();
}

State_E runExtendingRollingState(void)
{
    State_E next = EXTENDING_ROLLING_STATE;
    if(getMonitorTripped())
    {
        next = FAULT_STATE;
    }
    else if ((getDistanceSensorHealthy() && (getDistanceMM() > picking_dist_intervals[curr_dist_limit_idx])) || (millis() - extend_start_time_ms) > EXTEND_TIMEOUT_MS)
    {
        curr_dist_limit_idx++;
        next = RETRACTING_NO_ROLLING_STATE;
    }
    else if(wasStateButtonPressed())
    {
        // Abort auto-drop sequence on button press
        next = OFF_STATE;
    }
    return next;
}

void enterRetractingNoRollingState(State_E prev_state)
{
    setRollerMotorEnable(false);
    raiseRoller();
    retract_start_time_ms = millis();
}

State_E runRetractingNoRollingState(void)
{
    State_E next = RETRACTING_NO_ROLLING_STATE;
    if(getMonitorTripped())
    {
        next = FAULT_STATE;
    }
    else if((getDistanceMM() < RAISED_HEIGHT_THRESHOLD_MM) || ((millis() - retract_start_time_ms) > RETRACT_TIMEOUT_MS))
    {
        next = RETRACTED_ROLLING_STATE;
    }

    return next;
}

void enterRetractedRollingState(State_E prev_state)
{
    setRollerMotorEnable(true);
    spinFans();
    fan_start_time_ms = millis();
}

State_E runRetractedRollingState(void)
{
    State_E next = RETRACTED_ROLLING_STATE;
    if(getMonitorTripped())
    {
        next = FAULT_STATE;
    }
    else if((millis() - fan_start_time_ms) > FAN_SPIN_TIME_MS)
    {
        stopFans();
        // if auto-drop sequence is not complete, keep dropping
        if(curr_dist_limit_idx < NUMBER_OF_DISTANCE_INTERVALS)
        {
            next = EXTENDING_ROLLING_STATE;
        }
        else
        {
            next = RETRACTED_IDLE_STATE;
        }
    }

    return next;
}

void enterFaultState(State_E prev_state)
{
    setRollerMotorEnable(false);
    stopFans();
    dropRoller();
    Serial.print("FAULTED bits: ");
    Serial.println(getMonitorTripBits(), BIN);
}

State_E runFaultState(void)
{
    State_E next = FAULT_STATE;
    // Cannot exit fault state unless power reset
    return next;
}

static void TaskStateMachine(void *pvParameters)
{
    (void) pvParameters;

const TickType_t xDelay = TASK_PERIOD_MS / portTICK_PERIOD_MS;
    // Setup
    pinMode(STATE_BUTTON_PIN, INPUT_PULLUP);
    pinMode(SOLENOID_PIN, OUTPUT);
    pinMode(FAN_EN_PIN, OUTPUT);
    dropRoller();
    stopFans();

    // Loop
    while (1)
    {
        if (prev_state != state && stateFunction[state].enterState != NULL)
        {
            stateFunction[state].enterState(prev_state);
        }

        // Run state and get next state
        next_state = stateFunction[state].runState();

        if (next_state != state && stateFunction[state].exitState != NULL)
        {
            stateFunction[state].exitState(next_state);
        }
        prev_state = state;
        state = next_state;

        vTaskDelay(xDelay);
    }
}

// Public functions
void initStateMachineTask(UBaseType_t priority)
{
    xTaskCreate(
    TaskStateMachine
    ,  (const portCHAR *)"State Machine"
    ,  256
    ,  NULL
    ,  priority
    ,  NULL );
}

State_E getState(void)
{
    return state;
}
