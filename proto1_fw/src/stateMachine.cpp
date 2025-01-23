#include "stateMachine.h"
#include "monitor.h"
#include "rollerMotor.h"

#define TASK_PERIOD_MS 200U
#define STATE_BUTTON_PIN PA5
#define BUTTON_DEBOUNCE_MS (250U) // ms

#define SOLENOID_PIN PC9
#define CONVEYOR_STEP_PIN PA1
#define CONVEYOR_DIR_PIN PC1
#define CONVEYOR_EN_PIN PC0

State_E state = OFF_STATE;
State_E next_state = OFF_STATE;
State_E prev_state = OFF_STATE;
bool prevStateButton = true; // active low
uint32_t state_last_millis = 0U;

typedef struct {
    void (*enterState)(State_E prev_state);
    State_E (*runState)(void);
    void (*exitState)(State_E next_state);
} stateFunction_t;

// State functions, every state must have a run function
State_E runOffState(void);
void enterExtendedState(State_E prev_state);
State_E runExtendedState(void);
void enterRetractedState(State_E prev_state);
State_E runRetractedState(void);
void enterFaultState(State_E prev_state);
State_E runFaultState(void);

// Ensure all states get added to stateFunction
stateFunction_t stateFunction[STATE_COUNT] =
{
    [OFF_STATE] = {.enterState = NULL, .runState = &runOffState, .exitState = NULL},
    [EXTENDED_STATE] = {.enterState = &enterExtendedState, .runState = &runExtendedState, .exitState = NULL},
    [RETRACTED_STATE] = {.enterState = &enterRetractedState, .runState = &runRetractedState, .exitState = NULL},
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
    else if (currButton == true && buttonCount >=3U)
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

void spinConveyor(void)
{
    digitalWrite(CONVEYOR_EN_PIN, LOW);
    digitalWrite(CONVEYOR_DIR_PIN, LOW);
    analogWriteFrequency(500);
    analogWrite(CONVEYOR_STEP_PIN, 127); // Duty cycle does not matter, speed based on pwm freq
    analogWriteFrequency(PWM_FREQUENCY);
}

void stopConveyor(void)
{
    analogWrite(CONVEYOR_STEP_PIN, 0);
    digitalWrite(CONVEYOR_EN_PIN, HIGH);
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
        next = EXTENDED_STATE;
    }
    return next;
}

void enterExtendedState(State_E prev_state)
{
    setRollerMotorEnable(true);
    dropRoller();
}

State_E runExtendedState(void)
{
    State_E next = EXTENDED_STATE;
    if(getMonitorTripped())
    {
        next = FAULT_STATE;
    }
    else if(wasStateButtonPressed())
    {
        next = RETRACTED_STATE;
    }
    return next;
}

void enterRetractedState(State_E prev_state)
{
    raiseRoller();
}

State_E runRetractedState(void)
{
    State_E next = RETRACTED_STATE;
    if(getMonitorTripped())
    {
        next = FAULT_STATE;
    }
    else if(wasStateButtonPressed())
    {
        next = EXTENDED_STATE;
    }
    return next;
}

void enterFaultState(State_E prev_state)
{
    setRollerMotorEnable(false);
    stopConveyor();
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
    pinMode(CONVEYOR_STEP_PIN, OUTPUT);
    pinMode(CONVEYOR_DIR_PIN, OUTPUT);
    pinMode(CONVEYOR_EN_PIN, OUTPUT);
    dropRoller();
    stopConveyor();

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
