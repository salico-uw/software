#include "stateMachine.h"
#include "monitor.h"
#include "rollerMotor.h"

#define TASK_PERIOD_MS 200U
#define ENCODER_BUTTON_PIN PC5
#define BUTTON_DEBOUNCE_MS (200U) // ms

#define SOLENOID_PIN PC9
#define CONVEYOR_STEP_PIN PA6
#define CONVEYOR_DIR_PIN PC1
#define CONVEYOR_EN_PIN PC0

State_E state = OFF_STATE;
State_E next_state = OFF_STATE;
State_E prev_state = OFF_STATE;
bool prevButton = true; // active low
uint32_t last_millis = 0U;

typedef struct {
    void (*enterState)(State_E prev_state);
    State_E (*runState)(void);
    void (*exitState)(State_E next_state);
} stateFunction_t;

// State functions, every state must have a run function
State_E runOffState(void);
void exitOffState(State_E next_state);
State_E runSpeedState(void);
State_E runCurrentState(void);
void enterFaultState(State_E prev_state);
State_E runFaultState(void);

// Ensure all states get added to stateFunction
stateFunction_t stateFunction[STATE_COUNT] =
{
    [OFF_STATE] = {.enterState = NULL, .runState = &runOffState, .exitState = exitOffState},
    [SPEED_STATE] = {.enterState = NULL, .runState = &runSpeedState, .exitState = NULL},
    [CURRENT_STATE] = {.enterState = NULL, .runState = &runCurrentState, .exitState = NULL},
    [FAULT_STATE] = {.enterState = enterFaultState, .runState = &runFaultState, .exitState = NULL},
};

bool wasButtonPressed(){
  bool pressed = false;
  // handle overflow case
  if (millis() < last_millis)
  {
    last_millis = millis();
  }
  bool currButton = digitalRead(ENCODER_BUTTON_PIN);
  if(currButton == false && prevButton == true && (millis() - last_millis) > BUTTON_DEBOUNCE_MS)
  {
    pressed = true;
    last_millis = millis();
  }
  prevButton = currButton;
  return pressed;
}

void extendPistons(void)
{
    digitalWrite(SOLENOID_PIN, HIGH);
}

void retractPistons(void)
{
    digitalWrite(SOLENOID_PIN, LOW);
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
    else if(wasButtonPressed())
    {
        next = SPEED_STATE;
    }
    return next;
}

void exitOffState(State_E next_state)
{
    setRollerMotorEnable(true);
}

State_E runSpeedState(void)
{
    State_E next = SPEED_STATE;
    if(getMonitorTripped())
    {
        next = FAULT_STATE;
    }
    else if(wasButtonPressed())
    {
        next = CURRENT_STATE;
    }
    return next;
}

State_E runCurrentState(void)
{
    State_E next = CURRENT_STATE;
    if(getMonitorTripped())
    {
        next = FAULT_STATE;
    }
    else if(wasButtonPressed())
    {
        next = SPEED_STATE;
    }
    return next;
}

void enterFaultState(State_E prev_state)
{
    setRollerMotorEnable(false);
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
    pinMode(ENCODER_BUTTON_PIN, INPUT_PULLUP);
    pinMode(SOLENOID_PIN, OUTPUT);
    pinMode(CONVEYOR_STEP_PIN, OUTPUT);
    pinMode(CONVEYOR_DIR_PIN, OUTPUT);
    pinMode(CONVEYOR_EN_PIN, OUTPUT);
    extendPistons();
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
