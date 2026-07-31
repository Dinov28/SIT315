const byte DOOR_PIN = 8;
const byte WINDOW_PIN = 9;
const byte MOTION_PIN = 2;
const byte ALARM_LED_PIN = 5;
const byte HEARTBEAT_LED_PIN = 13;

const unsigned long DEBOUNCE_TIME_MS = 40;

enum SystemState {
  SAFE,
  WARNING,
  ALARM
};

SystemState currentState = SAFE;
SystemState previousState = SAFE;

// Values shared with the D2 external interrupt
volatile bool motionInterruptFlag = false;

// Values shared with the D8/D9 pin-change interrupt.
volatile bool doorInterruptFlag = false;
volatile bool windowInterruptFlag = false;
volatile bool capturedDoorState = false;
volatile bool capturedWindowState = false;
volatile byte previousPortBState = 0;

// Values shared with the Timer1 interrupt.
volatile bool timerInterruptFlag = false;
volatile unsigned long timerTickCount = 0;

// Stable sensor states used by the main program.
bool doorActive = false;
bool windowActive = false;

// Debounce state for D8 and D9.
bool doorDebouncePending = false;
bool windowDebouncePending = false;
bool doorCandidateState = false;
bool windowCandidateState = false;
unsigned long doorCandidateTime = 0;
unsigned long windowCandidateTime = 0;

// Debounce state for the D2 external interrupt.
bool motionAcceptedBefore = false;
unsigned long lastMotionAcceptedTime = 0;

unsigned long doorEventCount = 0;
unsigned long windowEventCount = 0;
unsigned long motionEventCount = 0;

bool heartbeatState = false;

void configureExternalInterrupt();
void configurePinChangeInterrupts();
void configureTimer1();
void handleMotionInterrupt();
void collectPinChangeEvents();
void applyDebouncedSensorStates();
void handleTimerEvent();
void thinkAboutState();
void actOnState();
void motionISR();

void setup()
{
  Serial.begin(9600);

  pinMode(DOOR_PIN, INPUT);
  pinMode(WINDOW_PIN, INPUT);
  pinMode(MOTION_PIN, INPUT);

  pinMode(ALARM_LED_PIN, OUTPUT);
  pinMode(HEARTBEAT_LED_PIN, OUTPUT);

  digitalWrite(ALARM_LED_PIN, LOW);
  digitalWrite(HEARTBEAT_LED_PIN, LOW);

  // Read the starting input conditions.
  doorActive = digitalRead(DOOR_PIN) == HIGH;
  windowActive = digitalRead(WINDOW_PIN) == HIGH;

  configureExternalInterrupt();
  configurePinChangeInterrupts();
  configureTimer1();

  Serial.println(" ");
  Serial.println("Interrupt-Driven Security System");
  Serial.println("External interrupt + PCINT + Timer1");
  Serial.println(" "); 
  Serial.println("[STATE] SAFE");
}

void loop()
{
  //SENSE
  handleMotionInterrupt();
  collectPinChangeEvents();
  applyDebouncedSensorStates();
  handleTimerEvent();

  //Think
  thinkAboutState();

  //act
  actOnState();
}

void configureExternalInterrupt()
{
  // The pull-down circuit changes from LOW to HIGH when pressed
  attachInterrupt(
    digitalPinToInterrupt(MOTION_PIN),
    motionISR,
    RISING
  );
}

void configurePinChangeInterrupts()
{
  noInterrupts();

  // Store the initial state
  previousPortBState = PINB;

  // D8 is PCINT0/PB0 and D9 is PCINT1/PB1.
  PCMSK0 |= bit(PCINT0);
  PCMSK0 |= bit(PCINT1);

  // Clear an old pending request before enabling the group.
  PCIFR = bit(PCIF0);

  // Enable the PORTB pin-change interrupt group.
  PCICR |= bit(PCIE0);

  interrupts();
}

void configureTimer1()
{
  noInterrupts();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  OCR1A = 15624;

  TCCR1B |= bit(WGM12);

  TCCR1B |= bit(CS12);
  TCCR1B |= bit(CS10);

  // Enable the Timer1 Compare A interrupt
  TIMSK1 |= bit(OCIE1A);

  interrupts();
}

// D2 external interrupt, record the event and return immediately
void motionISR()
{
  motionInterruptFlag = true;
}

// D8 and D9 share this pin-change interrupt vecto
ISR(PCINT0_vect)
{
  byte currentPortBState = PINB;
  byte changedPins = currentPortBState ^ previousPortBState;

  previousPortBState = currentPortBState;

  if (changedPins & bit(PB0)) {
    capturedDoorState = (currentPortBState & bit(PB0)) != 0;
    doorInterruptFlag = true;
  }

  if (changedPins & bit(PB1)) {
    capturedWindowState = (currentPortBState & bit(PB1)) != 0;
    windowInterruptFlag = true;
  }
}

// Timer1 interrupt, record the tick and return immediately.
ISR(TIMER1_COMPA_vect)
{
  timerTickCount++;
  timerInterruptFlag = true;
}

void handleMotionInterrupt()
{
  bool motionEvent;

  noInterrupts();
  motionEvent = motionInterruptFlag;
  motionInterruptFlag = false;
  interrupts();

  if (!motionEvent) {
    return;
  }

  unsigned long currentTime = millis();
  bool debounceFinished =
    !motionAcceptedBefore ||
    currentTime - lastMotionAcceptedTime >= DEBOUNCE_TIME_MS;

  if (debounceFinished) {
    motionAcceptedBefore = true;
    lastMotionAcceptedTime = currentTime;
    motionEventCount++;

    Serial.print("[EXTINT] Motion event #");
    Serial.print(motionEventCount);
    Serial.println(" detected on D2");
  }
}

void collectPinChangeEvents()
{
  bool doorEvent;
  bool windowEvent;
  bool newDoorState;
  bool newWindowState;

  noInterrupts();

  doorEvent = doorInterruptFlag;
  windowEvent = windowInterruptFlag;
  newDoorState = capturedDoorState;
  newWindowState = capturedWindowState;

  doorInterruptFlag = false;
  windowInterruptFlag = false;

  interrupts();

  unsigned long currentTime = millis();

  if (doorEvent) {
    doorCandidateState = newDoorState;
    doorCandidateTime = currentTime;
    doorDebouncePending = true;
  }

  if (windowEvent) {
    windowCandidateState = newWindowState;
    windowCandidateTime = currentTime;
    windowDebouncePending = true;
  }
}

void applyDebouncedSensorStates()
{
  unsigned long currentTime = millis();

  if (
    doorDebouncePending &&
    currentTime - doorCandidateTime >= DEBOUNCE_TIME_MS
  ) {
    bool stableDoorState = digitalRead(DOOR_PIN) == HIGH;

    if (stableDoorState == doorCandidateState) {
      doorDebouncePending = false;

      if (stableDoorState != doorActive) {
        doorActive = stableDoorState;
        doorEventCount++;

        Serial.print("[PCINT] Door event #");
        Serial.print(doorEventCount);
        Serial.print(": ");
        Serial.println(doorActive ? "ACTIVE" : "INACTIVE");
      }
    } else {
      doorCandidateState = stableDoorState;
      doorCandidateTime = currentTime;
    }
  }

  if (
    windowDebouncePending &&
    currentTime - windowCandidateTime >= DEBOUNCE_TIME_MS
  ) {
    bool stableWindowState = digitalRead(WINDOW_PIN) == HIGH;

    if (stableWindowState == windowCandidateState) {
      windowDebouncePending = false;

      if (stableWindowState != windowActive) {
        windowActive = stableWindowState;
        windowEventCount++;

        Serial.print("[PCINT] Window event #");
        Serial.print(windowEventCount);
        Serial.print(": ");
        Serial.println(windowActive ? "ACTIVE" : "INACTIVE");
      }
    } else {
      windowCandidateState = stableWindowState;
      windowCandidateTime = currentTime;
    }
  }
}

void handleTimerEvent()
{
  bool timerEvent;
  unsigned long copiedTickCount;

  noInterrupts();
  timerEvent = timerInterruptFlag;
  copiedTickCount = timerTickCount;
  timerInterruptFlag = false;
  interrupts();

  if (!timerEvent) {
    return;
  }

  heartbeatState = !heartbeatState;
  digitalWrite(
    HEARTBEAT_LED_PIN,
    heartbeatState ? HIGH : LOW
  );

  Serial.print("[TIMER] Heartbeat tick ");
  Serial.println(copiedTickCount);
}

void thinkAboutState()
{
  bool motionActive = digitalRead(MOTION_PIN) == HIGH;

  if (motionActive || (doorActive && windowActive)) {
    currentState = ALARM;
  } else if (doorActive || windowActive) {
    currentState = WARNING;
  } else {
    currentState = SAFE;
  }
}

void actOnState()
{
  if (currentState == previousState) {
    return;
  }

  Serial.print("[STATE] ");

  if (currentState == SAFE) {
    Serial.println("SAFE");
    digitalWrite(ALARM_LED_PIN, LOW);
  } else if (currentState == WARNING) {
    Serial.println("WARNING");
    digitalWrite(ALARM_LED_PIN, LOW);
  } else {
    Serial.println("ALARM");
    digitalWrite(ALARM_LED_PIN, HIGH);
  }

  previousState = currentState;
}
