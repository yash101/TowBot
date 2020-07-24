#include <FastGPIO.h>

#define nullptr NULL    // crapduino doesn't have nullptr (facepalm)

#define MOTOR_ENABLE 11
#define MOTOR_0_STEP 12
#define MOTOR_0_DIR 13
#define MOTOR_1_STEP 14
#define MOTOR_1_DIR 15
#define MOTOR_2_STEP 16
#define MOTOR_2_DIR 17
#define MOTOR_3_STEP 18
#define MOTOR_3_DIR 19

// we are targetting the enable pin, not the sleep pin so we don't have a setup period

#define CPU_SPEED 16000000
#define NUM_TASKS 16
#define TICKS_PER_SECOND 15625 // assuming prescaler = 1024

/**
Task Scheduler - schedules task for t ticks
*/
class Task;
class TaskScheduler;
bool nop_cb(void* arg, Task* tsk) {}
class Task {
private:
public:
  bool (*cb)(void* arg, Task* tsk);
  void* arg;
  int ticks_delay;
  int _calc_delay;
  bool in_use;
  Task* next;

  Task() :
    cb(nop_cb),
    arg(nullptr),
    ticks_delay(0),
    _calc_delay(0),
    in_use(false),
    next(nullptr)
  {}
};

class TaskScheduler {
private:
  Task* next;
public:
  void tmr_cb_compa() {
    // disable the interrupts
    TIMSK1 = 0x00;

    while (next != nullptr) {
      next->_calc_delay -= TCNT1;
      TCNT1 = 0x0000;
      // time to execute?
      if (next->_calc_delay <= 0) {
        // run the task callback
        Task* tsk = next;
        next = next->next;
        tsk->in_use = tsk->cb(tsk->arg, tsk);
      } else {
        // we are done :)
        break;
      }
    }

    // set the next timer interval
    if (next != nullptr) {
      // time left = next->_calc_delay
      OCR1A = next->_calc_delay;
      // enable the timer
      TIMSK1 |= 1 << OCIE1A;
    }
    TIMSK1 |= 1 << TOIE1;
  }

  void tmr_cb_of() {
    // s3gfaults haha :3)
    if (next == nullptr)
      return;

    // subtract 0xFFFF from root->_calc_delay
    next->_calc_delay -= (int) 0xFFFF;

    if (next->_calc_delay < (int) 0xFFFF) {
      OCR1A = next->_calc_delay;
      TIMSK1 |= (1 << OCIE1A); // enable the interrupt timer for the next task
    }
  }

  void init_tmr() {
    // no settings needed here
    TCCR1A = 0x0000; // timer 1 control reg a
    // prescaler = 1024; change to whatever as necessary. note: lower prescaler = higher overhead, tighter timing, lower latency
    TCCR1B = (1 << CS12) | (1 << CS10); // timer 1 control reg a <- prescaler
    // disable all interrupts - we don't need them yet
    TIMSK1 = 0x00; // timer 1 mask control reg
    OCR1A = 0xFFFF; // timer 1 compare a reg
  }

  // note: we want to avoid saturating the timer whenever possible!
  void addTask(Task* task) {
    // disable interrupts from this timer to make things atomic
    TIMSK1 = 0x00;

    // zero the timer so we don't accidentally overflow (safety)
    int prev_timer = TCNT1;
    TCNT1 = 0x0000;

    // check if this queue is empty
    if (next == nullptr) {
      next = task;
      next->_calc_delay = next->ticks_delay; // set the appropriate timing
      task->next = nullptr;
    } else {
      // next is the root of the task list
      // subtract prev_timer from the first task
      next->_calc_delay -= prev_timer;

      // add as first
      if (next->_calc_delay > task->ticks_delay) { // this works perfectly
        task->next = next;
        next = task;
        next->_calc_delay = next->ticks_delay;
        next->next->_calc_delay -= next->_calc_delay;
      } else {
        // find the correct location to add the task
        Task* iterator = next;
        int curdel = 0; // current delay up till task n
        while (iterator != nullptr) { // don't run off the end of the list
          curdel += iterator->_calc_delay; // delay up till the iterator

          // end of list
          if (iterator->next == nullptr) {
            task->next = nullptr;
            iterator->next = task;
            task->_calc_delay = task->ticks_delay - curdel;

            break;
          } else if (task->ticks_delay <= curdel + iterator->next->_calc_delay) {
            // add between iterator and iterator->next
            task->next = iterator->next;
            iterator->next = task;

            // time delay for this task
            iterator->next->_calc_delay = iterator->next->ticks_delay - curdel;

            // time delay for the next task (subtract the delay from the previous task)
            iterator->next->next->_calc_delay -= iterator->next->_calc_delay;

            break;
          }
          
          // advance the iterator
          iterator = iterator->next;
        }
      }
    }
    
    if (next->_calc_delay < 0xFFFF) { // within the next overflow cycle
      OCR1A = next->_calc_delay;
      TIMSK1 |= (1 << OCIE1A); // enable task interrupt
    }

    // enable overflow interrupt to make things tick again
    TIMSK1 |= (1 << TOIE1);
  }

  TaskScheduler() {
    init_tmr();
    next = nullptr;
  }
};

Task* getTask() {
  static Task _taskpool[NUM_TASKS];
  for (char i = 0; i < NUM_TASKS; i++) {
    if (!_taskpool[i].in_use) {
      _taskpool[i].in_use = true;
      return &_taskpool[i];
    }
  }
  return nullptr; // no tasks available :(
}

TaskScheduler& getScheduler() {
  static TaskScheduler sched;
  return sched;
}

ISR(TIMER1_COMPA_vect) {
  getScheduler().tmr_cb_compa();
}

ISR(TIMER1_OVF_vect) {
  getScheduler().tmr_cb_of();
}

template <
  short STEP,
  short EN,
  short DIR
>
class StepperMotor {
private:
  /** \brief target speed we want the stepper to move at (steps / sec)
   */
  int targetSpeed;

  /** \brief speed the stepper is moving at (steps / sec)
   */
  int currentSpeed;

  /** \brief number of ticks until next tick
   */
  int stepDelay;

  /** \brief acceleration (steps / sec^2)
   */
  unsigned int acceleration;

  Task stepTask;
  Task motionControlTask;

  inline void calculateDelay() {
    stepDelay = TICKS_PER_SECOND / abs(currentSpeed);
  }

  static bool step_cb(void* arg, Task* tsk) {
    StepperMotor<STEP, EN, DIR>* driver = reinterpret_cast<decltype(driver)>(arg);

    // don't actuate
    if (driver->currentSpeed == 0) {
      tsk->ticks_delay = TICKS_PER_SECOND / 100;
      getScheduler().addTask(tsk);
      return true;
    }

    // set the step pin high
    FastGPIO::Pin<STEP>::setOutput(HIGH);

    // re-add the task (do it before setting low to slow down the signal :))
    tsk->ticks_delay = driver->stepDelay;
    getScheduler().addTask(tsk);

    // set the step pin low to complete the step
    FastGPIO::Pin<STEP>::setOutput(LOW);
  }

  static bool motion_ctrl_cb(void* arg, Task* tsk) {
    StepperMotor<STEP, EN, DIR>* driver = reinterpret_cast<decltype(driver)>(arg);

    int deltaV = driver->targetSpeed - driver->currentSpeed; // make this zero

    if (deltaV < 0) {
    } else if (deltaV > 0) {
    } else {
    }
  }

public:
};

template <
  short step,
  short enable,
  short direction
>
class Stepper {
private:
  bool dir;
  bool switchDirection;
  bool motorPowered;

  int position;
  int targetPosition;
  
  int stepDelay; // interval between steps

  int stepsPerSecond;
  int targetStepsPerSecond;

  unsigned int targetStepsPerSecondPerSecond;

  // sets the pin high and starts the step
  // task is also used to boot motor
  Task stepTask;
  Task accelerationTask;

  void activateMotor() {
    FastGPIO::Pin<enable>::setOutput(HIGH);
    motorPowered = true;
  }

  void shutdownMotor() {
    // shutdown is immediate
    FastGPIO::Pin<enable>::setOutput(LOW);
    motorPowered = false;
  }

  void calculateDelay() {
    // stepDelay = (1 / stepsPerSecond) => TICKS_PER_SECOND / stepsPerSecond
    stepDelay = TICKS_PER_SECOND / stepsPerSecond;
  }

  static bool performStep(void* arg, Task* tsk) {
    // the stepper object was passed by pointer
    Stepper* stepper = reinterpret_cast<Stepper*>(arg);
    if (stepper->stepDelay != 0)
      FastGPIO::Pin<step>::setOutput(HIGH);

    if (stepper->motorPowered) {
      stepper->position += (stepper->dir) ? 1 : -1;
      tsk->ticks_delay = stepper->stepDelay;
      getScheduler().addTask(tsk);
    }

    FastGPIO::Pin<step>::setOutput(LOW);

    return stepper->motorPowered; // maintains the in_use flag
  }

  static bool accelerationStep(void* arg, Task* tsk) {
    Stepper* stepper = reinterpret_cast<Stepper*>(arg);

    // if stepper is off, cancel
    if (stepper->motorPowered) {
      tsk->ticks_delay = TICKS_PER_SECOND / 100;
      getScheduler().addTask(tsk); // run again
    }

    // turn off motor if not necessary
    if (stepper->targetStepsPerSecond == 0 && stepper->stepsPerSecond == 0)
      stepper->shutdownMotor();

    // current steps per second = stepper->stepsPerSecond
    // target steps per second = stepper->targetStepsPerSecond
    // maxDelta = stepper->targetStepsPerSecond * dt; dt = 0.01s (10 ms)

    // if we want to switch direction, switchDirection will be true

    // if we want to reverse we gotta stop first
    int target = (stepper->switchDirection && stepper->stepsPerSecond != 0) ? 0 : stepper->targetStepsPerSecond;

    // time to switch directions?
    if (stepper->switchDirection && stepper->stepsPerSecond == 0) {
      if (stepper->dir)
        FastGPIO::Pin<direction>::setOutput(HIGH);
      else
        FastGPIO::Pin<direction>::setOutput(LOW);

      stepper->switchDirection = false;
    }

    // slow down?
    if (stepper->stepsPerSecond > target) {
      stepper->stepsPerSecond = max(target, stepper->stepsPerSecond - (stepper->targetStepsPerSecondPerSecond / 100));
    }

    // speed up?
    if (stepper->stepsPerSecond < stepper->targetStepsPerSecond) {
      stepper->stepsPerSecond = min(target, stepper->stepsPerSecond + (stepper->targetStepsPerSecondPerSecond / 100));
    }

    stepper->calculateDelay();

    Serial.println(stepper->stepsPerSecond);
  }

  void startMotorTasks() {
    Serial.println("Starting motor tasks...");
    activateMotor();

    void* stpptr = reinterpret_cast<void*>(this);
    stepTask.cb = Stepper<step, enable, direction>::performStep;
    stepTask.ticks_delay = TICKS_PER_SECOND / 1000; // wait at least 1 ms
    stepTask.arg = stpptr;
    getScheduler().addTask(&stepTask);

    accelerationTask.cb = Stepper<step, enable, direction>::accelerationStep;
    accelerationTask.ticks_delay = TICKS_PER_SECOND / 1000; // wait at least 1 ms (right before steptask starts)
    accelerationTask.arg = stpptr;
    getScheduler().addTask(&accelerationTask);
  }

public:
  int getCurrentPosition() { return position; }

  void setTargetSpeed(int stepsPerSecond) {
    bool d = stepsPerSecond < 0;
    if (dir != d)
      switchDirection = true;
    dir = d;
    if (dir)
      stepsPerSecond = -stepsPerSecond;
    
    targetStepsPerSecond = stepsPerSecond;

    if (stepsPerSecond != 0 && !motorPowered)
      startMotorTasks();
  }

  void setAcceleration(unsigned int stepsPerSecondPerSecond) {
    targetStepsPerSecondPerSecond = stepsPerSecondPerSecond;

    // start motor if there's something to do
    if (targetStepsPerSecond > 0 && targetStepsPerSecondPerSecond > 0 && !motorPowered)
      startMotorTasks();
  }

  int getTargetSpeed() { return targetStepsPerSecond; }
  int getCurrentSpeed() { return stepsPerSecond; }
  int getAcceleration() { return targetStepsPerSecondPerSecond; }

  void initialize() {
    shutdownMotor();
    dir = false;
    switchDirection = false;

    position = 0;
    targetPosition = 0;
    stepDelay = 0;
    stepsPerSecond = 0;
    targetStepsPerSecondPerSecond = 200; // 1 RPM/s^2
    stepTask.in_use = false;
    accelerationTask.in_use = false;
  }
  
  Stepper() {
    initialize();
  }
};

LiquidCrystal mainDisplay(5, 4, 3, 2, 1, 0);
Stepper<STEPPER_STEP, STEPPER_EN, STEPPER_DIR> mainStepper;

void initializeDisplay() {
}

void initializeStepper() {
}

void setup() {
  Serial.begin(115200);
  initializeStepper();
}

void loop() {
    mainStepper.setTargetSpeed(1200);
    delay(4000);
    mainStepper.setTargetSpeed(0);
    delay(4000);
}
