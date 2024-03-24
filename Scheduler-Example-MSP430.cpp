// Scheduler.h example targeting the MSP-EXP430FR2433 board, which contains the MSP430FR2433 chip.
//
// This example toggles the red LED every second, and rapidly toggles the green LED upon pressing
// the button connected to GPIO P2.3.
#include <msp430.h> 
#include "Scheduler.h"

static constexpr uint32_t SysTickFreqHz = 256;

static void Sleep() {
    Toastbox::IntState ints; // Remember+restore current interrupt state upon return
    __bis_SR_register(GIE | LPM3_bits); // Sleep
}

static void SchedulerStackOverflow(size_t taskIdx) { for (;;); }

struct TaskButton;
struct TaskLED;
using Scheduler = Toastbox::Scheduler<
    std::ratio<1, SysTickFreqHz>,   // T_TicksPeriod: time period between ticks
    Sleep,                          // T_Sleep: function to put processor to sleep
    4,                              // T_StackGuardCount: number of pointer-sized stack guard elements to use
    SchedulerStackOverflow,         // T_StackOverflow: function to handle stack overflow
    nullptr,                        // T_StackInterrupt: unused
    
    // T_Tasks: list of tasks
    TaskButton,
    TaskLED
>;

struct TaskButton {
    static void Run() {
        for (;;) {
            // Wait for button press
            Scheduler::Wait([] { return _Pressed; });
            // Debounce
            Scheduler::Sleep(Scheduler::Ms<10>);
            
            // Toggle green LED (P1.1) 100 times when button is pressed
            for (int i=0; i<100; i++) {
                P1OUT ^= BIT1;
                Scheduler::Sleep(Scheduler::Ms<20>);
            }
            
            _Pressed = false;
        }
    }
    
    static void Pressed() {
        _Pressed = true;
    }
    
    static inline bool _Pressed = false;
    
    SchedulerStack(".stack.TaskButton")
    static inline uint8_t Stack[128];
};

struct TaskLED {
    static void Run() {
        // Toggle the red LED (P1.0) every second
        for (;;) {
            P1OUT ^= BIT0;
            Scheduler::Sleep(Scheduler::Ms<1000>);
        }
    }
    
    SchedulerStack(".stack.TaskLED")
    static inline uint8_t Stack[128];
};

[[gnu::interrupt(TIMER1_A1_VECTOR)]]
static void ISR_SysTick() {
    if (TA1IV != TA1IV_TAIFG) return;
    // Wake ourself if the scheduler says we should
    if (Scheduler::Tick()) __bic_SR_register_on_exit(LPM3_bits);
}

[[gnu::interrupt(PORT2_VECTOR)]]
static void ISR_Button() {
    if (P2IV != P2IV_P2IFG3) return;
    TaskButton::Pressed();
    __bic_SR_register_on_exit(LPM3_bits);
}

int main() {
    // Disable watchdog, unlock GPIOs
    WDTCTL = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;
    
    // P1.0 and P1.1 = outputs
    P1OUT = 0;
    P1DIR = BIT1 | BIT0;
    
    // P2.3 = input, enable interrupt
    P2OUT = BIT3;
    P2REN = BIT3;
    P2IES = BIT3;
    P2IE  = BIT3;
    P2IFG = 0;
    
    // Configure SysTick for frequency `SysTickFreqHz`
    TA1CCR0 = (32768/SysTickFreqHz)-1;
    TA1CTL = TASSEL__ACLK | MC__UP | TACLR | TAIE ;
    
    // Start our tasks and start the scheduler
    Scheduler::Start<TaskButton,TaskLED>();
    Scheduler::Run();
    return 0;
}
