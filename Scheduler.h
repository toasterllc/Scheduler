#pragma once
#include <type_traits>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <array>
#include <limits>
#include <ratio>
#include <tuple>

// SchedulerStack: macro to apply appropriate attributes to stack declarations
// gnu::used is apparently necessary for the gnu::section attribute to work when
// link-time optimization is enabled.
#define SchedulerStack(sect) [[gnu::section(sect), gnu::used]] alignas(void*)

namespace Toastbox {

// MARK: - IntState
class IntState {
public:
    static bool Get() {
#if defined(__MSP430__)
        return __get_SR_register() & GIE;
#elif defined(__arm__)
        return !__get_PRIMASK();
#else
        #error Task: Unsupported architecture
#endif
    }
    
    static void Set(bool en) {
#if defined(__MSP430__)
        if (en) __bis_SR_register(GIE);
        else    __bic_SR_register(GIE);
#elif defined(__arm__)
        if (en) __enable_irq();
        else    __disable_irq();
#else
        #error Task: Unsupported architecture
#endif
    }
    
    [[gnu::always_inline]]
    IntState() {
        _prev = Get();
    }
    
    [[gnu::always_inline]]
    IntState(bool en) {
        _prev = Get();
        Set(en);
    }
    
    IntState(const IntState& x) = delete;
    IntState(IntState&& x)      = delete;
    
    [[gnu::always_inline]]
    ~IntState() {
        Set(_prev);
    }
    
    [[gnu::always_inline]]
    void enable() {
        Set(true);
    }
    
    [[gnu::always_inline]]
    void disable() {
        Set(false);
    }
    
    [[gnu::always_inline]]
    void restore() {
        Set(_prev);
    }
    
private:
    bool _prev = false;
};

// MARK: - Scheduler
template<
typename T_TicksPeriod,             // T_TicksPeriod: a std::ratio specifying the period between Tick()
                                    //   calls, in seconds

void T_Sleep(),                     // T_Sleep: sleep function; invoked when no tasks have work to do.
                                    //   T_Sleep() is called with interrupts disabled, and interrupts must
                                    //   be disabled upon return. Implementations may temporarily enable
                                    //   interrupts if required for the CPU to wake from sleep, as long as
                                    //   interrupts are disabled upon return from T_Sleep().

size_t T_StackGuardCount,           // T_StackGuardCount: number of pointer-sized stack guard elements to use
void T_StackOverflow(size_t),       // T_StackOverflow: function to call when stack overflow is detected;
                                    //   argument is the task index (in T_Tasks) with the corrupted stack.

typename... T_Tasks                 // T_Tasks: list of tasks
>
class Scheduler {
public:
    using Ticks     = unsigned int;
    using Deadline  = Ticks;

private:
    using _TaskFn = void(*)();
    using _RunnableFn = bool(*)();
    
    // _Ticks(): returns the ceiled number of ticks required for T_Time to pass
    template<auto T_Time, typename T_Unit>
    static constexpr Ticks _Ticks() {
        using TicksPerUnitTime = std::ratio_divide<T_Unit, T_TicksPeriod>;
        using TicksRatio = std::ratio_multiply<std::ratio<T_Time>, TicksPerUnitTime>;
        const auto ticks = (TicksRatio::num + TicksRatio::den - 1) / TicksRatio::den;
        static_assert(ticks <= _TicksMax);
        return ticks;
    }
    
public:
    using TicksPeriod = T_TicksPeriod;
    
    // StackInit(): initialize the stack pointer to task 0's stack, which Scheduler.h expects before
    // Run() is called.
    [[gnu::always_inline]]
    static inline void StackInit() {
#if defined(__MSP430__)
        // MSP430 doesn't support a separate interrupt stack
        static_assert(_StackInterrupt == nullptr);
        
        if constexpr (sizeof(void*) == 2) {
            // Small memory model
            asm volatile("mov %0, sp" : : "i" (_StackTask0) : );
        } else {
            // Large memory model
            asm volatile("mov.a %0, sp" : : "i" (_StackTask0) : );
        }
        
#elif defined(__arm__)
        // Set the stack pointers: MSP+PSP (if task 0 has StackInterrupt member), or just MSP (otherwise)
        //
        // ARM cores conventionally initialize MSP to the SP value at the start of the vector table.
        // Instead, Scheduler.h expects to set SP manually via this function (and therefore the vector
        // table's SP value can contain anything). This technique works as long as StackInit() is called
        // early after chip reset, before the stack is needed.
        //
        // With this technique, we gain cleanliness (stacks exist as simple members inside of the task
        // struct, including the interrupt stack, instead of needing to declare stacks with asm directives
        // and reference them in the vector table), at the cost of needing to manually call this function
        // and a few extra instructions.
        
        if constexpr (_StackInterrupt != nullptr) {
            // We have an interrupt stack: set the MSP+PSP stack pointers
            asm volatile("ldr r0, =%0" : : "i" (_StackInterrupt) : );   // r0  = _StackInterrupt
            asm volatile("msr msp, r0" : : : );                         // msp = r0
            asm volatile("ldr r0, =%0" : : "i" (_StackTask0) : );       // r0  = _StackTask0
            asm volatile("msr psp, r0" : : : );                         // psp = r0
            
            // Make PSP the active stack
            asm volatile("mrs r0, control" : : : );                     // r0 = control
            asm volatile("orrs r0, r0, #2" : : : );                     // Set SPSEL bit (enable using PSP stack)
            asm volatile("msr control, r0" : : : );                     // control = r0
            asm volatile("isb" : : : );                                 // Instruction Synchronization Barrier
        
        } else {
            // We don't have an interrupt stack: simply set the current stack pointer
            asm volatile("ldr sp, =%0" : : "i" (_StackTask0) : );       // sp  = _StackTask0
        }
#else
        #error Task: Unsupported architecture
#endif
    }
    
    // Run(): scheduler entry point; runs task 0
    // StackInit() must be called before Run().
    [[noreturn]]
    static void Run() {
        // Init stack guards
        _StackGuardInit();
        // Run task 0
        _TaskRun();
        for (;;);
    }
    
    // Current(): returns whether any of T_Tasks are the currently-running task
    template<typename... T_Task>
    static bool Current() {
        return ((((_TaskCurr == &_TaskGet<T_Task>()) || ...)));
    }
    
    // Start(): start running T_Task with a specified function
    //
    // Interrupt context: allowed
    template<typename T_Task>
    static void Start(_TaskFn run) {
        // Disable ints (because _TaskStart requires it)
        IntState ints(false);
        _TaskStart(_TaskGet<T_Task>(), run, _StackTop<T_Task>());
    }
    
    // Start(): start running T_Tasks with their respective Run() functions
    //
    // Interrupt context: allowed
    template<typename... T_Task>
    static void Start() {
        // Disable ints (because _TaskStart requires it)
        IntState ints(false);
        ((_TaskStart(_TaskGet<T_Task>(), T_Task::Run, _StackTop<T_Task>()), ...));
    }
    
    // Stop(): stop T_Tasks
    // This explicitly does not affect the current task; see Abort() for that behavior.
    //
    // Interrupt context: allowed
    template<typename... T_Task>
    static void Stop() {
        // Disable ints (because _TaskStop requires it)
        IntState ints(false);
        ((_TaskStop(_TaskGet<T_Task>()), ...));
    }
    
    // Abort(): the same as Stop(), except if one of `T_Tasks` is the current task, immediately returns the scheduler
    //
    // Interrupt context: not allowed (because it enters the scheduler)
    template<typename... T_Task>
    static void Abort() {
        // Disable ints (because _TaskStop / _TaskSwap require it)
        IntState ints(false);
        ((_TaskStop(_TaskGet<T_Task>()), ...));
        if (Current<T_Task...>()) {
            // We stopped the current task so return to the scheduler
            _TaskSwap(_RunnableFalse);
        }
    }
    
    // Running(): returns whether any T_Tasks are running
    //
    // Interrupt context: allowed
    template<typename... T_Task>
    static bool Running() {
        // Disable ints (because we're accessing the same fields as Tick())
        IntState ints(false);
        return (((_TaskGet<T_Task>().runnable!=_RunnableFalse || _TaskGet<T_Task>().wakeDeadline) || ...));
    }
    
    // Wait(): waits until none of T_Tasks are running
    //
    // Interrupt context: not allowed (because it enters the scheduler)
    template<typename... T_Task>
    static void Wait() {
        return Wait([] { return !Running<T_Task...>(); });
    }
    
    // Yield(): yield current task to the scheduler
    //
    // Interrupt context: not allowed (because it enters the scheduler)
    static void Yield() {
        IntState ints(false);
        _TaskSwap(_RunnableTrue);
    }
    
    // Wait(fn): sleep current task until `fn` returns true.
    // `fn` must not cause any task to become runnable.
    // If it does, the scheduler may not notice that the task is runnable and
    // could go to sleep instead of running the task.
    //
    // If ints are disabled before calling Wait(), ints are guaranteed to have
    // remained disabled between `fn` executing and Wait() returning, and
    // therefore the condition that `fn` checks is guaranteed to remain true
    // after Wait() returns.
    //
    // Ints are disabled while calling `fn`
    //
    // Interrupt context: not allowed (because it enters the scheduler)
    static void Wait(_RunnableFn fn) {
        IntState ints(false);
        if (fn()) return;
        _TaskSwap(fn);
    }
    
    // Wait(): sleep current task until `fn` returns true, or `ticks` to pass.
    // Returns true if `fn` caused Wait() to return, and false if the timeout elapsed.
    //
    // See Wait() function above for more info.
    //
    // Interrupt context: not allowed (because it enters the scheduler)
    static bool Wait(Ticks ticks, _RunnableFn fn) {
        IntState ints(false);
        if (fn()) return true;
        _TaskSwap(fn, _DeadlineForTicks(ticks));
        return (bool)_TaskCurr->wakeDeadline;
    }
    
    // Context getter for current task
    template<typename T>
    static T Ctx() { return _TFromPtr<T>(_TaskCurr->ctx); }
    
    // Context setter for current task
    template<typename T>
    static void Ctx(const T& t) { _TaskCurr->ctx = _PtrFromT(t); }
    
    // WaitDeadline(): wait for a condition to become true, or for a deadline to pass.
    // Returns true if the condition became true before the deadline.
    //
    // For a deadline to be considered in the past, it must be in the range:
    //   [CurrentTime-TicksMax/2, CurrentTime]
    // For a deadline to be considered in the future, it must be in the range:
    //   [CurrentTime+1, CurrentTime+TicksMax/2+1]
    //
    // where TicksMax is the maximum value that the `Ticks` type can hold.
    //
    // See comment in function body for more info regarding the deadline parameter.
    //
    // See Wait() function above for more info.
    //
    // Interrupt context: not allowed (because it enters the scheduler)
    static bool WaitDeadline(Deadline deadline, _RunnableFn fn) {
        IntState ints(false);
        
        // Test whether `deadline` has already passed.
        //
        // Because _ISR.time rolls over periodically, it's impossible to differentiate
        // between `deadline` passing versus merely being far in the future. (For example,
        // consider the case where time is tracked with a uint8_t: if Deadline=127 and
        // CurrentTime=128, either Deadline passed one tick ago, or Deadline will pass
        // 255 ticks in the future.)
        //
        // To solve this ambiguity, we require deadlines to be within
        // [-TicksMax/2-1, +TicksMax/2] of _ISR.time (where TicksMax is the maximum
        // value that the `Ticks` type can hold), which allows us to employ the following
        // heuristic:
        //
        // For a deadline to be considered in the past, it must be in the range:
        //   [CurrentTime-TicksMax/2, CurrentTime]
        // For a deadline to be considered in the future, it must be in the range:
        //   [CurrentTime+1, CurrentTime+TicksMax/2+1]
        //
        // Now that ints are disabled (and therefore _ISR.time is unchanging), we
        // can employ the above heuristic to determine whether `deadline` has already passed.
        const bool past = deadline-_ISR.time-1 > _TicksMax/2;
        if (past) return false;
        if (fn()) return true;
        _TaskSwap(fn, deadline);
        return (bool)_TaskCurr->wakeDeadline;
    }
    
    template<auto T>
    static constexpr Ticks Us = _Ticks<T, std::micro>();
    
    template<auto T>
    static constexpr Ticks Ms = _Ticks<T, std::milli>();
    
    // Sleep(ticks): sleep current task for `ticks`
    //
    // Interrupt context: not allowed (because it enters the scheduler)
    static void Sleep(Ticks ticks) {
        IntState ints(false);
        _TaskSwap(_RunnableFalse, _DeadlineForTicks(ticks));
    }
    
    // Delay(ticks): delay current task for `ticks` without allowing other tasks to run
    // Enables interrupts at least once.
    //
    // Interrupt context: not allowed (because it calls T_Sleep)
    static void Delay(Ticks ticks) {
        IntState ints(false);
        _TaskCurr->wakeDeadline = _DeadlineForTicks(ticks);
        _ISR.wake.update = true;
        
        do {
            T_Sleep();
            // Let interrupts fire after waking
            IntState ints(true);
        } while (_TaskCurr->wakeDeadline);
    }
    
    // Tick(): notify scheduler that a tick has passed
    // Returns whether the CPU should wake to allow the scheduler to run.
    //
    // Interrupt context: can only be called from the interrupt context;
    // Tick() is intended to be called from an ISR that fires with period
    // `T_TicksPeriod`.
    static bool Tick() {
        _ISR.time++;
        
        // Wake tasks matching the current tick.
        bool wake = false;
        if (_ISR.wake.update || (_ISR.wake.pending && _ISR.wake.deadline==_ISR.time)) {
            // Wake the necessary tasks, and update _ISR.WakeDeadline
            Ticks wakeDelay = _TicksMax;
            std::optional<Deadline> wakeDeadline;
            for (_Task& task : _Tasks) {
                if (!task.wakeDeadline) continue;
                if (*task.wakeDeadline == _ISR.time) {
                    // The task's deadline has been hit; wake it
                    task.runnable = _RunnableTrue;
                    task.wakeDeadline = std::nullopt;
                    wake = true;
                
                } else {
                    // The task's deadline has not been hit; consider it as a candidate for the next _ISR.WakeDeadline
                    const Ticks d = *task.wakeDeadline-_ISR.time;
                    if (d <= wakeDelay) {
                        wakeDelay = d;
                        wakeDeadline = task.wakeDeadline;
                    }
                }
            }
            
            _ISR.wake.deadline = wakeDeadline.value_or(0);
            _ISR.wake.pending = (bool)wakeDeadline;
            _ISR.wake.update = false;
        }
        
        return wake;
    }
    
    // TickRequired(): whether Tick() invocations are currently required, as determined by whether there
    // are any tasks waiting for a deadline to pass.
    // If TickRequired() returns false, Tick() invocations aren't currently needed (and therefore
    // related hardware can be paused to save power, for example).
    //
    // Interrupt context: allowed
    static bool TickRequired() {
        IntState ints(false);
        return _ISR.wake.pending || _ISR.wake.update;
    }
    
    // CurrentTime(): returns the current time in ticks
    //
    // Note that this value only changes due to Tick() being called. Therefore if Tick() isn't
    // called because TickRequired()==false, then CurrentTime() won't be incremented until
    // TickRequired()==true.
    //
    // Interrupt context: allowed
    static Ticks CurrentTime() {
        IntState ints(false);
        return _ISR.time;
    }
    
private:
    // MARK: - Types
    
    using _StackGuard = uintptr_t[T_StackGuardCount];
    static constexpr Ticks _TicksMax = std::numeric_limits<Ticks>::max();
    static constexpr uintptr_t _StackGuardMagicNumber = (uintptr_t)0xCAFEBABEBABECAFE;
    
    struct _Task {
        _TaskFn run = nullptr;
        _RunnableFn runnable = nullptr;
        std::optional<Deadline> wakeDeadline;
        void* sp = nullptr;
        uintptr_t ctx = 0;
        _StackGuard* stackGuard = nullptr;
        _Task* next = nullptr;
    };
    
    // _DeadlineForTicks: returns the deadline for `ticks` in the future
    // We add 1 to account for the remainder of time left until the next tick arrives,
    // which is anywhere between [0,1) ticks. Ie the +1 has the effect of 'burning off'
    // this remainder time, so we can safely add `ticks` to that result, and guarantee
    // that we get a deadline that's at least `ticks` in the future.
    //
    // Ints: disabled (because we're accessing the same fields as Tick())
    static Deadline _DeadlineForTicks(Ticks ticks) {
        return _ISR.time+ticks+1;
    }
    
    template<typename T>
    static T _TFromPtr(uintptr_t x) {
        static_assert(sizeof(T) <= sizeof(uintptr_t));
        union { T t; uintptr_t ptr; } u = { .ptr = x };
        return u.t;
    }
    
    template<typename T>
    static uintptr_t _PtrFromT(T x) {
        static_assert(sizeof(T) <= sizeof(uintptr_t));
        union { T t; uintptr_t ptr; } u = { .t = x };
        return u.ptr;
    }
    
    // __TaskSwap(): architecture-specific function that saves the stack pointer into
    // _TaskPrev->sp and restores the stack pointer to _TaskCurr->sp. Steps:
    //
    //   (1) Push callee-saved regs onto stack (including $PC if needed for step #5 to work)
    //   (2) Save $SP into `_TaskPrev->sp`
    //   (3) Restore $SP from `_TaskCurr->sp`
    //   (4) Pop callee-saved registers from stack
    //   (5) Return to caller
    [[gnu::noinline, gnu::naked]] // Don't inline: PC must be pushed onto the stack when called
    static void __TaskSwap() {
#if defined(__MSP430__) && !defined(__LARGE_CODE_MODEL__)
        // MSP430, small memory model
        static_assert(sizeof(void*) == 2);
        #define _SchedulerStackAlign            1   // Count of pointer-sized registers to which the stack needs to be aligned
        #define _SchedulerStackSaveRegCount     7   // Count of pointer-sized registers that we save below (excluding $PC)
        asm volatile("pushm #7, r10" : : : );                           // (1)
        asm volatile("mov sp, %0" : "=m" (_TaskPrev->sp) : : );         // (2)
        asm volatile("mov %0, sp" : : "m" (_TaskCurr->sp) : );          // (3)
        asm volatile("popm #7, r10" : : : );                            // (4)
        asm volatile("ret" : : : );                                     // (5)
#elif defined(__MSP430__) && defined(__LARGE_CODE_MODEL__)
        // MSP430, large memory model
        static_assert(sizeof(void*) == 4);
        #define _SchedulerStackAlign            1   // Count of pointer-sized registers to which the stack needs to be aligned
        #define _SchedulerStackSaveRegCount     7   // Count of pointer-sized registers that we save below (excluding $PC)
        asm volatile("pushm.a #7, r10" : : : );                         // (1)
        asm volatile("mov.a sp, %0" : "=m" (_TaskPrev->sp) : : );       // (2)
        asm volatile("mov.a %0, sp" : : "m" (_TaskCurr->sp) : );        // (3)
        asm volatile("popm.a #7, r10" : : : );                          // (4)
        asm volatile("ret.a" : : : );                                   // (5)
#elif defined(__arm__)
        // ARM32
        static_assert(sizeof(void*) == 4);
        #define _SchedulerStackAlign            1   // Count of pointer-sized registers to which the stack needs to be aligned
        #define _SchedulerStackSaveRegCount     8   // Count of pointer-sized registers that we save below (excluding $PC)
        asm volatile("push {r4-r11,lr}" : : : );                        // (1)
        asm volatile("str sp, %0" : "=m" (_TaskPrev->sp) : : );         // (2)
        asm volatile("ldr sp, %0" : : "m" (_TaskCurr->sp) : );          // (3)
        asm volatile("pop {r4-r11,pc}" : : : );                         // (4)
#else
        #error Task: Unsupported architecture
#endif
    }
    
    // MARK: - Stack Guard
    static void _StackGuardInit() {
        // Initialize each task's stack guard
        if constexpr (_StackGuardEnabled) {
            for (_Task& task : _Tasks) {
                _StackGuardInit(*task.stackGuard);
            }
        }
        
        // Initialize the interrupt stack guard
        if constexpr (_StackInterruptGuardEnabled) _StackGuardInit(_StackInterruptGuard);
    }
    
    static void _StackGuardInit(_StackGuard& guard) {
        for (uintptr_t& x : guard) {
            x = _StackGuardMagicNumber;
        }
    }
    
    static void _StackGuardCheck(const _StackGuard& guard) {
        for (const uintptr_t& x : guard) {
            if (x != _StackGuardMagicNumber) {
                T_StackOverflow(_TaskCurr - &_Tasks[0]);
            }
        }
    }
    
    // _TaskStart(): reset a task to run a particular function
    //
    // Ints: disabled (because we're accessing the same fields as Tick())
    [[gnu::noinline]]
    static void _TaskStart(_Task& task, _TaskFn run, void* sp) {
        constexpr size_t SaveRegCount = _SchedulerStackSaveRegCount+1;
        constexpr size_t ExtraRegCount = SaveRegCount % _SchedulerStackAlign;
        constexpr size_t TotalRegCount = SaveRegCount + ExtraRegCount;
        void**const stackTop = (void**)sp;
        // Set task run function
        task.run = run;
        // Make task runnable
        task.runnable = _RunnableTrue;
        // Reset wake deadline
        task.wakeDeadline = std::nullopt;
        // Reset stack pointer
        task.sp = stackTop - TotalRegCount;
        // Push initial return address == _TaskRun
        *(stackTop-ExtraRegCount-1) = (void*)_TaskRun;
    }
    
    // _TaskStop(): reset a task to be stopped
    //
    // Ints: disabled (because we're accessing the same fields as Tick())
    [[gnu::noinline]]
    static void _TaskStop(_Task& task) {
        // Make task !runnable
        task.runnable = _RunnableFalse;
        // Reset wake deadline
        task.wakeDeadline = std::nullopt;
    }
    
    // _TaskRun(): task entry point
    //
    // Ints: don't care
    static void _TaskRun() {
        // Enable interrupts before entering the task for the first time
        IntState::Set(true);
        // Enter the task
        _TaskCurr->run();
        // Disable interrupts when returning to the scheduler after the task exits
        IntState::Set(false);
        // Next task
        _TaskSwap(_RunnableFalse);
    }
    
    // _TaskNext(): sets _TaskCurr to the next runnable task
    // Return `true` if we found a different task, or `false` if it's the same task that's currently running
    //
    // Ints: disabled
    static bool _TaskNext() {
        _TaskPrev = _TaskCurr;
        for (;;) {
            _TaskCurr = _TaskCurr->next;
            if (_TaskCurr->runnable()) return true;
            if (_TaskCurr == _TaskPrev) return false;
        }
    }
    
    // _TaskSwap(): saves _TaskCurr and restores the next runnable task
    //
    // Ints: disabled
    [[gnu::noinline]]
    static void _TaskSwap(_RunnableFn fn, std::optional<Deadline> wake=std::nullopt) {
        // Check stack guards
        if constexpr (_StackGuardEnabled) _StackGuardCheck(*_TaskCurr->stackGuard);
        if constexpr (_StackInterruptGuardEnabled) _StackGuardCheck(_StackInterruptGuard);
        
        // Update _TaskCurr's state
        _TaskCurr->runnable = fn;
        _TaskCurr->wakeDeadline = wake;
        if (wake) _ISR.wake.update = true;
        
        // Get the next runnable task, or sleep if no task wants to run
        while (!_TaskNext()) {
            T_Sleep();
            // Let interrupts fire after waking
            IntState ints(true);
        }
        
        __TaskSwap();
    }
    
    static bool _RunnableTrue() {
        return true;
    }
    
    static bool _RunnableFalse() {
        return false;
    }
    
    // _Task0RunFnOrNullptr(): returns T_Task's Run() function if it's task 0; otherwise returns nullptr
    template<typename T_Task>
    static constexpr _TaskFn _Task0RunFnOrNullptr() {
        if constexpr (std::is_same_v<T_Task, _Task0>) return T_Task::Run;
        return nullptr;
    }
    
    template<typename T_Task, typename=void>
    struct _StackInterruptExists : std::false_type {};
    
    template<typename T_Task>
    struct _StackInterruptExists<T_Task, std::void_t<decltype(T_Task::StackInterrupt)>> : std::true_type {};
    
    template<typename T_Task>
    static constexpr void* _StackInterruptTop() {
        if constexpr (_StackInterruptExists<T_Task>::value) {
            return (uint8_t*)T_Task::StackInterrupt + sizeof(T_Task::StackInterrupt);
        }
        return nullptr;
    }
    
    template<typename T_Task>
    static constexpr void* _StackInterruptBottom() {
        if constexpr (_StackInterruptExists<T_Task>::value) {
            return T_Task::StackInterrupt;
        }
        return nullptr;
    }
    
    template<typename T_Task>
    static constexpr void* _StackTop() {
        return (uint8_t*)T_Task::Stack + sizeof(T_Task::Stack);
    }
    
    // _TaskGet(): returns the _Task& for the given T_Task
    template<typename T_Task, size_t T_Delta=0>
    static constexpr _Task& _TaskGet() {
        static_assert((std::is_same_v<T_Task, T_Tasks> || ...), "invalid task");
        constexpr size_t idx = (_ElmIdx<T_Task, T_Tasks...>() + T_Delta) % std::size(_Tasks);
        return _Tasks[idx];
    }
    
    template<typename T_1, typename T_2=void, typename... T_s>
    static constexpr size_t _ElmIdx() {
        return std::is_same_v<T_1,T_2> ? 0 : 1 + _ElmIdx<T_1, T_s...>();
    }
    
    using _Task0 = std::tuple_element_t<0, std::tuple<T_Tasks...>>;
    static constexpr bool _StackGuardEnabled = (bool)T_StackGuardCount;
    static constexpr void* _StackTask0 = _StackTop<_Task0>();
    static constexpr void* _StackInterrupt = _StackInterruptTop<_Task0>();
    static constexpr bool _StackInterruptGuardEnabled = _StackGuardEnabled && _StackInterrupt;
    
    static inline _Task _Tasks[sizeof...(T_Tasks)] = {
        _Task{
            .run        = _Task0RunFnOrNullptr<T_Tasks>(),
            .runnable   = _RunnableFalse,
            .sp         = nullptr,
            .stackGuard = (_StackGuard*)T_Tasks::Stack,
            .next       = &_TaskGet<T_Tasks, 1>(),
        }...,
    };
    
    // _StackInterruptGuard: ideally this would be `static constexpr` instead of
    // `static inline`, but C++ doesn't allow constexpr reinterpret_cast.
    // In C++20 we could use std::bit_cast for this.
    static inline _StackGuard& _StackInterruptGuard = *(_StackGuard*)_StackInterruptBottom<_Task0>();
    static inline _Task* _TaskPrev = nullptr;
    static inline _Task* _TaskCurr = &_Tasks[0];
    
    static inline volatile struct {
        Ticks time = 0;
        struct {
            Deadline deadline = 0;
            bool pending = false;
            bool update = false;
        } wake;
    } _ISR;
};

} // namespace Toastbox
