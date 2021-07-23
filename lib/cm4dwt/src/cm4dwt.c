#include <stdio.h>
#include <cm4dwt/cm4dwt.h>
#include <core_cm4.h>
#include "console/console.h"

static int num_comparators = 0;
struct cm4dwt_comp {
  __IOM uint32_t COMP;
  __IOM uint32_t MASK;
  __IOM uint32_t FUNCTION;
};

void
DebugMon_HandlerC(unsigned long *args)
{
    /* Attribute unused is to silence compiler warnings,
     * the variables are placed here, so they can be inspected
     * by the debugger.
     */
    volatile unsigned long __attribute__((unused)) stacked_r0  = ((unsigned long)args[0]);
    volatile unsigned long __attribute__((unused)) stacked_r1  = ((unsigned long)args[1]);
    volatile unsigned long __attribute__((unused)) stacked_r2  = ((unsigned long)args[2]);
    volatile unsigned long __attribute__((unused)) stacked_r3  = ((unsigned long)args[3]);
    volatile unsigned long __attribute__((unused)) stacked_r12 = ((unsigned long)args[4]);
    volatile unsigned long __attribute__((unused)) stacked_lr  = ((unsigned long)args[5]);
    volatile unsigned long __attribute__((unused)) stacked_pc  = ((unsigned long)args[6]);
    volatile unsigned long __attribute__((unused)) stacked_psr = ((unsigned long)args[7]);

    // Configurable Fault Status Register
    // Consists of MMSR, BFSR and UFSR
    volatile unsigned long __attribute__((unused)) _CFSR = (*((volatile unsigned long *)(0xE000ED28))) ;

    // Hard Fault Status Register
    volatile unsigned long __attribute__((unused)) _HFSR = (*((volatile unsigned long *)(0xE000ED2C))) ;

    // Debug Fault Status Register
    volatile unsigned long __attribute__((unused)) _DFSR = (*((volatile unsigned long *)(0xE000ED30))) ;

    // Auxiliary Fault Status Register
    volatile unsigned long __attribute__((unused)) _AFSR = (*((volatile unsigned long *)(0xE000ED3C))) ;

    // Read the Fault Address Registers. These may not contain valid values.
    // Check BFARVALID/MMARVALID to see if they are valid values
    // MemManage Fault Address Register
    volatile unsigned long __attribute__((unused)) _MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
    // Bus Fault Address Register
    volatile unsigned long __attribute__((unused)) _BFAR = (*((volatile unsigned long *)(0xE000ED38))) ;

    volatile uint8_t __attribute__((unused)) watchpoint_number = 0;
    if (DWT->FUNCTION0 & DWT_FUNCTION_MATCHED_Msk){
        watchpoint_number = 0;
    } else if (DWT->FUNCTION1 & DWT_FUNCTION_MATCHED_Msk){
        watchpoint_number = 1;
    } else if (DWT->FUNCTION2 & DWT_FUNCTION_MATCHED_Msk){
        watchpoint_number = 2;
    }

    console_blocking_mode();
    console_printf("### wp %d triggered, pc: 0x%lx, lr: 0x%lx\n",
                   watchpoint_number, stacked_pc, stacked_lr);
    console_non_blocking_mode();

    __BKPT(0); //data watchpoint!
}

extern void DebugMon_Handler(void);
__attribute__((naked)) void DebugMon_Handler(void){
    __asm volatile (
            " movs r0,#4       \n"
            " movs r1, lr      \n"
            " tst r0, r1       \n"
            " beq _MSP2         \n"
            " mrs r0, psp      \n"
            " b _HALT2          \n"
            "_MSP2:               \n"
            " mrs r0, msp      \n"
            "_HALT2:              \n"
            " ldr r1,[r0,#20]  \n"
            " b DebugMon_HandlerC \n"
    );
}

static struct cm4dwt_comp*
get_comp(int watchpoint_index)
{
    struct cm4dwt_comp *c;
    if (!num_comparators) {
        return 0;
    }
    c = (struct cm4dwt_comp*)((uint8_t*)&DWT->COMP0 +
        watchpoint_index * (offsetof(DWT_Type, COMP1) -
                            offsetof(DWT_Type, COMP0))
        );
    return c;
}

void
cm4dwt_watchpoint_enable(uint8_t watchpoint_index,
                         uint32_t *word_address, uint32_t mask_2_pow_bytes,
                         cm4dwt_func_t dwt_function)
{
    CoreDebug->DEMCR = CoreDebug_DEMCR_TRCENA_Msk /*enable tracing*/ |
        CoreDebug_DEMCR_MON_EN_Msk /*enable debug interrupt*/;
    struct cm4dwt_comp *c = get_comp(watchpoint_index);
    assert(c);

    c->COMP = (uint32_t)word_address;
    c->MASK = (uint32_t)mask_2_pow_bytes;
    c->FUNCTION = (uint32_t)dwt_function;
}

int
cm4dwt_num_comparators(void)
{
    return num_comparators;
}

void
cm4dwt_watchpoint_disable(uint8_t watchpoint_index)
{
    struct cm4dwt_comp *c = get_comp(watchpoint_index);
    if (os_started()) {
        assert(c);
    }
    c->COMP = c->MASK = c->FUNCTION = 0;
}

void
cm4dwt_clear_stack(uint32_t margin)
{
    uint32_t limit = (uint32_t)__builtin_frame_address(0) - margin;
    for (uint32_t a=(uint32_t)g_current_task->t_stackbottom;
         a < limit;
         a+=4) {
        *((uint32_t*)a) = OS_STACK_PATTERN;
    }
}

void
cm4dwt_pkg_init(void)
{
    /* Need to explicitly set the interrupt vector here as os_arch_os_init resets
     * them all to default vector at boot. */
    NVIC_SetVector(DebugMonitor_IRQn, (uint32_t)DebugMon_Handler);
    num_comparators = (DWT->CTRL >> DWT_CTRL_NUMCOMP_Pos) & 0xF;

    /* Make sure watchpoints are all off */
    CoreDebug->DEMCR = 0;
    for (int i=0;i < num_comparators;i++) {
        cm4dwt_watchpoint_disable(i);
    }
}
