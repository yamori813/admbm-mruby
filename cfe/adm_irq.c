#include "adm5120.h"
#include "lib_types.h"
#include "sbmips32.h"
#include "cfe_irq.h"

static void
hw0_isr(void *arg)
{
print("@");

}

void adm_irq_enable(int adm_irql)
{
    uint32_t sr;

    sr = cfe_irq_disable();
    *(volatile uint32_t *)PHYS_TO_K1(INTC_BASE + IRQ_ENABLE_REG) = (1 << adm_irql);
    cfe_irq_enable(sr);
}

void adm_irq_disable(int adm_irql)
{
    uint32_t sr;

    sr = cfe_irq_disable();
    *(volatile uint32_t *)PHYS_TO_K1(INTC_BASE + IRQ_DISABLE_REG) = (1 << adm_irql);
    cfe_irq_enable(sr);
}

int adm_irq_init()
{

//    cfe_request_irq(0, hw0_isr, 0, CFE_IRQ_FLAGS_SHARED, 0);
//    adm_irq_enable(INT_LVL_TIMER);

    return 0;
}

int adm_timer_init()
{
uint32_t clock;

    clock = *(volatile uint32_t *)PHYS_TO_K1(SWCTRL_BASE + CODE_REG);
    clock = (clock & CODE_CLK_MASK) >> CODE_CLK_SHIFT;

    switch (clock)
    {
	case CPU_CLK_175MHZ:
	    return 175000000;
	case CPU_CLK_200MHZ:
	    return 200000000;
	case CPU_CLK_225MHZ:
	    return 225000000;
	case CPU_CLK_250MHZ:
	    return 250000000;
    }
}
