#define SRAM_START (0x20000000U)
#define SRAM_SIZE (32U * 1024U)
#define SRAM_END (SRAM_START + SRAM_SIZE)
#define STACK_POINTER_INIT_ADDRESS (SRAM_END)

#include <stdint.h>
#define ISR_VECTOR_SIZE_WORDS 114

extern uint32_t _etext, _sdata, _edata, _sbss, _ebss;

void main(void);
void init_system(void);
void reset_handler(void);
void default_handler(void);
void nmi_handler(void) __attribute__((weak));
void hard_fault_handler(void) __attribute__((weak));
void bus_fault_handler(void) __attribute__((weak));
void usage_fault_handler(void) __attribute__((weak));
void svcall_handler(void) __attribute__((weak));
void debug_monitor_handler(void) __attribute__((weak));
void pendsv_handler(void) __attribute__((weak));
void systick_handler(void) __attribute__((weak));

void nmi_handler(void) { default_handler(); }
void hard_fault_handler(void) { default_handler(); }
void bus_fault_handler(void) { default_handler(); }
void usage_fault_handler(void) { default_handler(); }
void svcall_handler(void) { default_handler(); }
void debug_monitor_handler(void) { default_handler(); }
void pendsv_handler(void) { default_handler(); }
void systick_handler(void) { default_handler(); }

const void *isr_vector[ISR_VECTOR_SIZE_WORDS] __attribute__((section(".isr_vector"))) = {
    (void *)STACK_POINTER_INIT_ADDRESS,
    reset_handler,
    nmi_handler,
    hard_fault_handler,
    bus_fault_handler,
    usage_fault_handler,
    0,
    0,
    0,
    0,
    0,
    svcall_handler,
    debug_monitor_handler,
    0,
    pendsv_handler,
    systick_handler,
};

void default_handler(void)
{
    while (1)
        ;
}

void reset_handler(void)
{
    // Copy .data from FLASH to SRAM
    uint32_t data_size = (uint32_t)&_edata - (uint32_t)&_sdata;
    uint8_t *flash_data = (uint8_t *)&_etext;
    uint8_t *sram_data = (uint8_t *)&_sdata;

    for (uint32_t i = 0; i < data_size; i++)
    {
        sram_data[i] = flash_data[i];
    }

    // Zero-fill .bss section in SRAM
    uint32_t bss_size = (uint32_t)&_ebss - (uint32_t)&_sbss;
    uint8_t *bss = (uint8_t *)&_sbss;

    for (uint32_t i = 0; i < bss_size; i++)
    {
        bss[i] = 0;
    }

    init_system();

    main();

    while (1)
    {
    }
}