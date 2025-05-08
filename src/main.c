#define PERIPHERAL_BASE (0x40000000U)
#define AHB1_BASE (PERIPHERAL_BASE + 0x20000U)
#define GPIOA_BASE (AHB1_BASE + 0x0U)
#define RCC_BASE (AHB1_BASE + 0x3800U)

#define RCC_AHB1ENR_OFFSET (0x30U)
#define RCC_AHB1ENR ((volatile uint32_t*) (RCC_BASE + RCC_AHB1ENR_OFFSET))
#define RCC_AHB1ENR_GPIOAEN (0x00U)

#define GPIO_MODER_OFFSET (0x00U)
#define GPIOA_MODER ((volatile uint32_t*) (GPIOA_BASE + GPIO_MODER_OFFSET))
#define GPIO_MODER_MODER5 (10U)
#define GPIO_ODR_OFFSET (0x14U)
#define GPIOA_ODR ((volatile uint32_t*) (GPIOA_BASE + GPIO_ODR_OFFSET))

#define LED_PIN 5

// Back EMF sensing
#define PIN_INPUT_BACK_EMF_VOLTAGE_1 0
#define PIN_INPUT_BACK_EMF_VOLTAGE_2 1
#define PIN_INPUT_BACK_EMF_VOLTAGE_3 2

#define PIN_OUTPUT_PWM_1 3
#define PIN_OUTPUT_PWM_2 4
#define PIN_OUTPUT_PWM_3 5

// PWM frequency
#define PWM_FREQUENCY 20000 // 20 kHz

// Rotor speed reference pin
#define PIN_INPUT_ROTORSPEED_REF 6

void init_system(void) {

}

void main(void)
{
  init_system();

  // ----------------------------------------------
  // FOC
  // ----------------------------------------------

  // Estimate the rotor position using Back EMF

  // Estimate the rotor speed

  // Clarke Transform

  // Park Transform

  // PID Controller

  // Inverse Park Transform

  // Space Vector Modulation

  // PWM modulation

}