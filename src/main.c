#include <stdint.h>
#include <math.h>

#define PERIPHERAL_BASE (0x40000000U)
#define AHB1_BASE (PERIPHERAL_BASE + 0x20000U)
#define GPIOA_BASE (AHB1_BASE + 0x0U)
#define RCC_BASE (AHB1_BASE + 0x3800U)

#define RCC_AHB1ENR_OFFSET (0x30U)
#define RCC_AHB2ENR_OFFSET (0x34U)
#define RCC_APB2ENR_OFFSET (0x60U)
#define RCC_AHB1ENR ((volatile uint32_t *)(RCC_BASE + RCC_AHB1ENR_OFFSET))
#define RCC_AHB2ENR ((volatile uint32_t *)(RCC_BASE + RCC_AHB2ENR_OFFSET))
#define RCC_APB2ENR ((volatile uint32_t *)(RCC_BASE + RCC_APB2ENR_OFFSET))
#define RCC_AHB1ENR_GPIOAEN (0x00U)

#define GPIO_MODER_OFFSET (0x00U)
#define GPIO_AFRH_OFFSET (0x24U)
#define GPIOA_MODER ((volatile uint32_t *)(GPIOA_BASE + GPIO_MODER_OFFSET))
#define GPIOA_AFRH ((volatile uint32_t *)(GPIOA_BASE + GPIO_AFRH_OFFSET))
#define GPIOB_BASE (AHB1_BASE + 0x400U)
#define GPIOB_MODER ((volatile uint32_t *)(GPIOB_BASE + GPIO_MODER_OFFSET))
#define GPIOC_BASE (AHB1_BASE + 0x800U)
#define GPIOC_MODER ((volatile uint32_t *)(GPIOC_BASE + GPIO_MODER_OFFSET))
#define GPIO_MODER_MODER5 (10U)
#define GPIO_ODR_OFFSET (0x14U)
#define GPIOA_ODR ((volatile uint32_t *)(GPIOA_BASE + GPIO_ODR_OFFSET))
#define GPIOC_ODR ((volatile uint32_t *)(GPIOC_BASE + GPIO_ODR_OFFSET))

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

// GPIO und Clock Konfigurationen
#define GPIOA_EN (1 << 0)
#define GPIOB_EN (1 << 1)
#define GPIOC_EN (1 << 2)

// ADC Definitionen
#define ADC1_BASE (PERIPHERAL_BASE + 0x12000U)
#define ADC2_BASE (PERIPHERAL_BASE + 0x12100U)
#define ADC_CR_OFFSET (0x08U)
#define ADC_CFGR_OFFSET (0x0CU)
#define ADC_SQR1_OFFSET (0x30U)
#define ADC_DR_OFFSET (0x40U)
#define ADC_CR_ADEN (1 << 0)
#define ADC_CR_ADSTART (1 << 2)
#define ADC_ISR_OFFSET (0x00U)
#define ADC_ISR_EOC (1 << 2)

#define ADC1_CR ((volatile uint32_t *)(ADC1_BASE + ADC_CR_OFFSET))
#define ADC1_CFGR ((volatile uint32_t *)(ADC1_BASE + ADC_CFGR_OFFSET))
#define ADC1_SQR1 ((volatile uint32_t *)(ADC1_BASE + ADC_SQR1_OFFSET))
#define ADC1_DR ((volatile uint32_t *)(ADC1_BASE + ADC_DR_OFFSET))
#define ADC1_ISR ((volatile uint32_t *)(ADC1_BASE + ADC_ISR_OFFSET))

// Timer-Definitionen für PWM
#define TIM1_BASE (PERIPHERAL_BASE + 0x12C00U)
#define TIM_CR1_OFFSET (0x00U)
#define TIM_CCMR1_OFFSET (0x18U)
#define TIM_CCMR2_OFFSET (0x1CU)
#define TIM_CCER_OFFSET (0x20U)
#define TIM_PSC_OFFSET (0x28U)
#define TIM_ARR_OFFSET (0x2CU)
#define TIM_CCR1_OFFSET (0x34U)
#define TIM_CCR2_OFFSET (0x38U)
#define TIM_CCR3_OFFSET (0x3CU)
#define TIM_BDTR_OFFSET (0x44U)

#define TIM1_CR1 ((volatile uint32_t *)(TIM1_BASE + TIM_CR1_OFFSET))
#define TIM1_CCMR1 ((volatile uint32_t *)(TIM1_BASE + TIM_CCMR1_OFFSET))
#define TIM1_CCMR2 ((volatile uint32_t *)(TIM1_BASE + TIM_CCMR2_OFFSET))
#define TIM1_CCER ((volatile uint32_t *)(TIM1_BASE + TIM_CCER_OFFSET))
#define TIM1_PSC ((volatile uint32_t *)(TIM1_BASE + TIM_PSC_OFFSET))
#define TIM1_ARR ((volatile uint32_t *)(TIM1_BASE + TIM_ARR_OFFSET))
#define TIM1_CCR1 ((volatile uint32_t *)(TIM1_BASE + TIM_CCR1_OFFSET))
#define TIM1_CCR2 ((volatile uint32_t *)(TIM1_BASE + TIM_CCR2_OFFSET))
#define TIM1_CCR3 ((volatile uint32_t *)(TIM1_BASE + TIM_CCR3_OFFSET))
#define TIM1_BDTR ((volatile uint32_t *)(TIM1_BASE + TIM_BDTR_OFFSET))

// Konstante für FOC
#define PI 3.14159265358979323846f
#define SQRT3 1.73205080757f
#define SYSTEM_CLOCK 170000000 // 170 MHz
#define PWM_PERIOD (SYSTEM_CLOCK / PWM_FREQUENCY)
#define ADC_VREF 3.3f

// LED Blinkparameter
#define LED_BLINK_PERIOD 10000000 // Ungefähr 0,5 Sekunden
#define LED_PIN_PC6 6

// PID-Regler Parameter
typedef struct
{
    float kp;         // Proportional-Verstärkung
    float ki;         // Integral-Verstärkung
    float kd;         // Differential-Verstärkung
    float setpoint;   // Sollwert
    float integral;   // Integrierte Regelabweichung
    float prev_error; // Vorherige Regelabweichung
    float output_min; // Minimale Ausgangsbegrenzung
    float output_max; // Maximale Ausgangsbegrenzung
} PIDController;

// FOC Variablen
typedef struct
{
    float alpha;   // Alpha-Komponente (statorfest)
    float beta;    // Beta-Komponente (statorfest)
    float d;       // D-Komponente (rotorfest)
    float q;       // Q-Komponente (rotorfest)
    float d_ref;   // D-Komponente Sollwert
    float q_ref;   // Q-Komponente Sollwert
    float v_alpha; // Ausgangsspannung Alpha-Komponente
    float v_beta;  // Ausgangsspannung Beta-Komponente
    float phi;     // Rotorposition (elektrischer Winkel)
    float omega;   // Rotordrehzahl (elektrisch)
    float duty1;   // Duty-Cycle Phase 1 (0-1)
    float duty2;   // Duty-Cycle Phase 2 (0-1)
    float duty3;   // Duty-Cycle Phase 3 (0-1)
} FOCState;

// Globale Variablen
FOCState foc_state = {0};
uint16_t bemf1, bemf2, bemf3, vbus, speed_ref;
float bemf1_filtered, bemf2_filtered, bemf3_filtered;
float bemf_filter_alpha = 0.05f;                                                  // Filterkonstante
PIDController pid_d = {0.5f, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f, -0.95f, 0.95f};        // d-Regler
PIDController pid_q = {1.0f, 0.2f, 0.0f, 0.0f, 0.0f, 0.0f, -0.95f, 0.95f};        // q-Regler
PIDController pid_speed = {0.01f, 0.005f, 0.0f, 0.0f, 0.0f, 0.0f, -10.0f, 10.0f}; // Drehzahlregler

volatile uint32_t system_ticks = 0;
uint8_t foc_ok = 0; // Flag für korrekte FOC-Funktion

void init_system(void)
{
    // Clock für GPIO aktivieren
    *RCC_AHB1ENR |= (GPIOA_EN | GPIOB_EN | GPIOC_EN);

    // GPIO Pins konfigurieren

    // GPIOA: PA0(ADC), PA4(ADC), PA8(PWM), PA9(PWM), PA10(PWM), PA12(PWM)
    *GPIOA_MODER &= ~(0x3U << (0 * 2) | 0x3U << (4 * 2) | 0x3U << (8 * 2) | 0x3U << (9 * 2) | 0x3U << (10 * 2) | 0x3U << (12 * 2));
    *GPIOA_MODER |= (0x3U << (0 * 2) | 0x3U << (4 * 2));                                       // Analog mode für ADC
    *GPIOA_MODER |= (0x2U << (8 * 2) | 0x2U << (9 * 2) | 0x2U << (10 * 2) | 0x2U << (12 * 2)); // Alternate function für PWM

    // GPIOB: PB11(ADC), PB12(ADC), PB15(PWM)
    *GPIOB_MODER &= ~(0x3U << (11 * 2) | 0x3U << (12 * 2) | 0x3U << (15 * 2));
    *GPIOB_MODER |= (0x3U << (11 * 2) | 0x3U << (12 * 2)); // Analog mode für ADC
    *GPIOB_MODER |= (0x2U << (15 * 2));                    // Alternate function für PWM

    // GPIOC: PC4(ADC), PC6(GPIO), PC13(PWM)
    *GPIOC_MODER &= ~(0x3U << (4 * 2) | 0x3U << (6 * 2) | 0x3U << (13 * 2));
    *GPIOC_MODER |= (0x3U << (4 * 2));  // Analog mode für ADC
    *GPIOC_MODER |= (0x1U << (6 * 2));  // Output mode für LED
    *GPIOC_MODER |= (0x2U << (13 * 2)); // Alternate function für PWM

    // Alternate function mapping für Timer-Kanäle
    // AF1: PA8(TIM1_CH1), PA9(TIM1_CH2), PA10(TIM1_CH3), PA12(TIM1_CH2N), PB15(TIM1_CH3N)
    // AF4: PC13(TIM1_CH1N)

    // Timer1 für PWM initialisieren
    // Enable TIM1 clock
    *(RCC_AHB1ENR + 0x4 / 4) |= (1 << 11); // TIM1 Clock enable in RCC_APB2ENR

    // Timer-Konfiguration für 20kHz PWM
    *TIM1_PSC = 0;              // Prescaler = 0 (keine Vorteiler)
    *TIM1_ARR = PWM_PERIOD - 1; // Auto-reload value

    // PWM-Mode 1 für Kanäle 1,2,3 konfigurieren
    *TIM1_CCMR1 &= ~(0xFF << 0);
    *TIM1_CCMR1 |= (0x6 << 4) | (0x6 << 12); // PWM mode 1 für CH1 und CH2
    *TIM1_CCMR2 &= ~(0xFF << 0);
    *TIM1_CCMR2 |= (0x6 << 4); // PWM mode 1 für CH3

    // Preload enable für alle Kanäle
    *TIM1_CCMR1 |= (1 << 3) | (1 << 11);
    *TIM1_CCMR2 |= (1 << 3);

    // Outputs enable (CCER)
    *TIM1_CCER |= (1 << 0) | (1 << 4) | (1 << 8);  // Enable CH1, CH2, CH3
    *TIM1_CCER |= (1 << 2) | (1 << 6) | (1 << 10); // Enable CH1N, CH2N, CH3N

    // Main output enable (BDTR)
    *TIM1_BDTR |= (1 << 15); // MOE=1

    // Initial duty cycle = 0
    *TIM1_CCR1 = 0;
    *TIM1_CCR2 = 0;
    *TIM1_CCR3 = 0;

    // Enable counter
    *TIM1_CR1 |= 1;

    // ADC initialisieren
    // Enable ADC1 und ADC2 clock
    *(RCC_AHB1ENR + 0x8 / 4) |= (1 << 8) | (1 << 9); // ADC1 & ADC2 Clock enable in RCC_AHB2ENR

    // ADC1 konfigurieren für VBUS, BEMF1, BEMF3, SPEED_REF Messung
    *ADC1_CR |= ADC_CR_ADEN; // ADC Enable

    // Aktiviere Status-LED als Betriebsanzeige
    *GPIOC_ODR |= (1 << 6);

    // UART-Initialisierung
    init_uart_debug();
}

// UART Definitionen
#define USART1_BASE (PERIPHERAL_BASE + 0x13800U)
#define USART_BRR_OFFSET (0x0CU)
#define USART_CR1_OFFSET (0x00U)
#define USART_ISR_OFFSET (0x1CU)
#define USART_TDR_OFFSET (0x28U)

#define USART1_BRR ((volatile uint32_t *)(USART1_BASE + USART_BRR_OFFSET))
#define USART1_CR1 ((volatile uint32_t *)(USART1_BASE + USART_CR1_OFFSET))
#define USART1_ISR ((volatile uint32_t *)(USART1_BASE + USART_ISR_OFFSET))
#define USART1_TDR ((volatile uint32_t *)(USART1_BASE + USART_TDR_OFFSET))

// UART-Initialisierung
void init_uart_debug()
{
    // GPIO für UART konfigurieren (z.B. PA9=TX, PA10=RX für USART1)
    *RCC_AHB2ENR |= (1 << 0);        // GPIOA-Takt aktivieren
    *GPIOA_MODER &= ~(3 << (9 * 2)); // PA9 Modus löschen
    *GPIOA_MODER |= (2 << (9 * 2));  // PA9 als alternative Funktion

    // Alternate Function für USART1 setzen
    *GPIOA_AFRH &= ~(0xF << ((9 - 8) * 4));
    *GPIOA_AFRH |= (7 << ((9 - 8) * 4)); // AF7 = USART1

    // USART1 Takt aktivieren und konfigurieren
    *RCC_APB2ENR |= (1 << 14);         // USART1 Takt einschalten
    *USART1_BRR = 170000000 / 115200;  // Baudrate 115200 bei 170MHz
    *USART1_CR1 = (1 << 0) | (1 << 3); // USART enable, TX enable

    // Sende ein Testzeichen und blinke die LED als Bestätigung
    uart_puts("UART initialized\r\n");

    // LED blinken zur Bestätigung
    for (int i = 0; i < 5; i++)
    {
        *GPIOC_ODR |= (1 << 6); // LED ein
        for (volatile int j = 0; j < 100000; j++)
            ;
        *GPIOC_ODR &= ~(1 << 6); // LED aus
        for (volatile int j = 0; j < 100000; j++)
            ;
    }
}

// Ein Zeichen senden
void uart_putc(char c)
{
    while (!(*USART1_ISR & (1 << 7)))
        ;            // Warte bis TX leer
    *USART1_TDR = c; // Sende Zeichen
}

// String senden
void uart_puts(const char *s)
{
    while (*s)
    {
        uart_putc(*s++);
    }
}

// Zahl als String senden
void uart_print_float(float value)
{
    char buffer[20];
    int int_part = (int)value;
    int frac_part = (int)((value - int_part) * 1000); // 3 Nachkommastellen

    // Integer-Teil konvertieren
    int i = 0;
    if (int_part == 0)
    {
        buffer[i++] = '0';
    }
    else
    {
        if (int_part < 0)
        {
            buffer[i++] = '-';
            int_part = -int_part;
            frac_part = -frac_part;
        }

        int temp = int_part;
        int digits = 0;
        while (temp > 0)
        {
            temp /= 10;
            digits++;
        }

        temp = int_part;
        i += digits;
        int j = i - 1;
        while (temp > 0)
        {
            buffer[j--] = '0' + (temp % 10);
            temp /= 10;
        }
    }

    // Dezimalpunkt
    buffer[i++] = '.';

    // Nachkommastellen
    buffer[i++] = '0' + ((frac_part / 100) % 10);
    buffer[i++] = '0' + ((frac_part / 10) % 10);
    buffer[i++] = '0' + (frac_part % 10);
    buffer[i] = '\0';

    uart_puts(buffer);
}

// Funktion zum Lesen eines ADC-Kanals
uint16_t read_adc1_channel(uint8_t channel)
{
    *ADC1_SQR1 = channel << 6;  // Set channel in SQR1
    *ADC1_CR |= ADC_CR_ADSTART; // Start conversion
    while (!(*ADC1_ISR & ADC_ISR_EOC))
        ;            // Wait for end of conversion
    return *ADC1_DR; // Read converted value
}

// Messung der Back-EMF-Spannungen
void measure_bemf()
{
    // ADC-Messungen der Back-EMF-Spannungen
    bemf1 = read_adc1_channel(4);      // PA4 - Back-EMF Phase 1 (ADC1_IN4)
    bemf2 = read_adc1_channel(5);      // PC4 - Back-EMF Phase 2 (ADC2_IN5)
    bemf3 = read_adc1_channel(14);     // PB11 - Back-EMF Phase 3 (ADC1_IN14)
    vbus = read_adc1_channel(1);       // PA0 - Bus-Spannung (ADC1_IN1)
    speed_ref = read_adc1_channel(11); // PB12 - Drehzahlreferenz (ADC1_IN11)

    // Tiefpassfilter für BEMF-Messungen
    bemf1_filtered = bemf1_filtered * (1.0f - bemf_filter_alpha) + (float)bemf1 * bemf_filter_alpha;
    bemf2_filtered = bemf2_filtered * (1.0f - bemf_filter_alpha) + (float)bemf2 * bemf_filter_alpha;
    bemf3_filtered = bemf3_filtered * (1.0f - bemf_filter_alpha) + (float)bemf3 * bemf_filter_alpha;
}

// Schätzung des Rotorwinkels über BEMF
void estimate_rotor_position()
{
    // Umrechnen der ADC-Werte in Spannungen
    float bemf1_v = bemf1_filtered * ADC_VREF / 4096.0f;
    float bemf2_v = bemf2_filtered * ADC_VREF / 4096.0f;
    float bemf3_v = bemf3_filtered * ADC_VREF / 4096.0f;
    float vbus_v = (float)vbus * ADC_VREF / 4096.0f;

    // Normalisierung der Back-EMF-Spannungen
    float bemf1_norm = bemf1_v / vbus_v;
    float bemf2_norm = bemf2_v / vbus_v;
    float bemf3_norm = bemf3_v / vbus_v;

    // Berechnung von alpha und beta Komponenten (Clarke-Transformation)
    float alpha = bemf1_norm;
    float beta = (bemf1_norm + 2.0f * bemf2_norm) / SQRT3;

    // Schätzung des Rotorwinkels
    foc_state.phi = atan2f(beta, alpha);

    // Sicherstellen dass der Winkel zwischen 0 und 2*PI liegt
    if (foc_state.phi < 0)
    {
        foc_state.phi += 2.0f * PI;
    }
}

// Simple Verzögerungsfunktion
void delay(uint32_t ticks)
{
    uint32_t start = system_ticks;
    while ((system_ticks - start) < ticks)
        ;
}

// PID-Regler Berechnung
float pid_calculate(PIDController *pid, float measurement)
{
    float error = pid->setpoint - measurement;

    // P-Term
    float p_term = pid->kp * error;

    // I-Term mit Anti-Windup
    pid->integral += pid->ki * error;
    if (pid->integral > pid->output_max)
    {
        pid->integral = pid->output_max;
    }
    else if (pid->integral < pid->output_min)
    {
        pid->integral = pid->output_min;
    }

    // D-Term
    float d_term = pid->kd * (error - pid->prev_error);
    pid->prev_error = error;

    // Gesamtausgang berechnen
    float output = p_term + pid->integral + d_term;

    // Ausgangsbegrenzung
    if (output > pid->output_max)
    {
        output = pid->output_max;
    }
    else if (output < pid->output_min)
    {
        output = pid->output_min;
    }

    return output;
}

// Inverse Park-Transformation
void inverse_park_transform()
{
    float sin_phi = sinf(foc_state.phi);
    float cos_phi = cosf(foc_state.phi);

    // Hier werden die Spannungen in rotierenden Koordinaten (d-q) in stationäre Koordinaten (alpha-beta) umgerechnet
    foc_state.v_alpha = foc_state.d * cos_phi - foc_state.q * sin_phi;
    foc_state.v_beta = foc_state.d * sin_phi + foc_state.q * cos_phi;
}

// Space Vector Modulation (SVM)
void space_vector_modulation()
{
    // Sektor bestimmen (1-6)
    float v_alpha = foc_state.v_alpha;
    float v_beta = foc_state.v_beta;

    // Konvertiere v_alpha und v_beta zu den drei Phasenspannungen
    float v1 = v_alpha;
    float v2 = -0.5f * v_alpha + (SQRT3 / 2.0f) * v_beta;
    float v3 = -0.5f * v_alpha - (SQRT3 / 2.0f) * v_beta;

    // Finde das Maximum und Minimum unter den drei Phasenspannungen
    float v_max = v1;
    if (v2 > v_max)
        v_max = v2;
    if (v3 > v_max)
        v_max = v3;

    float v_min = v1;
    if (v2 < v_min)
        v_min = v2;
    if (v3 < v_min)
        v_min = v3;

    // Berechne die Nullkomponente für Center-aligned PWM
    float v_offset = 0.5f * (v_max + v_min);

    // Berechne die endgültigen Duty-Cycles (0.0 - 1.0)
    foc_state.duty1 = 0.5f + 0.5f * (v1 - v_offset);
    foc_state.duty2 = 0.5f + 0.5f * (v2 - v_offset);
    foc_state.duty3 = 0.5f + 0.5f * (v3 - v_offset);

    // Begrenze die Duty-Cycles auf 0.0 - 0.95
    if (foc_state.duty1 > 0.95f)
        foc_state.duty1 = 0.95f;
    if (foc_state.duty2 > 0.95f)
        foc_state.duty2 = 0.95f;
    if (foc_state.duty3 > 0.95f)
        foc_state.duty3 = 0.95f;

    if (foc_state.duty1 < 0.05f)
        foc_state.duty1 = 0.05f;
    if (foc_state.duty2 < 0.05f)
        foc_state.duty2 = 0.05f;
    if (foc_state.duty3 < 0.05f)
        foc_state.duty3 = 0.05f;

    // Setze die berechneten Duty-Cycles im Timer
    uint32_t ccr1 = (uint32_t)(foc_state.duty1 * PWM_PERIOD);
    uint32_t ccr2 = (uint32_t)(foc_state.duty2 * PWM_PERIOD);
    uint32_t ccr3 = (uint32_t)(foc_state.duty3 * PWM_PERIOD);

    *TIM1_CCR1 = ccr1;
    *TIM1_CCR2 = ccr2;
    *TIM1_CCR3 = ccr3;
}

// Erhöht system_ticks
void increment_ticks()
{
    system_ticks++;
}

int main(void)
{
    init_system();

    // Initial-Werte (werden in diesem Test-Modus nicht verwendet)
    foc_state.phi = 0.0f;
    foc_state.omega = 0.0f;
    foc_state.d_ref = 0.0f;

    uint32_t last_led_toggle = 0;
    uint8_t led_state = 0;

    // Testmodus: Nur LED-Blinken ohne FOC-Algorithmus
    while (1)
    {
        increment_ticks();

        // Einfaches LED-Blinken alle LED_BLINK_PERIOD Ticks
        if (system_ticks - last_led_toggle > LED_BLINK_PERIOD)
        {
            led_state = !led_state;

            if (led_state)
            {
                *GPIOC_ODR |= (1 << LED_PIN_PC6); // LED ein
            }
            else
            {
                *GPIOC_ODR &= ~(1 << LED_PIN_PC6); // LED aus
            }

            last_led_toggle = system_ticks;
        }

        // Kurze Verzögerung für stabileren Betrieb
        for (volatile int i = 0; i < 1000; i++)
        {
            // NOP-Delay
        }

        // UART Debug-Ausgabe
        uart_puts("phi=");
        uart_print_float(foc_state.phi);
        uart_puts(" d=");
        uart_print_float(foc_state.d);
        uart_puts(" q=");
        uart_print_float(foc_state.q);
        uart_puts("\r\n");
    }

    return 0;
}