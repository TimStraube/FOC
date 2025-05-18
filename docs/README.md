# FOC

⚠️ **WARNING**: Work in progress. Not recommended to use yet.

## Info

The goal of this project is to implement and test sensorless field-oriented control (FOC) on a B-G431B-ESC1 microcontroller to control BLDC-motors.

## B-G431B-ESC1

![alt text](./img/B-G431B-ESC1.png)

### Pin table from UM2516

| Pin  | Pin-Nr. | Signal             | Solder Bridge | Funktion/Beschreibung                |
|------|---------|--------------------|--------------|------------------------------------|
| -    | 1       | VBAT               | 3V3          | Batterie-Backup für RTC           |
| PC13 | 2       | TIM1_CH1N          | -            | PWM Ausgang (N-Kanal) Phase 1     |
| PC14 | 3       | CAN_TERM           | R26          | CAN Terminierung                  |
| PC15 | 4       | N.C.               | -            | Nicht verbunden                   |
| PF0  | 5       | OSC-IN             | -            | 8MHz Oszillator Eingang           |
| PF1  | 6       | OSC-OUT            | R27          | 8MHz Oszillator Ausgang           |
| PG10 | 7       | NRST               | -            | Reset-Signal                      |
| PA0  | 8       | VBUS               | -            | Bus-Spannung                      |
| PA1  | 9       | Curr_fdbk1_OPAmp+  | -            | Strommessung Phase 1 (OPAmp+)     |
| PA2  | 10      | OP1_OUT            | -            | Operationsverstärker 1 Ausgang    |
| PA3  | 11      | Curr_fdbk1_OPAmp-  | -            | Strommessung Phase 1 (OPAmp-)     |
| PA4  | 12      | BEMF1              | -            | Back-EMF Spannung Phase 1         |
| PA5  | 13      | Curr_fdbk2_OPAmp-  | -            | Strommessung Phase 2 (OPAmp-)     |
| PA6  | 14      | OP2_OUT            | -            | Operationsverstärker 2 Ausgang    |
| PA7  | 15      | Curr_fdbk2_OPAmp+  | -            | Strommessung Phase 2 (OPAmp+)     |
| PC4  | 16      | BEMF2              | -            | Back-EMF Spannung Phase 2         |
| PB0  | 17      | Curr_fdbk3_OPAmp+  | -            | Strommessung Phase 3 (OPAmp+)     |
| PB1  | 18      | TP3                | -            | Testpunkt 3                       |
| PB2  | 19      | Curr_fdbk3_OPAmp-  | -            | Strommessung Phase 3 (OPAmp-)     |
| -    | 20      | VREF+              | 3v3          | Referenzspannung                  |
| -    | 21      | VDDA               | 3v3          | Analoge Versorgung                |
| PB10 | 22      | N.C.               | -            | Nicht verbunden                   |
| -    | 23      | VDD4               | 3V3          | Digitale Versorgung               |
| PB11 | 24      | BEMF3              | -            | Back-EMF Spannung Phase 3         |
| PB12 | 25      | POTENTIOMETER      | -            | Potentiometer-Eingang             |
| PB13 | 26      | N.C.               | -            | Nicht verbunden                   |
| PB14 | 27      | Temperature feedback| -           | Temperatur-Feedback               |
| PB15 | 28      | TIM1_CH3N          | -            | PWM Ausgang (N-Kanal) Phase 3     |
| PC6  | 29      | STATUS             | -            | Status-LED                        |
| PA8  | 30      | TIM1_CH1           | -            | PWM Ausgang Phase 1               |
| PA9  | 31      | TIM1_CH2           | -            | PWM Ausgang Phase 2               |
| PA10 | 32      | TIM1_CH3           | -            | PWM Ausgang Phase 3               |
| PA11 | 33      | CAN_RX             | -            | CAN-Bus Empfang                   |
| PA12 | 34      | TIM1_CH2N          | -            | PWM Ausgang (N-Kanal) Phase 2     |
| -    | 35      | VDD6               | 3V3          | Digitale Versorgung               |
| PA13 | 36      | SWDIO              | -            | Debug Interface I/O               |
| PA14 | 37      | SWCLK              | -            | Debug Interface Clock             |
| PA15 | 38      | PWM                | -            | PWM Signal                        |
| PC10 | 39      | BUTTON             | -            | Taster-Eingang                    |
| PC11 | 40      | CAN_SHDN,TP2       | -            | CAN Shutdown, Testpunkt 2         |
| PB3  | 41      | USART2_TX          | -            | UART Senden                       |
| PB4  | 42      | USART2_RX          | -            | UART Empfangen                    |
| PB5  | 43      | GPIO_BEMF          | -            | GPIO für Back-EMF Messung         |
| PB6  | 44      | A+/H1              | -            | Hall-Sensor 1                     |
| PB7  | 45      | B+/H2              | -            | Hall-Sensor 2                     |
| PB8  | 46      | Z+/H3              | -            | Hall-Sensor 3                     |
| PB9  | 47      | CAN_TX             | -            | CAN-Bus Senden                    |
| -    | 48      | VDD8               | 3V3          | Digitale Versorgung               |

### STM32G431CB

![alt text](./img/STM32G431CB%20Pinout.png)

### L6387



### STL180N6F7

#### MOSFET Parameter
| Parameter       | Description          | Value       | Unit  |
|-----------------|----------------------|-------------|-------|
| V_DS            | Voltage drain-source | 60          | V     |
| V_GS            | Voltage gate-source  | ±20          | V     |
| R_DS_on         | Max drain-source resistance | 2.4     | mΩ    |
| I_DS            | Drain current        | 120         | A     |

#### Save operating space
![alt text](./img/GADG101220181412SOA.png)

#### Characteristics
![alt text](./img/CADG101220181450OCH.png)

## System design

### Build process 

![alt text](./img/process%20build.png)

### Programmed pins

| Pin  | Pin-Nr. | Funktion      | Standard-Zustand | Beschreibung                    |
|------|---------|---------------|------------------|--------------------------------|
| PA0  | 8       | ADC1_IN1      | Analog Input     | Bus-Spannung (VBUS)            |
| PA4  | 12      | ADC1_IN4      | Analog Input     | Back-EMF Spannung Phase 1      |
| PC4  | 16      | ADC2_IN5      | Analog Input     | Back-EMF Spannung Phase 2      |
| PB11 | 24      | ADC1_IN14     | Analog Input     | Back-EMF Spannung Phase 3      |
| PB12 | 25      | ADC1_IN11     | Analog Input     | Drehzahlreferenz-Eingang       |
| PC6  | 29      | GPIO_Output   | Output PP        | Status-LED                     |
| PA8  | 30      | TIM1_CH1      | AF1, Output PP   | PWM Ausgang Phase 1            |
| PA9  | 31      | TIM1_CH2      | AF1, Output PP   | PWM Ausgang Phase 2            |
| PA10 | 32      | TIM1_CH3      | AF1, Output PP   | PWM Ausgang Phase 3            |
| PC13 | 2       | TIM1_CH1N     | AF4, Output PP   | PWM Ausgang (N-Kanal) Phase 1  |
| PA12 | 34      | TIM1_CH2N     | AF1, Output PP   | PWM Ausgang (N-Kanal) Phase 2  |
| PB15 | 28      | TIM1_CH3N     | AF1, Output PP   | PWM Ausgang (N-Kanal) Phase 3  |
| PB3  | 41      | USART2_TX     | AF7, Output PP   | Debug UART Senden              |
| PB4  | 42      | USART2_RX     | AF7, Input       | Debug UART Empfangen           |