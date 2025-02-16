# STM32F407DISC_DAYWISE

---

## **Day 1: Introduction to STM32 and Development Environment**
### **Theory:**
- **Overview of STM32 series**
  - What is STM32?
  - Different STM32 series (F0, F1, F4, etc.)
  - Why use STM32F407 DISC1?
- **Technical specifications of STM32F407 DISC1**
  - ARM Cortex-M4 core, 32-bit, 168 MHz
  - Floating Point Unit (FPU)
  - 192 KB SRAM, 1 MB Flash
  - Peripheral interfaces (GPIO, UART, I2C, SPI, ADC, DAC, etc.)
  - USB OTG, CAN, Ethernet
- **Development tools**
  - IDEs: STM32CubeIDE, Keil uVision, IAR Embedded Workbench
  - Debugging tools: ST-Link, OpenOCD
  - STM32CubeMX for project configuration

### **Practical:**
- Install **STM32CubeIDE** and **STM32CubeMX**
- Setup **STM32F407 DISC1** with ST-Link
- Create a basic STM32 project in STM32CubeIDE
- Generate and analyze the default HAL-based **main.c** file

---

## **Day 2: GPIO (General Purpose Input Output)**
### **Theory:**
- What is GPIO?
- Pin modes: Input, Output, Alternate Function, Analog
- Push-Pull vs Open-Drain
- Pull-up/Pull-down resistors
- Speed configuration (Low, Medium, High, Very High)
- Interrupts and External Interrupts (EXTI)

### **Practical:**
- Configure **GPIO as Output** to toggle an LED
- Configure **GPIO as Input** with a Push Button
- Implement **Interrupts (EXTI)**
- Use **HAL_GPIO_WritePin()**, **HAL_GPIO_ReadPin()**

---

## **Day 3: RCC (Reset and Clock Control)**
### **Theory:**
- Clock system in STM32
- Internal (HSI, LSI) and External (HSE, LSE) clocks
- PLL (Phase Locked Loop) for frequency scaling
- Configuring system clocks using STM32CubeMX

### **Practical:**
- Configure different system clock sources
- Set CPU speed to 168 MHz using **PLL**
- Measure clock output using MCO (Microcontroller Clock Output)
- Enable **RTC clock** and observe system behavior

---

## **Day 4: UART (Universal Asynchronous Receiver Transmitter)**
### **Theory:**
- What is UART?
- Baud rate, Parity, Stop bits, Flow control
- Polling, Interrupt, and DMA-based communication
- USART vs UART in STM32

### **Practical:**
- Send and receive data using UART (Polling)
- Use UART with **Interrupt mode**
- Implement **DMA for UART transmission**
- Test communication with **Serial Monitor (Putty, TeraTerm)**

---

## **Day 5: Timer and PWM (Pulse Width Modulation)**
### **Theory:**
- Introduction to Timers (Basic, General Purpose, Advanced)
- Timer registers: CNT, PSC, ARR
- PWM generation using Timers
- Applications of PWM (LED dimming, Motor control)

### **Practical:**
- Configure **Timer as a delay generator**
- Generate **PWM signal** using TIMx
- Control **LED brightness using PWM**
- Capture **PWM input using Input Capture Mode**

---

## **Day 6: ADC (Analog to Digital Converter)**
### **Theory:**
- ADC in STM32 (Resolution, Sampling Rate)
- Conversion modes: Single, Continuous, Discontinuous
- ADC Interrupt and DMA usage
- Voltage reference and calibration

### **Practical:**
- Read **Potentiometer** value using ADC
- Display ADC readings on **UART**
- Implement ADC with **Interrupt and DMA**

---

## **Day 7: DAC (Digital to Analog Converter)**
### **Theory:**
- Introduction to DAC
- DAC resolution (8-bit, 12-bit)
- DAC modes (Normal, Triangle Wave, Noise Wave)
- Using DMA with DAC

### **Practical:**
- Generate **Analog Signal using DAC**
- Implement **Sine Wave generation**
- Use DAC with **DMA for continuous output**

---

## **Day 8: SPI (Serial Peripheral Interface)**
### **Theory:**
- SPI Protocol (Master-Slave Communication)
- Clock Phase (CPHA) and Clock Polarity (CPOL)
- Full-Duplex, Half-Duplex, and Simplex Communication
- SPI Interrupts and DMA

### **Practical:**
- **SPI loopback test** (self-communication)
- Communicate between **two STM32 boards using SPI**
- Use **SPI with an external sensor/module (e.g., OLED, ADC chip)**

---

## **Day 9: I2C (Inter-Integrated Circuit)**
### **Theory:**
- I2C basics (Start, Stop, ACK/NACK)
- Standard, Fast, and High-Speed Modes
- Addressing modes (7-bit, 10-bit)
- I2C vs SPI comparison

### **Practical:**
- Interface **I2C EEPROM (24Cxx)**
- Read and write data using HAL_I2C functions
- Communicate with **I2C-based sensors (e.g., MPU6050, BMP180)**

---

## **Day 10: External Interrupts and NVIC (Nested Vector Interrupt Controller)**
### **Theory:**
- What are interrupts?
- NVIC and IRQ handling
- Priority levels in STM32
- ISR (Interrupt Service Routine) execution flow

### **Practical:**
- Configure **External Interrupt (Button Press)**
- Implement **Debouncing for push button**
- Control **LED using EXTI interrupts**

---

## **Day 11: Real-Time Clock (RTC) and Watchdog Timer**
### **Theory:**
- RTC functionality and battery backup
- Alarm, Wakeup Timer, and Timestamp
- Independent Watchdog (IWDG) vs Window Watchdog (WWDG)

### **Practical:**
- Set **RTC clock and display time on UART**
- Configure **RTC alarm and wake up from Sleep Mode**
- Implement **Watchdog Timer to reset MCU on failure**

---

## **Day 12: Low Power Modes and Power Management**
### **Theory:**
- Power modes (Sleep, Stop, Standby)
- Reducing power consumption in STM32
- Waking up MCU from low-power mode

### **Practical:**
- Put STM32 in **Sleep Mode**
- Wake up using **External Interrupt**
- Implement **Stop Mode and Standby Mode**

---

## **Day 13: USB Communication (USB CDC)**
### **Theory:**
- USB communication in STM32
- USB as Virtual COM Port (CDC)
- Configuring USB using HAL

### **Practical:**
- Implement **USB-to-Serial communication**
- Test USB CDC using a **PC terminal**
- Send/Receive data over USB

---

## **Day 14: FreeRTOS Basics with STM32**
### **Theory:**
- What is an RTOS? (Real-Time Operating System)
- FreeRTOS Tasks, Queues, Semaphores
- Task Scheduling (Preemptive vs Cooperative)
- Memory Management in FreeRTOS

### **Practical:**
- Create **two FreeRTOS tasks**
- Implement **Task Synchronization using Semaphores**
- Use **Queue for Inter-task Communication**

---

# **Day 1: Introduction to STM32 and Development Environment**

## **1. Overview of STM32 Series**
STM32 is a family of 32-bit microcontrollers developed by **STMicroelectronics**, based on the **ARM Cortex-M** processor architecture. It is widely used in **embedded systems, IoT, automotive, and industrial automation** due to its **high performance, low power consumption, and extensive peripheral support**.

### **Why STM32?**
- **High Processing Power** – Uses ARM Cortex-M cores, offering better performance than 8-bit and 16-bit MCUs.
- **Scalability** – Available in different series (F0, F1, F4, etc.), suitable for various applications.
- **Low Power** – Some series are optimized for battery-powered applications.
- **Rich Peripheral Set** – Supports **UART, SPI, I2C, CAN, Ethernet, USB, ADC, DAC, Timers, PWM, RTC**, etc.
- **Flexible Clock System** – Offers internal and external clock sources with **PLL-based frequency scaling**.
- **Development Tools** – Compatible with **STM32CubeIDE, Keil, IAR, and open-source tools**.

---

## **2. What is STM32?**
STM32 is a **series of microcontrollers (MCUs)** based on the **ARM Cortex-M** architecture. It is widely used for **real-time embedded applications** due to its **high-speed processing, power efficiency, and feature-rich peripherals**.

### **Key Features of STM32 MCUs:**
| Feature | Description |
|---------|------------|
| **Processor Core** | ARM Cortex-M0/M0+/M3/M4/M7 |
| **Clock Speed** | Ranges from **48 MHz to 550 MHz** |
| **Flash Memory** | **Up to 2 MB** for storing program code |
| **SRAM** | **Up to 1 MB** for data storage |
| **Peripherals** | GPIO, ADC, DAC, UART, SPI, I2C, CAN, USB, Ethernet, Timers, PWM, RTC, etc. |
| **Operating Voltage** | **1.8V - 3.6V** |
| **Power Modes** | Low Power, Sleep, Stop, Standby |
| **DSP & FPU Support** | Available in Cortex-M4 and M7 |

STM32 MCUs are grouped into **different series** based on application type, processing power, and power consumption.

---

## **3. Different STM32 Series (F0, F1, F4, etc.)**
STMicroelectronics classifies STM32 into several series based on **performance, power consumption, and target applications**.

| Series | Core | Clock Speed | Features | Application |
|--------|------|------------|----------|-------------|
| **STM32F0** | Cortex-M0 | Up to 48 MHz | Low-cost, Low-power | Basic embedded systems, IoT |
| **STM32F1** | Cortex-M3 | Up to 72 MHz | General-purpose | Industrial control, Home automation |
| **STM32F2** | Cortex-M3 | Up to 120 MHz | Advanced peripherals | Networking, Automation |
| **STM32F3** | Cortex-M4 | Up to 72 MHz | DSP and Floating Point Unit (FPU) | Sensor interfacing, Motor control |
| **STM32F4** | Cortex-M4 | Up to 180 MHz | High-performance, DSP, FPU | Robotics, Image processing |
| **STM32F7** | Cortex-M7 | Up to 216 MHz | High-speed, Advanced memory | Advanced AI/ML, Graphics |
| **STM32H7** | Cortex-M7 | Up to 550 MHz | Dual-core, Very High Performance | AI, Industrial Automation |
| **STM32L0/L1** | Cortex-M0/M3 | Low Power | Energy-efficient | Wearables, Battery-powered devices |
| **STM32L4/L5** | Cortex-M4/M33 | Up to 120 MHz | Ultra Low Power | Medical devices, IoT |
| **STM32G0/G4** | Cortex-M0+/M4 | Mid-range performance | High Efficiency | Industrial control |

### **STM32F4 vs Other Series**
| Feature | STM32F1 | STM32F3 | **STM32F4** | STM32F7 |
|---------|---------|---------|------------|---------|
| **Core** | Cortex-M3 | Cortex-M4 | Cortex-M4 | Cortex-M7 |
| **Speed** | 72 MHz | 72 MHz | **168-180 MHz** | 216 MHz |
| **Floating Point Unit** | No | Yes | **Yes** | **Yes** |
| **Power Consumption** | Medium | Medium | **High** | High |
| **Best For** | General | DSP & Motor Control | **High Performance, DSP, AI** | AI, Machine Learning |

---

## **4. Why Use STM32F407 DISC1?**
STM32F407 DISC1 (Discovery Board) is a **high-performance evaluation board** based on the **STM32F407VG** microcontroller. It is ideal for **learning and prototyping**.

### **Specifications of STM32F407 DISC1:**
| Feature | Description |
|---------|------------|
| **MCU** | STM32F407VG (Cortex-M4, 32-bit) |
| **Clock Speed** | **168 MHz** |
| **Flash Memory** | **1 MB** |
| **SRAM** | **192 KB** |
| **FPU (Floating Point Unit)** | **Yes** |
| **ADC** | 12-bit, 3.6 MSPS |
| **DAC** | 2 Channels |
| **Communication** | UART, SPI, I2C, CAN, USB OTG |
| **Timers** | Advanced Timers for PWM & Capture |
| **Power Supply** | USB or External 5V |
| **Other Features** | Built-in accelerometer, Audio DAC, ST-Link Debugger |

### **Advantages of STM32F407 DISC1**
- **High Processing Speed** (**168 MHz**) suitable for real-time applications.
- **Built-in Debugger (ST-Link/V2)** – No need for an external programmer.
- **Onboard Peripherals** – LEDs, Push Buttons, MEMS Accelerometer.
- **Support for FreeRTOS** – Can run an RTOS for multi-tasking.
- **USB OTG Support** – Can act as **USB Host and Device**.
- **Multiple Communication Interfaces** – UART, SPI, I2C, CAN, Ethernet.
- **Rich Development Ecosystem** – Supported by STM32CubeMX, Keil, IAR.

### **Applications of STM32F407 DISC1**
- **Embedded Systems Prototyping**
- **Real-time Signal Processing (DSP)**
- **Robotics and Motor Control**
- **Sensor Data Acquisition**
- **IoT and Wireless Communication**
- **AI & Machine Learning (Edge AI)**
- **Multimedia Applications (Audio Processing)**

---

## **Conclusion**
STM32 is a **powerful and versatile** family of microcontrollers, offering **high performance and a rich set of peripherals**. STM32F407 DISC1 is a great choice for **learning and developing embedded systems projects**, thanks to its **high clock speed, DSP capabilities, and extensive support for communication protocols**.

---


# **Day 2: GPIO (General Purpose Input/Output)**

## **Theory**

### **1. What is GPIO?**
GPIO (General Purpose Input/Output) is one of the most fundamental peripherals in microcontrollers. It allows the microcontroller to interact with external components like **LEDs, switches, sensors, motors, and displays**.

Each **GPIO pin** can be configured in different modes:
- **Input Mode** (for reading values from sensors or buttons)
- **Output Mode** (for controlling LEDs, motors, etc.)
- **Alternate Function Mode** (for communication protocols like UART, SPI, I2C)
- **Analog Mode** (for ADC/DAC operations)

In **STM32F407**, GPIOs are grouped into **ports** (GPIOA, GPIOB, GPIOC, etc.), and each port has **16 pins** labeled **Px0 to Px15** (e.g., PA0, PB5, PC13).

---

### **2. GPIO Pin Modes**
GPIO pins can operate in four different modes:

| Mode | Description | Example Use |
|------|------------|-------------|
| **Input** | Reads digital signals (HIGH/LOW) | Push buttons, Sensors |
| **Output** | Drives digital signals (HIGH/LOW) | LEDs, Relays, Motors |
| **Alternate Function** | Used for peripheral functions | UART, SPI, I2C, PWM |
| **Analog** | Used for ADC/DAC | Reading sensor values, Audio processing |

---

### **3. Push-Pull vs Open-Drain**
When a GPIO is configured as an **Output**, it can operate in either:
- **Push-Pull Mode** (default)
- **Open-Drain Mode**

| Mode | Description | Application |
|------|------------|-------------|
| **Push-Pull** | Actively drives **HIGH (VCC)** or **LOW (GND)** | LEDs, Relays |
| **Open-Drain** | Can only pull the pin **LOW** (Needs an external pull-up resistor for HIGH) | I2C communication, Multi-device bus |

---

### **4. Pull-up/Pull-down Resistors**
GPIO input pins need **pull-up or pull-down resistors** to define a default state.

| Type | Function | Application |
|------|----------|-------------|
| **Pull-up** | Keeps pin HIGH when no input is given | Button as Active-Low |
| **Pull-down** | Keeps pin LOW when no input is given | Button as Active-High |

STM32 allows enabling **internal pull-up/down resistors** via software, eliminating the need for external components.

---

### **5. Speed Configuration (Low, Medium, High, Very High)**
STM32 GPIOs can be configured for different **speed levels**, affecting power consumption and signal integrity.

| Speed | Max Frequency | Application |
|-------|--------------|-------------|
| **Low** | ~2 MHz | Low-speed communication |
| **Medium** | ~10 MHz | General-purpose applications |
| **High** | ~50 MHz | High-speed GPIO switching |
| **Very High** | ~100 MHz | High-frequency signals, Clocks |

Faster speeds increase **power consumption** and may cause **signal integrity issues** if not properly handled.

---

### **6. Interrupts and External Interrupts (EXTI)**
GPIOs can be used to trigger **interrupts**, allowing the microcontroller to respond to events (e.g., button press) without continuously polling the pin.

- **GPIO Interrupts (EXTI - External Interrupt)**
  - Allows STM32 to **wake up from sleep mode** or execute tasks when an input changes.
  - Configured using **HAL_EXTI API**.

| EXTI Type | Trigger Condition |
|-----------|------------------|
| **Rising Edge** | Interrupt triggers when the pin changes from LOW → HIGH |
| **Falling Edge** | Interrupt triggers when the pin changes from HIGH → LOW |
| **Both Edges** | Interrupt triggers on any change in state |

---

## **Practical Implementation**
### **1. Configure GPIO as Output to Toggle an LED**
🔹 **Goal:** Turn an LED ON and OFF using GPIO Output Mode.

**Steps:**
1. Open **STM32CubeIDE**.
2. Configure a GPIO pin (e.g., **PA5**) as **Output**.
3. Use **HAL_GPIO_WritePin()** to toggle the LED.

```c
#include "stm32f4xx_hal.h"

void SystemClock_Config(void);
void GPIO_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();

    while (1) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(500); // 500ms delay
    }
}

void GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
```
---

### **2. Configure GPIO as Input with a Push Button**
🔹 **Goal:** Read a button press and turn on an LED.

**Steps:**
1. Configure a button pin (e.g., **PC13**) as **Input**.
2. Configure an LED pin (e.g., **PA5**) as **Output**.
3. Use **HAL_GPIO_ReadPin()** to check button state.

```c
void GPIO_Init(void) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure LED pin as Output
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure Button pin as Input
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // Use Pull-up since button is Active-Low
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();

    while (1) {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        }
    }
}
```

---

### **3. Implement Interrupts (EXTI)**
🔹 **Goal:** Trigger an interrupt when a button is pressed.

**Steps:**
1. Configure a GPIO pin (e.g., **PC13**) as an **External Interrupt (EXTI)**.
2. Define an **Interrupt Handler** function.
3. Toggle an LED when the interrupt occurs.

```c
void GPIO_Init(void) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // LED as Output
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Button as External Interrupt
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // Falling edge trigger
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Enable EXTI Line Interrupt
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

// Interrupt Handler
void EXTI15_10_IRQHandler(void) {
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Toggle LED
    }
}
```

---

## **Summary**
- **GPIO is used for input/output operations**.
- **Push-Pull vs Open-Drain** determines output behavior.
- **Pull-up/Pull-down resistors** help define a default state.
- **GPIO Interrupts (EXTI)** allow event-driven execution.
- **HAL functions**: 
  - `HAL_GPIO_WritePin()`
  - `HAL_GPIO_ReadPin()`
  - `HAL_NVIC_EnableIRQ()`


# **Day 3: RCC (Reset and Clock Control)**

## **Theory**

### **1. Clock System in STM32**
The clock system in STM32 microcontrollers plays a crucial role in determining the performance and power consumption of the system. The **Reset and Clock Control (RCC)** module manages the system clock and peripheral clocks.

#### **Key Components of the Clock System:**
- **System Clock (SYSCLK):** The main clock driving the CPU and peripherals.
- **AHB (Advanced High-Performance Bus) Clock:** Drives high-speed peripherals like GPIO, DMA, etc.
- **APB1 (Advanced Peripheral Bus 1) Clock:** Drives low-speed peripherals (Timers, UART, SPI, I2C, etc.).
- **APB2 (Advanced Peripheral Bus 2) Clock:** Drives high-speed peripherals (USART1, SPI1, ADC1, etc.).

In STM32F407, the maximum **CPU clock frequency is 168 MHz**.

---

### **2. Internal (HSI, LSI) and External (HSE, LSE) Clocks**
STM32 has multiple clock sources:

| Clock Source | Frequency | Purpose |
|-------------|-----------|---------|
| **HSI (High-Speed Internal)** | 16 MHz | Used as the default system clock |
| **HSE (High-Speed External)** | 8-25 MHz | Used with an external crystal for accuracy |
| **LSI (Low-Speed Internal)** | ~32 kHz | Used for RTC (low power) |
| **LSE (Low-Speed External)** | 32.768 kHz | Used for RTC with external crystal |

#### **HSI vs HSE**
- **HSI** is factory-calibrated but less accurate.
- **HSE** requires an external crystal, providing higher accuracy for communication protocols like UART.

#### **LSI vs LSE**
- **LSI** is an internal clock for low-power applications (less accurate).
- **LSE** is a highly accurate external crystal (used for RTC and watchdog timers).

---

### **3. PLL (Phase Locked Loop) for Frequency Scaling**
The **Phase Locked Loop (PLL)** is used to increase the system clock speed. The PLL takes a lower-frequency input (HSI or HSE) and multiplies it to achieve higher speeds.

**Formula for calculating PLL Output Frequency:**
\[
f_{PLL} = \frac{f_{input}}{M} \times N \div P
\]

- **M (Division Factor)**: Reduces input frequency before multiplication.
- **N (Multiplication Factor)**: Increases the frequency.
- **P (Division Factor)**: Selects the final system clock.

For **168 MHz system clock**, using **HSE = 8 MHz**:
- **M = 8**
- **N = 336**
- **P = 2**

\[
\frac{8}{8} \times 336 \div 2 = 168 MHz
\]

---

### **4. Configuring System Clocks using STM32CubeMX**
STM32CubeMX provides a graphical interface to configure clocks easily.

**Steps to set SYSCLK to 168 MHz using PLL:**
1. Open **STM32CubeMX**.
2. Select **STM32F407VGTx**.
3. Go to **Clock Configuration** tab.
4. Select **HSE (8 MHz) as the PLL source**.
5. Set:
   - **M = 8**
   - **N = 336**
   - **P = 2**
6. Set AHB and APB clocks:
   - **AHB Prescaler = 1** (168 MHz)
   - **APB1 Prescaler = 4** (42 MHz)
   - **APB2 Prescaler = 2** (84 MHz)
7. Click **Generate Code**.

---

## **Practical Implementation**
### **1. Configure Different System Clock Sources**
🔹 **Goal:** Switch between HSI, HSE, and PLL.

```c
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Configure HSE as clock source
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        while (1);
    }

    // Configure the SYSCLK and buses
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        while (1);
    }
}
```

---

### **2. Set CPU Speed to 168 MHz using PLL**
🔹 **Goal:** Configure PLL to achieve **168 MHz system clock**.

```c
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Enable HSE and configure PLL
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        while (1);
    }

    // Configure system clock
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        while (1);
    }
}
```

---

### **3. Measure Clock Output using MCO (Microcontroller Clock Output)**
🔹 **Goal:** Output the system clock signal on **PA8 (MCO1)** to measure using an oscilloscope.

```c
void GPIO_MCO_Config(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Select MCO1 source as SYSCLK
    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
}
```

---

### **4. Enable RTC Clock and Observe System Behavior**
🔹 **Goal:** Enable the RTC clock using **LSE (32.768 kHz)**.

```c
void RTC_Config(void) {
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSE_CONFIG(RCC_LSE_ON);

    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    __HAL_RCC_RTC_ENABLE();
}
```

---

## **Summary**
- **STM32 clock sources:** HSI, HSE, LSI, LSE.
- **PLL multiplies clock frequency** to achieve 168 MHz.
- **MCO outputs clock signals** for debugging.
- **RTC uses LSE for accurate timekeeping**.

# **Day 4: UART (Universal Asynchronous Receiver Transmitter)**

## **Theory**

### **1. What is UART?**
**UART (Universal Asynchronous Receiver-Transmitter)** is a serial communication protocol that enables data exchange between devices over two wires:
- **TX (Transmit)**
- **RX (Receive)**

It is **asynchronous**, meaning it does not require a clock signal like SPI or I2C.

---

### **2. Key UART Parameters**
| Parameter | Description |
|-----------|------------|
| **Baud Rate** | Speed of data transmission (e.g., 9600, 115200 bps) |
| **Parity** | Error detection mechanism (**None, Even, Odd**) |
| **Stop Bits** | Defines end of transmission (**1, 1.5, or 2 bits**) |
| **Flow Control** | Controls data flow (**None, RTS/CTS**) |

**Baud Rate Formula:**
\[
BaudRate = \frac{USART\ Clock}{16 \times USARTDIV}
\]

---

### **3. Polling, Interrupt, and DMA-based Communication**
#### **1️⃣ Polling Mode**
- CPU continuously checks UART status.
- Simple but inefficient (CPU is busy waiting).

#### **2️⃣ Interrupt Mode**
- UART generates an **interrupt** when data is available.
- CPU executes other tasks while waiting for data.
- More efficient than polling.

#### **3️⃣ DMA Mode**
- **Direct Memory Access (DMA)** moves data between memory and UART without CPU involvement.
- Ideal for high-speed, continuous data transfer.

---

### **4. USART vs UART in STM32**
| Feature | UART | USART |
|---------|------|-------|
| **Clock Signal** | No | Optional |
| **Full-Duplex** | Yes | Yes |
| **Half-Duplex** | Yes | Yes |
| **Synchronous Mode** | No | Yes (Uses External Clock) |
| **STM32 Support** | Most STM32 MCUs support USART (which can work as UART) |

---

## **Practical Implementation**
### **1. Send and Receive Data using UART (Polling)**
🔹 **Goal:** Transmit and receive data using **polling mode**.

```c
#include "stm32f4xx_hal.h"

UART_HandleTypeDef huart2;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART2_UART_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    char msg[] = "Hello UART!\r\n";

    while (1) {
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg) - 1, HAL_MAX_DELAY);
        HAL_Delay(1000);
    }
}

// UART2 Initialization (Baud rate: 115200)
void MX_USART2_UART_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        while (1);
    }
}
```
📌 **Expected Output:** "Hello UART!" will be printed every second on the serial monitor.

---

### **2. Use UART with Interrupt Mode**
🔹 **Goal:** Receive data using **interrupt-based communication**.

```c
#include "stm32f4xx_hal.h"

UART_HandleTypeDef huart2;
uint8_t rxData;

void SystemClock_Config(void);
void MX_USART2_UART_Init(void);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        HAL_UART_Transmit(&huart2, &rxData, 1, 100);
        HAL_UART_Receive_IT(&huart2, &rxData, 1);
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_USART2_UART_Init();

    HAL_UART_Receive_IT(&huart2, &rxData, 1);

    while (1);
}

// UART2 Initialization
void MX_USART2_UART_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK) {
        while (1);
    }
}
```
📌 **Expected Behavior:** Any received character is echoed back.

---

### **3. Implement DMA for UART Transmission**
🔹 **Goal:** Send large data without CPU intervention using **DMA mode**.

```c
#include "stm32f4xx_hal.h"

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

char dmaMsg[] = "UART DMA Transmission\r\n";

void SystemClock_Config(void);
void MX_USART2_UART_Init(void);
void MX_DMA_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_DMA_Init();
    MX_USART2_UART_Init();

    while (1) {
        HAL_UART_Transmit_DMA(&huart2, (uint8_t *)dmaMsg, sizeof(dmaMsg) - 1);
        HAL_Delay(1000);
    }
}

// UART2 Initialization
void MX_USART2_UART_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK) {
        while (1);
    }
}

// DMA Initialization
void MX_DMA_Init(void) {
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_usart2_tx.Instance = DMA1_Stream6;
    hdma_usart2_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;

    HAL_DMA_Init(&hdma_usart2_tx);
    __HAL_LINKDMA(&huart2, hdmatx, hdma_usart2_tx);
}
```
📌 **Expected Behavior:** "UART DMA Transmission" is sent every second via DMA.

---

### **4. Test Communication with Serial Monitor**
To verify data transmission:
1. **Connect UART to PC** via USB-to-Serial (e.g., FTDI, CP2102).
2. **Open** **Putty/TeraTerm**.
3. Set:
   - Baud rate: **115200**
   - Data Bits: **8**
   - Parity: **None**
   - Stop Bits: **1**
   - Flow Control: **None**
4. Observe the transmitted messages.

---

## **Summary**
- **Polling Mode** is simple but inefficient.
- **Interrupt Mode** improves CPU efficiency.
- **DMA Mode** is best for high-speed data transfer.
- **Test using Putty/TeraTerm** to verify output.


# **Day 5: Timer and PWM (Pulse Width Modulation)**

## **Theory**

### **1. Introduction to Timers in STM32**
STM32 MCUs have different types of timers:
- **Basic Timers (TIM6, TIM7)** → Used for time base generation (e.g., delays).
- **General Purpose Timers (TIM2-TIM5, TIM9-TIM14)** → Used for timekeeping, PWM, input capture, and output compare.
- **Advanced Timers (TIM1, TIM8)** → Used for motor control, advanced PWM with dead-time insertion.

---

### **2. Timer Registers**
| Register | Description |
|----------|------------|
| **CNT (Counter Register)** | Holds current timer count value |
| **PSC (Prescaler Register)** | Divides input clock to slow down the timer |
| **ARR (Auto-Reload Register)** | Defines timer overflow (maximum count value) |

#### **Timer Frequency Formula**
\[
F_{timer} = \frac{F_{clock}}{(PSC + 1) \times (ARR + 1)}
\]

---

### **3. PWM (Pulse Width Modulation)**
PWM generates a square wave with a variable **duty cycle**.

| **Duty Cycle (%)** | **ON Time / Period** |
|--------------------|----------------------|
| 0%  | Always OFF |
| 50% | Equal ON and OFF times |
| 100% | Always ON |

🛠 **Applications of PWM**:
- LED brightness control
- Motor speed control
- Servo motor positioning
- Audio signal generation

---

## **Practical Implementation**

### **1. Configure Timer as a Delay Generator**
🔹 **Goal:** Use TIM2 to generate a 1-second delay.

```c
#include "stm32f4xx_hal.h"

TIM_HandleTypeDef htim2;

void SystemClock_Config(void);
void MX_TIM2_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_TIM2_Init();

    HAL_TIM_Base_Start(&htim2);

    while (1) {
        while (__HAL_TIM_GET_COUNTER(&htim2) < 1000); // Wait 1 second
        __HAL_TIM_SET_COUNTER(&htim2, 0); // Reset counter
    }
}

// Timer2 Initialization
void MX_TIM2_Init(void) {
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 8399;  // 84MHz / (8399+1) = 10kHz
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 9999;     // 10kHz / 10k = 1 sec
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        while (1);
    }
}
```
📌 **Expected Behavior:** Timer increments and resets every second.

---

### **2. Generate PWM Signal using TIMx**
🔹 **Goal:** Generate a **1kHz PWM signal with 50% duty cycle** using TIM3.

```c
#include "stm32f4xx_hal.h"

TIM_HandleTypeDef htim3;

void SystemClock_Config(void);
void MX_TIM3_Init(void);
void MX_GPIO_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM3_Init();

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    while (1);
}

// Timer3 PWM Initialization
void MX_TIM3_Init(void) {
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 83;   // 84MHz / (83+1) = 1MHz
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999;     // 1MHz / 1000 = 1kHz PWM
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        while (1);
    }

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 500;  // 50% Duty Cycle (500/1000)
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
}
```
📌 **Expected Behavior:** A **1kHz PWM signal** is generated on TIM3 Channel 1.

---

### **3. Control LED Brightness using PWM**
🔹 **Goal:** Vary LED brightness using **PWM duty cycle**.

```c
#include "stm32f4xx_hal.h"

TIM_HandleTypeDef htim3;

void SystemClock_Config(void);
void MX_TIM3_Init(void);
void MX_GPIO_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM3_Init();

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    while (1) {
        for (uint16_t duty = 0; duty <= 999; duty += 100) {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
            HAL_Delay(100);
        }
    }
}

// Timer3 PWM Initialization
void MX_TIM3_Init(void) {
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 83;   
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999;     
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        while (1);
    }

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
}
```
📌 **Expected Behavior:** LED brightness gradually increases.

---

### **4. Capture PWM Input using Input Capture Mode**
🔹 **Goal:** Measure the frequency and duty cycle of an **external PWM signal**.

```c
#include "stm32f4xx_hal.h"

TIM_HandleTypeDef htim2;
uint32_t IC_Val1 = 0, IC_Val2 = 0, Difference = 0;
uint8_t Is_First_Captured = 0;
float Frequency = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        if (Is_First_Captured == 0) {
            IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            Is_First_Captured = 1;
        } else {
            IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            Difference = (IC_Val2 > IC_Val1) ? (IC_Val2 - IC_Val1) : ((0xFFFF - IC_Val1) + IC_Val2);
            Frequency = 84000000 / Difference; // Assuming 84MHz Timer Clock
            Is_First_Captured = 0;
        }
    }
}

void MX_TIM2_IC_Init(void) {
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 83;  // 1MHz Timer Clock
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFF;
    
    HAL_TIM_IC_Init(&htim2);

    TIM_IC_InitTypeDef sConfigIC = {0};
    sConfigIC.ICPolarity = TIM_ICPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    
    HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_TIM2_IC_Init();

    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

    while (1);
}
```
📌 **Expected Behavior:** Captures PWM frequency and prints it.

---

## **Summary**
- **Timers** generate delays, PWM signals, and capture external inputs.
- **PWM** is useful for LED dimming and motor control.
- **Input Capture Mode** measures external PWM frequencies.


# **Day 6: ADC (Analog to Digital Converter)**

## **Theory**

### **1. ADC in STM32**
An **Analog-to-Digital Converter (ADC)** converts an analog voltage signal into a digital value.

| **Feature** | **Description** |
|------------|----------------|
| **Resolution** | 12-bit (0-4095), 10-bit (0-1023), 8-bit (0-255) |
| **Sampling Rate** | Defines how fast the ADC samples the input |
| **Channels** | Multiple input pins can be used (e.g., ADC1, ADC2) |
| **Reference Voltage (Vref)** | Determines the max ADC value (e.g., 3.3V → 4095 for 12-bit) |

#### **ADC Formula**
\[
V_{input} = \frac{ADC_{Value}}{2^n - 1} \times V_{ref}
\]
Where:
- **n** = ADC resolution (e.g., 12-bit → 4095 max)
- **Vref** = Reference voltage (e.g., 3.3V)

---

### **2. ADC Conversion Modes**
1. **Single Conversion Mode** – Converts once and stops.
2. **Continuous Conversion Mode** – Repeats conversions.
3. **Discontinuous Mode** – Converts a set number of samples.
4. **Scan Mode** – Reads multiple channels sequentially.

---

### **3. ADC with Interrupt and DMA**
- **Polling Mode** → Blocks CPU until conversion is complete.
- **Interrupt Mode** → CPU is notified when conversion finishes.
- **DMA Mode** → Directly transfers ADC data to memory without CPU usage.

---

## **Practical Implementation**

### **1. Read Potentiometer Value using ADC**
🔹 **Goal:** Read **analog voltage from a potentiometer** and print it over UART.

#### **Connections**
- **Potentiometer**:
  - VCC → 3.3V
  - GND → GND
  - Output → **PA0 (ADC1_IN0)**

#### **Code:**
```c
#include "stm32f4xx_hal.h"
#include <stdio.h>

ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;
char msg[50];

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_ADC1_Init(void);
void MX_USART2_UART_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_USART2_UART_Init();

    while (1) {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        uint32_t adc_val = HAL_ADC_GetValue(&hadc1);
        
        float voltage = (adc_val / 4095.0) * 3.3;
        sprintf(msg, "ADC Value: %ld, Voltage: %.2fV\r\n", adc_val, voltage);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        
        HAL_Delay(500);
    }
}

void MX_ADC1_Init(void) {
    hadc1.Instance = ADC1;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;

    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        while (1);
    }

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}
```
📌 **Expected Behavior:** ADC value and voltage are printed via UART every 500ms.

---

### **2. ADC with Interrupt Mode**
🔹 **Goal:** Read ADC using **interrupts** instead of polling.

```c
#include "stm32f4xx_hal.h"
#include <stdio.h>

ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;
char msg[50];
uint32_t adc_value;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_ADC1_Init(void);
void MX_USART2_UART_Init(void);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        adc_value = HAL_ADC_GetValue(&hadc1);
        float voltage = (adc_value / 4095.0) * 3.3;
        sprintf(msg, "ADC Interrupt: %ld, Voltage: %.2fV\r\n", adc_value, voltage);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_USART2_UART_Init();

    HAL_ADC_Start_IT(&hadc1); // Start ADC in Interrupt mode

    while (1);
}
```
📌 **Expected Behavior:** UART prints ADC values whenever an interrupt occurs.

---

### **3. ADC with DMA (Direct Memory Access)**
🔹 **Goal:** Use DMA to transfer ADC data to memory **without CPU involvement**.

```c
#include "stm32f4xx_hal.h"
#include <stdio.h>

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
UART_HandleTypeDef huart2;
char msg[50];
uint32_t adc_val;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_ADC1_Init(void);
void MX_USART2_UART_Init(void);
void MX_DMA_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_USART2_UART_Init();

    HAL_ADC_Start_DMA(&hadc1, &adc_val, 1);

    while (1) {
        float voltage = (adc_val / 4095.0) * 3.3;
        sprintf(msg, "ADC DMA: %ld, Voltage: %.2fV\r\n", adc_val, voltage);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        
        HAL_Delay(500);
    }
}

void MX_DMA_Init(void) {
    __HAL_RCC_DMA2_CLK_ENABLE();
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;

    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
        while (1);
    }

    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
}
```
📌 **Expected Behavior:** ADC data is continuously stored in memory and printed via UART.

---

## **Summary**
- **ADC Polling Mode** → Waits for conversion (simple but inefficient).
- **ADC Interrupt Mode** → Notifies CPU when conversion is complete.
- **ADC DMA Mode** → Transfers ADC data to memory without CPU usage.


# **Day 7: DAC (Digital to Analog Converter)**

## **Theory**

### **1. Introduction to DAC**
A **Digital-to-Analog Converter (DAC)** converts digital values into an analog voltage signal.

| **Feature**   | **Description** |
|--------------|----------------|
| **Resolution** | 8-bit (0-255) or 12-bit (0-4095) |
| **Reference Voltage (Vref)** | Determines the output voltage range |
| **Output Modes** | Normal, Triangle Wave, Noise Wave |
| **DMA Support** | Allows continuous waveform generation without CPU intervention |

#### **DAC Output Formula**
\[
V_{out} = \frac{DAC_{Value}}{2^n - 1} \times V_{ref}
\]
Where:
- **n** = DAC resolution (8-bit or 12-bit)
- **Vref** = Reference voltage (e.g., 3.3V)

---

### **2. DAC Modes**
1. **Normal Mode** → Directly outputs digital values as analog voltage.
2. **Triangle Wave Mode** → Generates a triangular waveform.
3. **Noise Wave Mode** → Outputs random noise for testing.

---

### **3. DAC with DMA**
- **Without DMA**: CPU continuously writes values to the DAC.
- **With DMA**: DAC fetches data from memory automatically for continuous waveform generation.

---

## **Practical Implementation**

### **1. Generate Analog Signal using DAC**
🔹 **Goal:** Output a fixed analog voltage on **PA4 (DAC Channel 1)**.

#### **Connections**
- **PA4 (DAC_OUT1)** → Connect to an oscilloscope or LED driver.

#### **Code:**
```c
#include "stm32f4xx_hal.h"

DAC_HandleTypeDef hdac;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DAC_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DAC_Init();

    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
    
    while (1) {
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048); // ~1.65V for 3.3V Vref
        HAL_Delay(1000);
    }
}

void MX_DAC_Init(void) {
    hdac.Instance = DAC;
    HAL_DAC_Init(&hdac);

    DAC_ChannelConfTypeDef sConfig = {0};
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1);
}
```
📌 **Expected Behavior:** A fixed **1.65V output** (for Vref = 3.3V).

---

### **2. Generate Sine Wave using DAC**
🔹 **Goal:** Generate a **sine wave** using lookup table.

#### **Code:**
```c
#include "stm32f4xx_hal.h"
#include <math.h>

DAC_HandleTypeDef hdac;
TIM_HandleTypeDef htim6;

#define SINE_SAMPLES 32
uint32_t sine_wave[SINE_SAMPLES];

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DAC_Init(void);
void MX_TIM6_Init(void);

void Generate_SineWave(void) {
    for (int i = 0; i < SINE_SAMPLES; i++) {
        sine_wave[i] = (uint32_t)((sin(2 * M_PI * i / SINE_SAMPLES) + 1) * 2048);
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DAC_Init();
    MX_TIM6_Init();

    Generate_SineWave();

    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
    HAL_TIM_Base_Start(&htim6);

    while (1) {
        for (int i = 0; i < SINE_SAMPLES; i++) {
            HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, sine_wave[i]);
            HAL_Delay(1);
        }
    }
}

void MX_TIM6_Init(void) {
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 84 - 1;
    htim6.Init.Period = 10 - 1;
    HAL_TIM_Base_Init(&htim6);
}
```
📌 **Expected Behavior:** Generates a **sine wave** on PA4.

---

### **3. Use DAC with DMA for Continuous Output**
🔹 **Goal:** Output a **continuous sine wave using DMA**.

#### **Code:**
```c
#include "stm32f4xx_hal.h"
#include <math.h>

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;
TIM_HandleTypeDef htim6;

#define SINE_SAMPLES 32
uint32_t sine_wave[SINE_SAMPLES];

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DAC_Init(void);
void MX_DMA_Init(void);
void MX_TIM6_Init(void);

void Generate_SineWave(void) {
    for (int i = 0; i < SINE_SAMPLES; i++) {
        sine_wave[i] = (uint32_t)((sin(2 * M_PI * i / SINE_SAMPLES) + 1) * 2048);
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_DAC_Init();
    MX_TIM6_Init();

    Generate_SineWave();
    
    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, sine_wave, SINE_SAMPLES, DAC_ALIGN_12B_R);
    HAL_TIM_Base_Start(&htim6);

    while (1);
}

void MX_DMA_Init(void) {
    __HAL_RCC_DMA1_CLK_ENABLE();
    hdma_dac1.Instance = DMA1_Stream5;
    hdma_dac1.Init.Channel = DMA_CHANNEL_7;
    hdma_dac1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_dac1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dac1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dac1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_dac1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_dac1.Init.Mode = DMA_CIRCULAR;
    hdma_dac1.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_dac1);
    
    __HAL_LINKDMA(&hdac, DMA_Handle1, hdma_dac1);
}
```
📌 **Expected Behavior:** DAC continuously outputs a **sine wave without CPU load**.

---

## **Summary**
- **Normal Mode:** Direct digital-to-analog conversion.
- **Sine Wave Generation:** Uses lookup table with delay/timer.
- **DAC with DMA:** Continuous output without CPU intervention.

🚀 This completes **Day 7**! Let me know if you need modifications.


# **Day 8: SPI (Serial Peripheral Interface)**

## **Theory**

### **1. Introduction to SPI**
SPI (Serial Peripheral Interface) is a **high-speed, full-duplex communication protocol** used for connecting microcontrollers with peripherals like sensors, ADCs, displays, and memory chips.

| **Feature**          | **Description** |
|---------------------|----------------|
| **Communication**  | Master-Slave |
| **Clock Signal**   | Synchronous (SCK) |
| **Data Lines**     | MOSI (Master Out, Slave In), MISO (Master In, Slave Out) |
| **Chip Select**    | CS (Chip Select) or NSS (Slave Select) |

---

### **2. SPI Clock Configuration (CPOL & CPHA)**

SPI has two clock settings:
- **Clock Polarity (CPOL)**:
  - **CPOL = 0** → Clock is **low** when idle.
  - **CPOL = 1** → Clock is **high** when idle.

- **Clock Phase (CPHA)**:
  - **CPHA = 0** → Data sampled on **first clock edge**.
  - **CPHA = 1** → Data sampled on **second clock edge**.

| **Mode** | **CPOL** | **CPHA** | **Clock Behavior** |
|---------|--------|--------|----------------|
| **Mode 0** | 0 | 0 | Idle Low, Sample on Rising |
| **Mode 1** | 0 | 1 | Idle Low, Sample on Falling |
| **Mode 2** | 1 | 0 | Idle High, Sample on Falling |
| **Mode 3** | 1 | 1 | Idle High, Sample on Rising |

---

### **3. SPI Communication Types**
- **Full-Duplex** → Data sent & received simultaneously.
- **Half-Duplex** → Only one direction at a time.
- **Simplex** → Only transmitting or receiving.

---

### **4. SPI Interrupts & DMA**
- **Interrupt Mode**: CPU gets notified when SPI transmission is complete.
- **DMA Mode**: Transfers data without CPU involvement.

---

## **Practical Implementation**

### **1. SPI Loopback Test (Self-Communication)**
🔹 **Goal:** Connect **MOSI to MISO** and verify SPI data transmission.

#### **Connections**
| **Pin**  | **STM32** |
|----------|----------|
| **MOSI** | PA7 |
| **MISO** | PA6 |
| **SCK**  | PA5 |
| **CS**   | PA4 |

#### **Code:**
```c
#include "stm32f4xx_hal.h"

SPI_HandleTypeDef hspi1;
uint8_t txData = 0x55, rxData = 0x00;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_SPI1_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();

    while (1) {
        HAL_SPI_TransmitReceive(&hspi1, &txData, &rxData, 1, HAL_MAX_DELAY);
        HAL_Delay(500);
    }
}

void MX_SPI1_Init(void) {
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    HAL_SPI_Init(&hspi1);
}
```
📌 **Expected Behavior:** The STM32 sends `0x55`, receives the same byte, and confirms SPI operation.

---

### **2. SPI Communication Between Two STM32 Boards**
🔹 **Goal:** Setup **one STM32 as SPI Master** and **another as SPI Slave**.

#### **Connections**
| **Pin**  | **Master (STM32 A)** | **Slave (STM32 B)** |
|----------|---------------------|---------------------|
| **MOSI** | PA7  | PA7  |
| **MISO** | PA6  | PA6  |
| **SCK**  | PA5  | PA5  |
| **CS**   | PA4  | PA4  |

---

#### **Master Code (STM32 A)**
```c
#include "stm32f4xx_hal.h"

SPI_HandleTypeDef hspi1;
uint8_t masterTxData = 0xAA, masterRxData = 0x00;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_SPI1_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();

    while (1) {
        HAL_SPI_TransmitReceive(&hspi1, &masterTxData, &masterRxData, 1, HAL_MAX_DELAY);
        HAL_Delay(500);
    }
}
```

---

#### **Slave Code (STM32 B)**
```c
#include "stm32f4xx_hal.h"

SPI_HandleTypeDef hspi1;
uint8_t slaveTxData = 0x55, slaveRxData = 0x00;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_SPI1_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();

    while (1) {
        HAL_SPI_TransmitReceive(&hspi1, &slaveTxData, &slaveRxData, 1, HAL_MAX_DELAY);
        HAL_Delay(500);
    }
}
```
📌 **Expected Behavior:** Master sends `0xAA`, Slave sends `0x55`, and both exchange data.

---

### **3. SPI with an External Sensor (e.g., OLED Display)**
🔹 **Goal:** Communicate with **OLED Display (SSD1306) using SPI**.

#### **Connections**
| **Pin**  | **STM32 (Master)** | **SSD1306 OLED** |
|----------|------------------|------------------|
| **MOSI** | PA7  | SDA |
| **SCK**  | PA5  | SCL |
| **CS**   | PA4  | CS |
| **DC**   | PB0  | DC |
| **RESET**| PB1  | RES |

---

#### **SPI OLED Code**
```c
#include "stm32f4xx_hal.h"
#include "ssd1306.h"

SPI_HandleTypeDef hspi1;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_SPI1_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    
    ssd1306_Init();
    ssd1306_SetCursor(10, 10);
    ssd1306_WriteString("Hello SPI", Font_11x18, White);
    ssd1306_UpdateScreen();

    while (1);
}
```
📌 **Expected Behavior:** Displays `"Hello SPI"` on OLED.

---

## **Summary**
- **Loopback Test** → Verifies SPI communication.
- **Master-Slave Communication** → Data exchange between two STM32.
- **SPI with Sensor/Display** → Interfaces with an **OLED using SPI**.


# **Day 9: I2C (Inter-Integrated Circuit)**

## **Theory**

### **1. Introduction to I2C**
I2C (Inter-Integrated Circuit) is a **multi-master, multi-slave, synchronous serial communication protocol** used for connecting microcontrollers with sensors, EEPROMs, and other peripherals.

| **Feature**          | **Description** |
|----------------------|----------------|
| **Communication**   | Multi-Master, Multi-Slave |
| **Data Lines**      | SDA (Serial Data), SCL (Serial Clock) |
| **Speed Modes**     | Standard (100 kbps), Fast (400 kbps), High-Speed (3.4 Mbps) |
| **Addressing**      | 7-bit & 10-bit |
| **Pull-up Resistors** | Required for SDA & SCL |

---

### **2. I2C Bus Operation**
- **Start Condition:** Master pulls **SDA low** while **SCL is high**.
- **Stop Condition:** Master releases **SDA** while **SCL is high**.
- **ACK/NACK:** Acknowledgment mechanism used for error detection.

---

### **3. I2C Addressing**
- **7-bit Addressing:** Commonly used.
- **10-bit Addressing:** Less common, used for large device networks.

---

### **4. I2C vs SPI**
| **Feature** | **I2C** | **SPI** |
|------------|--------|--------|
| **Speed**  | Up to 3.4 Mbps | Up to 50 Mbps |
| **Wires**  | 2 (SDA, SCL) | 4 (MOSI, MISO, SCK, CS) |
| **Communication** | Multi-Master, Multi-Slave | Single-Master, Multi-Slave |
| **Addressing** | 7-bit / 10-bit | No Addressing |
| **Use Cases** | Sensors, EEPROM, RTC | High-speed displays, ADCs |

---

## **Practical Implementation**

### **1. Interface I2C EEPROM (24Cxx)**
🔹 **Goal:** Read & write data to a **24Cxx EEPROM**.

#### **Connections**
| **Pin**  | **STM32** | **24Cxx EEPROM** |
|----------|----------|-----------------|
| **SDA**  | PB7      | SDA |
| **SCL**  | PB6      | SCL |

---

#### **Code: Write Data to EEPROM**
```c
#include "stm32f4xx_hal.h"

I2C_HandleTypeDef hi2c1;
uint8_t eeprom_addr = 0xA0; // 24Cxx EEPROM address

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);

void EEPROM_Write(uint16_t memAddr, uint8_t data) {
    uint8_t buffer[2] = {memAddr, data};
    HAL_I2C_Master_Transmit(&hi2c1, eeprom_addr, buffer, 2, HAL_MAX_DELAY);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

    EEPROM_Write(0x00, 0x55);  // Write 0x55 to memory address 0x00
    while (1);
}
```

---

#### **Code: Read Data from EEPROM**
```c
uint8_t EEPROM_Read(uint16_t memAddr) {
    uint8_t data;
    HAL_I2C_Master_Transmit(&hi2c1, eeprom_addr, &memAddr, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, eeprom_addr, &data, 1, HAL_MAX_DELAY);
    return data;
}
```
📌 **Expected Behavior:** Stores `0x55` in EEPROM and retrieves it successfully.

---

### **2. Communicate with I2C-based Sensor (MPU6050)**
🔹 **Goal:** Read acceleration & gyroscope data from **MPU6050**.

#### **Connections**
| **Pin**  | **STM32** | **MPU6050** |
|----------|----------|------------|
| **SDA**  | PB7      | SDA |
| **SCL**  | PB6      | SCL |

---

#### **Code: Read Sensor Data**
```c
#define MPU6050_ADDR 0xD0 // MPU6050 I2C address
uint8_t accel_data[6];

void MPU6050_Read_Accel(void) {
    uint8_t reg = 0x3B; // Accel data register
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, accel_data, 6, HAL_MAX_DELAY);
}
```
📌 **Expected Behavior:** Reads raw acceleration data from MPU6050.

---

## **Summary**
- **I2C EEPROM (24Cxx)** → Read & write data.
- **MPU6050 Sensor** → Read acceleration & gyroscope data.


# **Day 10: External Interrupts and NVIC (Nested Vector Interrupt Controller)**

## **Theory**

### **1. What are Interrupts?**
Interrupts are events that temporarily pause the main execution flow to execute a special function called an **Interrupt Service Routine (ISR)**. Once the ISR completes, the processor resumes the previous execution.

### **2. NVIC (Nested Vector Interrupt Controller)**
The **NVIC (Nested Vector Interrupt Controller)** is responsible for:
- **Prioritizing** interrupts
- **Enabling/Disabling** specific interrupts
- **Nested execution** (higher-priority interrupts can interrupt lower-priority ones)

### **3. Interrupt Priority in STM32**
- STM32 has **16 priority levels**.
- Lower numerical values indicate **higher priority**.
- Priority grouping allows for **preemption** (interrupting an ongoing ISR).

### **4. ISR (Interrupt Service Routine) Execution Flow**
1. **Interrupt event occurs** (e.g., button press).
2. **Processor saves context** and jumps to ISR.
3. **ISR executes the required task**.
4. **Processor restores context** and resumes execution.

---

## **Practical Implementation**

### **1. Configure External Interrupt (Button Press)**
🔹 **Goal:** Toggle an LED when a button is pressed.

#### **Connections**
| **Pin**  | **STM32 (F407)** | **Component** |
|----------|----------------|--------------|
| **Button** | PA0 (EXTI0) | Push Button |
| **LED** | PD12 | LED |

---

#### **Code: External Interrupt for Button Press**
```c
#include "stm32f4xx_hal.h"

void SystemClock_Config(void);
void MX_GPIO_Init(void);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_0) { // Check if PA0 caused the interrupt
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); // Toggle LED
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    while (1) {
        // Main loop does nothing, LED is controlled by the ISR
    }
}

void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure LED pin (Output)
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    // Configure Button pin (Interrupt)
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Enable Interrupt in NVIC
    HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void EXTI0_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}
```
📌 **Expected Behavior:** Pressing the button toggles the LED.

---

### **2. Implement Debouncing for Push Button**
🔹 **Goal:** Avoid multiple detections of a single button press due to bouncing.

#### **Modify ISR to include Debouncing**
```c
volatile uint32_t last_interrupt_time = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    uint32_t current_time = HAL_GetTick(); // Get system time (ms)
    if (GPIO_Pin == GPIO_PIN_0 && (current_time - last_interrupt_time > 200)) { 
        last_interrupt_time = current_time;
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); // Toggle LED
    }
}
```
📌 **Expected Behavior:** Button presses will be detected only if at least 200ms has passed since the last press.

---

### **Summary**
- **Interrupts** allow asynchronous event handling.
- **NVIC** manages interrupt priorities.
- **EXTI** is used for external interrupts (e.g., button press).
- **Debouncing** prevents false triggering.



# **Day 11: Real-Time Clock (RTC) and Watchdog Timer**

## **Theory**

### **1. RTC (Real-Time Clock) in STM32**
The **RTC (Real-Time Clock)** is a **low-power** peripheral that keeps track of time and date, even when the microcontroller is in low-power mode.

#### **Key Features of RTC**
- Keeps time and date accurately.
- Works even when the MCU is powered off (if a backup battery is connected).
- Has **alarm functions** to trigger events at specific times.
- Can generate **wake-up events** from low-power modes.
- Can log timestamps for events.

#### **RTC Battery Backup**
The RTC is powered by a separate **VBAT** pin when the main power is off. A **3V coin cell** (e.g., CR2032) is usually used.

---

### **2. RTC Alarm, Wakeup Timer, and Timestamp**
- **Alarm:** Triggers an event at a predefined time.
- **Wakeup Timer:** Periodically wakes up the MCU from Sleep Mode.
- **Timestamp:** Saves time data on an external event (e.g., power failure detection).

---

### **3. Watchdog Timer (WDT)**
The **Watchdog Timer** is a safety feature that **resets the MCU** if it detects a software malfunction (e.g., infinite loop or crash).

#### **Types of Watchdog Timers**
1. **Independent Watchdog (IWDG)**
   - Runs on an independent **low-speed clock (LSI)**.
   - Cannot be stopped once started.
   - Used for critical failure recovery.

2. **Window Watchdog (WWDG)**
   - Can only be refreshed within a specific time window.
   - Helps prevent unintended code execution.

---

## **Practical Implementation**

### **1. Set RTC Clock and Display Time on UART**
🔹 **Goal:** Initialize RTC and send time to the UART terminal every second.

#### **Connections**
| **Component** | **Connection** |
|--------------|--------------|
| USB to UART (FTDI) | UART2 (PA2: TX, PA3: RX) |

#### **Code: RTC Time Display on UART**
```c
#include "stm32f4xx_hal.h"
#include <stdio.h>

UART_HandleTypeDef huart2;
RTC_HandleTypeDef hrtc;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART2_UART_Init(void);
void MX_RTC_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_RTC_Init();

    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;
    char buffer[50];

    while (1) {
        HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
        HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

        snprintf(buffer, sizeof(buffer), "Time: %02d:%02d:%02d\n", 
                 sTime.Hours, sTime.Minutes, sTime.Seconds);
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
        HAL_Delay(1000); // 1-second delay
    }
}

void MX_RTC_Init(void) {
    __HAL_RCC_RTC_ENABLE();
    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 127;
    hrtc.Init.SynchPrediv = 255;
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    HAL_RTC_Init(&hrtc);
}
```
📌 **Expected Behavior:** The UART terminal displays the current time every second.

---

### **2. Configure RTC Alarm and Wake Up from Sleep Mode**
🔹 **Goal:** Set an RTC alarm that wakes up the MCU from Sleep Mode.

#### **Code: RTC Alarm Configuration**
```c
void MX_RTC_Alarm_Init(void) {
    RTC_AlarmTypeDef sAlarm = {0};

    sAlarm.AlarmTime.Hours = 12;
    sAlarm.AlarmTime.Minutes = 30;
    sAlarm.AlarmTime.Seconds = 0;
    sAlarm.Alarm = RTC_ALARM_A;
    sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
    sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
    HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN);

    HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
}

void RTC_Alarm_IRQHandler(void) {
    HAL_RTC_AlarmIRQHandler(&hrtc);
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); // Toggle LED on wake-up
}
```
📌 **Expected Behavior:** The microcontroller wakes up and toggles an LED at **12:30:00 PM**.

---

### **3. Implement Watchdog Timer to Reset MCU on Failure**
🔹 **Goal:** Reset the microcontroller if a system hang occurs.

#### **Code: Independent Watchdog (IWDG)**
```c
IWDG_HandleTypeDef hiwdg;

void MX_IWDG_Init(void) {
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = 1000;
    HAL_IWDG_Init(&hiwdg);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_IWDG_Init();  // Start Watchdog

    while (1) {
        HAL_Delay(800); // Must refresh before 1 second
        HAL_IWDG_Refresh(&hiwdg); // Reset watchdog timer
    }
}
```
📌 **Expected Behavior:** The MCU **resets** if the `HAL_IWDG_Refresh()` function is not called within **1 second**.

---

## **Summary**
- **RTC** maintains time, even in low-power mode.
- **Alarm** and **Wakeup Timer** allow time-based MCU wake-up.
- **IWDG** resets MCU on failure.
- **WWDG** prevents unexpected execution errors.


# **Day 12: Low Power Modes and Power Management**

## **Theory**

### **1. Power Modes in STM32**
STM32 microcontrollers have different **low-power modes** to reduce power consumption:

| **Mode**   | **CPU** | **Peripherals** | **Wake-Up Sources** | **Power Consumption** |
|------------|--------|----------------|----------------------|-----------------------|
| **Sleep**  | Off    | On              | Any interrupt/event  | Low                   |
| **Stop**   | Off    | Off (Retains RAM) | EXTI, RTC Alarm     | Very Low              |
| **Standby** | Off   | Off (Loses RAM)  | Reset, RTC Alarm    | Ultra-Low             |

---

### **2. Reducing Power Consumption in STM32**
- Lowering **clock speed** reduces power usage.
- Disabling unused **peripherals** (UART, SPI, ADC, etc.).
- Using **low-power peripherals** (LP timers, LP ADC).
- Entering **low-power modes** when idle.

---

### **3. Waking Up MCU from Low-Power Mode**
- **Sleep Mode:** Wakes up with any interrupt.
- **Stop Mode:** Wakes up using **external interrupt (EXTI)** or **RTC alarm**.
- **Standby Mode:** Wakes up using **Reset button** or **RTC alarm**.

---

## **Practical Implementation**

### **1. Put STM32 in Sleep Mode**
🔹 **Goal:** Put MCU in Sleep Mode and wake it up using an external button.

#### **Code: Sleep Mode with External Interrupt**
```c
#include "stm32f4xx_hal.h"

void SystemClock_Config(void);
void MX_GPIO_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    while (1) {
        HAL_SuspendTick();   // Disable SysTick for low power
        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
        HAL_ResumeTick();    // Re-enable SysTick after wake-up
    }
}

void EXTI0_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_0) {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); // Toggle LED
    }
}
```
📌 **Expected Behavior:** MCU enters **Sleep Mode** and wakes up when the button (PA0) is pressed.

---

### **2. Implement Stop Mode and Wake Up via EXTI**
🔹 **Goal:** Put MCU in Stop Mode and wake it up using a button.

#### **Code: Stop Mode with EXTI Wake-Up**
```c
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    while (1) {
        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
        SystemClock_Config(); // Reconfigure system clock after wake-up
    }
}
```
📌 **Expected Behavior:** MCU enters **Stop Mode**, wakes up on button press, and resumes execution.

---

### **3. Implement Standby Mode and Wake Up via Reset**
🔹 **Goal:** Put MCU in Standby Mode and wake it up using a **Reset or RTC Alarm**.

#### **Code: Standby Mode with RTC Alarm Wake-Up**
```c
void Enter_Standby_Mode(void) {
    HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
    HAL_PWR_EnterSTANDBYMode();
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    
    if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET) {
        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); // Indicate wake-up
    } else {
        Enter_Standby_Mode();
    }

    while (1);
}
```
📌 **Expected Behavior:** MCU **enters Standby Mode** and **wakes up via Reset or RTC Alarm**.

---

## **Summary**
- **Sleep Mode**: CPU stops, peripherals run, wakes up with any interrupt.
- **Stop Mode**: Everything stops except RAM, wakes up via EXTI or RTC.
- **Standby Mode**: Power off, RAM lost, wakes up via Reset or RTC.


# **Day 13: USB Communication (USB CDC)**

## **Theory**

### **1. USB Communication in STM32**
USB (Universal Serial Bus) is a widely used communication protocol for data transfer between microcontrollers and computers. STM32 microcontrollers support different USB modes:
- **USB Device** (CDC, HID, MSC, etc.)
- **USB Host** (Connects to USB devices like flash drives, keyboards)
- **USB OTG (On-The-Go)** (Acts as both Host and Device)

---

### **2. USB as Virtual COM Port (CDC)**
USB CDC (Communications Device Class) allows the STM32 to appear as a **Virtual COM Port (VCP)** on a computer, enabling serial communication over USB.  
🔹 **Advantages of USB CDC over UART:**
- Faster data transfer (up to **12 Mbps** in Full Speed mode)
- No need for an external USB-to-Serial adapter
- Can communicate with **PC terminal software** (Putty, TeraTerm, etc.)

---

### **3. Configuring USB using HAL**
STM32 HAL (Hardware Abstraction Layer) provides a **USB Device Middleware** to implement USB CDC communication. The key files for USB CDC are:
- **usbd_cdc_if.c** → Implements USB communication
- **usbd_cdc.h** → USB CDC header file
- **usb_device.c** → Initializes USB middleware

---

## **Practical Implementation**

### **1. Implement USB-to-Serial Communication**
🔹 **Goal:** Configure STM32 as a USB CDC device and send/receive data using a PC.

#### **Steps:**
1. Open **STM32CubeMX** and enable **USB Device (CDC Class)**.
2. Generate code and open **usb_device.c**.
3. Modify `CDC_Receive_FS()` to handle received data.

#### **Code: USB CDC Send/Receive Data**
📌 **Modify `usbd_cdc_if.c`**
```c
#include "usbd_cdc_if.h"

uint8_t USB_RX_Buffer[64];

int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len) {
    memcpy(USB_RX_Buffer, Buf, *Len);  // Copy received data
    CDC_Transmit_FS(USB_RX_Buffer, *Len);  // Echo back data
    return (USBD_OK);
}

void USB_SendData(uint8_t* data, uint16_t size) {
    CDC_Transmit_FS(data, size);
}
```
📌 **Expected Behavior:** Data sent to the STM32 **echoes back** to the PC.

---

### **2. Test USB CDC using a PC Terminal**
🔹 **Steps:**
1. Connect STM32 to a PC via **USB cable**.
2. Open **Device Manager** → Check if STM32 appears as a **COM Port**.
3. Open **Putty/TeraTerm** → Select **Baud Rate: 115200**.
4. Type text and observe **echoed response**.

---

### **3. Send/Receive Data Over USB**
🔹 **Modify `main.c` to send data periodically**
```c
#include "usbd_cdc_if.h"

uint8_t message[] = "Hello from STM32 USB CDC!\r\n";

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_USB_DEVICE_Init();  // Initialize USB CDC

    while (1) {
        USB_SendData(message, sizeof(message));
        HAL_Delay(1000);  // Send every second
    }
}
```
📌 **Expected Behavior:** The STM32 sends **"Hello from STM32 USB CDC!"** every second to the PC terminal.

---

## **Summary**
- **USB CDC** allows STM32 to act as a **Virtual COM Port**.
- **HAL API** simplifies USB communication.
- **PC terminal software** (Putty, TeraTerm) can test USB CDC.


# **Day 14: FreeRTOS Basics with STM32**

## **Theory**

### **1. What is an RTOS (Real-Time Operating System)?**
An **RTOS (Real-Time Operating System)** is designed for real-time applications where tasks must execute within defined time constraints. Unlike traditional operating systems, RTOS provides:
- **Deterministic task scheduling**
- **Multitasking support**
- **Efficient CPU utilization**

🔹 **Why use an RTOS?**
- Handles multiple tasks efficiently.
- Provides real-time response.
- Manages system resources effectively.

🔹 **Types of Task Scheduling in RTOS:**
1. **Preemptive Scheduling:** Higher priority tasks can interrupt lower-priority ones.
2. **Cooperative Scheduling:** Tasks voluntarily give control to others.

---

### **2. FreeRTOS Fundamentals**
FreeRTOS is an **open-source RTOS** widely used in embedded systems.

🔹 **Key Components:**
- **Tasks:** Independent execution units.
- **Queues:** Message passing between tasks.
- **Semaphores:** Synchronization between tasks.
- **Timers:** Event triggering at intervals.

---

### **3. Memory Management in FreeRTOS**
- **Static Allocation:** Memory assigned at compile time.
- **Dynamic Allocation:** Uses heap, but requires careful management to avoid fragmentation.

---

## **Practical Implementation**

### **1. Creating Two FreeRTOS Tasks**
🔹 **Goal:** Run two tasks that blink an LED and send a message via UART.

#### **Steps:**
1. Enable **FreeRTOS** in **STM32CubeMX**.
2. Create **two tasks** in `main.c`.

#### **Code: Create Two Tasks**
📌 **Modify `main.c`**
```c
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"

void LED_Task(void *pvParameters);
void UART_Task(void *pvParameters);

int main(void) {
    HAL_Init();
    SystemClock_Config();

    xTaskCreate(LED_Task, "LED Task", 128, NULL, 1, NULL);
    xTaskCreate(UART_Task, "UART Task", 128, NULL, 1, NULL);

    vTaskStartScheduler();  // Start FreeRTOS scheduler

    while (1);  // Should never reach here
}

void LED_Task(void *pvParameters) {
    while (1) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        vTaskDelay(pdMS_TO_TICKS(500));  // 500ms delay
    }
}

void UART_Task(void *pvParameters) {
    uint8_t message[] = "Hello from FreeRTOS!\r\n";
    while (1) {
        HAL_UART_Transmit(&huart2, message, sizeof(message), HAL_MAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(1000));  // 1s delay
    }
}
```
📌 **Expected Behavior:** 
- LED blinks every **500ms**.
- UART sends `"Hello from FreeRTOS!"` every **1s**.

---

### **2. Implement Task Synchronization using Semaphores**
🔹 **Goal:** Ensure UART task waits for LED toggle using a semaphore.

#### **Steps:**
1. Enable **FreeRTOS Semaphores** in STM32CubeMX.
2. Modify **UART Task** to wait for semaphore.

#### **Code: Use Semaphore for Synchronization**
📌 **Modify `main.c`**
```c
#include "semphr.h"

SemaphoreHandle_t xSemaphore;

void LED_Task(void *pvParameters);
void UART_Task(void *pvParameters);

int main(void) {
    HAL_Init();
    SystemClock_Config();

    xSemaphore = xSemaphoreCreateBinary();  // Create semaphore

    xTaskCreate(LED_Task, "LED Task", 128, NULL, 1, NULL);
    xTaskCreate(UART_Task, "UART Task", 128, NULL, 1, NULL);

    vTaskStartScheduler();  // Start FreeRTOS scheduler

    while (1);
}

void LED_Task(void *pvParameters) {
    while (1) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        xSemaphoreGive(xSemaphore);  // Release semaphore
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void UART_Task(void *pvParameters) {
    uint8_t message[] = "LED Toggled!\r\n";
    while (1) {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
            HAL_UART_Transmit(&huart2, message, sizeof(message), HAL_MAX_DELAY);
        }
    }
}
```
📌 **Expected Behavior:** 
- UART message `"LED Toggled!"` only prints after LED toggles.

---

### **3. Using Queue for Inter-task Communication**
🔹 **Goal:** Pass data between **LED Task** and **UART Task** using a **Queue**.

#### **Steps:**
1. Enable **FreeRTOS Queues** in STM32CubeMX.
2. Modify **Tasks** to send and receive messages.

#### **Code: Use Queue for Communication**
📌 **Modify `main.c`**
```c
#include "queue.h"

QueueHandle_t xQueue;

void LED_Task(void *pvParameters);
void UART_Task(void *pvParameters);

int main(void) {
    HAL_Init();
    SystemClock_Config();

    xQueue = xQueueCreate(5, sizeof(uint8_t));

    xTaskCreate(LED_Task, "LED Task", 128, NULL, 1, NULL);
    xTaskCreate(UART_Task, "UART Task", 128, NULL, 1, NULL);

    vTaskStartScheduler();  

    while (1);
}

void LED_Task(void *pvParameters) {
    uint8_t led_status = 1;
    while (1) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        xQueueSend(xQueue, &led_status, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void UART_Task(void *pvParameters) {
    uint8_t received_status;
    while (1) {
        if (xQueueReceive(xQueue, &received_status, portMAX_DELAY) == pdTRUE) {
            HAL_UART_Transmit(&huart2, (uint8_t*)"Queue Received!\r\n", 18, HAL_MAX_DELAY);
        }
    }
}
```
📌 **Expected Behavior:** 
- LED Task sends a message via Queue.
- UART Task prints `"Queue Received!"` every time the LED toggles.

---

## **Summary**
- **FreeRTOS** enables multitasking in STM32.
- **Semaphores** synchronize task execution.
- **Queues** allow message passing between tasks.

-----
-----
-----
-----

---

## **1. LED Blinking using HAL**
🔹 **Objective**: Blink an LED on the STM32F407 Discovery Board using HAL functions.

### **Code: LED Blinking**
```c
#include "stm32f4xx.h"  // Include STM32 HAL library

void SystemClock_Config(void);
void GPIO_Init(void);
void Delay_ms(uint32_t ms);

int main(void) {
    HAL_Init();             // Initialize the HAL Library
    SystemClock_Config();   // Configure system clock
    GPIO_Init();            // Initialize GPIO for LED

    while (1) {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); // Toggle LED (PD12)
        Delay_ms(500);                          // Delay of 500ms
    }
}

void GPIO_Init(void) {
    __HAL_RCC_GPIOD_CLK_ENABLE();  // Enable clock for GPIOD

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_12;        // LED connected to PD12
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Output mode, push-pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;       // No pull-up/down resistor
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Low speed
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);   // Initialize GPIO
}

void Delay_ms(uint32_t ms) {
    HAL_Delay(ms); // HAL built-in delay function
}

void SystemClock_Config(void) {
    // Default system clock settings (HSE = 8MHz, PLL enabled, 168MHz)
}
```
### **Explanation:**
1. **HAL_Init()** initializes STM32 HAL.
2. **SystemClock_Config()** sets the system clock (optional).
3. **GPIO_Init()** enables GPIO port D and configures **PD12** as an output.
4. **HAL_GPIO_TogglePin()** toggles the LED state every 500ms.
5. **HAL_Delay(ms)** introduces a delay.

---

## **2. Switch Interfacing using HAL**
🔹 **Objective**: Read input from a push button (connected to PA0) to control LED (PD12).

### **Code: Switch Interfacing**
```c
#include "stm32f4xx.h"

void SystemClock_Config(void);
void GPIO_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();

    while (1) {
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) { // If button pressed
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); // Toggle LED
            HAL_Delay(300);  // Debounce delay
        }
    }
}

void GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();  // Enable clock for GPIOA
    __HAL_RCC_GPIOD_CLK_ENABLE();  // Enable clock for GPIOD

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // LED Configuration (PD12)
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    // Button Configuration (PA0)
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;  // Input mode for button
    GPIO_InitStruct.Pull = GPIO_NOPULL; // No pull-up/down
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
```
### **Explanation:**
1. **PA0 is configured as input**, and **PD12 as output**.
2. **HAL_GPIO_ReadPin()** checks if the button is pressed.
3. If **button is pressed**, **HAL_GPIO_TogglePin()** toggles the LED.
4. **Debounce delay (300ms)** prevents unintended multiple toggles.

---
Great! Let’s move to the **next step: 7-Segment Display Interfacing** with STM32F407 using HAL.  

---

## **3. 7-Segment Display Interfacing using HAL**
🔹 **Objective**: Display numbers (0–9) on a **Common Cathode** 7-segment display using GPIO.

### **Wiring:**
| 7-Segment Pin | Connected To (STM32F407) |
|--------------|--------------------------|
| **a** | PB0 |
| **b** | PB1 |
| **c** | PB2 |
| **d** | PB3 |
| **e** | PB4 |
| **f** | PB5 |
| **g** | PB6 |
| **dp (dot)** | PB7 |

### **Code: 7-Segment Display**
```c
#include "stm32f4xx.h"

void SystemClock_Config(void);
void GPIO_Init(void);
void Display_Number(uint8_t num);
void Delay_ms(uint32_t ms);

// 7-Segment Lookup Table (Common Cathode)
const uint8_t segment_map[10] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F  // 9
};

int main(void) {
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();

    while (1) {
        for (uint8_t i = 0; i < 10; i++) { // Loop from 0 to 9
            Display_Number(i);
            Delay_ms(1000);  // Display each number for 1 sec
        }
    }
}

void GPIO_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();  // Enable clock for GPIOB

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                          GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7; // All segment pins
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Display_Number(uint8_t num) {
    uint8_t data = segment_map[num]; // Get segment data
    for (uint8_t i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(GPIOB, (1 << i), (data >> i) & 0x01); // Set each segment pin
    }
}

void Delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}
```

---

### **Explanation:**
1. **7-Segment Common Cathode**: LOW (0) turns OFF the segment, HIGH (1) turns it ON.
2. **segment_map[]** stores the binary patterns for displaying **0-9**.
3. **Display_Number(num)**:
   - Extracts the **binary pattern** from `segment_map[]`.
   - Sets the **corresponding GPIO pins** to display the number.
4. **Loop (0-9)**: Cycles through numbers **0 to 9**, updating the display every second.

---

### **4. DC Motor Interfacing with L298N using STM32F407 (HAL)**
🔹 **Objective**: Control the direction and speed of a **DC motor** using the **L298N motor driver** and **PWM**.

---

### **Connections:**
| L298N Pin | STM32F407 Pin |
|-----------|--------------|
| IN1 | PA0 |
| IN2 | PA1 |
| ENA (PWM) | PA2 |

- **IN1 & IN2** control the **direction**.
- **ENA (PWM)** controls the **speed**.

---

### **Code: DC Motor Control**
```c
#include "stm32f4xx.h"

void SystemClock_Config(void);
void GPIO_Init(void);
void PWM_Init(void);
void Motor_Control(uint8_t direction, uint8_t speed);
void Delay_ms(uint32_t ms);

TIM_HandleTypeDef htim2;

int main(void) {
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    PWM_Init();

    while (1) {
        // Rotate Motor Forward at 50% Speed
        Motor_Control(1, 50);
        Delay_ms(3000);

        // Stop Motor
        Motor_Control(0, 0);
        Delay_ms(1000);

        // Rotate Motor Backward at 75% Speed
        Motor_Control(2, 75);
        Delay_ms(3000);

        // Stop Motor
        Motor_Control(0, 0);
        Delay_ms(1000);
    }
}

void GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE(); // Enable clock for GPIOA

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure IN1 (PA0) & IN2 (PA1) as Output
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void PWM_Init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE(); // Enable clock for TIM2

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 84 - 1; // 1 MHz timer clock
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 100 - 1; // 100 steps for duty cycle
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim2);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0; // Default: Motor off
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
}

void Motor_Control(uint8_t direction, uint8_t speed) {
    if (direction == 1) { // Forward
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    } else if (direction == 2) { // Backward
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    } else { // Stop
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, speed); // Set PWM Duty Cycle (0-100)
}

void Delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}
```

---

### **Explanation:**
1. **Motor Direction Control:**
   - **IN1 = HIGH, IN2 = LOW** → Forward Rotation 🡆
   - **IN1 = LOW, IN2 = HIGH** → Reverse Rotation 🡄
   - **IN1 = LOW, IN2 = LOW** → Stop ✋

2. **PWM for Speed Control:**
   - **TIM2 Channel 3 (PA2)** is configured for **PWM**.
   - Duty cycle varies from **0% (stop) to 100% (full speed)**.

3. **Motor Operation:**
   - Forward @ 50% speed → 3 sec
   - Stop → 1 sec
   - Reverse @ 75% speed → 3 sec
   - Stop → 1 sec (Loop repeats)

---

### **5. LCD 16x2 Interfacing in 4-bit Mode with STM32F407 (HAL)**
🔹 **Objective**: Interface a **16x2 LCD** in **4-bit mode** using **GPIO**.

---

### **Connections:**
| LCD Pin | STM32F407 Pin |
|---------|--------------|
| RS | PB0 |
| RW | GND (Always Write Mode) |
| EN | PB1 |
| D4 | PB4 |
| D5 | PB5 |
| D6 | PB6 |
| D7 | PB7 |
| VSS | GND |
| VDD | +5V |
| VEE | Potentiometer (Contrast Control) |

---

### **Code: LCD 4-bit Mode**
```c
#include "stm32f4xx.h"

void SystemClock_Config(void);
void GPIO_Init(void);
void LCD_Init(void);
void LCD_Send_Cmd(uint8_t cmd);
void LCD_Send_Data(uint8_t data);
void LCD_Send_String(char *str);
void LCD_Enable(void);
void Delay_ms(uint32_t ms);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    LCD_Init();

    while (1) {
        LCD_Send_String("Hello, STM32!");
        HAL_Delay(2000);
        
        LCD_Send_Cmd(0x01); // Clear display
        HAL_Delay(500);

        LCD_Send_String("LCD 4-bit Mode");
        HAL_Delay(2000);

        LCD_Send_Cmd(0x01); // Clear display
        HAL_Delay(500);
    }
}

void GPIO_Init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE(); // Enable clock for GPIOB

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure RS, EN, D4-D7 as Output
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void LCD_Init(void) {
    HAL_Delay(20); // Power ON delay

    // Set LCD in 4-bit mode
    LCD_Send_Cmd(0x33);
    LCD_Send_Cmd(0x32);

    LCD_Send_Cmd(0x28); // 4-bit mode, 2-line, 5x8 dots
    LCD_Send_Cmd(0x0C); // Display ON, Cursor OFF
    LCD_Send_Cmd(0x06); // Entry Mode: Auto Increment
    LCD_Send_Cmd(0x01); // Clear Display
}

void LCD_Send_Cmd(uint8_t cmd) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // RS = 0 (Command Mode)

    // Send Upper Nibble
    GPIOB->ODR = (GPIOB->ODR & 0xFF0F) | ((cmd & 0xF0) >> 4);
    LCD_Enable();

    // Send Lower Nibble
    GPIOB->ODR = (GPIOB->ODR & 0xFF0F) | (cmd & 0x0F);
    LCD_Enable();
}

void LCD_Send_Data(uint8_t data) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // RS = 1 (Data Mode)

    // Send Upper Nibble
    GPIOB->ODR = (GPIOB->ODR & 0xFF0F) | ((data & 0xF0) >> 4);
    LCD_Enable();

    // Send Lower Nibble
    GPIOB->ODR = (GPIOB->ODR & 0xFF0F) | (data & 0x0F);
    LCD_Enable();
}

void LCD_Send_String(char *str) {
    while (*str) {
        LCD_Send_Data(*str++);
    }
}

void LCD_Enable(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // EN = 1
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // EN = 0
    HAL_Delay(1);
}

void Delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}
```

---

### **Explanation:**
1. **4-bit Mode Communication:**
   - Only **D4-D7** are connected, reducing pin usage.
   - Commands are sent **in two parts** (Upper & Lower Nibble).

2. **LCD Initialization:**
   - **Function Set (0x28)** → 4-bit, 2-line, 5x8 dots.
   - **Display ON (0x0C)** → Display ON, Cursor OFF.
   - **Entry Mode (0x06)** → Auto increment cursor.
   - **Clear Display (0x01)** → Clears LCD.

3. **LCD Operation:**
   - Displays `"Hello, STM32!"` for **2 sec**.
   - Clears LCD and shows `"LCD 4-bit Mode"`.
   - Repeats in a loop.

---

### **6. LCD 16x2 Interfacing using I2C with STM32F407 (HAL)**
🔹 **Objective**: Interface a **16x2 LCD** using **I2C module (PCF8574)** with **STM32F407 DISC**.

---

### **Connections (I2C LCD - PCF8574 to STM32F407)**
| LCD Pin | PCF8574 Pin | STM32F407 Pin |
|---------|------------|--------------|
| SDA | SDA | PB9 |
| SCL | SCL | PB8 |
| VCC | VCC | +5V |
| GND | GND | GND |

**I2C Address for PCF8574:** `0x27` or `0x3F` (depends on module version).

---

### **Code: LCD I2C Mode**
```c
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#define LCD_ADDR 0x27 << 1  // I2C Address (shifted for HAL)
#define LCD_BACKLIGHT 0x08  // Backlight ON
#define LCD_ENABLE 0x04     // Enable bit
#define LCD_CMD 0           // Command mode
#define LCD_DATA 1          // Data mode

I2C_HandleTypeDef hi2c1;

void SystemClock_Config(void);
void I2C1_Init(void);
void LCD_Init(void);
void LCD_Send_Cmd(uint8_t cmd);
void LCD_Send_Data(uint8_t data);
void LCD_Send_String(char *str);
void LCD_Write_4Bits(uint8_t data, uint8_t mode);
void LCD_Enable_Signal(uint8_t data);
void Delay_ms(uint32_t ms);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    I2C1_Init();
    LCD_Init();

    while (1) {
        LCD_Send_String("Hello, I2C LCD!");
        HAL_Delay(2000);

        LCD_Send_Cmd(0x01); // Clear Display
        HAL_Delay(500);

        LCD_Send_String("STM32 I2C Mode");
        HAL_Delay(2000);

        LCD_Send_Cmd(0x01); // Clear Display
        HAL_Delay(500);
    }
}

void I2C1_Init(void) {
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

void LCD_Init(void) {
    HAL_Delay(50); // Power-on delay

    LCD_Send_Cmd(0x33); // Initialize LCD
    LCD_Send_Cmd(0x32); // Set to 4-bit mode
    LCD_Send_Cmd(0x28); // 4-bit, 2-line, 5x8 dots
    LCD_Send_Cmd(0x0C); // Display ON, Cursor OFF
    LCD_Send_Cmd(0x06); // Auto Increment
    LCD_Send_Cmd(0x01); // Clear Display
}

void LCD_Send_Cmd(uint8_t cmd) {
    LCD_Write_4Bits(cmd & 0xF0, LCD_CMD);
    LCD_Write_4Bits((cmd << 4) & 0xF0, LCD_CMD);
}

void LCD_Send_Data(uint8_t data) {
    LCD_Write_4Bits(data & 0xF0, LCD_DATA);
    LCD_Write_4Bits((data << 4) & 0xF0, LCD_DATA);
}

void LCD_Send_String(char *str) {
    while (*str) {
        LCD_Send_Data(*str++);
    }
}

void LCD_Write_4Bits(uint8_t data, uint8_t mode) {
    uint8_t data_out = data | LCD_BACKLIGHT | mode;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, &data_out, 1, 10);
    LCD_Enable_Signal(data_out);
}

void LCD_Enable_Signal(uint8_t data) {
    uint8_t temp = data | LCD_ENABLE;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, &temp, 1, 10);
    HAL_Delay(1);
    temp &= ~LCD_ENABLE;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, &temp, 1, 10);
    HAL_Delay(1);
}

void Delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}
```

---

### **Explanation:**
1. **I2C Communication with PCF8574:**
   - PCF8574 extends **I2C** for controlling LCD.
   - LCD data is sent **4 bits at a time** via I2C.
   - **Backlight and Enable control** are handled via I2C.

2. **LCD Initialization:**
   - **0x33, 0x32** → Switch LCD to **4-bit mode**.
   - **0x28** → Set LCD for **4-bit, 2-line**.
   - **0x0C** → Display ON, Cursor OFF.
   - **0x01** → Clear Display.

3. **LCD Operations:**
   - `"Hello, I2C LCD!"` is displayed for **2 sec**.
   - Clears LCD, displays `"STM32 I2C Mode"` for **2 sec**.
   - Loop repeats.

---
### **7. UART Communication with STM32F407 (HAL)**
🔹 **Objective**: Send and receive data over UART using HAL.

---

### **Connections (STM32F407 to USB-TTL Converter)**
| STM32F407 Pin | USB-TTL Converter |
|--------------|------------------|
| **PA2 (TX)** | **RX** |
| **PA3 (RX)** | **TX** |
| **GND** | **GND** |

💡 **Use a USB-TTL Converter** (CP2102/FTDI) to connect the STM32F407 to a PC.

---

### **Code: UART Communication (Send & Receive)**
```c
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <string.h>

UART_HandleTypeDef huart2;

void SystemClock_Config(void);
void UART2_Init(void);
void UART_Send_String(char *str);
void UART_Receive_String(void);

uint8_t rxBuffer[50];  // Buffer to store received data

int main(void) {
    HAL_Init();
    SystemClock_Config();
    UART2_Init();

    char msg[] = "UART Initialized!\r\n";
    UART_Send_String(msg);

    while (1) {
        UART_Receive_String();  // Wait for user input
        UART_Send_String("Received: ");
        UART_Send_String((char *)rxBuffer);
        UART_Send_String("\r\n");
    }
}

void UART2_Init(void) {
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

void UART_Send_String(char *str) {
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

void UART_Receive_String(void) {
    HAL_UART_Receive(&huart2, rxBuffer, sizeof(rxBuffer), HAL_MAX_DELAY);
}
```

---

### **Explanation:**
1. **UART Initialization:**
   - `PA2` → **TX** (Transmit).
   - `PA3` → **RX** (Receive).
   - Baud rate **115200**.
   - 8-bit data, 1 stop bit, no parity.

2. **UART Send & Receive:**
   - `UART_Send_String()` → Transmits a string.
   - `UART_Receive_String()` → Receives user input into `rxBuffer`.
   - Loops indefinitely, echoing received data.

3. **Testing on PC:**
   - Use **PuTTY/Tera Term**.
   - Set **115200 baud rate**.
   - Type a message → STM32 echoes back with `"Received: <your message>"`.

---

### **8. UART with ADC (STM32F407 - HAL)**
🔹 **Objective**: Read ADC values and send them via UART.

---

### **Connections**
| Component | STM32F407 Pin |
|-----------|-------------|
| Potentiometer (Middle Pin) | PA0 (ADC Input) |
| Potentiometer (Other Pins) | VCC, GND |
| UART TX (PA2) | USB-TTL RX |
| UART RX (PA3) | USB-TTL TX |

---

### **Code: Read ADC and Send Data via UART**
```c
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
void ADC1_Init(void);
void UART2_Init(void);
void UART_Send_String(char *str);
uint32_t Read_ADC(void);

char msg[50]; // Buffer for UART messages

int main(void) {
    HAL_Init();
    SystemClock_Config();
    ADC1_Init();
    UART2_Init();

    UART_Send_String("ADC & UART Initialized!\r\n");

    while (1) {
        uint32_t adcValue = Read_ADC();
        float voltage = (adcValue / 4095.0) * 3.3;  // Convert ADC value to voltage

        sprintf(msg, "ADC Value: %lu, Voltage: %.2fV\r\n", adcValue, voltage);
        UART_Send_String(msg);

        HAL_Delay(1000);  // Send every 1 second
    }
}

void ADC1_Init(void) {
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hadc1.Instance = ADC1;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    HAL_ADC_Init(&hadc1);
}

void UART2_Init(void) {
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

uint32_t Read_ADC(void) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    return HAL_ADC_GetValue(&hadc1);
}

void UART_Send_String(char *str) {
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}
```

---

### **Explanation:**
1. **ADC Initialization:**
   - Uses **PA0** as an **analog input**.
   - 12-bit resolution → Values range from **0 to 4095**.
   - Converts voltage **0V - 3.3V**.

2. **UART Initialization:**
   - **PA2 (TX) → USB-TTL RX**.
   - **PA3 (RX) → USB-TTL TX**.
   - Baud rate: **115200**.

3. **Reading and Sending Data:**
   - `Read_ADC()` reads ADC values.
   - Converts ADC value to voltage (`voltage = (adcValue / 4095.0) * 3.3`).
   - Sends the ADC value and voltage via **UART**.
   - **Every second**, it sends data to the terminal.

---

### **Testing:**
1. Connect a **potentiometer** to **PA0**.
2. Open **PuTTY/Tera Term** (Baud: 115200).
3. Rotate the potentiometer and observe **ADC value & voltage**.

---

### **9. Timer (STM32F407 - HAL)**
🔹 **Objective**: Generate a 1-second delay using Timer.

---

### **Connections**
| Component | STM32F407 Pin |
|-----------|-------------|
| LED       | PA5 (Onboard LED) |

---

### **Code: Timer-based LED Blinking**
```c
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

TIM_HandleTypeDef htim2;

void SystemClock_Config(void);
void Timer2_Init(void);
void GPIO_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    Timer2_Init();

    HAL_TIM_Base_Start(&htim2); // Start Timer

    while (1) {
        if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE)) {  // Check if timer has overflowed
            __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);  // Clear the flag
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Toggle LED
        }
    }
}

void Timer2_Init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 16000 - 1;  // Scale down 16 MHz to 1 kHz
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 1000 - 1;  // 1-second delay (1000 ms)
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim2);
}

void GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
```

---

### **Explanation:**
1. **Timer Initialization (TIM2)**
   - Prescaler: `16000 - 1` → Reduces **16 MHz** to **1 kHz**.
   - Period: `1000 - 1` → Generates an **interrupt every 1 second**.

2. **LED Control**
   - Checks if **TIM2 overflowed**.
   - Toggles LED **every second**.

---

### **Testing:**
1. Flash code onto the STM32F407 board.
2. Observe the **onboard LED (PA5) toggling every second**.

---

### **10. PWM (STM32F407 - HAL)**
🔹 **Objective**: Generate a PWM signal to control LED brightness.

---

### **Connections**
| Component | STM32F407 Pin |
|-----------|-------------|
| LED       | PA5 (PWM Output) |

---

### **Code: PWM LED Dimming**
```c
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

TIM_HandleTypeDef htim2;
void SystemClock_Config(void);
void PWM_Init(void);
void GPIO_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    PWM_Init();

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Start PWM on Timer 2, Channel 1

    while (1) {
        for (int duty = 0; duty <= 100; duty += 10) {  // Increase brightness
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (duty * 100));  
            HAL_Delay(500);
        }
        for (int duty = 100; duty >= 0; duty -= 10) {  // Decrease brightness
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (duty * 100));  
            HAL_Delay(500);
        }
    }
}

void PWM_Init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();  // Enable Timer 2 clock

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 16 - 1;  // 16 MHz / 16 = 1 MHz Timer clock
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 10000 - 1;  // Set PWM period to 10 ms (100 Hz)
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim2);

    TIM_OC_InitTypeDef sConfigPWM = {0};
    sConfigPWM.OCMode = TIM_OCMODE_PWM1;
    sConfigPWM.Pulse = 0;  // Initial Duty Cycle
    sConfigPWM.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigPWM.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigPWM, TIM_CHANNEL_1);
}

void GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_5;  
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;  // Alternate Function Mode for PWM
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;  // TIM2 CH1 Alternate Function
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
```

---

### **Explanation:**
1. **PWM Configuration**
   - Timer **Prescaler**: `16 - 1` → Reduces **16 MHz** to **1 MHz**.
   - **Period**: `10000 - 1` → Generates **100 Hz PWM signal**.
   - **Duty Cycle** controlled via `__HAL_TIM_SET_COMPARE()`.

2. **LED Brightness Control**
   - Gradually **increases** and **decreases** brightness **every 500ms**.

---

### **Testing:**
1. Flash code onto the STM32F407 board.
2. Observe **LED brightness smoothly increasing and decreasing**.

---

### **11. Interrupts (Internal & Timer Interrupt) - STM32F407 HAL**
🔹 **Objective**: Handle button press using **External Interrupt (EXTI)** and generate periodic events using a **Timer Interrupt**.

---

### **Connections**
| Component | STM32F407 Pin |
|-----------|-------------|
| Push Button | PC13 (EXTI Input) |
| LED | PA5 (Output) |

---

### **Code: External & Timer Interrupt**
```c
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

TIM_HandleTypeDef htim2;

void SystemClock_Config(void);
void GPIO_Init(void);
void Timer2_Init(void);
void EXTI_Init(void);

int led_state = 0;

int main(void) {
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    Timer2_Init();
    EXTI_Init();

    HAL_TIM_Base_Start_IT(&htim2);  // Start Timer2 with Interrupt

    while (1) {
        // Main loop does nothing, all handled by interrupts
    }
}

// External Interrupt Handler (Button Press)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_13) {  // Check if PC13 triggered
        led_state = !led_state;  // Toggle LED state
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, (GPIO_PinState)led_state);
    }
}

// Timer Interrupt Handler (Blink LED every second)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  // Toggle LED on PA5
    }
}

void GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // LED Output (PA5)
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Button Input (PC13)
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // Interrupt on falling edge
    GPIO_InitStruct.Pull = GPIO_PULLUP;  // Enable pull-up resistor
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Timer2_Init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 16000 - 1;  // 16 MHz / 16000 = 1 kHz timer clock
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 1000 - 1;  // 1 kHz / 1000 = 1 Hz (1-second interrupt)
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim2);

    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void EXTI_Init(void) {
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

// Timer2 ISR
void TIM2_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim2);
}

// External Interrupt ISR
void EXTI15_10_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}
```

---

### **Explanation**
1. **Button Interrupt (EXTI)**
   - Button on `PC13` triggers **falling edge interrupt**.
   - ISR toggles LED state (`PA5`).

2. **Timer Interrupt (TIM2)**
   - Configured to generate an **interrupt every 1 second**.
   - ISR toggles LED on `PA5` periodically.

3. **NVIC Configuration**
   - **EXTI15_10_IRQn** handles button press.
   - **TIM2_IRQn** handles timer interrupts.

---

### **Testing**
1. **Button Press** → LED toggles instantly.
2. **Every 1 second** → LED blinks automatically.

---

### **Done! ✅**
This concludes the **Interrupt Handling** section. 🚀

----
----
----
----
---

### **Step 1: LED Blinking with Delay (Basic GPIO Output)**  
#### **Theory:**  
- GPIO (General Purpose Input/Output) is used to control LEDs.  
- The STM32F407 Discovery board has built-in LEDs (e.g., **Green: PD12, Orange: PD13, Red: PD14, Blue: PD15**).  
- We can turn an LED ON/OFF using HAL functions:  
  - `HAL_GPIO_WritePin(GPIOx, PIN, STATE);`  
  - `HAL_Delay(ms);` for adding delay.  

---

### **Practical: Blinking LED on STM32F407 DISC1**
#### **Steps to Implement:**
1. **Open STM32CubeIDE and Create a New Project.**  
2. **Select STM32F407 Discovery Board.**  
3. **Go to Pin Configuration:**
   - Set **PD12 (Green LED)** as **GPIO_Output**.  
4. **Generate Code and Write the Following in `main.c`:**

```c
#include "main.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

int main(void)
{
    HAL_Init();                // Initialize HAL Library
    SystemClock_Config();       // Configure System Clock
    MX_GPIO_Init();             // Initialize GPIO

    while (1)
    {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);  // Toggle LED on PD12
        HAL_Delay(500);                          // Wait for 500ms
    }
}

/* GPIO Initialization Function */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();  // Enable Clock for GPIOD

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Set as Output Push-Pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;          // No Pull-up/Pull-down
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Low speed
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // This function is auto-generated by STM32CubeMX
}
```

---

### **Explanation of the Code:**
- **HAL_Init();** → Initializes STM32 HAL Library.  
- **SystemClock_Config();** → Configures the system clock.  
- **MX_GPIO_Init();** → Initializes GPIO for LED.  
- **HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);** → Toggles LED.  
- **HAL_Delay(500);** → Adds 500ms delay.  

---

### **Expected Output:**  
- The **Green LED (PD12)** will **blink every 500ms**.  

---

### **Next Step: Push Button Input (GPIO Input Handling)**

### **Step 2: Push Button Interfacing (GPIO Input Handling)**  
#### **Theory:**  
- GPIO pins can be configured as **inputs** to read button states.  
- The STM32F407 Discovery board has a **USER button (PA0)** connected to **ground** when pressed.  
- We use **HAL_GPIO_ReadPin(GPIOx, PIN)** to read button state.  

---

### **Practical: Control LED with Push Button**
#### **Steps to Implement:**  
1. **Open STM32CubeIDE and Create a New Project.**  
2. **Select STM32F407 Discovery Board.**  
3. **Go to Pin Configuration:**
   - Set **PA0 (Button)** as **GPIO_Input**.  
   - Set **PD12 (LED)** as **GPIO_Output**.  
4. **Generate Code and Write the Following in `main.c`:**  

```c
#include "main.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

int main(void)
{
    HAL_Init();                 // Initialize HAL Library
    SystemClock_Config();        // Configure System Clock
    MX_GPIO_Init();              // Initialize GPIO

    while (1)
    {
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)  // Check if button is pressed
        {
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);  // Toggle LED
            HAL_Delay(200);  // Debounce delay
        }
    }
}

/* GPIO Initialization Function */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();  // Enable Clock for GPIOA (Button)
    __HAL_RCC_GPIOD_CLK_ENABLE();  // Enable Clock for GPIOD (LED)

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure LED (PD12)
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    // Configure Button (PA0)
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;  // No Pull-up/Pull-down
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of the Code:**  
- **HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)** → Reads button state.  
- **If button is pressed (PA0 == HIGH), toggle LED.**  
- **HAL_Delay(200)** → Debounce delay to avoid multiple detections.  

---

### **Expected Output:**  
- Pressing the **USER button (PA0)** **toggles the LED (PD12) ON/OFF**.  

---

### **Next Step: 7-Segment Display Interfacing**  

### **Step 3: 7-Segment Display Interfacing**  
#### **Theory:**  
- A **7-segment display** consists of **8 LEDs (A to G + DP)**.  
- Each segment is controlled by **GPIO pins**.  
- Two types: **Common Anode (CA)** & **Common Cathode (CC)**.  
- We use **GPIO_WritePin()** to turn ON/OFF each segment.  

---

### **Practical: Display Numbers (0-9) on 7-Segment**  
#### **Steps to Implement:**  
1. **Connect 7-Segment Display to STM32F407:**  
   - **Common Cathode**: GND to **GND**.  
   - **Segments (A-G, DP)** → Any **GPIO Output Pins**.  

2. **Configure GPIO Pins for Output:**  
   - Assign **PD0 - PD6** to **A-G Segments**.  

3. **Generate Code and Modify `main.c`:**  

```c
#include "main.h"

// Define segment connections (Common Cathode)
#define SEG_A GPIO_PIN_0
#define SEG_B GPIO_PIN_1
#define SEG_C GPIO_PIN_2
#define SEG_D GPIO_PIN_3
#define SEG_E GPIO_PIN_4
#define SEG_F GPIO_PIN_5
#define SEG_G GPIO_PIN_6

GPIO_TypeDef *SEG_PORT = GPIOD;

// Lookup table for numbers 0-9
const uint8_t SEGMENT_MAP[10] = {
    0b0111111, // 0
    0b0000110, // 1
    0b1011011, // 2
    0b1001111, // 3
    0b1100110, // 4
    0b1101101, // 5
    0b1111101, // 6
    0b0000111, // 7
    0b1111111, // 8
    0b1101111  // 9
};

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void displayNumber(uint8_t num);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    while (1)
    {
        for (uint8_t i = 0; i < 10; i++)
        {
            displayNumber(i);  // Show number on 7-segment
            HAL_Delay(1000);   // Delay 1s between numbers
        }
    }
}

/* Function to Display Number */
void displayNumber(uint8_t num)
{
    uint8_t data = SEGMENT_MAP[num];

    HAL_GPIO_WritePin(SEG_PORT, SEG_A, (data >> 0) & 1);
    HAL_GPIO_WritePin(SEG_PORT, SEG_B, (data >> 1) & 1);
    HAL_GPIO_WritePin(SEG_PORT, SEG_C, (data >> 2) & 1);
    HAL_GPIO_WritePin(SEG_PORT, SEG_D, (data >> 3) & 1);
    HAL_GPIO_WritePin(SEG_PORT, SEG_E, (data >> 4) & 1);
    HAL_GPIO_WritePin(SEG_PORT, SEG_F, (data >> 5) & 1);
    HAL_GPIO_WritePin(SEG_PORT, SEG_G, (data >> 6) & 1);
}

/* GPIO Initialization */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
- **SEGMENT_MAP** → Stores **bit patterns** for numbers **0-9**.  
- **displayNumber(num)** → Controls GPIO pins based on **lookup table**.  
- **HAL_GPIO_WritePin()** → Turns ON/OFF each segment.  
- **Loop through 0-9 with 1s delay** to display numbers sequentially.  

---

### **Expected Output:**  
- The **7-segment display** cycles through **0-9** every **1 second**.  

---

### **Next Step: DC Motor Control with L298N**  


### **Step 4: DC Motor Control with L298N**  

#### **Theory:**  
- **L298N** is a **dual H-Bridge motor driver** that controls **speed** and **direction** of **DC motors**.  
- Requires **two input pins** (IN1, IN2) to control **direction**.  
- **PWM on ENA pin** controls **speed**.  

---

### **Practical: Control DC Motor Using STM32F407 + L298N**  
#### **Connections:**  
| L298N Pin | STM32 Pin | Function |
|-----------|----------|----------|
| **ENA**   | **PA0** (PWM) | Speed Control |
| **IN1**   | **PA1** (GPIO) | Motor Direction 1 |
| **IN2**   | **PA2** (GPIO) | Motor Direction 2 |
| **GND**   | **GND** | Common Ground |
| **VCC**   | **5V** | Power |
| **Motor Outputs** | **DC Motor** | Connect to Motor |

---

### **Steps to Implement:**  
1. **Configure GPIO Pins for Direction (IN1, IN2)**.  
2. **Generate PWM on ENA for Speed Control**.  
3. **Write functions for motor control** (Forward, Reverse, Stop).  
4. **Use PWM to vary speed dynamically**.  

---

### **Code for Controlling DC Motor (PWM + Direction)**  

```c
#include "main.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
void setMotorDirection(uint8_t direction);
void setMotorSpeed(uint16_t speed);

TIM_HandleTypeDef htim2;  // Timer for PWM

#define MOTOR_FORWARD  1
#define MOTOR_REVERSE  2
#define MOTOR_STOP     0

#define IN1 GPIO_PIN_1
#define IN2 GPIO_PIN_2
#define ENA TIM_CHANNEL_1  // PWM on TIM2_CH1
#define MOTOR_PORT GPIOA

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();

    HAL_TIM_PWM_Start(&htim2, ENA);  // Start PWM

    while (1)
    {
        setMotorDirection(MOTOR_FORWARD);
        setMotorSpeed(50);  // 50% Speed
        HAL_Delay(3000);

        setMotorDirection(MOTOR_REVERSE);
        setMotorSpeed(100);  // 100% Speed
        HAL_Delay(3000);

        setMotorDirection(MOTOR_STOP);
        HAL_Delay(2000);
    }
}

/* Function to Set Motor Direction */
void setMotorDirection(uint8_t direction)
{
    if (direction == MOTOR_FORWARD)
    {
        HAL_GPIO_WritePin(MOTOR_PORT, IN1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_PORT, IN2, GPIO_PIN_RESET);
    }
    else if (direction == MOTOR_REVERSE)
    {
        HAL_GPIO_WritePin(MOTOR_PORT, IN1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_PORT, IN2, GPIO_PIN_SET);
    }
    else  // MOTOR_STOP
    {
        HAL_GPIO_WritePin(MOTOR_PORT, IN1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_PORT, IN2, GPIO_PIN_RESET);
    }
}

/* Function to Set Motor Speed (PWM) */
void setMotorSpeed(uint16_t speed)
{
    if (speed > 100) speed = 100;  // Limit to 100%
    __HAL_TIM_SET_COMPARE(&htim2, ENA, speed * 10);
}

/* GPIO Initialization */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = IN1 | IN2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR_PORT, &GPIO_InitStruct);
}

/* Timer 2 Initialization for PWM */
static void MX_TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    TIM_OC_InitTypeDef sConfigOC = {0};
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 1600 - 1;  
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 1000 - 1;  
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim2);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, ENA);
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
- **IN1 & IN2 Control Motor Direction**.  
- **PWM on ENA Controls Speed** (0-100%).  
- **setMotorDirection()** sets forward, reverse, or stop.  
- **setMotorSpeed()** adjusts PWM duty cycle.  
- **TIM2 used for PWM Generation**.  

---

### **Expected Output:**  
- **Motor moves forward (50% speed) for 3s**.  
- **Motor moves reverse (100% speed) for 3s**.  
- **Motor stops for 2s**.  
- **Repeats in a loop**.  

---

### **Next Step: 4-Bit LCD Interfacing**  

### **Step 5: LCD Interfacing (4-bit Mode) with STM32F407**  

#### **Theory:**  
- **16x2 LCD** works in **8-bit** or **4-bit mode**.  
- **4-bit mode** reduces **GPIO pin usage**.  
- Uses **RS, RW, E, and D4-D7** for communication.  
- **HD44780 controller** handles data display.  

---

### **Practical: Interface 16x2 LCD in 4-bit Mode**  
#### **Connections:**  
| LCD Pin | STM32 Pin | Function |
|---------|----------|----------|
| **VSS** | **GND** | Ground |
| **VDD** | **5V** | Power |
| **V0** | **Potentiometer** | Contrast Adjustment |
| **RS** | **PA0** | Register Select |
| **RW** | **GND** | Always Write Mode |
| **E**  | **PA1** | Enable |
| **D4** | **PA2** | Data Line |
| **D5** | **PA3** | Data Line |
| **D6** | **PA4** | Data Line |
| **D7** | **PA5** | Data Line |

---

### **Steps to Implement:**  
1. **Configure GPIO Pins for LCD Control & Data.**  
2. **Write a function to send commands.**  
3. **Write a function to send data (characters).**  
4. **Create a function to display strings.**  
5. **Initialize LCD and print a message.**  

---

### **Code for 4-bit LCD Interfacing with STM32F407**  

```c
#include "main.h"
#include <string.h>

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void LCD_Send_Command(uint8_t cmd);
void LCD_Send_Char(char data);
void LCD_Send_String(char *str);
void LCD_Init(void);
void LCD_Enable(void);

#define RS GPIO_PIN_0
#define E  GPIO_PIN_1
#define D4 GPIO_PIN_2
#define D5 GPIO_PIN_3
#define D6 GPIO_PIN_4
#define D7 GPIO_PIN_5
#define LCD_PORT GPIOA

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    LCD_Init();  // Initialize LCD

    LCD_Send_String("Hello STM32!");
    HAL_Delay(2000);
    LCD_Send_Command(0x01);  // Clear Screen
    HAL_Delay(1000);
    LCD_Send_String("4-bit LCD Mode");

    while (1)
    {
    }
}

/* Function to Initialize LCD */
void LCD_Init(void)
{
    HAL_Delay(50);  // Wait for LCD to power up
    LCD_Send_Command(0x33); // Initialize in 4-bit mode
    LCD_Send_Command(0x32);
    LCD_Send_Command(0x28); // 2 lines, 5x8 matrix
    LCD_Send_Command(0x0C); // Display ON, Cursor OFF
    LCD_Send_Command(0x06); // Auto-increment cursor
    LCD_Send_Command(0x01); // Clear screen
    HAL_Delay(2);
}

/* Function to Send Commands */
void LCD_Send_Command(uint8_t cmd)
{
    HAL_GPIO_WritePin(LCD_PORT, RS, GPIO_PIN_RESET); // RS = 0 (Command Mode)

    // Send Upper Nibble
    HAL_GPIO_WritePin(LCD_PORT, D4, (cmd >> 4) & 1);
    HAL_GPIO_WritePin(LCD_PORT, D5, (cmd >> 5) & 1);
    HAL_GPIO_WritePin(LCD_PORT, D6, (cmd >> 6) & 1);
    HAL_GPIO_WritePin(LCD_PORT, D7, (cmd >> 7) & 1);
    LCD_Enable();

    // Send Lower Nibble
    HAL_GPIO_WritePin(LCD_PORT, D4, (cmd >> 0) & 1);
    HAL_GPIO_WritePin(LCD_PORT, D5, (cmd >> 1) & 1);
    HAL_GPIO_WritePin(LCD_PORT, D6, (cmd >> 2) & 1);
    HAL_GPIO_WritePin(LCD_PORT, D7, (cmd >> 3) & 1);
    LCD_Enable();
}

/* Function to Send Character */
void LCD_Send_Char(char data)
{
    HAL_GPIO_WritePin(LCD_PORT, RS, GPIO_PIN_SET); // RS = 1 (Data Mode)

    // Send Upper Nibble
    HAL_GPIO_WritePin(LCD_PORT, D4, (data >> 4) & 1);
    HAL_GPIO_WritePin(LCD_PORT, D5, (data >> 5) & 1);
    HAL_GPIO_WritePin(LCD_PORT, D6, (data >> 6) & 1);
    HAL_GPIO_WritePin(LCD_PORT, D7, (data >> 7) & 1);
    LCD_Enable();

    // Send Lower Nibble
    HAL_GPIO_WritePin(LCD_PORT, D4, (data >> 0) & 1);
    HAL_GPIO_WritePin(LCD_PORT, D5, (data >> 1) & 1);
    HAL_GPIO_WritePin(LCD_PORT, D6, (data >> 2) & 1);
    HAL_GPIO_WritePin(LCD_PORT, D7, (data >> 3) & 1);
    LCD_Enable();
}

/* Function to Send String */
void LCD_Send_String(char *str)
{
    while (*str)
    {
        LCD_Send_Char(*str++);
    }
}

/* Function to Enable LCD (Pulse E pin) */
void LCD_Enable(void)
{
    HAL_GPIO_WritePin(LCD_PORT, E, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_PORT, E, GPIO_PIN_RESET);
    HAL_Delay(1);
}

/* GPIO Initialization */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = RS | E | D4 | D5 | D6 | D7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LCD_PORT, &GPIO_InitStruct);
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
- **LCD_Init()** → Initializes LCD in 4-bit mode.  
- **LCD_Send_Command()** → Sends command (e.g., clear screen).  
- **LCD_Send_Char()** → Sends single character to display.  
- **LCD_Send_String()** → Prints full string on LCD.  
- **LCD_Enable()** → Generates a short pulse on **E** pin.  

---

### **Expected Output:**  
- **"Hello STM32!"** displayed for **2 seconds**.  
- LCD clears, then shows **"4-bit LCD Mode"**.  

---

### **Next Step: LCD Interfacing Using I2C**  


### **Step 6: LCD Interfacing Using I2C with STM32F407**  

#### **Theory:**  
- I2C reduces the number of GPIO pins needed for LCD interfacing.  
- Uses an **I2C to LCD module (PCF8574)**, requiring only **SDA** and **SCL**.  
- Communicates via **7-bit I2C address** (default: **0x27 or 0x3F**).  

---

### **Practical: Interface 16x2 LCD Using I2C**  

#### **Connections:**  
| LCD I2C Module (PCF8574) | STM32 Pin | Function |
|-------------------------|----------|----------|
| **GND**  | **GND**  | Ground |
| **VCC**  | **5V**   | Power |
| **SDA**  | **PB7**  | I2C1 Data |
| **SCL**  | **PB6**  | I2C1 Clock |

---

### **Steps to Implement:**  
1. **Enable I2C Peripheral in STM32CubeMX.**  
2. **Initialize I2C Peripheral in Code.**  
3. **Send Commands & Data to LCD Using I2C.**  
4. **Display a String on LCD.**  

---

### **Code for LCD Interfacing Using I2C with STM32F407**  

```c
#include "main.h"
#include <string.h>

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void LCD_Init(void);
void LCD_Send_Command(uint8_t cmd);
void LCD_Send_Data(uint8_t data);
void LCD_Send_String(char *str);
void LCD_Enable(void);

#define LCD_ADDR 0x27 << 1  // I2C address (shifted for write operation)
#define LCD_BACKLIGHT 0x08  // Backlight ON
#define ENABLE 0x04         // Enable bit

I2C_HandleTypeDef hi2c1;

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();

    LCD_Init();  // Initialize LCD

    LCD_Send_String("Hello STM32 I2C!");
    HAL_Delay(2000);
    LCD_Send_Command(0x01);  // Clear Screen
    HAL_Delay(1000);
    LCD_Send_String("I2C LCD Interface");

    while (1)
    {
    }
}

/* Function to Initialize LCD */
void LCD_Init(void)
{
    HAL_Delay(50);  // Wait for LCD to power up
    LCD_Send_Command(0x33); // Initialize in 4-bit mode
    LCD_Send_Command(0x32);
    LCD_Send_Command(0x28); // 2 lines, 5x8 matrix
    LCD_Send_Command(0x0C); // Display ON, Cursor OFF
    LCD_Send_Command(0x06); // Auto-increment cursor
    LCD_Send_Command(0x01); // Clear screen
    HAL_Delay(2);
}

/* Function to Send Commands */
void LCD_Send_Command(uint8_t cmd)
{
    uint8_t upper_nibble = (cmd & 0xF0) | LCD_BACKLIGHT;
    uint8_t lower_nibble = ((cmd << 4) & 0xF0) | LCD_BACKLIGHT;

    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, &upper_nibble, 1, HAL_MAX_DELAY);
    LCD_Enable();
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, &lower_nibble, 1, HAL_MAX_DELAY);
    LCD_Enable();
}

/* Function to Send Data */
void LCD_Send_Data(uint8_t data)
{
    uint8_t upper_nibble = (data & 0xF0) | LCD_BACKLIGHT | 0x01;
    uint8_t lower_nibble = ((data << 4) & 0xF0) | LCD_BACKLIGHT | 0x01;

    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, &upper_nibble, 1, HAL_MAX_DELAY);
    LCD_Enable();
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, &lower_nibble, 1, HAL_MAX_DELAY);
    LCD_Enable();
}

/* Function to Send String */
void LCD_Send_String(char *str)
{
    while (*str)
    {
        LCD_Send_Data(*str++);
    }
}

/* Function to Enable LCD (Pulse E bit) */
void LCD_Enable(void)
{
    uint8_t data = ENABLE | LCD_BACKLIGHT;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(1);
    data &= ~ENABLE;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, &data, 1, HAL_MAX_DELAY);
    HAL_Delay(1);
}

/* I2C Initialization */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

/* GPIO Initialization */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
- **LCD_Init()** → Initializes LCD in **4-bit I2C mode**.  
- **LCD_Send_Command()** → Sends commands via I2C (e.g., clear screen).  
- **LCD_Send_Data()** → Sends single character over I2C.  
- **LCD_Send_String()** → Prints full string on LCD.  
- **LCD_Enable()** → Sends enable pulse over I2C.  

---

### **Expected Output:**  
1. **"Hello STM32 I2C!"** displayed for **2 seconds**.  
2. LCD clears, then shows **"I2C LCD Interface"**.  

---

### **Next Step: UART Communication**  


### **Step 7: UART Communication with STM32F407**  

#### **Theory:**  
- **UART (Universal Asynchronous Receiver-Transmitter)** enables serial communication between STM32 and PC or other devices.  
- Uses **Tx (PA9) and Rx (PA10)** pins.  
- Baud rate: **115200 bps** (adjustable).  

---

### **Practical: Send & Receive Data Over UART**  

#### **Connections (Using USB-to-Serial Converter or ST-Link V2)**  
| STM32F407 Pin | USB-to-Serial (PL2303/CP2102) | Function |
|--------------|----------------------------|----------|
| **GND**      | **GND**                      | Ground |
| **PA9**      | **RXD**                      | UART Transmit (Tx) |
| **PA10**     | **TXD**                      | UART Receive (Rx) |

---

### **Steps to Implement:**  
1. **Enable UART Peripheral in STM32CubeMX.**  
2. **Initialize UART in Code.**  
3. **Transmit and Receive Data via UART.**  
4. **Monitor Output Using a Serial Terminal (Tera Term, PuTTY, or RealTerm).**  

---

### **Code for UART Communication with STM32F407**  

```c
#include "main.h"
#include <string.h>

UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

void UART_SendString(char *str);
void UART_ReceiveString(void);

char rxData[100]; // Buffer to store received data

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    UART_SendString("UART Initialized!\r\n");

    while (1)
    {
        UART_SendString("Enter a message: ");
        UART_ReceiveString();  // Receive input from serial terminal
        UART_SendString("\r\nReceived: ");
        UART_SendString(rxData);
        UART_SendString("\r\n");
    }
}

/* Function to Send a String via UART */
void UART_SendString(char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

/* Function to Receive Data via UART */
void UART_ReceiveString(void)
{
    memset(rxData, 0, sizeof(rxData)); // Clear buffer
    HAL_UART_Receive(&huart2, (uint8_t *)rxData, sizeof(rxData), HAL_MAX_DELAY);
}

/* UART2 Initialization */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

/* GPIO Initialization */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
- **UART_SendString()** → Sends a string via UART.  
- **UART_ReceiveString()** → Receives input from UART.  
- **Uses DMA or polling mode** for data transfer.  

---

### **Expected Output (Serial Terminal at 115200 bps):**  
```
UART Initialized!
Enter a message: 
```
- User types **"Hello STM32!"**  
- Output:  
  ```
  Received: Hello STM32!
  ```

---

### **Next Step: UART with ADC**  


### **Step 8: UART with ADC (Analog-to-Digital Conversion) on STM32F407**  

#### **Theory:**  
- **ADC (Analog to Digital Converter)** converts analog signals (e.g., from a potentiometer) into digital values.  
- **UART (Universal Asynchronous Receiver-Transmitter)** is used to send the ADC readings to a serial terminal.  
- ADC resolution: **12-bit (0-4095)** → Voltage range: **0V to 3.3V**  

---

### **Practical: Read ADC & Send Data Over UART**  

#### **Connections (Using a Potentiometer on PA0 - ADC1_IN0)**  
| STM32F407 Pin | Component | Function |
|--------------|----------|----------|
| **PA0**      | Potentiometer (Middle Pin) | ADC Input |
| **3.3V**     | Potentiometer (VCC) | Power |
| **GND**      | Potentiometer (GND) | Ground |

#### **Connections for UART (Using USB-to-Serial Converter or ST-Link V2)**  
| STM32F407 Pin | USB-to-Serial (PL2303/CP2102) | Function |
|--------------|----------------------------|----------|
| **GND**      | **GND**                      | Ground |
| **PA9**      | **RXD**                      | UART Transmit (Tx) |
| **PA10**     | **TXD**                      | UART Receive (Rx) |

---

### **Steps to Implement:**  
1. **Enable ADC & UART in STM32CubeMX.**  
2. **Initialize ADC and UART in Code.**  
3. **Read ADC value & Convert to Voltage.**  
4. **Send ADC data over UART.**  
5. **Monitor ADC readings on Serial Terminal (Tera Term, PuTTY, or RealTerm).**  

---

### **Code for ADC + UART Communication with STM32F407**  

```c
#include "main.h"
#include <stdio.h>

ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);

uint32_t ADC_Read(void);
void UART_SendString(char *str);

char buffer[50]; // Buffer for sending ADC value over UART

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_USART2_UART_Init();

    UART_SendString("ADC & UART Initialized!\r\n");

    while (1)
    {
        uint32_t adcValue = ADC_Read();    // Read ADC value
        float voltage = (adcValue * 3.3f) / 4095.0f; // Convert to voltage

        sprintf(buffer, "ADC Value: %lu, Voltage: %.2fV\r\n", adcValue, voltage);
        UART_SendString(buffer); // Send ADC data over UART

        HAL_Delay(1000); // Delay 1 second
    }
}

/* Function to Read ADC Value */
uint32_t ADC_Read(void)
{
    HAL_ADC_Start(&hadc1); // Start ADC conversion
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); // Wait for conversion
    return HAL_ADC_GetValue(&hadc1); // Get ADC value
}

/* Function to Send a String via UART */
void UART_SendString(char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

/* ADC1 Initialization */
static void MX_ADC1_Init(void)
{
    hadc1.Instance = ADC1;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    HAL_ADC_Init(&hadc1);
}

/* UART2 Initialization */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

/* GPIO Initialization */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
- **ADC_Read()** → Reads ADC value from **PA0 (ADC1_IN0)**.  
- **Voltage Calculation:**  
  \[
  V = \left(\frac{\text{ADC Value} \times 3.3V}{4095}\right)
  \]
- **UART_SendString()** → Sends ADC data over UART.  
- **Data sent every 1 second** using `HAL_Delay(1000)`.  

---

### **Expected Output (Serial Terminal at 115200 bps):**  
```
ADC & UART Initialized!
ADC Value: 2048, Voltage: 1.65V
ADC Value: 1024, Voltage: 0.83V
ADC Value: 3072, Voltage: 2.48V
...
```

---

### **Next Step: Timer Basics on STM32**  

### **Step 9: Timer Basics on STM32F407**  

#### **Theory:**  
- **Timers** are used for generating delays, measuring time intervals, and creating periodic events.  
- STM32F407 has **multiple timers** (TIM1-TIM14).  
- Timers can operate in **counter mode, PWM mode, or input capture mode**.  
- **Prescaler & Auto-reload register (ARR)** determine the timer period.  

---

### **Practical: Generate a 1-second delay using TIM2**  

#### **Steps to Implement:**  
1. **Enable TIM2 in STM32CubeMX** (Clock: 84MHz, Prescaler: 8399, ARR: 9999).  
2. **Configure the timer in interrupt mode**.  
3. **Toggle an LED every 1 second** in the timer interrupt handler.  

---

### **Code for Timer-Based LED Blinking**  

```c
#include "main.h"

TIM_HandleTypeDef htim2;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();

    HAL_TIM_Base_Start_IT(&htim2);  // Start TIM2 in interrupt mode

    while (1)
    {
        // Main loop remains empty, as LED toggling is handled in Timer interrupt
    }
}

/* Timer2 Interrupt Handler */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); // Toggle LED on PD12
    }
}

/* TIM2 Initialization */
static void MX_TIM2_Init(void)
{
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 8399;  // Set prescaler to 8399
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 9999;  // Set ARR to 9999
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim2);
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* GPIO Initialization (LED on PD12) */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}

/* Timer2 IRQ Handler */
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}
```

---

### **Explanation of Code:**  
- **TIM2 is configured** for a 1-second period:  
  \[
  \text{Timer Frequency} = \frac{84 MHz}{(\text{Prescaler} + 1)}
  \]  
  \[
  = \frac{84,000,000}{(8399 + 1)} = 10,000 \text{ Hz}
  \]
  \[
  \text{Timer Period} = \frac{\text{ARR} + 1}{\text{Timer Frequency}} = \frac{10,000}{10,000} = 1 \text{ sec}
  \]  
- **Interrupt is triggered every 1 second**, toggling LED **PD12**.  
- **TIM2_IRQHandler()** calls `HAL_TIM_IRQHandler()` to handle interrupts.  

---

### **Expected Output:**  
- **LED on PD12 blinks every 1 second** using Timer2.  

---

### **Next Step: PWM Generation using Timer**  

### **Step 10: PWM Generation using Timer on STM32F407**  

#### **Theory:**  
- **Pulse Width Modulation (PWM)** is used to control the **brightness of LEDs**, **motor speed**, etc.  
- PWM signal has **variable duty cycle** (percentage of HIGH time in one cycle).  
- **STM32 Timers** can generate PWM using **Output Compare Mode**.  
- A **higher duty cycle** means **brighter LED or faster motor speed**.  

---

### **Practical: Generate PWM on TIM3 (Channel 1) to control LED brightness**  

#### **Steps to Implement:**  
1. **Enable TIM3 and configure PWM mode** in STM32CubeMX.  
2. **Set PWM frequency and duty cycle** using Prescaler & ARR.  
3. **Output PWM on GPIO (e.g., PA6 - TIM3 Channel 1)**.  
4. **Gradually increase/decrease brightness** by modifying duty cycle.  

---

### **Code for LED Brightness Control using PWM**  

```c
#include "main.h"

TIM_HandleTypeDef htim3;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);

int dutyCycle = 0;

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM3_Init();

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // Start PWM on TIM3 Channel 1

    while (1)
    {
        for (dutyCycle = 0; dutyCycle <= 100; dutyCycle += 5)
        {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (dutyCycle * htim3.Init.Period) / 100);
            HAL_Delay(100);
        }
        for (dutyCycle = 100; dutyCycle >= 0; dutyCycle -= 5)
        {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (dutyCycle * htim3.Init.Period) / 100);
            HAL_Delay(100);
        }
    }
}

/* TIM3 Initialization for PWM */
static void MX_TIM3_Init(void)
{
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 83;  // Timer clock = 1 MHz
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999;  // PWM frequency = 1 kHz
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim3);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;  // Initial duty cycle = 0%
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
}

/* GPIO Initialization (PA6 as PWM Output) */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
- **TIM3 is configured in PWM mode** with a **1 kHz frequency** (Prescaler = 83, ARR = 999).  
- **PA6 is set as PWM output** (TIM3 Channel 1).  
- **PWM duty cycle** is adjusted from **0% to 100% and back** in a loop.  
- `__HAL_TIM_SET_COMPARE()` sets the duty cycle dynamically.  

---

### **Expected Output:**  
- **LED brightness gradually increases and decreases** continuously.  

---

### **Next Step: Interrupts (Internal & Timer Interrupts)**  


### **Step 11: Interrupts (Internal & Timer Interrupts) on STM32F407**  

#### **Theory:**  
- **Interrupts** allow the MCU to respond to events without constantly checking (polling).  
- **Types of Interrupts in STM32:**  
  1. **External Interrupts (e.g., Button Press - EXTI)**  
  2. **Timer Interrupts (e.g., Periodic tasks using TIMx)**  
  3. **Peripheral Interrupts (e.g., UART, ADC, I2C, etc.)**  
- **NVIC (Nested Vector Interrupt Controller)** manages **priority and handling**.  

---

### **Practical 1: External Interrupt using Push Button (PA0) to Toggle LED**  

#### **Steps to Implement:**  
1. **Enable GPIOA and configure PA0 as an EXTI interrupt pin**.  
2. **Enable EXTI0 in NVIC and attach an ISR (Interrupt Service Routine)**.  
3. **Toggle LED (PA5) whenever the button is pressed**.  

---

### **Code for External Interrupt (Button Press)**
```c
#include "main.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    while (1)
    {
        // Main loop does nothing, waiting for button interrupt
    }
}

/* Interrupt Callback Function */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Toggle LED
    }
}

/* GPIO Initialization */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure LED pin (PA5) as output
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure Button pin (PA0) as EXTI interrupt
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Enable Interrupt in NVIC
    HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/* EXTI ISR Handler */
void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
- **PA0 is configured as an external interrupt (EXTI0)**.  
- **LED on PA5 toggles** when the button is pressed.  
- **NVIC enables EXTI0 interrupt with priority 2**.  
- **HAL_GPIO_EXTI_Callback() is triggered on button press**.  

---

### **Expected Output:**  
- **LED (PA5) toggles ON/OFF whenever the button (PA0) is pressed**.  

---

## **Practical 2: Timer Interrupt using TIM2 (1s Interval Blinking LED)**
### **Steps to Implement:**  
1. **Enable TIM2 and configure it to trigger an interrupt every 1s**.  
2. **Enable TIM2 in NVIC and attach an ISR**.  
3. **Toggle LED (PA5) inside the ISR**.  

---

### **Code for Timer Interrupt (TIM2 - 1s Blink)**
```c
#include "main.h"

TIM_HandleTypeDef htim2;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();

    HAL_TIM_Base_Start_IT(&htim2);  // Start Timer Interrupt

    while (1)
    {
        // Main loop does nothing, LED blinks in ISR
    }
}

/* Timer Interrupt Callback Function */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Toggle LED every 1s
    }
}

/* TIM2 Initialization */
static void MX_TIM2_Init(void)
{
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 8399;  // 84 MHz / (8399 + 1) = 10 kHz
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 9999;  // 10 kHz / (9999 + 1) = 1 Hz (1s)
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim2);

    // Enable Timer Interrupt
    HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* TIM2 ISR Handler */
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}

/* GPIO Initialization */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
- **TIM2 is configured to generate an interrupt every 1 second**.  
- **ISR toggles LED on PA5 when the timer interrupt occurs**.  
- **NVIC enables TIM2 interrupt with priority 2**.  

---

### **Expected Output:**  
- **LED blinks every 1 second automatically** using the timer interrupt.  

---

## **Summary of Interrupt Examples**
| Example | Interrupt Type | Trigger Event | Action |
|---------|---------------|--------------|--------|
| **Button Press** | **External (EXTI0)** | Rising Edge on PA0 | Toggle LED |
| **Timer Interrupt** | **TIM2 Update Event** | Every 1 second | Toggle LED |

---

### **Next Step: FreeRTOS Task Scheduling on STM32**


### **Step 12: FreeRTOS Basics on STM32F407**  

#### **Theory:**  
- **FreeRTOS** is a real-time operating system (RTOS) that manages tasks efficiently.  
- **Key FreeRTOS Components:**  
  1. **Tasks** – Independent functions running in parallel.  
  2. **Task Scheduling** – Determines task execution order (Preemptive or Cooperative).  
  3. **Semaphores** – Used for task synchronization.  
  4. **Queues** – Used for inter-task communication.  
  5. **Memory Management** – Dynamic allocation using Heap_1 to Heap_5.  

---

### **Practical 1: Creating Two FreeRTOS Tasks**
**Objective:** Create two tasks that blink two LEDs at different intervals.  

#### **Steps to Implement:**
1. **Initialize FreeRTOS and create two tasks.**  
2. **One task blinks LED1 every 500ms.**  
3. **Another task blinks LED2 every 1000ms.**  
4. **Start the FreeRTOS scheduler.**  

---

### **Code for Creating Two FreeRTOS Tasks**
```c
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void LED_Task1(void *pvParameters);
void LED_Task2(void *pvParameters);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    // Create Task 1 - LED1 Blink (500ms)
    xTaskCreate(LED_Task1, "LED1", 128, NULL, 1, NULL);

    // Create Task 2 - LED2 Blink (1000ms)
    xTaskCreate(LED_Task2, "LED2", 128, NULL, 1, NULL);

    // Start FreeRTOS Scheduler
    vTaskStartScheduler();

    while (1)
    {
        // Should never reach here
    }
}

/* Task 1: LED1 Blink Every 500ms */
void LED_Task1(void *pvParameters)
{
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Toggle LED1
        vTaskDelay(pdMS_TO_TICKS(500));       // Delay 500ms
    }
}

/* Task 2: LED2 Blink Every 1000ms */
void LED_Task2(void *pvParameters)
{
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6); // Toggle LED2
        vTaskDelay(pdMS_TO_TICKS(1000));       // Delay 1000ms
    }
}

/* GPIO Initialization */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure LED1 (PA5)
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure LED2 (PA6)
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
- **xTaskCreate()** creates two tasks:  
  - **LED_Task1:** Blinks LED1 every **500ms**.  
  - **LED_Task2:** Blinks LED2 every **1000ms**.  
- **vTaskDelay(pdMS_TO_TICKS(x))** suspends the task for `x` milliseconds.  
- **vTaskStartScheduler()** starts FreeRTOS task management.  

---

### **Expected Output:**  
- **LED1 (PA5) blinks every 500ms.**  
- **LED2 (PA6) blinks every 1000ms.**  

---

## **Practical 2: Task Synchronization Using a Semaphore**
### **Objective:**  
- Use a **binary semaphore** to synchronize task execution.  
- **Button Press (PA0) triggers LED blinking.**  

---

### **Steps to Implement:**  
1. **Create a binary semaphore.**  
2. **One task waits for the semaphore signal.**  
3. **Button press (EXTI) releases the semaphore.**  
4. **LED toggles on receiving semaphore.**  

---

### **Code for Task Synchronization Using Semaphore**
```c
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void LED_Task(void *pvParameters);
SemaphoreHandle_t xBinarySemaphore;

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    // Create a binary semaphore
    xBinarySemaphore = xSemaphoreCreateBinary();

    // Create LED task
    xTaskCreate(LED_Task, "LED_Task", 128, NULL, 1, NULL);

    // Start FreeRTOS Scheduler
    vTaskStartScheduler();

    while (1)
    {
        // Should never reach here
    }
}

/* LED Task - Waits for Semaphore */
void LED_Task(void *pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdTRUE)
        {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Toggle LED
        }
    }
}

/* Button Interrupt Callback */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)
    {
        xSemaphoreGiveFromISR(xBinarySemaphore, NULL);
    }
}

/* GPIO Initialization */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure LED (PA5)
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure Button (PA0)
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Enable EXTI0 interrupt
    HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/* EXTI ISR */
void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
- **xSemaphoreCreateBinary()** creates a binary semaphore.  
- **LED_Task() waits for semaphore** (blocks until button press).  
- **Button Press (PA0) triggers EXTI0, which gives the semaphore**.  
- **Semaphore release unblocks LED_Task(), toggling the LED**.  

---

### **Expected Output:**  
- **LED blinks once whenever the button (PA0) is pressed**.  

---

## **Summary of FreeRTOS Examples**
| Example | FreeRTOS Feature | Description |
|---------|-----------------|-------------|
| **Two Tasks** | **Task Scheduling** | LED1 (500ms), LED2 (1000ms) |
| **Button Press LED** | **Binary Semaphore** | Button triggers LED toggle |

---

### **Next Step: FreeRTOS Queues for Inter-Task Communication**  


### **Step 13: FreeRTOS Queues for Inter-Task Communication**  

#### **Theory:**  
- **Queues in FreeRTOS** are used to **pass messages between tasks**.  
- A queue is a **FIFO (First In, First Out) buffer** for storing and retrieving data.  
- **Common Use Cases:**  
  - Sending sensor data from **ADC Task** to **Processing Task**.  
  - Communicating between **UART Task** and **Main Task**.  
  - Passing commands between tasks.  

---

### **Practical: Using FreeRTOS Queue for Inter-Task Communication**  
#### **Objective:**  
- Create **Task 1 (Sender)** that **sends a number to a queue** every second.  
- Create **Task 2 (Receiver)** that **receives and prints the number** over UART.  

---

### **Steps to Implement:**  
1. **Create a queue** to hold integer values.  
2. **Task 1 (Sender)** sends a **counter value** to the queue every 1000ms.  
3. **Task 2 (Receiver)** reads the queue and **sends the value over UART**.  
4. **Start FreeRTOS scheduler**.  

---

### **Code for FreeRTOS Queue Example**
```c
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stdio.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void Sender_Task(void *pvParameters);
void Receiver_Task(void *pvParameters);

UART_HandleTypeDef huart2;
QueueHandle_t xQueue;

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    // Create a queue of size 5 (storing int values)
    xQueue = xQueueCreate(5, sizeof(int));

    if (xQueue != NULL)
    {
        // Create Sender Task
        xTaskCreate(Sender_Task, "Sender", 128, NULL, 1, NULL);
        // Create Receiver Task
        xTaskCreate(Receiver_Task, "Receiver", 128, NULL, 1, NULL);

        // Start FreeRTOS Scheduler
        vTaskStartScheduler();
    }

    while (1)
    {
        // Should never reach here
    }
}

/* Sender Task: Sends a counter value to the queue */
void Sender_Task(void *pvParameters)
{
    int count = 0;
    while (1)
    {
        count++;
        xQueueSend(xQueue, &count, portMAX_DELAY); // Send count to queue
        vTaskDelay(pdMS_TO_TICKS(1000));          // Delay 1 sec
    }
}

/* Receiver Task: Reads from queue and prints to UART */
void Receiver_Task(void *pvParameters)
{
    int receivedValue;
    char msg[50];

    while (1)
    {
        if (xQueueReceive(xQueue, &receivedValue, portMAX_DELAY) == pdTRUE)
        {
            sprintf(msg, "Received: %d\r\n", receivedValue);
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
        }
    }
}

/* UART2 Initialization */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

/* GPIO Initialization */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
- **xQueueCreate(5, sizeof(int))** creates a queue to hold **5 integers**.  
- **Sender_Task()** increments a counter and sends it to the queue every **1000ms**.  
- **Receiver_Task()** waits for a new value, reads it, and sends it over **UART2**.  
- **HAL_UART_Transmit()** prints received values on a terminal.  

---

### **Expected Output (UART Terminal, Baud 115200)**  
```
Received: 1
Received: 2
Received: 3
...
```

---

### **Summary of FreeRTOS Queue Example**  
| **Component** | **Function** |
|--------------|-------------|
| **Queue** | Stores integer values |
| **Sender Task** | Sends counter values every second |
| **Receiver Task** | Reads queue and prints via UART |

---

## **Next Step: FreeRTOS Memory Management (Dynamic Allocation)**  

### **Step 14: FreeRTOS Memory Management (Dynamic Allocation)**  

#### **Theory:**  
- FreeRTOS provides different memory allocation methods to manage **heap usage**.  
- Common memory management schemes in FreeRTOS:  
  1. **heap_1:** Simple allocation (no freeing).  
  2. **heap_2:** Allows freeing but not memory fragmentation handling.  
  3. **heap_3:** Uses C’s **malloc()** and **free()**.  
  4. **heap_4:** Best for embedded, with **memory defragmentation**.  
  5. **heap_5:** Similar to heap_4 but allows **multiple memory regions**.  

---

### **Practical: Allocating and Freeing Memory Dynamically**  
#### **Objective:**  
- Allocate memory dynamically for an array inside a task.  
- Use **pvPortMalloc()** (FreeRTOS equivalent of malloc).  
- Free memory using **vPortFree()**.  
- Print results over UART.

---

### **Steps to Implement:**  
1. Create a **Task** that dynamically allocates an array.  
2. Check if memory allocation was successful.  
3. Use the memory (store and print values).  
4. Free the allocated memory.  
5. Repeat allocation after a delay.  

---

### **Code for FreeRTOS Dynamic Memory Allocation**
```c
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void Memory_Task(void *pvParameters);

UART_HandleTypeDef huart2;

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    // Create a task to test dynamic memory allocation
    xTaskCreate(Memory_Task, "MemoryTest", 256, NULL, 1, NULL);

    // Start FreeRTOS Scheduler
    vTaskStartScheduler();

    while (1)
    {
        // Should never reach here
    }
}

/* Task to test FreeRTOS dynamic memory allocation */
void Memory_Task(void *pvParameters)
{
    char msg[50];

    while (1)
    {
        // Allocate memory for an array of 10 integers
        int *arr = (int *)pvPortMalloc(10 * sizeof(int));

        if (arr == NULL)
        {
            sprintf(msg, "Memory Allocation Failed!\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
        }
        else
        {
            // Assign values and print them
            for (int i = 0; i < 10; i++)
            {
                arr[i] = i * 10;
                sprintf(msg, "arr[%d] = %d\r\n", i, arr[i]);
                HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
            }

            // Free allocated memory
            vPortFree(arr);
            sprintf(msg, "Memory Freed\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Wait 2 seconds before next allocation
    }
}

/* UART2 Initialization */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

/* GPIO Initialization */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
- **pvPortMalloc(10 * sizeof(int))** dynamically allocates an array of **10 integers**.  
- If allocation fails, it prints **"Memory Allocation Failed!"**.  
- The program assigns **values to the array** and prints them over **UART**.  
- **vPortFree(arr)** releases the allocated memory.  
- **Task repeats allocation every 2 seconds**.  

---

### **Expected Output (UART Terminal, Baud 115200)**  
```
arr[0] = 0
arr[1] = 10
arr[2] = 20
...
arr[9] = 90
Memory Freed
```

---

### **Summary of FreeRTOS Memory Management**  
| **Function** | **Purpose** |
|-------------|-------------|
| **pvPortMalloc()** | Allocates memory dynamically |
| **vPortFree()** | Frees allocated memory |
| **heap_4** | Best choice for FreeRTOS |

---

## **Next Step: FreeRTOS Mutex for Task Synchronization**  


### **Step 15: FreeRTOS Mutex for Task Synchronization**  

#### **Theory:**  
- **Mutex (Mutual Exclusion)** is used to prevent **race conditions** when multiple tasks access a shared resource.  
- Unlike binary semaphores, **Mutexes** support **priority inheritance**, reducing priority inversion problems.  
- A Mutex is locked using **xSemaphoreTake()** and released using **xSemaphoreGive()**.  

---

### **Practical: Synchronizing UART Access with a Mutex**  
#### **Objective:**  
- Create **two tasks** that print messages over **UART**.  
- Use a **Mutex** to ensure **only one task** prints at a time.  

---

### **Steps to Implement:**  
1. **Create a Mutex** before starting the scheduler.  
2. **Two tasks (Task1 and Task2)** will try to print over **UART**.  
3. Each task will **take the Mutex**, print a message, **release the Mutex**, and then delay.  
4. The Mutex ensures **messages don't mix up in UART output**.  

---

### **Code for FreeRTOS Mutex Synchronization**
```c
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stdio.h"

UART_HandleTypeDef huart2;
SemaphoreHandle_t uartMutex;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void Task1(void *pvParameters);
void Task2(void *pvParameters);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    // Create Mutex before starting tasks
    uartMutex = xSemaphoreCreateMutex();

    if (uartMutex != NULL)
    {
        // Create tasks
        xTaskCreate(Task1, "Task1", 256, NULL, 1, NULL);
        xTaskCreate(Task2, "Task2", 256, NULL, 1, NULL);

        // Start FreeRTOS Scheduler
        vTaskStartScheduler();
    }

    while (1)
    {
        // Should never reach here
    }
}

/* Task 1 - Prints a message using Mutex */
void Task1(void *pvParameters)
{
    char msg[50];
    while (1)
    {
        if (xSemaphoreTake(uartMutex, portMAX_DELAY) == pdTRUE)
        {
            sprintf(msg, "Task 1 is running\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
            xSemaphoreGive(uartMutex); // Release Mutex
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay 1 second
    }
}

/* Task 2 - Prints a message using Mutex */
void Task2(void *pvParameters)
{
    char msg[50];
    while (1)
    {
        if (xSemaphoreTake(uartMutex, portMAX_DELAY) == pdTRUE)
        {
            sprintf(msg, "Task 2 is running\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
            xSemaphoreGive(uartMutex); // Release Mutex
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay 1 second
    }
}

/* UART2 Initialization */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

/* GPIO Initialization */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
1. **Create a Mutex** using `xSemaphoreCreateMutex()`.  
2. **Task1 and Task2** both attempt to print a message over **UART**.  
3. Each task **takes the Mutex**, prints, then **releases the Mutex**.  
4. **Ensures only one task prints at a time**, avoiding mixed-up messages.  

---

### **Expected Output (UART Terminal, Baud 115200)**  
```
Task 1 is running
Task 2 is running
Task 1 is running
Task 2 is running
...
```

---

### **Summary of FreeRTOS Mutexes**  
| **Function** | **Purpose** |
|-------------|-------------|
| **xSemaphoreCreateMutex()** | Creates a Mutex |
| **xSemaphoreTake()** | Locks (takes) the Mutex |
| **xSemaphoreGive()** | Unlocks (releases) the Mutex |

---

## **Next Step: FreeRTOS Counting Semaphore for Event Handling**  


### **Step 16: FreeRTOS Counting Semaphore for Event Synchronization**  

#### **Theory:**  
- A **Counting Semaphore** is used when multiple instances of an event need to be counted (e.g., button presses, sensor triggers).  
- Unlike a binary semaphore (which allows only 0 or 1), a counting semaphore **increments and decrements** within a set limit.  
- Tasks can **wait** for a specific count and process events when the count is available.  

---

### **Practical: Counting Button Presses Using Semaphore**  
#### **Objective:**  
- Use an **external push button** to **increase a counting semaphore**.  
- A task will **process** the button presses whenever it detects an event.  

---

### **Steps to Implement:**  
1. **Create a counting semaphore** before the scheduler starts.  
2. **Configure an external interrupt (EXTI) for the button press.**  
3. **Interrupt Handler (ISR) will increment the semaphore count.**  
4. **A task waits for the semaphore count to process button presses.**  

---

### **Code for FreeRTOS Counting Semaphore (Button Press Handler)**
```c
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stdio.h"

UART_HandleTypeDef huart2;
SemaphoreHandle_t buttonSemaphore;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void ButtonTask(void *pvParameters);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    // Create Counting Semaphore (max count 5, initial count 0)
    buttonSemaphore = xSemaphoreCreateCounting(5, 0);

    if (buttonSemaphore != NULL)
    {
        // Create Task
        xTaskCreate(ButtonTask, "ButtonTask", 256, NULL, 1, NULL);

        // Start FreeRTOS Scheduler
        vTaskStartScheduler();
    }

    while (1)
    {
        // Should never reach here
    }
}

/* Task to Process Button Press Events */
void ButtonTask(void *pvParameters)
{
    char msg[50];
    while (1)
    {
        // Wait for a button press event
        if (xSemaphoreTake(buttonSemaphore, portMAX_DELAY) == pdTRUE)
        {
            sprintf(msg, "Button Press Detected!\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
        }
    }
}

/* External Interrupt Callback (Triggered on Button Press) */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0) // Check if it's the button pin
    {
        xSemaphoreGiveFromISR(buttonSemaphore, NULL); // Increment the semaphore
    }
}

/* GPIO Initialization (Button on PA0, EXTI Interrupt) */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // Interrupt on Rising Edge
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Enable EXTI Interrupt in NVIC
    HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/* UART2 Initialization */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
1. **Create a Counting Semaphore** using `xSemaphoreCreateCounting(5, 0)`.  
   - Max count = **5** (stores up to 5 events).  
   - Initial count = **0** (no events initially).  
2. **EXTI Interrupt Handler** (`HAL_GPIO_EXTI_Callback`) increments the semaphore when the button is pressed.  
3. **Button Task** waits for the semaphore and prints a message each time a button press is detected.  

---

### **Expected Output (UART Terminal, Baud 115200)**  
```
Button Press Detected!
Button Press Detected!
Button Press Detected!
...
```

---

### **Summary of FreeRTOS Counting Semaphore**  
| **Function** | **Purpose** |
|-------------|-------------|
| **xSemaphoreCreateCounting(max, initial)** | Creates a counting semaphore |
| **xSemaphoreGive()** | Increments the semaphore count |
| **xSemaphoreTake()** | Decrements the semaphore count |
| **xSemaphoreGiveFromISR()** | Used in Interrupts to signal an event |

---

## **Next Step: FreeRTOS Software Timers for Periodic Tasks**  


### **Step 17: FreeRTOS Software Timers for Periodic Tasks**  

#### **Theory:**  
- **Software Timers** in FreeRTOS allow you to execute a function **periodically** or **after a delay** without blocking the CPU.  
- Unlike **hardware timers**, they run **within the FreeRTOS kernel** and do not require dedicated MCU timers.  
- Can be **one-shot** (runs once) or **auto-reload** (repeats after a period).  
- Used for **periodic events** like blinking an LED, monitoring a sensor, or timeout handling.  

---

### **Practical: Blinking an LED Using a FreeRTOS Software Timer**  
#### **Objective:**  
- Create a **software timer** to toggle an LED **every 1 second**.  
- Use **Auto-Reload mode** to make it run continuously.  

---

### **Steps to Implement:**  
1. **Create a Software Timer** before the scheduler starts.  
2. **Define a Timer Callback Function** that toggles the LED.  
3. **Start the Timer** when the scheduler starts.  
4. **Observe the LED blinking periodically.**  

---

### **Code for FreeRTOS Software Timer (LED Blinker)**
```c
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#define LED_PIN GPIO_PIN_13
#define LED_PORT GPIOC

TimerHandle_t ledTimerHandle;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void LED_Timer_Callback(TimerHandle_t xTimer);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    // Create a software timer (Auto-Reload mode, period = 1000ms)
    ledTimerHandle = xTimerCreate("LED Timer", pdMS_TO_TICKS(1000), pdTRUE, (void *)0, LED_Timer_Callback);

    if (ledTimerHandle != NULL)
    {
        xTimerStart(ledTimerHandle, 0); // Start the timer
    }

    vTaskStartScheduler(); // Start FreeRTOS

    while (1)
    {
        // Should never reach here
    }
}

/* Timer Callback Function (Executed every 1 sec) */
void LED_Timer_Callback(TimerHandle_t xTimer)
{
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN); // Toggle LED
}

/* GPIO Initialization (LED on PC13) */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
1. **Create a Software Timer** using `xTimerCreate()` with:  
   - **Timer Name:** `"LED Timer"`  
   - **Period:** `1000ms` (1 second)  
   - **Auto-Reload:** `pdTRUE` (runs continuously)  
   - **Callback Function:** `LED_Timer_Callback()`  
2. **Start the Timer** using `xTimerStart()` before the scheduler starts.  
3. **Timer Callback Function** toggles the LED every second.  
4. **LED blinks periodically** without blocking other tasks.  

---

### **Expected Behavior:**  
- The LED on **PC13** will blink every **1 second**.  

---

### **Summary of FreeRTOS Software Timers**  
| **Function** | **Purpose** |
|-------------|-------------|
| **xTimerCreate(name, period, auto-reload, id, callback)** | Creates a software timer |
| **xTimerStart(timer, timeout)** | Starts the timer |
| **xTimerStop(timer, timeout)** | Stops the timer |
| **xTimerChangePeriod(timer, newPeriod, timeout)** | Changes the timer period |
| **xTimerReset(timer, timeout)** | Resets the timer |

---

## **Next Step: FreeRTOS Event Groups for Multi-task Synchronization**  


### **Step 18: FreeRTOS Event Groups for Multi-Task Synchronization**  

#### **Theory:**  
- **Event Groups** in FreeRTOS allow tasks to synchronize using **bit flags** (similar to binary semaphores but more flexible).  
- Multiple tasks can **set, wait for, or clear specific bits** in an event group.  
- Useful for **task synchronization, signaling, and multi-event handling**.  

---

### **Practical: Synchronizing Two Tasks Using an Event Group**  
#### **Objective:**  
- **Task 1:** Sets a bit in an event group every 2 seconds.  
- **Task 2:** Waits for the event and toggles an LED when the event occurs.  

---

### **Steps to Implement:**  
1. **Create an Event Group** before the scheduler starts.  
2. **Task 1:** Sets a specific bit in the event group every 2 seconds.  
3. **Task 2:** Waits for the bit to be set, toggles an LED, and clears the bit.  

---

### **Code for FreeRTOS Event Groups**
```c
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

#define LED_PIN GPIO_PIN_13
#define LED_PORT GPIOC
#define EVENT_BIT (1 << 0) // Define event bit 0

EventGroupHandle_t eventGroupHandle;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void Task1_SetEvent(void *pvParameters);
void Task2_WaitEvent(void *pvParameters);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    // Create an Event Group
    eventGroupHandle = xEventGroupCreate();

    // Create two tasks
    xTaskCreate(Task1_SetEvent, "Task 1", 128, NULL, 1, NULL);
    xTaskCreate(Task2_WaitEvent, "Task 2", 128, NULL, 2, NULL);

    vTaskStartScheduler(); // Start FreeRTOS

    while (1)
    {
        // Should never reach here
    }
}

/* Task 1: Sets the Event Bit every 2 seconds */
void Task1_SetEvent(void *pvParameters)
{
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for 2 seconds
        xEventGroupSetBits(eventGroupHandle, EVENT_BIT); // Set event bit
    }
}

/* Task 2: Waits for the Event Bit and Toggles LED */
void Task2_WaitEvent(void *pvParameters)
{
    while (1)
    {
        xEventGroupWaitBits(eventGroupHandle, EVENT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN); // Toggle LED
    }
}

/* GPIO Initialization (LED on PC13) */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
1. **Create an Event Group** using `xEventGroupCreate()`.  
2. **Task 1:** Uses `xEventGroupSetBits()` to **set a bit** in the event group every 2 seconds.  
3. **Task 2:** Uses `xEventGroupWaitBits()` to **wait for the bit**, toggles an LED, and clears the bit.  
4. **The LED blinks every 2 seconds** when Task 2 detects the event.  

---

### **Expected Behavior:**  
- Every **2 seconds**, the LED **toggles** when Task 2 detects the event.  

---

### **Summary of FreeRTOS Event Groups**  
| **Function** | **Purpose** |
|-------------|-------------|
| **xEventGroupCreate()** | Creates an event group |
| **xEventGroupSetBits(eventGroup, bitsToSet)** | Sets specific bits in an event group |
| **xEventGroupClearBits(eventGroup, bitsToClear)** | Clears specific bits |
| **xEventGroupWaitBits(eventGroup, bitsToWaitFor, clearOnExit, waitForAll, timeout)** | Waits for bits to be set |

---

## **Next Step: FreeRTOS Message Queues for Task Communication**  


### **Step 19: FreeRTOS Message Queues for Task Communication**  

#### **Theory:**  
- **Message Queues** in FreeRTOS allow tasks to exchange data safely.  
- One task **sends data** into the queue, and another task **receives it**.  
- Queues store **multiple messages** in FIFO (First-In-First-Out) order.  
- Useful for **task synchronization and inter-task communication**.  

---

### **Practical: Sending and Receiving Data Between Two Tasks Using a Queue**  
#### **Objective:**  
- **Task 1:** Sends an **incrementing number** every second.  
- **Task 2:** Reads the number and **toggles an LED** if the number is even.  

---

### **Steps to Implement:**  
1. **Create a Queue** before the scheduler starts.  
2. **Task 1:** Puts an integer into the queue every second.  
3. **Task 2:** Reads the integer from the queue and toggles an LED if the value is even.  

---

### **Code for FreeRTOS Message Queue**
```c
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define LED_PIN GPIO_PIN_13
#define LED_PORT GPIOC

QueueHandle_t messageQueue;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void Task1_Send(void *pvParameters);
void Task2_Receive(void *pvParameters);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    // Create a queue to store integers (each message is sizeof(int))
    messageQueue = xQueueCreate(5, sizeof(int));

    // Create two tasks
    xTaskCreate(Task1_Send, "Task 1", 128, NULL, 1, NULL);
    xTaskCreate(Task2_Receive, "Task 2", 128, NULL, 2, NULL);

    vTaskStartScheduler(); // Start FreeRTOS

    while (1)
    {
        // Should never reach here
    }
}

/* Task 1: Sends an incrementing number to the queue */
void Task1_Send(void *pvParameters)
{
    int count = 0;
    while (1)
    {
        count++; // Increment count
        xQueueSend(messageQueue, &count, portMAX_DELAY); // Send data to queue
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second
    }
}

/* Task 2: Receives data from the queue and toggles LED if even */
void Task2_Receive(void *pvParameters)
{
    int receivedValue;
    while (1)
    {
        if (xQueueReceive(messageQueue, &receivedValue, portMAX_DELAY) == pdTRUE)
        {
            if (receivedValue % 2 == 0) // Check if even
            {
                HAL_GPIO_TogglePin(LED_PORT, LED_PIN); // Toggle LED
            }
        }
    }
}

/* GPIO Initialization (LED on PC13) */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
1. **Create a Queue** using `xQueueCreate(5, sizeof(int))`.  
2. **Task 1:** Uses `xQueueSend()` to send an incrementing number every second.  
3. **Task 2:** Uses `xQueueReceive()` to read the number and **toggles the LED** if it is even.  

---

### **Expected Behavior:**  
- **Every second**, Task 1 sends an incrementing number to the queue.  
- Task 2 reads the number and toggles the LED **only if the number is even**.  

---

### **Summary of FreeRTOS Queue Functions**  
| **Function** | **Purpose** |
|-------------|-------------|
| **xQueueCreate(length, itemSize)** | Creates a queue |
| **xQueueSend(queue, &data, timeout)** | Sends data to the queue |
| **xQueueReceive(queue, &buffer, timeout)** | Reads data from the queue |

---

## **Next Step: FreeRTOS Timers for Periodic Tasks**  


### **Step 20: FreeRTOS Software Timers for Periodic Tasks**  

#### **Theory:**  
- FreeRTOS **software timers** allow functions to execute **periodically** without needing a dedicated task.  
- Timers run in the **background** using the FreeRTOS **timer service task**.  
- There are **one-shot timers** (execute once) and **periodic timers** (execute repeatedly).  
- Useful for **blinking an LED**, **monitoring sensors**, or **triggering events at fixed intervals**.

---

### **Practical: Blink LED Every 500ms Using a FreeRTOS Timer**  
#### **Objective:**  
- Create a **FreeRTOS software timer** that **toggles an LED** every 500ms.

---

### **Steps to Implement:**  
1. **Create a timer** before starting the scheduler.  
2. **Set the timer period** to **500ms** (for LED blinking).  
3. **Start the timer** inside `main()`.  
4. **Define the timer callback function** to toggle the LED.  

---

### **Code for FreeRTOS Timer**
```c
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#define LED_PIN GPIO_PIN_13
#define LED_PORT GPIOC

TimerHandle_t LEDTimer;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void LED_Timer_Callback(TimerHandle_t xTimer);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    // Create a software timer (500ms interval, auto-reload mode)
    LEDTimer = xTimerCreate("LED Timer", pdMS_TO_TICKS(500), pdTRUE, 0, LED_Timer_Callback);

    if (LEDTimer != NULL)
    {
        xTimerStart(LEDTimer, 0); // Start the timer
    }

    vTaskStartScheduler(); // Start FreeRTOS

    while (1)
    {
        // Should never reach here
    }
}

/* Timer Callback Function */
void LED_Timer_Callback(TimerHandle_t xTimer)
{
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN); // Toggle LED
}

/* GPIO Initialization (LED on PC13) */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
1. **Create a Timer** using `xTimerCreate()`, with:
   - **500ms period** (`pdMS_TO_TICKS(500)`)
   - **Auto-reload enabled** (`pdTRUE`)
   - **Callback function** `LED_Timer_Callback()`
2. **Start the Timer** using `xTimerStart()`.
3. **Timer Callback Function** toggles the LED every 500ms.

---

### **Expected Behavior:**  
- The LED **blinks every 500ms** without needing a dedicated FreeRTOS task.

---

### **Summary of FreeRTOS Timer Functions**  
| **Function** | **Purpose** |
|-------------|-------------|
| **xTimerCreate(name, period, autoReload, ID, callback)** | Creates a timer |
| **xTimerStart(timer, timeout)** | Starts the timer |
| **xTimerStop(timer, timeout)** | Stops the timer |
| **xTimerChangePeriod(timer, newPeriod, timeout)** | Changes timer period |

---

## **Next Step: FreeRTOS Event Groups for Task Synchronization**  


### **Step 21: FreeRTOS Event Groups for Task Synchronization**  

#### **Theory:**  
- **Event Groups** in FreeRTOS allow multiple tasks to synchronize using **bitwise flags**.  
- Tasks can **wait for specific bits** to be set before continuing execution.  
- Useful for **inter-task communication**, **synchronization**, and **event-driven programming**.  

---

### **Practical: Synchronizing Two Tasks Using Event Groups**  
#### **Objective:**  
- Create an **Event Group**.  
- Task 1 sets an event bit every **1 second**.  
- Task 2 waits for the event bit and then **toggles an LED**.  

---

### **Steps to Implement:**  
1. **Create an Event Group** before starting the scheduler.  
2. **Task 1:** Set the event flag every 1 second.  
3. **Task 2:** Wait for the event flag and toggle the LED.  
4. **Clear the event flag** after processing.  

---

### **Code for FreeRTOS Event Groups**
```c
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

#define LED_PIN GPIO_PIN_13
#define LED_PORT GPIOC

#define EVENT_BIT (1 << 0) // Define event bit

EventGroupHandle_t eventGroup;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void Task1_SetEvent(void *pvParameters);
void Task2_WaitForEvent(void *pvParameters);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    eventGroup = xEventGroupCreate(); // Create event group

    xTaskCreate(Task1_SetEvent, "Task1", 128, NULL, 1, NULL);
    xTaskCreate(Task2_WaitForEvent, "Task2", 128, NULL, 1, NULL);

    vTaskStartScheduler(); // Start FreeRTOS

    while (1)
    {
        // Should never reach here
    }
}

/* Task 1: Set Event Every 1 Second */
void Task1_SetEvent(void *pvParameters)
{
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second
        xEventGroupSetBits(eventGroup, EVENT_BIT); // Set event flag
    }
}

/* Task 2: Wait for Event and Toggle LED */
void Task2_WaitForEvent(void *pvParameters)
{
    while (1)
    {
        xEventGroupWaitBits(eventGroup, EVENT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN); // Toggle LED
    }
}

/* GPIO Initialization */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
1. **Create an Event Group** using `xEventGroupCreate()`.  
2. **Task 1:** Sets the event flag every **1 second** using `xEventGroupSetBits()`.  
3. **Task 2:** Waits for the event flag using `xEventGroupWaitBits()` before toggling the LED.  
4. The event bit is **cleared after processing** using `pdTRUE` in `xEventGroupWaitBits()`.  

---

### **Expected Behavior:**  
- The LED **toggles every second** based on the event flag set by Task 1.  

---

### **Summary of FreeRTOS Event Group Functions**  
| **Function** | **Purpose** |
|-------------|-------------|
| **xEventGroupCreate()** | Creates an event group |
| **xEventGroupSetBits(group, bits)** | Sets event flags |
| **xEventGroupWaitBits(group, bits, clear, all, timeout)** | Waits for event flags |
| **xEventGroupClearBits(group, bits)** | Clears specific event flags |

---

## **Next Step: FreeRTOS Mutex for Resource Sharing**  


### **Step 22: FreeRTOS Mutex for Resource Sharing**  

#### **Theory:**  
- **Mutex (Mutual Exclusion)** is used to protect **shared resources** (e.g., UART, LCD, variables).  
- Ensures that **only one task** accesses the resource at a time.  
- Prevents **race conditions** and **data corruption** in multitasking environments.  

---

### **Practical: Protecting UART Communication with Mutex**  
#### **Objective:**  
- Create a **Mutex** to synchronize access to **UART**.  
- Two tasks will **attempt to send messages** via UART.  
- Mutex ensures only **one task** accesses UART at a time.  

---

### **Steps to Implement:**  
1. **Create a Mutex** before starting the scheduler.  
2. **Task 1:** Prints `"Task 1: UART Access"` every 1 second.  
3. **Task 2:** Prints `"Task 2: UART Access"` every 2 seconds.  
4. Both tasks **acquire and release the Mutex** before accessing UART.  

---

### **Code for FreeRTOS Mutex**
```c
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define UART_TIMEOUT 100

UART_HandleTypeDef huart2;
SemaphoreHandle_t uartMutex;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void Task1_Print(void *pvParameters);
void Task2_Print(void *pvParameters);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    uartMutex = xSemaphoreCreateMutex(); // Create Mutex

    xTaskCreate(Task1_Print, "Task1", 128, NULL, 1, NULL);
    xTaskCreate(Task2_Print, "Task2", 128, NULL, 1, NULL);

    vTaskStartScheduler(); // Start FreeRTOS

    while (1)
    {
        // Should never reach here
    }
}

/* Task 1: Prints Message Every 1 Second */
void Task1_Print(void *pvParameters)
{
    char msg[] = "Task 1: UART Access\r\n";
    while (1)
    {
        if (xSemaphoreTake(uartMutex, portMAX_DELAY) == pdTRUE)
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg) - 1, UART_TIMEOUT);
            xSemaphoreGive(uartMutex); // Release Mutex
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 sec
    }
}

/* Task 2: Prints Message Every 2 Seconds */
void Task2_Print(void *pvParameters)
{
    char msg[] = "Task 2: UART Access\r\n";
    while (1)
    {
        if (xSemaphoreTake(uartMutex, portMAX_DELAY) == pdTRUE)
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg) - 1, UART_TIMEOUT);
            xSemaphoreGive(uartMutex); // Release Mutex
        }
        vTaskDelay(pdMS_TO_TICKS(2000)); // Wait 2 sec
    }
}

/* UART Initialization */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

/* GPIO Initialization */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
1. **Create a Mutex** using `xSemaphoreCreateMutex()`.  
2. **Task 1 and Task 2** try to access UART.  
3. Each task **locks the Mutex** using `xSemaphoreTake()`.  
4. Task sends data over UART using `HAL_UART_Transmit()`.  
5. After sending, **release the Mutex** using `xSemaphoreGive()`.  
6. **Task 1 waits 1 second, Task 2 waits 2 seconds** before retrying.  

---

### **Expected Behavior:**  
- `"Task 1: UART Access"` prints every **1 second**.  
- `"Task 2: UART Access"` prints every **2 seconds**.  
- Messages **never overlap** due to Mutex protection.  

---

### **Summary of FreeRTOS Mutex Functions**  
| **Function** | **Purpose** |
|-------------|-------------|
| **xSemaphoreCreateMutex()** | Creates a Mutex |
| **xSemaphoreTake(mutex, timeout)** | Locks the Mutex |
| **xSemaphoreGive(mutex)** | Releases the Mutex |

---

## **Next Step: FreeRTOS Software Timers**  


### **Step 23: FreeRTOS Software Timers**  

#### **Theory:**  
- **Software Timers** allow periodic or one-shot execution of functions in FreeRTOS.  
- Unlike hardware timers, software timers **don't require hardware resources**.  
- Useful for **delayed function execution** or **periodic tasks** (e.g., blinking an LED, watchdog reset).  

---

### **Practical: Blinking an LED Using FreeRTOS Software Timer**  
#### **Objective:**  
- Create a **software timer** that toggles an LED every **500ms**.  
- The timer callback function will execute on **timer expiry**.  
- The LED toggling will be **handled by the software timer** instead of a task.  

---

### **Steps to Implement:**  
1. **Create a software timer** before starting FreeRTOS.  
2. **Define a callback function** that toggles an LED.  
3. **Start the timer** when FreeRTOS begins execution.  
4. Timer **expires every 500ms**, and the LED toggles.  

---

### **Code for FreeRTOS Software Timer**
```c
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#define LED_TOGGLE_PERIOD pdMS_TO_TICKS(500) // 500ms

TimerHandle_t ledTimerHandle; // Timer handle

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void LedTimerCallback(TimerHandle_t xTimer);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    // Create software timer: Periodic mode, 500ms, Auto-start
    ledTimerHandle = xTimerCreate("LedTimer", LED_TOGGLE_PERIOD, pdTRUE, NULL, LedTimerCallback);
    
    if (ledTimerHandle != NULL)
    {
        xTimerStart(ledTimerHandle, 0); // Start timer
    }

    vTaskStartScheduler(); // Start FreeRTOS

    while (1)
    {
        // Should never reach here
    }
}

/* Timer Callback Function */
void LedTimerCallback(TimerHandle_t xTimer)
{
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Toggle LED
}

/* GPIO Initialization */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
1. **Create Software Timer:**  
   ```c
   ledTimerHandle = xTimerCreate("LedTimer", LED_TOGGLE_PERIOD, pdTRUE, NULL, LedTimerCallback);
   ```
   - Name: `"LedTimer"`
   - Period: **500ms** (converted to ticks)
   - Mode: **Periodic** (auto-repeats)
   - Callback: `LedTimerCallback()` executes on timer expiry.

2. **Start the Timer:**
   ```c
   xTimerStart(ledTimerHandle, 0);
   ```
   - Starts the **500ms timer** before FreeRTOS starts.

3. **Toggle LED in Callback Function:**
   ```c
   void LedTimerCallback(TimerHandle_t xTimer)
   {
       HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
   }
   ```
   - Executes **every 500ms**, toggling LED.  

---

### **Expected Behavior:**  
- **LED blinks every 500ms** using the software timer.  
- **No dedicated task is required**, reducing CPU usage.  

---

### **Summary of FreeRTOS Software Timer Functions**  
| **Function** | **Purpose** |
|-------------|-------------|
| **xTimerCreate()** | Creates a software timer |
| **xTimerStart()** | Starts the timer |
| **xTimerStop()** | Stops the timer |
| **xTimerReset()** | Resets the timer period |
| **xTimerChangePeriod()** | Modifies the timer interval |

---

## **Next Step: FreeRTOS Event Groups**  


### **Step 24: FreeRTOS Event Groups**  

#### **Theory:**  
- **Event Groups** are a way for tasks to communicate using **bit flags**.  
- Each **bit** in an **event group** represents a separate **event or condition**.  
- Tasks can **wait for specific events**, or **set/clear event bits** when conditions occur.  
- **Useful for:**  
  - Synchronizing multiple tasks.  
  - Managing hardware events like **button presses, data ready signals, etc.**  
  - **Efficient alternative to polling.**  

---

### **Practical: Synchronizing Two Tasks Using Event Groups**  
#### **Objective:**  
- One task (**Task A**) **waits for an event bit** before proceeding.  
- Another task (**Task B**) **sets the event bit** after a delay.  
- When the event is set, **Task A resumes execution**.  

---

### **Steps to Implement:**  
1. **Create an event group** before starting FreeRTOS.  
2. **Define two tasks:**  
   - **Task A:** Waits for an event.  
   - **Task B:** Sets the event after 2 seconds.  
3. **Run FreeRTOS Scheduler** to execute the tasks.  

---

### **Code for FreeRTOS Event Groups**
```c
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

#define EVENT_BIT (1 << 0) // Event bit 0

EventGroupHandle_t eventGroup; // Event Group Handle

void TaskA(void *pvParameters);
void TaskB(void *pvParameters);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    // Create Event Group
    eventGroup = xEventGroupCreate();

    // Create Tasks
    xTaskCreate(TaskA, "TaskA", 128, NULL, 2, NULL);
    xTaskCreate(TaskB, "TaskB", 128, NULL, 1, NULL);

    // Start Scheduler
    vTaskStartScheduler();

    while (1)
    {
        // Should never reach here
    }
}

/* Task A - Waits for Event */
void TaskA(void *pvParameters)
{
    while (1)
    {
        printf("Task A: Waiting for event...\n");

        // Wait for event bit 0 to be set
        xEventGroupWaitBits(eventGroup, EVENT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

        printf("Task A: Event received! Continuing...\n");

        vTaskDelay(pdMS_TO_TICKS(1000)); // Simulate work
    }
}

/* Task B - Sets Event */
void TaskB(void *pvParameters)
{
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(2000)); // Delay for 2 seconds

        printf("Task B: Setting event...\n");

        // Set event bit 0
        xEventGroupSetBits(eventGroup, EVENT_BIT);
    }
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
1. **Create Event Group:**  
   ```c
   eventGroup = xEventGroupCreate();
   ```
   - Allocates memory for an **event group** to store bit flags.

2. **Task A: Waits for the Event Bit**
   ```c
   xEventGroupWaitBits(eventGroup, EVENT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
   ```
   - **Blocks execution** until **EVENT_BIT** is set by another task.
   - **Clears the bit after receiving** (`pdTRUE`).
   - **Waits indefinitely** (`portMAX_DELAY`).

3. **Task B: Sets the Event Bit**
   ```c
   xEventGroupSetBits(eventGroup, EVENT_BIT);
   ```
   - After **2 seconds**, Task B **sets the event**.
   - Task A **resumes execution**.

---

### **Expected Behavior:**  
- Task A **waits** for an event.  
- Task B **sets** the event after 2 seconds.  
- Task A **continues execution** when the event is received.  
- This repeats in a loop.  

---

### **Summary of FreeRTOS Event Group Functions**  
| **Function** | **Purpose** |
|-------------|-------------|
| **xEventGroupCreate()** | Creates an event group |
| **xEventGroupSetBits()** | Sets event bits |
| **xEventGroupClearBits()** | Clears event bits |
| **xEventGroupWaitBits()** | Waits for an event bit |
| **xEventGroupGetBits()** | Reads event group bits |

---

## **Next Step: FreeRTOS Message Queues**  


### **Step 25: FreeRTOS Message Queues**  

#### **Theory:**  
- **Queues** are used for **inter-task communication** in FreeRTOS.  
- They allow tasks to **send and receive messages** (data structures, integers, etc.).  
- **Key Benefits:**  
  - Tasks can communicate **safely** without shared memory issues.  
  - Supports **FIFO (First-In-First-Out)** behavior.  
  - Can be used for **sensor data, logs, control commands, etc.**  

---

### **Practical: Sending and Receiving Messages Using a Queue**  
#### **Objective:**  
- Create a queue to hold **integer values**.  
- Task A **sends a number** to the queue.  
- Task B **receives the number** and prints it.  

---

### **Steps to Implement:**  
1. **Create a queue** before starting FreeRTOS.  
2. **Define two tasks:**  
   - **Task A:** Sends numbers to the queue every second.  
   - **Task B:** Reads numbers from the queue and prints them.  
3. **Run FreeRTOS Scheduler** to execute the tasks.  

---

### **Code for FreeRTOS Queue Example**
```c
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

QueueHandle_t myQueue; // Queue handle

void TaskA(void *pvParameters);
void TaskB(void *pvParameters);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    // Create a queue that holds 5 integers
    myQueue = xQueueCreate(5, sizeof(int));

    // Create Tasks
    xTaskCreate(TaskA, "TaskA", 128, NULL, 2, NULL);
    xTaskCreate(TaskB, "TaskB", 128, NULL, 1, NULL);

    // Start Scheduler
    vTaskStartScheduler();

    while (1)
    {
        // Should never reach here
    }
}

/* Task A - Sends data to Queue */
void TaskA(void *pvParameters)
{
    int value = 0;
    while (1)
    {
        value++;
        printf("Task A: Sending %d to queue\n", value);

        // Send value to queue (wait up to 100ms if queue is full)
        xQueueSend(myQueue, &value, pdMS_TO_TICKS(100));

        vTaskDelay(pdMS_TO_TICKS(1000)); // Send data every 1 sec
    }
}

/* Task B - Receives data from Queue */
void TaskB(void *pvParameters)
{
    int receivedValue;
    while (1)
    {
        // Wait indefinitely until data is received
        if (xQueueReceive(myQueue, &receivedValue, portMAX_DELAY))
        {
            printf("Task B: Received %d from queue\n", receivedValue);
        }
    }
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
1. **Create a Queue**  
   ```c
   myQueue = xQueueCreate(5, sizeof(int));
   ```
   - Creates a queue with **5 slots**, each storing an **integer**.  

2. **Task A: Sends Data to the Queue**  
   ```c
   xQueueSend(myQueue, &value, pdMS_TO_TICKS(100));
   ```
   - Increments `value` and **sends it to the queue**.  
   - Waits **up to 100ms** if the queue is full.  
   - Runs every **1 second** (`vTaskDelay(1000ms)`).  

3. **Task B: Reads Data from the Queue**  
   ```c
   xQueueReceive(myQueue, &receivedValue, portMAX_DELAY);
   ```
   - **Waits indefinitely** (`portMAX_DELAY`) for data.  
   - Reads the value **when available** and prints it.  

---

### **Expected Behavior:**  
- **Task A sends numbers** `1, 2, 3...` every second.  
- **Task B reads and prints** the numbers from the queue.  
- The process **repeats indefinitely**.  

---

### **Summary of FreeRTOS Queue Functions**  
| **Function** | **Purpose** |
|-------------|-------------|
| **xQueueCreate()** | Creates a queue |
| **xQueueSend()** | Sends data to a queue |
| **xQueueReceive()** | Receives data from a queue |
| **uxQueueMessagesWaiting()** | Checks the number of items in the queue |
| **uxQueueSpacesAvailable()** | Checks available space in the queue |

---

## **Next Step: FreeRTOS Mutexes (Avoiding Task Conflicts)**  


### **Step 26: FreeRTOS Mutexes (Mutual Exclusion Semaphores)**  

#### **Theory:**  
- A **Mutex (Mutual Exclusion Semaphore)** is used to **prevent multiple tasks from accessing shared resources** simultaneously.  
- Unlike binary semaphores, **mutexes include priority inheritance**, preventing priority inversion.  
- Commonly used for **protecting shared variables, peripherals (UART, SPI, I2C), or filesystems**.  

---

### **Practical: Using a Mutex to Protect Shared UART Output**  
#### **Objective:**  
- Create a **mutex** to control access to UART.  
- Two tasks will try to **print messages** over UART.  
- Mutex ensures only **one task prints at a time**.  

---

### **Steps to Implement:**  
1. **Create a mutex** before starting FreeRTOS.  
2. **Define two tasks** that both try to use UART.  
3. **Each task takes the mutex before printing** and releases it after.  
4. **Run FreeRTOS Scheduler** to execute the tasks.  

---

### **Code for FreeRTOS Mutex Example**
```c
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

SemaphoreHandle_t uartMutex; // Mutex handle

void Task1(void *pvParameters);
void Task2(void *pvParameters);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    // Create a mutex before starting tasks
    uartMutex = xSemaphoreCreateMutex();

    // Create Tasks
    xTaskCreate(Task1, "Task1", 128, NULL, 2, NULL);
    xTaskCreate(Task2, "Task2", 128, NULL, 1, NULL);

    // Start Scheduler
    vTaskStartScheduler();

    while (1)
    {
        // Should never reach here
    }
}

/* Task 1: Prints a message */
void Task1(void *pvParameters)
{
    while (1)
    {
        // Take the mutex before printing
        if (xSemaphoreTake(uartMutex, portMAX_DELAY))
        {
            printf("Task 1: Hello from Task 1\n");

            // Release the mutex after printing
            xSemaphoreGive(uartMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay 1 sec
    }
}

/* Task 2: Prints a message */
void Task2(void *pvParameters)
{
    while (1)
    {
        // Take the mutex before printing
        if (xSemaphoreTake(uartMutex, portMAX_DELAY))
        {
            printf("Task 2: Hello from Task 2\n");

            // Release the mutex after printing
            xSemaphoreGive(uartMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay 1 sec
    }
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
1. **Create a Mutex**  
   ```c
   uartMutex = xSemaphoreCreateMutex();
   ```
   - Creates a **mutex** to control **UART access**.  

2. **Task 1 and Task 2: Taking and Releasing Mutex**  
   ```c
   if (xSemaphoreTake(uartMutex, portMAX_DELAY))
   ```
   - **Waits indefinitely** for the mutex.  
   - If another task holds the mutex, it **blocks** until it becomes available.  

   ```c
   printf("Task 1: Hello from Task 1\n");
   ```
   - Prints a message **after acquiring the mutex**.  

   ```c
   xSemaphoreGive(uartMutex);
   ```
   - **Releases the mutex** after printing, allowing another task to print.  

3. **vTaskDelay(1000ms)**  
   - Ensures that **tasks do not continuously request the mutex**.  

---

### **Expected Behavior:**  
- **Task 1 and Task 2 alternate printing messages**.  
- **Mutex ensures that only one task prints at a time**.  
- Prevents **UART output corruption** from concurrent access.  

---

### **Summary of FreeRTOS Mutex Functions**  
| **Function** | **Purpose** |
|-------------|-------------|
| **xSemaphoreCreateMutex()** | Creates a mutex |
| **xSemaphoreTake()** | Locks (takes) the mutex |
| **xSemaphoreGive()** | Unlocks (releases) the mutex |

---

## **Next Step: FreeRTOS Event Groups (Task Synchronization)**  


### **Step 27: FreeRTOS Event Groups (Task Synchronization)**  

#### **Theory:**  
- **Event Groups** allow tasks to **synchronize** using **bitwise flags**.  
- Unlike queues and semaphores, they enable multiple tasks to **wait for specific conditions** before proceeding.  
- Each **bit in the event group** represents a different event, and tasks can **wait for one or multiple events**.  

---

### **Practical: Synchronizing Two Tasks Using an Event Group**  
#### **Objective:**  
- Create an **Event Group** to synchronize two tasks.  
- **Task 1 sets an event** after performing an operation.  
- **Task 2 waits for the event** before proceeding.  

---

### **Steps to Implement:**  
1. **Create an Event Group** before starting FreeRTOS.  
2. **Define two tasks:**  
   - **Task 1:** Sets an event bit after a delay.  
   - **Task 2:** Waits for the event bit before executing.  
3. **Run the FreeRTOS Scheduler** to execute tasks.  

---

### **Code for FreeRTOS Event Group Example**
```c
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

EventGroupHandle_t eventGroup; // Handle for Event Group

#define EVENT_BIT (1 << 0) // Define event bit (Bit 0)

void Task1(void *pvParameters);
void Task2(void *pvParameters);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    // Create an event group before starting tasks
    eventGroup = xEventGroupCreate();

    // Create Tasks
    xTaskCreate(Task1, "Task1", 128, NULL, 2, NULL);
    xTaskCreate(Task2, "Task2", 128, NULL, 1, NULL);

    // Start Scheduler
    vTaskStartScheduler();

    while (1)
    {
        // Should never reach here
    }
}

/* Task 1: Sets the event bit */
void Task1(void *pvParameters)
{
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(2000)); // Simulate some processing
        printf("Task 1: Setting Event Bit\n");

        // Set event bit
        xEventGroupSetBits(eventGroup, EVENT_BIT);
    }
}

/* Task 2: Waits for the event bit */
void Task2(void *pvParameters)
{
    while (1)
    {
        printf("Task 2: Waiting for Event Bit\n");

        // Wait until event bit is set
        xEventGroupWaitBits(eventGroup, EVENT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

        printf("Task 2: Event Received! Proceeding...\n");

        vTaskDelay(pdMS_TO_TICKS(1000)); // Simulate processing
    }
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
1. **Create an Event Group**  
   ```c
   eventGroup = xEventGroupCreate();
   ```
   - Creates an **Event Group** to store synchronization flags.  

2. **Task 1: Setting the Event Bit**  
   ```c
   xEventGroupSetBits(eventGroup, EVENT_BIT);
   ```
   - **Sets Bit 0** to indicate an event has occurred.  

3. **Task 2: Waiting for the Event Bit**  
   ```c
   xEventGroupWaitBits(eventGroup, EVENT_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
   ```
   - **Blocks until Bit 0 is set** by Task 1.  
   - The **pdTRUE** parameter clears the bit after it is received.  
   - The **portMAX_DELAY** means it waits indefinitely.  

4. **Tasks Print Messages**  
   - **Task 1:** Prints `"Task 1: Setting Event Bit"` every 2 sec.  
   - **Task 2:** Waits, then prints `"Task 2: Event Received! Proceeding..."`.  

---

### **Expected Behavior:**  
- **Task 2 starts waiting** for the event bit.  
- **Task 1 sets the bit after 2 seconds**.  
- **Task 2 detects the event and proceeds**.  
- The cycle **repeats indefinitely**.  

---

### **Summary of FreeRTOS Event Group Functions**  
| **Function** | **Purpose** |
|-------------|-------------|
| **xEventGroupCreate()** | Creates an Event Group |
| **xEventGroupSetBits()** | Sets an event bit |
| **xEventGroupWaitBits()** | Waits for an event bit |
| **xEventGroupClearBits()** | Clears a specific bit |

---

## **Next Step: FreeRTOS Software Timers**  


### **Step 28: FreeRTOS Software Timers**  

#### **Theory:**  
- FreeRTOS **Software Timers** allow executing functions **at fixed intervals** without using hardware timers.  
- They **run in the background** without blocking tasks.  
- Useful for **periodic actions** like LED blinking, watchdog resets, or sensor polling.  

---

### **Practical: Blinking an LED using a FreeRTOS Software Timer**  
#### **Objective:**  
- Create a **software timer** that toggles an LED **every 1 second**.  

---

### **Steps to Implement:**  
1. **Create a Timer Handle** before starting the scheduler.  
2. **Define a Timer Callback Function** that toggles an LED.  
3. **Create and Start the Timer** in the `main()` function.  
4. **Let FreeRTOS manage the timer** while running other tasks.  

---

### **Code for FreeRTOS Software Timer Example**
```c
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#define LED_PIN GPIO_PIN_13
#define LED_PORT GPIOC

TimerHandle_t myTimerHandle; // Handle for the software timer

void TimerCallback(TimerHandle_t xTimer); // Timer callback function

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    // Initialize LED GPIO (Assuming LED is on PC13)
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    // Create a software timer
    myTimerHandle = xTimerCreate("LED_Timer", pdMS_TO_TICKS(1000), pdTRUE, (void *)0, TimerCallback);

    if (myTimerHandle != NULL)
    {
        xTimerStart(myTimerHandle, 0); // Start timer
    }

    // Start FreeRTOS Scheduler
    vTaskStartScheduler();

    while (1)
    {
        // Should never reach here
    }
}

/* Timer Callback Function */
void TimerCallback(TimerHandle_t xTimer)
{
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN); // Toggle LED
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
1. **Create a Timer Handle**  
   ```c
   TimerHandle_t myTimerHandle;
   ```
   - Stores the timer reference.

2. **Define a Timer Callback Function**  
   ```c
   void TimerCallback(TimerHandle_t xTimer)
   {
       HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
   }
   ```
   - Toggles the LED **every time the timer expires**.

3. **Create a Software Timer**  
   ```c
   myTimerHandle = xTimerCreate("LED_Timer", pdMS_TO_TICKS(1000), pdTRUE, (void *)0, TimerCallback);
   ```
   - `"LED_Timer"` → Timer name.  
   - `pdMS_TO_TICKS(1000)` → Converts 1000ms to RTOS ticks.  
   - `pdTRUE` → Makes the timer **auto-restart**.  
   - `TimerCallback` → Function executed **every second**.  

4. **Start the Timer**  
   ```c
   xTimerStart(myTimerHandle, 0);
   ```
   - Starts the timer **immediately** after creation.  

---

### **Expected Behavior:**  
- The **LED blinks every second** without using a hardware timer.  
- The FreeRTOS **scheduler handles the timing** efficiently.  

---

### **Summary of FreeRTOS Timer Functions**  
| **Function** | **Purpose** |
|-------------|-------------|
| **xTimerCreate()** | Creates a software timer |
| **xTimerStart()** | Starts the timer |
| **xTimerStop()** | Stops the timer |
| **xTimerReset()** | Resets the timer |
| **xTimerChangePeriod()** | Changes the timer interval |

---

## **Next Step: FreeRTOS Message Queues**  


### **Step 29: FreeRTOS Message Queues**  

#### **Theory:**  
- A **Queue** is a **FIFO (First-In-First-Out) buffer** used for **task communication** in FreeRTOS.  
- Tasks **send and receive data** using queues without blocking execution.  
- Used for **sensor data sharing, event handling, and inter-task communication**.  

---

### **Practical: Sending and Receiving Data Using a Queue**  
#### **Objective:**  
- Create a **producer task** that sends a number (0-9) to a queue every second.  
- Create a **consumer task** that receives the number and prints it via UART.  

---

### **Steps to Implement:**  
1. **Create a Queue Handle** before starting the scheduler.  
2. **Define Producer and Consumer Tasks.**  
3. **Producer Task:** Sends data (0-9) to the queue every second.  
4. **Consumer Task:** Reads data from the queue and sends it via UART.  
5. **Start the FreeRTOS Scheduler.**  

---

### **Code for FreeRTOS Message Queue Example**
```c
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "usart.h"

QueueHandle_t myQueue;  // Queue handle

void ProducerTask(void *pvParameters);
void ConsumerTask(void *pvParameters);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    
    // Initialize UART
    MX_USART2_UART_Init(); 

    // Create a queue of size 5 (each element is an int)
    myQueue = xQueueCreate(5, sizeof(int));

    if (myQueue != NULL)
    {
        // Create Producer and Consumer tasks
        xTaskCreate(ProducerTask, "Producer", 128, NULL, 1, NULL);
        xTaskCreate(ConsumerTask, "Consumer", 128, NULL, 1, NULL);

        // Start FreeRTOS Scheduler
        vTaskStartScheduler();
    }

    while (1)
    {
        // Should never reach here
    }
}

/* Producer Task */
void ProducerTask(void *pvParameters)
{
    int count = 0;
    while (1)
    {
        xQueueSend(myQueue, &count, portMAX_DELAY); // Send number to queue
        count = (count + 1) % 10; // Increment (0-9)
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay 1 sec
    }
}

/* Consumer Task */
void ConsumerTask(void *pvParameters)
{
    int receivedValue;
    char msg[20];

    while (1)
    {
        if (xQueueReceive(myQueue, &receivedValue, portMAX_DELAY) == pdTRUE)
        {
            sprintf(msg, "Received: %d\r\n", receivedValue);
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
        }
    }
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
1. **Create a Queue Handle**  
   ```c
   QueueHandle_t myQueue;
   ```
   - Stores reference to the queue.

2. **Create a Queue with Size 5**  
   ```c
   myQueue = xQueueCreate(5, sizeof(int));
   ```
   - Holds **5 integers** at a time.

3. **Producer Task**  
   ```c
   xQueueSend(myQueue, &count, portMAX_DELAY);
   ```
   - Sends `count` (0-9) to the queue every second.

4. **Consumer Task**  
   ```c
   xQueueReceive(myQueue, &receivedValue, portMAX_DELAY);
   ```
   - Reads value from the queue.  
   - Prints it over **UART**.

---

### **Expected Behavior:**  
- Every second, **Producer Task** sends numbers `0-9` to the queue.  
- **Consumer Task** reads the numbers and prints:  
  ```
  Received: 0
  Received: 1
  Received: 2
  ...
  ```

---

### **Summary of FreeRTOS Queue Functions**  
| **Function** | **Purpose** |
|-------------|-------------|
| **xQueueCreate()** | Creates a queue |
| **xQueueSend()** | Sends data to the queue |
| **xQueueReceive()** | Receives data from the queue |
| **uxQueueMessagesWaiting()** | Gets the number of items in the queue |

---

## **Next Step: FreeRTOS Event Groups**  


### **Step 31: FreeRTOS Software Timers**  

#### **Theory:**  
- **Software Timers** allow tasks to execute **periodically** or after a **one-time delay**.  
- FreeRTOS timers run **without blocking tasks**.  
- Timer callbacks execute in the **timer service task**.  

---

### **Practical: Blinking LED using a FreeRTOS Timer**  
#### **Objective:**  
- Create a **software timer** that toggles an LED every **1 second**.  
- Use a **timer callback function** to handle LED toggling.  

---

### **Steps to Implement:**  
1. **Create a Timer Handle.**  
2. **Define Timer Callback Function.**  
3. **Create and Start the Timer in `main()`.**  
4. **The Callback function toggles an LED every 1 second.**  

---

### **Code for FreeRTOS Software Timer Example**
```c
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#define LED_PIN GPIO_PIN_13
#define LED_PORT GPIOC

TimerHandle_t myTimerHandle;  // Timer handle

void TimerCallback(TimerHandle_t xTimer);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    
    // Initialize GPIO for LED
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    // Create Software Timer (1 sec period, auto-reload)
    myTimerHandle = xTimerCreate("LED_Timer", pdMS_TO_TICKS(1000), pdTRUE, 0, TimerCallback);

    if (myTimerHandle != NULL)
    {
        xTimerStart(myTimerHandle, 0);  // Start Timer
        vTaskStartScheduler();  // Start FreeRTOS Scheduler
    }

    while (1)
    {
        // Should never reach here
    }
}

/* Timer Callback Function */
void TimerCallback(TimerHandle_t xTimer)
{
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);  // Toggle LED
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    // Auto-generated by STM32CubeMX
}
```

---

### **Explanation of Code:**  
1. **Create a Timer Handle**  
   ```c
   TimerHandle_t myTimerHandle;
   ```
   - Stores **timer instance**.

2. **Create a Software Timer**  
   ```c
   myTimerHandle = xTimerCreate("LED_Timer", pdMS_TO_TICKS(1000), pdTRUE, 0, TimerCallback);
   ```
   - Timer named **"LED_Timer"**.  
   - Period = **1000 ms (1 sec)**.  
   - **pdTRUE** → Auto-reloads after expiration.  
   - Callback function → **TimerCallback()**.  

3. **Start Timer**  
   ```c
   xTimerStart(myTimerHandle, 0);
   ```
   - Starts timer **immediately**.  

4. **Timer Callback Function**  
   ```c
   void TimerCallback(TimerHandle_t xTimer)
   {
       HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
   }
   ```
   - Toggles **LED** every second.  

---

### **Expected Behavior:**  
- **Every 1 second**, the LED **toggles ON/OFF** automatically.  

---

### **Summary of FreeRTOS Timer Functions**  
| **Function** | **Purpose** |
|-------------|-------------|
| **xTimerCreate()** | Creates a software timer |
| **xTimerStart()** | Starts a timer |
| **xTimerStop()** | Stops a timer |
| **xTimerChangePeriod()** | Changes timer period |
| **xTimerDelete()** | Deletes a timer |

---

## **Next Step: FreeRTOS Message Queues**  
