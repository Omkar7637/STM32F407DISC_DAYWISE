# STM32F407DISC_DAYWISE

Here's a **14-day** structured plan to teach the **STM32F407 DISC1** board using **HAL (Hardware Abstraction Layer)**, covering both **theory and practicals**.

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

## **Final Thoughts:**
This plan provides a **structured** approach covering **STM32F407 peripherals** with **HAL**, ensuring **both theoretical** and **practical** knowledge.  
Let me know if you need modifications or additional details! 🚀

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

This covers **Day 1 (Theory)**. Let me know if you want **further modifications or practical session details**! 🚀

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

This covers **Day 2**! 🚀 Let me know if you need modifications.

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

This completes **Day 3**! 🚀 Let me know if you need modifications.

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

🚀 This completes **Day 4**! Let me know if you need changes.

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

🚀 This completes **Day 5**! Let me know if you need modifications.

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

🚀 This completes **Day 6**! Let me know if you need modifications.

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

🚀 This completes **Day 8**! Let me know if you need modifications.


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

🚀 This completes **Day 9**! Let me know if you need modifications.


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

🚀 This completes **Day 10**! Let me know if you need modifications.


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

🚀 This completes **Day 11**! Let me know if you need modifications.


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

🚀 **Day 12 Complete!** Let me know if you need modifications.


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

🚀 **Day 13 Complete!** Let me know if you need modifications.


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

🚀 **Day 14 Complete!** Let me know if you need any modifications.

