# Indirect Sensor-Based Field-Oriented Control (FOC) for a Three-Phase AC Induction Motor Using Space Vector Pulse Width Modulation (SVPWM)

## 1. Overview
This project implements Indirect Sensored Field-Oriented Control (FOC) for a Three-Phase AC Induction Motor (ACI). It utilizes a rotor position sensor, Clarke and Park Transforms, Proportional-Integral (PI) Controllers, and Space Vector Pulse Width Modulation (SVPWM) to achieve precise speed and torque control. The goal is to ensure optimal motor performance with reduced harmonic distortion, improved dynamic response, and better efficiency.

## 2. Features
### 2.1 Sensored Field-Oriented Control (FOC)
- Uses Position Sensor (Encoder or Hall sensors) to determine rotor position.
- Decouples Torque (i_q) and Magnetizing (i_d) components of current for independent control.
- Enhances dynamic response, efficiency, and torque per ampere ratio.

### 2.2 Space Vector Pulse Width Modulation (SVPWM)
- Advanced PWM technique for better DC bus utilization.
- Reduces harmonic distortion and enhances efficiency.
- Utilizes six switching vectors for optimum waveform synthesis.

### 2.3 Control Structure
- **Speed PI Controller**: Adjusts reference current (i_q) to regulate motor speed.
- **Current Control Loop**: Adjusts voltage to regulate motor currents.
- **Clarke & Park Transforms** for three-phase current conversion.
- **Inverse Clarke Transform** to generate three-phase voltages.
- **SVPWM (Space Vector PWM)** for efficient inverter switching.

## 3. System Block Diagram
```
                  +----------------+
                  | Speed PI Controller |
                  +----------------+
                        |
                        V
      +---------------------------------+
      | Current Control (i_d, i_q) |
      +---------------------------------+
                        |
                        V
      +-------------------------------+
      |    Inverse Park Transform    |
      +-------------------------------+
                        |
                        V
      +----------------------------------+
      |   Inverse Clarke Transform   |
      +----------------------------------+
                        |
                        V
       +-------------------------+
       |        SVPWM            |
       +-------------------------+
                        |
                        V
       +--------------------------+
       | 3-Phase Inverter          |
       +------------------------------+
                        |
                        V
       +----------------------------+
       | AC Induction Motor         |
       +----------------------------+
                        |
                        V
       +-------------------------+
       | Rotor Encoder           |
       +-------------------------+
                        |
                        V
       +-------------------------------+
       | Position Feedback (Theta)     |
       +-------------------------------+
```

---

## 4. Code Explanation

### 4.1 Include Required Header Files
```c
#include <math.h>
#include <stdint.h>
#include "device.h"
#include "pwm.h"
#include "adc.h"
#include "control.h"
#include "encoder.h"
```
- `math.h`: Provides mathematical functions such as `sin()`, `cos()`, and `atan2()` for trigonometric calculations.
- `stdint.h`: Standard integer types for improved portability.
- `pwm.h`: PWM driver functions.
- `adc.h`: ADC driver functions for acquiring motor currents and rotor speed.
- `control.h`: Contains PI controllers and field-oriented control logic.
- `encoder.h`: Handles encoder data acquisition for rotor position feedback.

### 4.2 Theory of Field-Oriented Control (FOC)
FOC is a control method that allows decoupled torque and flux control in AC motors. It improves dynamic response and efficiency by controlling motor current in a rotating reference frame instead of a stationary frame.

#### Clarke Transform (ABC → αβ)
- Converts the three-phase stator currents (i_a, i_b, i_c) into two-phase stationary coordinates (i_alpha, i_beta).

#### Park Transform (αβ → dq)
- Converts stationary reference frame currents (i_alpha, i_beta) into a rotating d-q frame using the rotor position (θ) obtained from the encoder.

#### Speed PI Controller
- Compares actual motor speed (from encoder) with the reference speed.
- Generates the required quadrature current `i_q` to regulate speed.

#### Current Control PI Loop
- Uses two PI controllers to regulate direct and quadrature currents (`i_d` and `i_q`) by generating `v_d` and `v_q` voltage references.

#### Inverse Park Transform (d-q → αβ)
- Converts back from rotating d-q reference frame to stationary αβ frame.

#### Inverse Clarke Transform (αβ → ABC)
- Converts two-phase voltages (`v_alpha`, `v_beta`) back to three-phase signals (`v_a`, `v_b`, `v_c`).

#### Space Vector Pulse Width Modulation (SVPWM)
- Synthesizes a sinusoidal three-phase voltage waveform for efficient motor operation.
- Optimizes PWM switching to reduce harmonics and improve performance.

#### Three-Phase Inverter
- Uses six-switch IGBT/MOSFET bridge to drive the motor phases.

#### Rotor Position Feedback (Encoder)
- A rotary encoder measures the rotor position and speed.
- This information is fed into the Park transform to rotate the d-q frame.

### 4.3 Code Implementation
#### Global Variables
```c
float i_alpha, i_beta;  // Alpha-beta current components
float i_d, i_q;         // Direct-quadrature currents
float v_d, v_q;         // Direct-quadrature voltage components
float v_alpha, v_beta;  // Inverse Clarke transform results
float theta;            // Rotor position from encoder
float omega_actual;     // Actual rotor speed
float omega_ref = 1500; // Reference speed (RPM)
```

#### Encoder Read (Position Feedback)
```c
float encoder_read() {
    return get_encoder_position();  // Read encoder feedback
}
```

#### Clarke Transform (ABC to αβ)
```c
void clarke_transform(float i_a, float i_b, float i_c) {
    i_alpha = i_a;
    i_beta = (i_a + 2 * i_b) / sqrt(3);
}
```

#### Park Transform (αβ → dq)
```c
void park_transform(float theta) {
    float sin_theta = sin(theta);
    float cos_theta = cos(theta);
    
    i_d = i_alpha * cos(theta) + i_beta * sin(theta);
    i_q = -i_alpha * sin(theta) + i_beta * cos(theta);
}
```

#### Speed PI Controller
```c
void speed_control(float omega_actual) {
    static float integral = 0;
    float error = omega_ref - omega_actual;
    float Kp = 0.01, Ki = 0.005;
    
    integral += error * Ki;
    if (integral > 1.0) integral = 1.0;
    if (integral < -1.0) integral = -1.0;
    
    v_q = (Kp * error) + integral;
}
```

#### Inverse Park Transform (dq → αβ)
```c
void inverse_park_transform(float theta) {
    v_alpha = v_d * cos(theta) - v_q * sin(theta);
    v_beta = v_q * cos(theta) + v_d * sin(theta);
}
```

#### Space Vector PWM (SVPWM)
```c
void space_vector_pwm(float v_alpha, float v_beta) {
    float v_ref = sqrt(v_alpha * v_alpha + v_beta * v_beta);
    float angle = atan2(v_beta, v_alpha);
    float sector = (angle / M_PI) * 3.0;
    compute_pwm_duty_cycle(v_alpha, v_beta);
}
```

## 5. Conclusion
This project provides a comprehensive approach to Field-Oriented Control (FOC) of an Induction Motor using Space Vector PWM (SVPWM). Let me know if you need refinements, hardware details, or optimizations! 🚀

