# Indirect Sensor-Based Field-Oriented Control (FOC) for a Three-Phase AC Induction Motor Using Space Vector Pulse Width Modulation (SVPWM)



**Step 1: Understanding the SOGI Block Diagram**

The Second-Order Generalized Integrator (SOGI) is used to generate two orthogonal components from an input signal \( v(t) \). These components are:

1. \( v' (t) \) - The in-phase component.
2. \( qv' (t) \) - The quadrature-phase component (90\(^\circ\) phase shift).

The SOGI structure consists of:
- A proportional gain \( k \).
- Two integrators represented as \( \frac{1}{s} \).
- A feedback loop incorporating the angular frequency \( \omega \).

---

**Step 2: Writing the Fundamental SOGI Equation**

Mathematically, the SOGI transfer function is expressed as:

\[
G_L(s) = \frac{\omega s}{s^2 + \omega^2}
\]

This represents a second-order transfer function with a resonant frequency \( \omega \), which acts as a band-pass filter.

---

**Step 3: Expressing the Transfer Function Relations**

From the block diagram, the system's governing equations are:

1. The differential equation for \( v' (s) \):
   \[
   sV'(s) = k\omega [V(s) - V'(s)] - \omega QV'(s)
   \]
2. The quadrature-phase component is given by:
   \[
   sQV'(s) = \omega V'(s)
   \]
   This implies:
   \[
   QV'(s) = \frac{\omega}{s} V'(s)
   \]

---

**Step 4: Deriving \( H_d(s) \)**

To derive \( H_d(s) \), substitute \( QV'(s) = \frac{\omega}{s} V'(s) \) into the first equation:

\[
   sV'(s) + k\omega V'(s) + \omega^2 \frac{1}{s} V'(s) = k\omega V(s)
\]

Factor \( V'(s) \):

\[
   V'(s) \left( s + k\omega + \frac{\omega^2}{s} \right) = k\omega V(s)
\]

Multiply by \( s \) to clear the fraction:

\[
   V'(s) (s^2 + k\omega s + \omega^2) = k\omega s V(s)
\]

Solving for \( H_d(s) = \frac{V'(s)}{V(s)} \):

\[
   H_d(s) = \frac{k\omega s}{s^2 + k\omega s + \omega^2}
\]

This equation represents the band-pass filter behavior of the SOGI, extracting the in-phase component.

---

**Step 5: Deriving \( H_q(s) \)**

Since:

\[
H_q(s) = \frac{QV'(s)}{V(s)}
\]

Using \( QV'(s) = \frac{\omega}{s} V'(s) \), we substitute \( V'(s) \) from \( H_d(s) \):

\[
H_q(s) = \frac{\omega}{s} H_d(s)
\]

Substituting \( H_d(s) \):

\[
H_q(s) = \frac{\omega}{s} \cdot \frac{k\omega s}{s^2 + k\omega s + \omega^2}
\]

Simplifying:

\[
H_q(s) = \frac{k\omega^2}{s^2 + k\omega s + \omega^2}
\]

This equation represents the quadrature-phase component with a 90\(^\circ\) phase shift.

---

**Conclusion**

The derived transfer functions are:

1. **In-phase transfer function** (band-pass filter):
   \[
   H_d(s) = \frac{k\omega s}{s^2 + k\omega s + \omega^2}
   \]
2. **Quadrature-phase transfer function**:
   \[
   H_q(s) = \frac{k\omega^2}{s^2 + k\omega s + \omega^2}
   \]

These equations confirm that:
- \( H_d(s) \) extracts the in-phase component.
- \( H_q(s) \) generates the quadrature-phase component with a 90\(^\circ\) shift.

These properties make the SOGI highly useful in phase-locked loops (PLLs) and grid synchronization applications.



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
    i_beta = (i_a + 2.0f * i_b) / sqrtf(3.0f);
}
```

#### Park Transform (αβ → dq)
```c
void park_transform(float theta) {
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);
    
    i_d = i_alpha * cos_theta + i_beta * sin_theta;
    i_q = -i_alpha * sin_theta + i_beta * cos_theta;
}
```

#### Speed PI Controller
```c
void speed_control(float omega_actual) {
    static float integral = 0;
    float error = omega_ref - omega_actual;
    float Kp = 0.01f, Ki = 0.005f;
    
    integral += error * Ki;
    integral = fminf(fmaxf(integral, -1.0f), 1.0f);
    
    v_q = (Kp * error) + integral;
}
```

#### Inverse Park Transform (dq → αβ)
```c
void inverse_park_transform(float theta) {
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);
    
    v_alpha = v_d * cos_theta - v_q * sin_theta;
    v_beta = v_q * cos_theta + v_d * sin_theta;
}
```

#### SVPWM Generation
```c
#include "driverlib.h"

void generate_svpwm_pulses(float v_alpha, float v_beta) {
    float v_ref = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
    float angle = atan2f(v_beta, v_alpha);
    int sector_num = ((int)((angle / M_PI) * 3.0f)) % 6;
    
    float t1, t2, t0;
    float sqrt3 = sqrtf(3.0f);
    
    switch (sector_num) {
        case 0:
            t1 = sqrt3 * v_ref * sinf(M_PI / 3.0f - angle);
            t2 = sqrt3 * v_ref * sinf(angle);
            break;
        case 1:
            t1 = sqrt3 * v_ref * sinf(angle - M_PI / 3.0f);
            t2 = sqrt3 * v_ref * sinf(2 * M_PI / 3.0f - angle);
            break;
        case 2:
            t1 = sqrt3 * v_ref * sinf(2 * M_PI / 3.0f - angle);
            t2 = sqrt3 * v_ref * sinf(angle - M_PI);
            break;
        case 3:
            t1 = sqrt3 * v_ref * sinf(angle - M_PI);
            t2 = sqrt3 * v_ref * sinf(4 * M_PI / 3.0f - angle);
            break;
        case 4:
            t1 = sqrt3 * v_ref * sinf(4 * M_PI / 3.0f - angle);
            t2 = sqrt3 * v_ref * sinf(angle - 5 * M_PI / 3.0f);
            break;
        case 5:
            t1 = sqrt3 * v_ref * sinf(angle - 5 * M_PI / 3.0f);
            t2 = sqrt3 * v_ref * sinf(2 * M_PI - angle);
            break;
    }
    
    t0 = 1.0f - t1 - t2;
    
    PWM_setDutyCycle(PWM1_BASE, PWM_OUT_1, t1);
    PWM_setDutyCycle(PWM1_BASE, PWM_OUT_2, t2);
    PWM_setDutyCycle(PWM1_BASE, PWM_OUT_3, t0);
}
```

#### Main Loop
```c
int main() {
    encoder_init();
    
    while (1) {
        theta = encoder_read();
        park_transform(theta);
        speed_control(omega_actual);
        inverse_park_transform(theta);
        space_vector_pwm(v_alpha, v_beta);
    }
}
```

## 5. Conclusion
This project provides a comprehensive approach to Field-Oriented Control (FOC) of an Induction Motor using Space Vector PWM (SVPWM). Let me know if you need refinements, hardware details, or optimizations! 🚀

