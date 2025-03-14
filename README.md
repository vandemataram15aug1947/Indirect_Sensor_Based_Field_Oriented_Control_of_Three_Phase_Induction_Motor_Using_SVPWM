# Indirect_Sensor_Based_Field_Oriented_Control_of_Three_Phase_Induction_Motor_Using_SVPWM


# Sensored FOC Control of Three-Phase AC Induction Motor Using SVPWM

## Overview
This project implements **Sensored Field-Oriented Control (FOC)** for a **Three-Phase AC Induction Motor (ACI)** using **Space Vector Pulse Width Modulation (SVPWM)**. The control algorithm is implemented on an embedded system (e.g., **TMS320F28379D**) and aims to achieve precise speed and torque control using a rotor position sensor.

---

## Features
- **Sensored FOC**: Uses an encoder or Hall sensors for rotor position feedback.
- **Space Vector PWM (SVPWM)**: Optimized switching scheme for reduced harmonic distortion.
- **PI Controllers**: Speed and current control loops.
- **Clarke and Park Transforms**: Converts three-phase currents to a rotating reference frame.
- **Inverse Clarke Transform**: Converts D-Q frame voltages back to three-phase.
- **Real-time feedback and monitoring**.

---

## Block Diagram

```text
       Reference Speed
              |
      +----------------+
      | Speed PI Loop  |
      +----------------+
              |
      +----------------+
      | Current Control |
      +----------------+
              |
    +----------------------+
    | Inverse Park Transform |
    +----------------------+
              |
    +----------------------+
    | Inverse Clarke Transform |
    +----------------------+
              |
    +----------------------+
    |      SVPWM         |
    +----------------------+
              |
       3-Phase Inverter
              |
    +----------------+
    |  AC Induction  |
    |     Motor      |
    +----------------+
              |
        Encoder/Hall Sensor
              |
      +----------------+
      | Position Feedback |
      +----------------+
```

---

## Code Explanation (Step-by-Step)

### 1. **Header Files**
```c
#include <math.h>
#include <stdint.h>
#include "device.h"
#include "pwm.h"
#include "adc.h"
#include "control.h"
#include "encoder.h"
```
- `math.h`: Used for mathematical calculations.
- `stdint.h`: Standard integer types.
- `device.h`: Includes microcontroller-specific configurations.
- `pwm.h`: Manages PWM generation.
- `adc.h`: Handles ADC sampling for current and voltage.
- `control.h`: Implements FOC control logic.
- `encoder.h`: Handles position sensor readings.

---

### 2. **Global Variables**
```c
float i_alpha, i_beta;   // Clarke transform outputs
float i_d, i_q;          // Park transform outputs
float v_d, v_q;          // Voltage references
float v_alpha, v_beta;   // Inverse Clarke outputs
float theta;             // Rotor angle from sensor
float omega_ref = 100.0; // Reference speed (rad/s)
```
- `i_alpha, i_beta`: Two-phase equivalent currents.
- `i_d, i_q`: Direct and quadrature currents (after Park transform).
- `v_d, v_q`: Voltage references for d-q axis.
- `v_alpha, v_beta`: Alpha-beta voltages after inverse Clarke transform.
- `theta`: Rotor position obtained from the encoder.
- `omega_ref`: Desired motor speed.

---

### 3. **Encoder Reading**
```c
float encoder_read() {
    return get_encoder_position();
}
```
- Reads the **rotor position** from the encoder sensor.

---

### 4. **Clarke Transform (Converts 3-phase to 2-phase Alpha-Beta)**
```c
void clarke_transform(float i_a, float i_b, float i_c) {
    i_alpha = i_a;
    i_beta = (i_a + 2 * i_b) / sqrt(3);
}
```
- Converts three-phase currents `(i_a, i_b, i_c)` to two-phase `(i_alpha, i_beta)`.
- Uses standard Clarke Transform equations.

---

### 5. **Park Transform (Converts Alpha-Beta to D-Q Frame)**
```c
void park_transform(float theta) {
    i_d = i_alpha * cos(theta) + i_beta * sin(theta);
    i_q = -i_alpha * sin(theta) + i_beta * cos(theta);
}
```
- Converts stationary frame `(i_alpha, i_beta)` to rotating frame `(i_d, i_q)`.
- Uses rotor angle `theta` obtained from the sensor.

---

### 6. **Speed and Current Controllers**
```c
void speed_control(float omega_actual) {
    float error = omega_ref - omega_actual;
    v_q = pi_controller(error); // PI controller output
    v_d = 0; // No direct-axis voltage for indirect FOC
}
```

---

### 7. **Inverse Park Transform (Generates Alpha-Beta Voltages)**
```c
void inverse_park_transform(float theta) {
    v_alpha = v_d * cos(theta) - v_q * sin(theta);
    v_beta = v_q * cos(theta) + v_d * sin(theta);
}
```

---

### 8. **Inverse Clarke Transform (Converts Alpha-Beta to Three-Phase Voltages)**
```c
void inverse_clarke_transform() {
    float v_a = v_alpha;
    float v_b = (-v_alpha + sqrt(3) * v_beta) / 2;
    float v_c = (-v_alpha - sqrt(3) * v_beta) / 2;
}
```

---

### 9. **Space Vector PWM (SVPWM) Implementation**
```c
void svpwm(float v_alpha, float v_beta) {
    float v_ref = sqrt(v_alpha * v_alpha + v_beta * v_beta);
    float angle = atan2(v_beta, v_alpha);
    float sector = (angle / M_PI) * 3.0;
    generate_svpwm_pulses(v_ref, angle);
}
```

---

### 10. **Main Loop**
```c
void main() {
    device_init();
    pwm_init();
    adc_init();
    encoder_init();
    
    while (1) {
        float i_a, i_b, i_c, omega_actual;
        adc_read(&i_a, &i_b, &i_c, &omega_actual);
        theta = encoder_read();
        
        clarke_transform(i_a, i_b, i_c);
        park_transform(theta);
        speed_control(omega_actual);
        inverse_park_transform(theta);
        inverse_clarke_transform();
        svpwm(v_alpha, v_beta);
    }
}
```


