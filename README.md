# ESP32_EnhancedPWM

ESP32_EnhancedPWM is an Arduino library for the ESP32 that provides advanced PWM control using the built-in LEDC peripheral. This library extends standard PWM functionality by supporting extended frequency ranges—including frequencies below 1 Hz—and enabling fractional frequency adjustments through an enhanced manual divider calculation.

## Enhanced Features in Detail

1. **Extended Frequency Support**:
   - **Low-Frequency PWM:** Generate PWM signals with frequencies below 1 Hz, ideal for applications such as low-speed motor control, slow LED dimming, or other low-power scenarios.
   - **Versatile Timing:** Accommodates use cases that require very long PWM periods, which are not achievable with standard PWM libraries.

2. **Fractional Divider Calculation**:
   - **Precise Frequency Tuning:** Utilizes a fixed-point divider where the divider is represented as:
     
     divider = A + (B / 256)
     
     This method splits the divider into an integer component (A) and a fractional component (B), allowing for more granular control over the PWM frequency.
   - **Enhanced Accuracy:** Achieve frequencies that aren’t exact integer divisors of the source clock, providing more precise control over the output frequency.

3. **Flexible Resolution**:
   - **Customizable Duty Cycle:** Supports PWM resolutions from 1 bit up to the maximum supported by the ESP32 LEDC hardware. This flexibility allows for finer control over the duty cycle, which is useful for applications requiring precise brightness or speed control.
   - **Optimized Performance:** Enables developers to choose the optimal balance between resolution and frequency for their specific application.

4. **Multiple Clock Sources**:
   - **LEDC_USE_APB_CLK:** Uses the APB clock (typically 80 MHz), which is ideal for most PWM applications due to its high frequency and robust performance.
   - **LEDC_USE_XTAL_CLK:** Uses the external crystal oscillator (typically 40 MHz) if available, providing an alternative clock source.
   - **LEDC_USE_REF_TICK:** Uses the reference tick clock (1 MHz), making it ideal for generating very low frequencies.
   - **LEDC_USE_RTC8M_CLK:** Uses the RTC clock (typically 8Mhz), suitable for specific low-power or precise timing applications.

## Installation

1. **Download the Library:**
   - Clone or download the repository to your local machine.
   - Place the folder `ESP32_EnhancedPWM` in your Arduino libraries directory (typically located at `~/Documents/Arduino/libraries`).

2. **Include the Library in Your Sketch:**

   ```cpp
   #include <ESP32_EnhancedPWM.h>
   ```

## Usage Example

Below is a simple example to get you started. This example initializes a PWM signal on inbuilt LED, channel 0, with a frequency of 0.5 Hz, an 8-bit resolution, inverted and uses the reference tick clock for extended low-frequency support.

```cpp
#include <Arduino.h>
#include <ESP32_EnhancedPWM.h>

ESP32_EnhancedPWM pwm;

void setup()
{
  delay(5000);

  Serial.begin(115200);

  // Initialize PWM on builtin LED, channel 0, frequency 0.5 Hz, 8-bit resolution,
  // non-inverted output, using the LEDC_USE_REF_TICK clock.
  if (!pwm.begin(LED_BUILTIN, 0, 0.5f, 8, true, LEDC_USE_REF_TICK))
  {
    Serial.println("PWM initialization failed!");
    while (1);
  }
  else
  {
    Serial.println("PWM initialized successfully!");
  }

  // Set duty cycle to 50%
  pwm.setDutyNormalized(0.5);
}

void loop()
{
  // PWM is running in hardware; the main loop can perform other tasks.
}
```

## API Reference

### `bool begin(uint8_t pin, uint8_t channel, float frequency, uint8_t resolution = 8, bool isInverted = false, ledc_clk_cfg_t clk = LEDC_USE_APB_CLK)`
Initializes and configures the LEDC PWM channel and timer.

- **Parameters:**
  - `pin`: The GPIO pin number for PWM output.
  - `channel`: The LEDC channel to use (0 to SOC_LEDC_CHANNEL_NUM - 1).
  - `frequency`: The desired PWM frequency in Hz.
  - `resolution`: The PWM resolution in bits (default is 8).
  - `isInverted`: If true, inverts the PWM output.
  - `clk`: The LEDC clock source. Options include:
    - `LEDC_USE_APB_CLK` (default)
    - `LEDC_USE_XTAL_CLK`
    - `LEDC_USE_REF_TICK`
    - `LEDC_USE_RTC8M_CLK`
- **Returns:** `true` if initialization is successful; `false` otherwise.

### `void setDuty(uint32_t dutyValue)`
Sets the PWM duty cycle using an integer value constrained to the maximum allowed by the configured resolution.

### `void setDutyAndResetTimer(uint32_t dutyValue)`
Sets the PWM duty cycle using an integer value constrained to the maximum allowed by the configured resolution and resets the timer.

### `void setDutyNormalized(float dutyNormalized)`
Sets the PWM duty cycle using a normalized value between 0.0 and 1.0.

### `uint8_t getResolution() const`
Returns the PWM duty resolution in bits.

### `uint32_t getFrequency() const`
Returns the current PWM frequency as configured in hardware.

### `float getExactFrequency() const`
Returns the computed PWM frequency as determined by the divider calculation, including any fractional adjustments.

### `uint32_t getDuty() const`
Returns the current PWM duty cycle value.

### `uint32_t getMaxDuty() const`
Returns the maximum possible duty cycle value based on the configured resolution.

### `uint8_t getBoardPin() const`
Returns the board-specific GPIO pin used for the PWM output.

_For additional internal functions such as the divider calculations and clock source resolution, please refer to the source code documentation._

## Compatibility

This library has been **tested only on the Arduino Nano ESP32-S3** so far.  
Other ESP32 variants **may work**, but they have **not been verified yet**.  

If you successfully use this library on a different ESP32 board, please consider submitting an issue or pull request to help expand compatibility.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request on GitHub.

## Acknowledgments

- Inspired by Khoi's [ESP32_FastPWM](https://github.com/khoih-prog/ESP32_FastPWM) library.
- Special thanks to the ESP32 community for their ongoing support and contributions.
