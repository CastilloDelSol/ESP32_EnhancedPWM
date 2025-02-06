#include <Arduino.h>
#include <ESP32_Enhanced_PWM.h>

ESP32_Enhanced_PWM pwm;

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