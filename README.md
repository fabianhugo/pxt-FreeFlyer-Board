# FreeFlyer Board for Calliope mini

MakeCode extension for the FreeFlyer board on the Calliope mini 3. It adds blocks for four sensors on the I2C bus:

| Sensor | Measures | I2C address |
|---|---|---|
| Bosch BMP585 | air pressure, temperature, altitude | 0x46 / 0x47 |
| Bosch BME690 | temperature, air pressure, humidity, gas resistance | 0x76 / 0x77 |
| Bosch BMI323 | acceleration, rotation rate, temperature | 0x68 / 0x69 |
| Sensirion SCD4x (SCD40, SCD41) | CO₂, temperature, humidity | 0x62 |

## Example

```blocks
bmp585.setReferenceHeight(34)
basic.forever(function () {
    scd4x.set_ambient_pressure(bmp585.pressure())
    serial.writeValue("CO2", scd4x.get_co2())
    serial.writeValue("hPa", bmp585.pressure())
    serial.writeValue("alt", bmp585.altitudeCalibrated())
    serial.writeValue("hum", bme690.humidity())
    serial.writeValue("az", bmi323.acceleration(BMI323Axis.Z))
    basic.pause(5000)
})
```

## Notes

- **No init block needed.** Every sensor initializes itself on first use. The `init` blocks return whether a sensor was found.
- **BMP585 altitude:** use `set current height to … m` at a known height. The weather shifts the result by about 8 m per hPa, so calibrate again when you need absolute values.
- **BME690 gas resistance:** a new sensor needs a long burn-in (roughly 24 h) before absolute values mean much. Until then, use it as a trend.
- **BMI323 axes:** x, y, z and their signs match the Calliope's built-in accelerometer. `align axes with Calliope` switches to the raw chip axes.
- **SCD4x:** new values come every 5 s. Automatic self-calibration is switched off, because it assumes regular exposure to 400 ppm outdoor air. For accurate CO₂, pass the current air pressure with `set air pressure to … hPa`, for example from the BMP585. For manual calibration, put the sensor in air with a known CO₂ level (outdoors is about 420 ppm) and use `calibrate CO2 to … ppm` (advanced blocks). It waits until the sensor has measured for 3 minutes, and the correction stays stored in the sensor.
- **Troubleshooting:** the Bosch drivers have a `debug` block that writes diagnostic lines to the serial port.

## License

MIT
