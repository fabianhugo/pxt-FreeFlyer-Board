/**
 * Minimal proof-of-concept driver for the Bosch BME690 (temperature, pressure,
 * humidity, gas) on the Calliope mini v3 via I2C.
 *
 * Port of the floating-point path of Bosch's BME690_SensorAPI (bme69x.c).
 * The BME690 has the same chip ID (0x61) as BME680/688 but different calibration
 * parameters and formulas, so this driver gives wrong values on a BME680/688.
 *
 * Uses forced mode: every measurement heats the gas plate once (300 °C, 100 ms),
 * then reads all values in one 17-byte burst.
 */
//% color=#00a3b4 icon="" block="BME690"
namespace bme690 {
    const REG_COEFF3 = 0x00      // 5 bytes
    const REG_FIELD0 = 0x1D      // 17 bytes of measurement data
    const REG_RES_HEAT0 = 0x5A
    const REG_GAS_WAIT0 = 0x64
    const REG_CTRL_GAS_0 = 0x70
    const REG_CTRL_GAS_1 = 0x71
    const REG_CTRL_HUM = 0x72
    const REG_CTRL_MEAS = 0x74
    const REG_CONFIG = 0x75
    const REG_COEFF1 = 0x8A      // 23 bytes
    const REG_CHIP_ID = 0xD0
    const REG_SOFT_RESET = 0xE0
    const REG_COEFF2 = 0xE1      // 14 bytes
    const REG_VARIANT_ID = 0xF0

    const CHIP_ID = 0x61
    const CMD_SOFT_RESET = 0xB6

    // oversampling codes: 1 = x1, 2 = x2, 3 = x4, 4 = x8, 5 = x16
    const OS_T = 2  // x2
    const OS_P = 3  // x4
    const OS_H = 1  // x1
    let heaterTemp = 300  // °C
    let heaterMs = 100

    let addr = 0
    let debug = false
    let repeatedStart = false
    const RETRIES = 3
    let gapMs = 5
    const SLOW_MS = 50
    const MAX_AGE_MS = 500  // reuse a measurement younger than this

    // calibration parameters (names as in bme69x.c)
    let parT1 = 0
    let parT2 = 0
    let parT3 = 0
    let parP1 = 0
    let parP2 = 0
    let parP3 = 0
    let parP4 = 0
    let parP5 = 0
    let parP6 = 0
    let parP7 = 0
    let parP8 = 0
    let parP9 = 0
    let parP10 = 0
    let parP11 = 0
    let parH1 = 0
    let parH2 = 0
    let parH3 = 0
    let parH4 = 0
    let parH5 = 0
    let parH6 = 0
    let parG1 = 0
    let parG2 = 0
    let parG3 = 0
    let resHeatRange = 0
    let resHeatVal = 0

    let lastTemp = 0
    let lastPress = 0
    let lastHum = 0
    let lastGas = 0
    let gasValid = false
    let lastMeasure = 0

    function hex(n: number): string {
        const d = "0123456789ABCDEF"
        return d.charAt((n >> 4) & 0xF) + d.charAt(n & 0xF)
    }

    function log(msg: string): void {
        if (debug) serial.writeLine(input.runningTime() + " [bme690] " + msg)
    }

    function logSlow(what: string, t0: number): void {
        const dt = input.runningTime() - t0
        if (dt > SLOW_MS) log("SLOW " + what + " took " + dt + " ms")
    }

    // I2C write with up to RETRIES attempts; returns last error code (0 = ok)
    function i2cWrite(buf: Buffer, repeat: boolean, what: string): number {
        let err = 0
        for (let i = 0; i < RETRIES; i++) {
            basic.pause(gapMs) // idle bus time before every transaction
            const t0 = input.runningTime()
            err = pins.i2cWriteBuffer(addr, buf, repeat)
            logSlow(what, t0)
            if (!err) return 0
            log(what + " err " + err + " (try " + (i + 1) + ")")
        }
        return err
    }

    function writeReg(reg: number, value: number): void {
        const buf = pins.createBuffer(2)
        buf[0] = reg
        buf[1] = value
        i2cWrite(buf, false, "wr 0x" + hex(reg) + "=0x" + hex(value))
    }

    function readBuf(reg: number, len: number): Buffer {
        const r = pins.createBuffer(1)
        r[0] = reg
        i2cWrite(r, repeatedStart, "rd 0x" + hex(reg) + " addr")
        basic.pause(gapMs) // gap between address write and data read
        const t0 = input.runningTime()
        const b = pins.i2cReadBuffer(addr, len)
        logSlow("rd 0x" + hex(reg) + " data", t0)
        return b
    }

    function readReg(reg: number): number {
        return readBuf(reg, 1)[0]
    }

    function s8(v: number): number {
        return v > 127 ? v - 256 : v
    }

    function s16(v: number): number {
        return v > 32767 ? v - 65536 : v
    }

    function probe(a: number): boolean {
        addr = a
        const id = readReg(REG_CHIP_ID)
        log("probe 0x" + hex(a) + " chip id 0x" + hex(id))
        return id == CHIP_ID
    }

    function readCalibration(): void {
        // same 42-byte layout as coeff_array in bme69x.c get_calib_data()
        const c: number[] = []
        const parts = [readBuf(REG_COEFF1, 23), readBuf(REG_COEFF2, 14), readBuf(REG_COEFF3, 5)]
        for (const part of parts) {
            for (let i = 0; i < part.length; i++) c.push(part[i])
        }

        parT1 = (c[32] << 8) | c[31]
        parT2 = (c[1] << 8) | c[0]
        parT3 = s8(c[2])

        parP5 = s16((c[5] << 8) | c[4])
        parP6 = s16((c[7] << 8) | c[6])
        parP7 = s8(c[8])
        parP8 = s8(c[9])
        parP1 = s16((c[11] << 8) | c[10])
        parP2 = (c[13] << 8) | c[12]
        parP3 = s8(c[14])
        parP4 = s8(c[15])
        parP9 = s16((c[19] << 8) | c[18])
        parP10 = s8(c[20])
        parP11 = s8(c[21])

        parH5 = (c[23] << 4) | (c[24] >> 4)
        if (parH5 > 2047) parH5 -= 4096
        parH1 = (c[25] << 4) | (c[24] & 0x0F)
        if (parH1 > 2047) parH1 -= 4096
        parH2 = s8(c[26])
        parH4 = s8(c[27])
        parH3 = c[28]
        parH6 = c[29]

        parG1 = s8(c[35])
        parG2 = s16((c[34] << 8) | c[33])
        parG3 = s8(c[36])

        resHeatRange = (c[39] & 0x30) >> 4
        resHeatVal = s8(c[37])

        log("calib T1=" + parT1 + " P1=" + parP1 + " H1=" + parH1 + " H5=" + parH5 + " G1=" + parG1)
    }

    function calcResHeat(temp: number, ambTemp: number): number {
        if (temp > 400) temp = 400
        const var1 = parG1 / 16 + 49
        const var2 = (parG2 / 32768) * 0.0005 + 0.00235
        const var3 = parG3 / 1024
        const var4 = var1 * (1 + var2 * temp)
        const var5 = var4 + var3 * ambTemp
        const r = 3.4 * (var5 * (4 / (4 + resHeatRange)) * (1 / (1 + resHeatVal * 0.002)) - 25)
        return Math.max(0, Math.min(255, Math.floor(r)))
    }

    function calcGasWait(dur: number): number {
        if (dur >= 0xFC0) return 0xFF
        let factor = 0
        while (dur > 0x3F) {
            dur = Math.idiv(dur, 4)
            factor++
        }
        return dur + factor * 64
    }

    function calcTemperature(adc: number): number {
        const cf = adc - parT1 * 256
        return cf * (parT2 / Math.pow(2, 30)) + cf * cf * (parT3 / Math.pow(2, 48))
    }

    function calcPressure(adc: number, t: number): number {
        const o = parP1 * 8
        const tk10 = parP2 / Math.pow(2, 6)
        const tk20 = parP3 / Math.pow(2, 8)
        const tk30 = parP4 / Math.pow(2, 15)
        const s = (parP5 - 16384) / Math.pow(2, 20)
        const tk1s = (parP6 - 16384) / Math.pow(2, 29)
        const tk2s = parP7 / Math.pow(2, 32)
        const tk3s = parP8 / Math.pow(2, 37)
        const nls = parP9 / Math.pow(2, 48)
        const tknls = parP10 / Math.pow(2, 48)
        const nls3 = parP11 / Math.pow(2, 65)

        const t2 = t * t
        const t3 = t2 * t
        return o + tk10 * t + tk20 * t2 + tk30 * t3 +
            adc * (s + tk1s * t + tk2s * t2 + tk3s * t3) +
            adc * adc * (nls + tknls * t) +
            adc * adc * adc * nls3
    }

    function calcHumidity(adc: number, t: number): number {
        const tComp = t * 5120 - 76800
        const oh = parH1 * 64
        const sh = parH5 / 65536
        const tk10h = parH2 / 16384
        const tk1sh = parH4 / Math.pow(2, 26)
        const tk2sh = parH3 / Math.pow(2, 26)
        const hlin2 = parH6 / Math.pow(2, 19)

        const hoff = adc - (oh + tk10h * tComp)
        const hsens = hoff * sh * (1 + tk1sh * tComp + tk1sh * tk2sh * tComp * tComp)
        const h = hsens * (1 - hlin2 * hsens)
        return Math.max(0, Math.min(100, h))
    }

    function calcGasResistance(adc: number, range: number): number {
        const var1 = 262144 >> range
        const var2 = 4096 + (adc - 512) * 3
        return 1000000 * var1 / var2  // Ohm
    }

    function configure(): void {
        // humidity oversampling only takes effect after the following ctrl_meas write
        writeReg(REG_CTRL_HUM, OS_H)
        writeReg(REG_CTRL_MEAS, (OS_T << 5) | (OS_P << 2))  // sleep mode
        writeReg(REG_CONFIG, 0)                            // IIR filter off
        // heater set-point 0: used by forced mode
        const rh = calcResHeat(heaterTemp, 25)
        writeReg(REG_RES_HEAT0, rh)
        writeReg(REG_GAS_WAIT0, calcGasWait(heaterMs))
        log("heater " + heaterTemp + " C " + heaterMs + " ms: res_heat=0x" + hex(rh) +
            " gas_wait=0x" + hex(calcGasWait(heaterMs)) +
            " (G1=" + parG1 + " G2=" + parG2 + " G3=" + parG3 +
            " range=" + resHeatRange + " val=" + resHeatVal + ")")
        writeReg(REG_CTRL_GAS_0, 0x00)  // heat_off = 0 -> heater enabled
        writeReg(REG_CTRL_GAS_1, 0x20)  // run_gas = 1, nb_conv = 0
    }

    /**
     * Turn serial debug output on or off.
     */
    //% block="BME690 debug %on"
    //% on.shadow=toggleOnOff
    //% weight=10
    export function setDebug(on: boolean): void {
        debug = on
    }

    /**
     * Gas heater set-point: temperature in °C (200..400) and duration in ms.
     * Default 300 °C, 100 ms. Takes effect on the next measurement.
     */
    //% block="BME690 heater %tempC °C for %ms ms"
    //% tempC.defl=300 tempC.min=200 tempC.max=400
    //% ms.defl=100 ms.min=1 ms.max=4000
    //% weight=8
    export function setHeater(tempC: number, ms: number): void {
        heaterTemp = Math.max(200, Math.min(400, tempC))
        heaterMs = Math.max(1, Math.min(4000, ms))
        if (addr != 0) configure()
    }

    /**
     * Pause before every I2C transaction (ms). Default 5 ms.
     */
    //% block="BME690 I2C gap %ms ms"
    //% ms.defl=5 ms.min=0 ms.max=100
    //% weight=7
    export function setGap(ms: number): void {
        gapMs = Math.max(0, ms)
    }

    /**
     * Detect and configure the sensor. Tries I2C address 0x76, then 0x77.
     * Returns true if a sensor with chip ID 0x61 was found.
     */
    //% block="BME690 init"
    //% weight=100
    export function init(): boolean {
        if (!probe(0x76) && !probe(0x77)) {
            addr = 0
            log("sensor not found")
            return false
        }
        writeReg(REG_SOFT_RESET, CMD_SOFT_RESET)
        basic.pause(10)
        log("variant id 0x" + hex(readReg(REG_VARIANT_ID)))
        readCalibration()
        configure()
        return true
    }

    function measure(): void {
        const now = input.runningTime()
        if (lastMeasure > 0 && now - lastMeasure < MAX_AGE_MS) return
        if (addr == 0 && !init()) return

        // heater resistance depends on ambient temperature: use the last reading
        if (lastMeasure > 0) writeReg(REG_RES_HEAT0, calcResHeat(heaterTemp, Math.round(lastTemp)))

        // trigger: TPH conversion (~20 ms) + heater (heaterMs)
        writeReg(REG_CTRL_MEAS, (OS_T << 5) | (OS_P << 2) | 1)
        basic.pause(20 + heaterMs)

        // wait for new_data, as bme69x.c does (5 tries, 10 ms apart)
        let b: Buffer = null
        for (let i = 0; i < 5; i++) {
            b = readBuf(REG_FIELD0, 17)
            if (b[0] & 0x80) break
            basic.pause(10)
        }
        if (debug) {
            let s = "raw"
            for (let i = 0; i < 17; i++) s += " " + hex(b[i])
            log(s)
        }
        if (!(b[0] & 0x80)) {
            log("no new data, re-init on next read")
            addr = 0
            return  // keep previous values
        }

        const adcP = (b[2] << 16) | (b[3] << 8) | b[4]
        const adcT = (b[5] << 16) | (b[6] << 8) | b[7]
        const adcH = (b[8] << 8) | b[9]
        const adcG = (b[15] << 2) | (b[16] >> 6)
        const gasRange = b[16] & 0x0F

        const t = calcTemperature(adcT)
        const p = calcPressure(adcP, t)
        // outside the sensor's range -> corrupted read, or sensor reset
        if (p < 30000 || p > 110000 || t < -40 || t > 85) {
            log("implausible p=" + p + " t=" + t + ", re-init on next read")
            addr = 0
            return
        }
        lastMeasure = now
        lastTemp = t
        lastPress = p
        lastHum = calcHumidity(adcH, t)
        // gas valid + heater stable
        gasValid = (b[16] & 0x30) == 0x30
        if (gasValid) lastGas = calcGasResistance(adcG, gasRange)
        if (debug) log("gas adc=" + adcG + " range=" + gasRange)
        else log("gas not valid (heater not stable?)")
    }

    /**
     * Temperature in °C.
     */
    //% block="BME690 temperature (°C)"
    //% weight=90
    export function temperature(): number {
        measure()
        return Math.round(lastTemp * 100) / 100
    }

    /**
     * Air pressure in hPa.
     */
    //% block="BME690 pressure (hPa)"
    //% weight=85
    export function pressure(): number {
        measure()
        return Math.round(lastPress) / 100
    }

    /**
     * Relative humidity in %.
     */
    //% block="BME690 humidity (\\%)"
    //% weight=80
    export function humidity(): number {
        measure()
        return Math.round(lastHum * 10) / 10
    }

    /**
     * Gas resistance in kOhm. Higher = cleaner air. Needs some minutes of
     * operation to stabilise; returns the last valid value if the current
     * reading is invalid.
     */
    //% block="BME690 gas resistance (kΩ)"
    //% weight=75
    export function gasResistance(): number {
        measure()
        return Math.round(lastGas / 10) / 100
    }
}
