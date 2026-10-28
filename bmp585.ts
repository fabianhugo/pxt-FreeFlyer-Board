/**
 * Minimal proof-of-concept driver for the Bosch BMP585 pressure sensor
 * (also works with BMP581) on the Calliope mini v3 via I2C.
 *
 * Like the bmpxxx.py MicroPython reference: the sensor is put into continuous
 * (non-stop) mode once in init. A reading is then a single burst read of the
 * data registers, with no register writes during operation.
 */
//% color=#0069b4 icon="" block="BMP585"
namespace bmp585 {
    const REG_CHIP_ID = 0x01
    const REG_INT_STATUS = 0x27
    const REG_STATUS = 0x28
    const REG_TEMP_XLSB = 0x1D   // 0x1D..0x1F temp, 0x20..0x22 pressure
    const REG_OSR_CONFIG = 0x36
    const REG_ODR_CONFIG = 0x37
    const REG_CMD = 0x7E

    const CMD_SOFT_RESET = 0xB6
    const CHIP_ID_BMP581 = 0x50
    const CHIP_ID_BMP585 = 0x51

    // press_en=1, osr_p=x8 (3), osr_t=x1 (0)
    const OSR_CONFIG = 0x40 | (3 << 3) | 0
    // deep_dis=1, pwr_mode=standby (0) / non-stop (3)
    const ODR_STANDBY = 0x80 | 0x00
    const ODR_NON_STOP = 0x80 | 0x03

    let addr = 0
    let debug = false
    let trace = false
    let repeatedStart = false
    const RETRIES = 3
    let gapMs = 5
    const SLOW_MS = 50
    const MAX_AGE_MS = 100  // reuse a measurement younger than this
    let lastTemp = 0
    let lastPress = 0
    let lastMeasure = 0

    function hex(n: number): string {
        const d = "0123456789ABCDEF"
        return d.charAt((n >> 4) & 0xF) + d.charAt(n & 0xF)
    }

    function log(msg: string): void {
        if (debug) serial.writeLine(input.runningTime() + " [bmp585] " + msg)
    }

    function traceLog(msg: string): void {
        if (trace) serial.writeLine(input.runningTime() + " [i2c] " + msg)
    }

    // log I2C calls that block for longer than SLOW_MS
    function logSlow(what: string, t0: number): void {
        const dt = input.runningTime() - t0
        if (dt > SLOW_MS) log("SLOW " + what + " took " + dt + " ms")
    }

    // I2C write with up to RETRIES attempts; returns last error code (0 = ok)
    function i2cWrite(buf: Buffer, repeat: boolean, what: string): number {
        let err = 0
        for (let i = 0; i < RETRIES; i++) {
            basic.pause(gapMs) // idle bus time before every transaction
            traceLog(what)
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
        traceLog("rd 0x" + hex(reg) + " data x" + len)
        const t0 = input.runningTime()
        const b = pins.i2cReadBuffer(addr, len)
        logSlow("rd 0x" + hex(reg) + " data", t0)
        traceLog("rd done")
        return b
    }

    function readReg(reg: number): number {
        return readBuf(reg, 1)[0]
    }

    function probe(a: number): boolean {
        addr = a
        const id = readReg(REG_CHIP_ID)
        log("probe 0x" + hex(a) + " chip id 0x" + hex(id))
        return id == CHIP_ID_BMP585 || id == CHIP_ID_BMP581
    }

    function configure(): void {
        // config registers may only be changed in standby
        writeReg(REG_ODR_CONFIG, ODR_STANDBY)
        basic.pause(5) // mode change takes up to 4 ms
        writeReg(REG_OSR_CONFIG, OSR_CONFIG)
        writeReg(REG_ODR_CONFIG, ODR_NON_STOP)
        basic.pause(5)
        log("OSR_CONFIG=0x" + hex(readReg(REG_OSR_CONFIG)) +
            " ODR_CONFIG=0x" + hex(readReg(REG_ODR_CONFIG)))
    }

    /**
     * Turn serial debug output on or off.
     */
    //% block="BMP585 debug %on"
    //% on.shadow=toggleOnOff
    //% weight=10
    export function setDebug(on: boolean): void {
        debug = on
    }

    /**
     * Log every single I2C transaction to serial (very verbose).
     * The last line before a hang shows the transaction that blocked.
     */
    //% block="BMP585 trace I2C %on"
    //% on.shadow=toggleOnOff
    //% weight=9
    export function setTrace(on: boolean): void {
        trace = on
    }

    /**
     * Use I2C repeated start (no STOP) between register address and read.
     * Default off: a full STOP is sent, which the BMP585 also accepts.
     */
    //% block="BMP585 repeated start %on"
    //% on.shadow=toggleOnOff
    //% weight=8
    export function setRepeatedStart(on: boolean): void {
        repeatedStart = on
    }

    /**
     * Pause before every I2C transaction (ms). Larger = slower but gives the bus more
     * idle time. Default 5 ms.
     */
    //% block="BMP585 I2C gap %ms ms"
    //% ms.defl=5 ms.min=0 ms.max=100
    //% weight=7
    export function setGap(ms: number): void {
        gapMs = Math.max(0, ms)
    }

    /**
     * Detect and configure the sensor. Tries I2C address 0x46, then 0x47.
     * Returns true if a BMP585/BMP581 was found.
     */
    //% block="BMP585 init"
    //% weight=100
    export function init(): boolean {
        basic.pause(3) // power-up time t_powup is 2 ms
        if (!probe(0x46) && !probe(0x47)) {
            addr = 0
            log("sensor not found")
            return false
        }
        writeReg(REG_CMD, CMD_SOFT_RESET)
        basic.pause(5)
        // wait for NVM ready (bit 1), give up after ~50 ms
        let status = 0
        for (let i = 0; i < 10; i++) {
            status = readReg(REG_STATUS)
            if (status & 0x02) break
            basic.pause(5)
        }
        log("STATUS=0x" + hex(status))
        readReg(REG_INT_STATUS) // clear POR flag from the soft reset
        configure()
        return true
    }

    function measure(): void {
        // pressure/temperature/altitude called back to back share one reading
        const now = input.runningTime()
        if (lastMeasure > 0 && now - lastMeasure < MAX_AGE_MS) return
        if (addr == 0 && !init()) return

        // sensor converts continuously; just read the latest result
        // (2 I2C transactions: address write, data read)
        const b = readBuf(REG_TEMP_XLSB, 6)
        if (debug) {
            let s = "raw"
            for (let i = 0; i < 6; i++) s += " " + hex(b[i])
            log(s)
        }
        let rawT = b[0] | (b[1] << 8) | (b[2] << 16)
        if (rawT & 0x800000) rawT -= 0x1000000 // sign-extend 24 bit
        const rawP = b[3] | (b[4] << 8) | (b[5] << 16)
        const t = rawT / 65536      // °C
        const p = rawP / 64         // Pa

        // outside the sensor's range (300..1250 hPa, -40..85 °C) -> corrupted read,
        // or the sensor reset and lost its config (data reads 0)
        if (p < 30000 || p > 125000 || t < -40 || t > 85) {
            log("implausible p=" + p + " t=" + t + ", re-init on next read")
            addr = 0
            return  // keep previous values
        }
        lastMeasure = now
        lastTemp = t
        lastPress = p
    }

    /**
     * Air pressure in hPa.
     */
    //% block="BMP585 pressure (hPa)"
    //% weight=90
    export function pressure(): number {
        measure()
        return Math.round(lastPress) / 100
    }

    /**
     * Temperature in °C.
     */
    //% block="BMP585 temperature (°C)"
    //% weight=80
    export function temperature(): number {
        measure()
        return Math.round(lastTemp * 100) / 100
    }

    /**
     * Barometric altitude in m, relative to the given sea-level pressure.
     */
    //% block="BMP585 altitude (m) at sea level pressure %seaLevel hPa"
    //% seaLevel.defl=1013.25
    //% weight=70
    export function altitude(seaLevel: number): number {
        measure()
        const hPa = lastPress / 100
        return Math.round(44330 * (1 - Math.pow(hPa / seaLevel, 1 / 5.255)) * 10) / 10
    }

    let refSeaLevel = 1013.25  // hPa, set by setReferenceHeight()

    /**
     * Calibrate altitude: tell the sensor its current height above sea level in m.
     * Averages 5 readings and derives the matching sea-level pressure, which
     * "BMP585 altitude (m)" then uses. Repeat when the weather changes.
     */
    //% block="BMP585 set current height to %height m"
    //% height.defl=0
    //% weight=69
    export function setReferenceHeight(height: number): void {
        let sum = 0
        let n = 0
        for (let i = 0; i < 5; i++) {
            measure()
            if (lastPress > 0) {
                sum += lastPress
                n++
            }
            basic.pause(MAX_AGE_MS + 20) // make sure the next measure() reads fresh data
        }
        if (n == 0) {
            log("setReferenceHeight: no valid reading")
            return
        }
        const hPa = sum / n / 100
        refSeaLevel = hPa / Math.pow(1 - height / 44330, 5.255)
        log("reference: " + height + " m at " + hPa + " hPa -> sea level " + refSeaLevel + " hPa")
    }

    /**
     * Altitude in m, using the reference from "BMP585 set current height".
     * Without calibration, uses the standard 1013.25 hPa.
     */
    //% block="BMP585 altitude (m)"
    //% weight=68
    export function altitudeCalibrated(): number {
        return altitude(refSeaLevel)
    }
}
