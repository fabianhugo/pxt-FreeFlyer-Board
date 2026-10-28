/**
 * Minimal proof-of-concept driver for the Bosch BMI323 IMU (accelerometer,
 * gyroscope, temperature) on the Calliope mini v3 via I2C.
 *
 * Follows Bosch's BMI323_SensorAPI (bmi3.c / bmi323.c): 16-bit registers,
 * little-endian, and every I2C read starts with 2 dummy bytes.
 * No configuration file upload is needed (unlike the BMI270).
 */

enum BMI323Axis {
    //% block="x"
    X = 0,
    //% block="y"
    Y = 1,
    //% block="z"
    Z = 2,
    //% block="strength"
    Strength = 3
}

enum BMI323AccelRange {
    //% block="±2 g"
    G2 = 0,
    //% block="±4 g"
    G4 = 1,
    //% block="±8 g"
    G8 = 2,
    //% block="±16 g"
    G16 = 3
}

enum BMI323GyroRange {
    //% block="±125 °/s"
    Dps125 = 0,
    //% block="±250 °/s"
    Dps250 = 1,
    //% block="±500 °/s"
    Dps500 = 2,
    //% block="±1000 °/s"
    Dps1000 = 3,
    //% block="±2000 °/s"
    Dps2000 = 4
}

//% color=#5c2d91 icon="" block="BMI323"
namespace bmi323 {
    const REG_CHIP_ID = 0x00
    const REG_ERR = 0x01
    const REG_ACC_DATA_X = 0x03  // 0x03..0x05 acc, 0x06..0x08 gyr, 0x09 temp
    const REG_FEATURE_IO1 = 0x11
    const REG_FEATURE_IO2 = 0x12
    const REG_FEATURE_IO_STATUS = 0x14
    const REG_ACC_CONF = 0x20
    const REG_GYR_CONF = 0x21
    const REG_FEATURE_CTRL = 0x40
    const REG_CMD = 0x7E

    const CHIP_ID = 0x43
    const CMD_SOFT_RESET = 0xDEAF
    const DUMMY_BYTES = 2        // I2C reads return 2 dummy bytes first
    const INVALID = -32768       // 0x8000: no valid data (sensor off / not ready)

    const ODR_100HZ = 0x08
    const MODE_HIGH_PERF = 0x07  // lowest noise; averaging setting is ignored

    let addr = 0
    let debug = false
    const RETRIES = 3
    let gapMs = 2
    const SLOW_MS = 50
    const MAX_AGE_MS = 10        // x/y/z read back to back share one sample

    // Calliope axes from chip axes, measured on the test board (2026-10-28):
    // x = -chip y, y = -chip x, z = -chip z (a proper rotation, so it applies to the gyro too)
    const ALIGN_SRC = [1, 0, 2]
    let alignAxes = true

    let accRange: number = BMI323AccelRange.G4
    let gyrRange: number = BMI323GyroRange.Dps500

    const acc = [0, 0, 0]        // g
    const gyr = [0, 0, 0]        // °/s
    let lastTemp = 0             // °C
    let lastMeasure = 0

    function hex(n: number): string {
        const d = "0123456789ABCDEF"
        return d.charAt((n >> 12) & 0xF) + d.charAt((n >> 8) & 0xF) +
            d.charAt((n >> 4) & 0xF) + d.charAt(n & 0xF)
    }

    function log(msg: string): void {
        if (debug) serial.writeLine(input.runningTime() + " [bmi323] " + msg)
    }

    function logSlow(what: string, t0: number): void {
        const dt = input.runningTime() - t0
        if (dt > SLOW_MS) log("SLOW " + what + " took " + dt + " ms")
    }

    // I2C write with up to RETRIES attempts; returns last error code (0 = ok)
    function i2cWrite(buf: Buffer, what: string): number {
        let err = 0
        for (let i = 0; i < RETRIES; i++) {
            basic.pause(gapMs) // idle bus time before every transaction
            const t0 = input.runningTime()
            err = pins.i2cWriteBuffer(addr, buf, false)
            logSlow(what, t0)
            if (!err) return 0
            log(what + " err " + err + " (try " + (i + 1) + ")")
        }
        return err
    }

    function writeWord(reg: number, value: number): void {
        const buf = pins.createBuffer(3)
        buf[0] = reg
        buf[1] = value & 0xFF         // LSB first
        buf[2] = (value >> 8) & 0xFF
        i2cWrite(buf, "wr 0x" + hex(reg) + "=0x" + hex(value))
    }

    // reads n 16-bit words starting at reg; returns 2*n data bytes (dummy bytes removed)
    function readWords(reg: number, n: number): Buffer {
        const r = pins.createBuffer(1)
        r[0] = reg
        i2cWrite(r, "rd 0x" + hex(reg) + " addr")
        basic.pause(gapMs)
        const t0 = input.runningTime()
        const raw = pins.i2cReadBuffer(addr, DUMMY_BYTES + 2 * n)
        logSlow("rd 0x" + hex(reg) + " data", t0)
        const b = pins.createBuffer(2 * n)
        for (let i = 0; i < 2 * n; i++) b[i] = raw[DUMMY_BYTES + i]
        return b
    }

    function readWord(reg: number): number {
        const b = readWords(reg, 1)
        return b[0] | (b[1] << 8)
    }

    function s16(v: number): number {
        return v > 32767 ? v - 65536 : v
    }

    function probe(a: number): boolean {
        addr = a
        const id = readWord(REG_CHIP_ID) & 0xFF
        log("probe 0x" + hex(a) + " chip id 0x" + hex(id))
        return id == CHIP_ID
    }

    function configure(): void {
        // ACC_CONF / GYR_CONF: odr[3:0] range[6:4] bw[7]=0 (ODR/2) avg[10:8] mode[14:12]
        writeWord(REG_ACC_CONF, (MODE_HIGH_PERF << 12) | (accRange << 4) | ODR_100HZ)
        writeWord(REG_GYR_CONF, (MODE_HIGH_PERF << 12) | (gyrRange << 4) | ODR_100HZ)
        basic.pause(50)  // let the first samples arrive
        const err = readWord(REG_ERR)
        log("ACC_CONF=0x" + hex(readWord(REG_ACC_CONF)) + " GYR_CONF=0x" + hex(readWord(REG_GYR_CONF)) +
            " ERR=0x" + hex(err))
        if (err & 0x0020) log("accel config error")
        if (err & 0x0040) log("gyro config error")
        lastMeasure = 0
    }

    /**
     * Turn serial debug output on or off.
     */
    //% block="BMI323 debug %on"
    //% on.shadow=toggleOnOff
    //% weight=10
    export function setDebug(on: boolean): void {
        debug = on
    }

    /**
     * On (default): x/y/z and their signs match the Calliope's built-in
     * acceleration sensor. Off: raw chip axes as in the BMI323 datasheet.
     */
    //% block="BMI323 align axes with Calliope %on"
    //% on.shadow=toggleOnOff
    //% on.defl=true
    //% weight=40
    export function setAlignAxes(on: boolean): void {
        alignAxes = on
        lastMeasure = 0
    }

    /**
     * Pause before every I2C transaction (ms). Default 2 ms.
     */
    //% block="BMI323 I2C gap %ms ms"
    //% ms.defl=2 ms.min=0 ms.max=100
    //% weight=7
    export function setGap(ms: number): void {
        gapMs = Math.max(0, ms)
    }

    /**
     * Detect, reset and configure the sensor. Tries I2C address 0x68, then 0x69.
     * Returns true if a BMI323 was found.
     */
    //% block="BMI323 init"
    //% weight=100
    export function init(): boolean {
        basic.pause(2)
        if (!probe(0x68) && !probe(0x69)) {
            addr = 0
            log("sensor not found")
            return false
        }
        // soft reset, then enable the feature engine, exactly as bmi3_soft_reset() does
        writeWord(REG_CMD, CMD_SOFT_RESET)
        basic.pause(2)  // 1.5 ms
        writeWord(REG_FEATURE_IO2, 0x012C)
        writeWord(REG_FEATURE_IO_STATUS, 0x0001)
        writeWord(REG_FEATURE_CTRL, 0x0001)
        let ok = false
        for (let i = 0; i < 10; i++) {
            basic.pause(100)
            if (readWord(REG_FEATURE_IO1) & 0x0001) {
                ok = true
                break
            }
        }
        log("feature engine " + (ok ? "ready" : "NOT ready (basic data still works)"))
        configure()
        return true
    }

    /**
     * Accelerometer range. Smaller range = finer resolution. Default ±4 g.
     */
    //% block="BMI323 set acceleration range %range"
    //% weight=50
    export function setAccelRange(range: BMI323AccelRange): void {
        accRange = range
        if (addr != 0) configure()
    }

    /**
     * Gyroscope range. Smaller range = finer resolution. Default ±500 °/s.
     */
    //% block="BMI323 set rotation range %range"
    //% weight=49
    export function setGyroRange(range: BMI323GyroRange): void {
        gyrRange = range
        if (addr != 0) configure()
    }

    function measure(): void {
        const now = input.runningTime()
        if (lastMeasure > 0 && now - lastMeasure < MAX_AGE_MS) return
        if (addr == 0 && !init()) return

        // 7 words: acc x/y/z, gyr x/y/z, temp
        const b = readWords(REG_ACC_DATA_X, 7)
        const w: number[] = []
        let allZero = true
        for (let i = 0; i < 7; i++) {
            const v = s16(b[2 * i] | (b[2 * i + 1] << 8))
            w.push(v)
            if (v != 0) allZero = false
        }
        if (debug) {
            let s = "raw"
            for (let i = 0; i < 7; i++) s += " " + w[i]
            log(s)
        }
        if (allZero) {
            // a failed I2C read returns zeros; real data is never all zero
            log("all-zero read, re-init on next read")
            addr = 0
            return  // keep previous values
        }

        const gFull = 2 << accRange                        // 2, 4, 8, 16 g
        const dpsFull = 125 << gyrRange                    // 125 ... 2000 °/s
        for (let i = 0; i < 3; i++) {
            // chip axis feeding output axis i, and its sign
            const src = alignAxes ? ALIGN_SRC[i] : i
            const sign = alignAxes ? -1 : 1
            if (w[src] != INVALID) acc[i] = sign * w[src] * gFull / 32768
            if (w[3 + src] != INVALID) gyr[i] = sign * w[3 + src] * dpsFull / 32768
        }
        if (w[6] != INVALID) lastTemp = w[6] / 512 + 23
        lastMeasure = now
    }

    function pick(v: number[], axis: BMI323Axis): number {
        if (axis == BMI323Axis.Strength) return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        return v[axis]
    }

    /**
     * Acceleration in mg (1000 mg = 1 g) on one axis, or the total strength.
     */
    //% block="BMI323 acceleration (mg) %axis"
    //% weight=90
    export function acceleration(axis: BMI323Axis): number {
        measure()
        return Math.round(pick(acc, axis) * 10000) / 10
    }

    /**
     * Rotation rate in °/s on one axis, or the total strength.
     */
    //% block="BMI323 rotation rate (°/s) %axis"
    //% weight=85
    export function rotationRate(axis: BMI323Axis): number {
        measure()
        return Math.round(pick(gyr, axis) * 100) / 100
    }

    /**
     * Chip temperature in °C.
     */
    //% block="BMI323 temperature (°C)"
    //% weight=80
    export function temperature(): number {
        measure()
        return Math.round(lastTemp * 100) / 100
    }
}
