/**
 * Driver for the Sensirion SCD4x CO2 sensor (SCD40, SCD41) on the Calliope mini v3 via I2C.
 *
 * Follows Sensirion's embedded-i2c-scd4x driver: every 16-bit word is followed by
 * a CRC-8 byte, which is checked. Automatic self-calibration (ASC) is switched off,
 * because it assumes the sensor regularly sees 400 ppm fresh air.
 */
//% weight=80 color=#00A654 icon="" block="CO2-Sensor SCD4x"
namespace scd4x {

    export enum SCD4X_T_UNIT {
        //% block="°C"
        C = 0,
        //% block="°F"
        F = 1
    }

    const SCD4X_I2C_ADDR = 0x62;
    const RETRIES = 3;

    let co2 = 0;
    let temperature = 0;
    let relative_humidity = 0;
    let measurement_start = 0;
    const MIN_RUN_BEFORE_FRC_MS = 3 * 60 * 1000;

    // The simulator runs this TS body; on hardware TD_NOOP removes the call.
    // In the simulator, I2C reads return zeros (CRC fails), so these values are kept.
    //% shim=TD_NOOP
    function set_simulator_defaults() {
        co2 = 450;
        temperature = 21.5;
        relative_humidity = 68.2;
    }

    set_simulator_defaults();
    init();

    // CRC-8 as in the SCD4x datasheet: polynomial 0x31, init 0xFF
    function crc8(b0: number, b1: number): number {
        let crc = 0xFF;
        const data = [b0, b1];
        for (let i = 0; i < 2; i++) {
            crc ^= data[i];
            for (let bit = 0; bit < 8; bit++) {
                crc = (crc & 0x80) ? ((crc << 1) ^ 0x31) & 0xFF : (crc << 1) & 0xFF;
            }
        }
        return crc;
    }

    function write_buffer(buf: Buffer): boolean {
        for (let i = 0; i < RETRIES; i++) {
            if (pins.i2cWriteBuffer(SCD4X_I2C_ADDR, buf, false) == 0) return true;
            basic.pause(1);
        }
        return false;
    }

    function send_command(cmd: number): boolean {
        const buf = pins.createBuffer(2);
        buf.setNumber(NumberFormat.UInt16BE, 0, cmd);
        return write_buffer(buf);
    }

    function send_command_with_word(cmd: number, word: number): boolean {
        const buf = pins.createBuffer(5);
        buf.setNumber(NumberFormat.UInt16BE, 0, cmd);
        buf.setNumber(NumberFormat.UInt16BE, 2, word);
        buf[4] = crc8(buf[2], buf[3]);
        return write_buffer(buf);
    }

    // reads n words (each 2 bytes + CRC) in one transaction; null if any CRC fails
    function read_words(number_of_words: number): number[] {
        const buffer = pins.i2cReadBuffer(SCD4X_I2C_ADDR, number_of_words * 3, false);
        const words: number[] = [];
        for (let i = 0; i < number_of_words; i++) {
            if (crc8(buffer[3 * i], buffer[3 * i + 1]) != buffer[3 * i + 2]) return null;
            words.push(buffer.getNumber(NumberFormat.UInt16BE, 3 * i));
        }
        return words;
    }

    function get_data_ready_status(): boolean {
        if (!send_command(0xE4B8)) return false;
        basic.pause(1);
        const words = read_words(1);
        if (!words) return false;
        return (words[0] & 0x07FF) > 0;
    }

    function read_measurement() {
        // only read measurement if data is available, else use last measurement
        if (!get_data_ready_status()) {
            return
        }
        if (!send_command(0xEC05)) return;
        basic.pause(1);
        const values = read_words(3);
        if (!values) return;  // CRC error: keep last measurement
        co2 = values[0];
        temperature = -45 + (175 * values[1] / (1 << 16));
        temperature = Math.round(temperature * 10) / 10;
        relative_humidity = 100 * values[2] / (1 << 16);
        relative_humidity = Math.round(relative_humidity * 10) / 10;
    }

    // from any state to idle; configuration commands only work in idle mode
    function stop_and_wait() {
        send_command(0x3F86);
        basic.pause(500);
    }

    /**
     * Initialise: stop a measurement that may still run from before a reset,
     * switch automatic self-calibration off and start continuous measurement.
     */
    export function init() {
        basic.pause(30);  // power-up time
        stop_and_wait();
        // ASC off (volatile; not persisted, to spare the EEPROM, since init runs at every start)
        send_command_with_word(0x2416, 0);
        basic.pause(1);
        start_continuous_measurement();
    }

    /**
     * perform a factory reset
     */
    //% blockId="SCD4X_PERFORM_FACTORY_RESET" block="factory reset"
    //% weight=60 blockGap=8 advanced=true
    export function perform_factory_reset() {
        stop_and_wait();
        send_command(0x3632);
        basic.pause(1200);
        init();
    }

    /**
     * start continuous measurement. Call this before reading measurements
     */
    //% blockId="SCD4X_START_CONTINUOUS_MEASUREMENT" block="start continuous measurement"
    //% weight=70 blockGap=8 advanced=true
    export function start_continuous_measurement() {
        send_command(0x21B1);
        measurement_start = input.runningTime();
    }

    /**
     * stop continuous measurement. Call this to stop SCD4x internal measurements
     */
    //% blockId="SCD4X_STOP_CONTINUOUS_MEASUREMENT" block="stop continuous measurement"
    //% weight=70 blockGap=8 advanced=true
    export function stop_continuous_measurement() {
        stop_and_wait();
    }

    /**
     * Compensate the CO2 measurement for the current air pressure, e.g. from the
     * BMP585. Can be called while measuring; repeat when the pressure changes.
     */
    //% blockId="SCD4X_SET_AMBIENT_PRESSURE" block="set air pressure to %hPa hPa"
    //% hPa.defl=1013
    //% weight=75 blockGap=8 advanced=true
    export function set_ambient_pressure(hPa: number) {
        const value = Math.round(Math.max(700, Math.min(1200, hPa)));
        send_command_with_word(0xE000, value);
        basic.pause(1);
    }

    /**
     * Manual calibration (forced recalibration): the sensor must be in air with a
     * known, constant CO2 concentration, e.g. outdoors (about 420 ppm) or next to a
     * reference instrument. Waits until the sensor has measured for 3 minutes,
     * then calibrates and restarts the measurement. The correction is stored in the
     * sensor and survives a power cycle; "factory reset" erases it.
     * Returns the applied correction in ppm, or -32768 if the calibration failed.
     */
    //% blockId="SCD4X_CALIBRATE" block="calibrate CO2 to %ppm ppm"
    //% ppm.defl=420
    //% weight=74 blockGap=8 advanced=true
    export function calibrate(ppm: number): number {
        // Sensirion: run in periodic mode for at least 3 minutes before the FRC
        const wait = MIN_RUN_BEFORE_FRC_MS - (input.runningTime() - measurement_start);
        if (wait > 0) basic.pause(wait);
        stop_and_wait();
        const target = Math.round(Math.max(0, Math.min(40000, ppm)));
        let correction = -32768;
        if (send_command_with_word(0x362F, target)) {
            basic.pause(400);
            const words = read_words(1);
            // 0xFFFF: failed because the sensor was not measuring before
            if (words && words[0] != 0xFFFF) correction = words[0] - 0x8000;
        }
        start_continuous_measurement();
        return correction;
    }

    /**
     * get CO2 in ppm. Call this at most once every 5 seconds, else last measurement value will be returned
     */
    //% blockId="SCD4X_GET_CO2" block="CO2 (ppm)"
    //% weight=80 blockGap=8
    export function get_co2() {
        read_measurement();
        return co2;
    }

    /**
     * get temperature. Call this at most once every 5 seconds, else last measurement value will be returned
     */
    //% blockId="SCD4X_GET_TEMPERATURE" block="temperature %unit"
    //% weight=80 blockGap=8
    export function get_temperature(unit: SCD4X_T_UNIT = SCD4X_T_UNIT.C) {
        read_measurement();
        if (unit == SCD4X_T_UNIT.C) {
            return temperature;
        }
        return 32 + ((temperature * 9) / 5);
    }

    /**
     * get relative humidity in %. Call this at most once every 5 seconds, else last measurement value will be returned
     */
    //% blockId="SCD4X_GET_RELATIVE_HUMIDITY" block="relative humidity (\\%)"
    //% weight=80 blockGap=8
    export function get_relative_humidity() {
        read_measurement();
        return relative_humidity;
    }
}
