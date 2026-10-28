bmp585.setDebug(true)
serial.writeLine("init: " +  bmp585.init().toString())
basic.forever(function () {
    serial.writeValue("pressure", bmp585.pressure())
    serial.writeValue("temp", bmp585.temperature())
    serial.writeValue("alt", bmp585.altitude(1013.25))
    basic.pause(500)
})
