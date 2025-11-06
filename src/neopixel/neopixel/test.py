import numpy
import spidev

def neopixel_spi_write_orig(spi, data):
    d = numpy.array(data).ravel()
    tx = numpy.zeros(len(d) * 4, dtype = numpy.uint8)
    for ibit in range(4):
        tx[3 - ibit::4] = ((d >> (2 * ibit + 1)) & 1) * 0x60 + \
                          ((d >> (2 * ibit + 0)) & 1) * 0x06 + 0x88
    spi.xfer(tx.tolist(), int(4/1.25e-6))

def write2812_numpy4(spi,data):
    #print spi
    d=numpy.array(data).ravel()

    #z.B. Farbwert dd = 0xAA -> 0b10101010:
    #DIN der LED:  dd = 0xAA -> tx = 1110 1100 1110 1100 1110 1100 1110 1100
    tx=numpy.zeros(len(d)*4, dtype=numpy.uint8)
    reset = numpy.zeros(6*4, dtype=numpy.uint8)
    for ibit in range(4):
        tx[3 - ibit::4] = ((d >> (2 * ibit + 1)) & 1) * 0x60 + \
                          ((d >> (2 * ibit + 0)) & 1) * 0x06 +  0x88
        #print [hex(v) for v in tx]
    #print [hex(v) for v in tx]
    print(tx.tolist())
    print(int(4.0/1.25e-6))
    spi.xfer(tx.tolist(), int(4.0/1.25e-6)) #works, on Zero (initially didn't?)
    #spi.xfer(tx.tolist(), int(4/1.20e-6))  #works, no flashes on Zero, Works on Raspberry 3
    #spi.xfer(tx.tolist(), int(4/1.15e-6))  #works, no flashes on Zero
    #spi.xfer(tx.tolist(), int(4/1.05e-6))  #works, no flashes on Zero
    #spi.xfer(tx.tolist(), int(4/.95e-6))  #works, no flashes on Zero
    #spi.xfer(tx.tolist(), int(4/.90e-6))  #works, no flashes on Zero
    #spi.xfer(tx.tolist(), int(4/.85e-6))  #doesn't work (first 4 LEDS work, others have flashing colors)
    #spi.xfer(tx.tolist(), int(4/.65e-6))  #doesn't work on Zero; Works on Raspberry 3
    #spi.xfer(tx.tolist(), int(4/.55e-6))  #doesn't work on Zero; Works on Raspberry 3
    #spi.xfer(tx.tolist(), int(4/.50e-6))  #doesn't work on Zero; Doesn't work on Raspberry 3 (bright colors)
    #spi.xfer(tx.tolist(), int(4/.45e-6))  #doesn't work on Zero; Doesn't work on Raspberry 3
    #spi.xfer(tx.tolist(), int(8e6))

def main():
    n_led = 1

    spi = spidev.SpiDev()
    #spi.open_path("/dev/spidev0.0")
    spi.open(0,0)
    # spi.max_speed_hz = 4*8000000
    # spi.mode = 0b00
    write2812_numpy4(spi, [[255,0,0]]*n_led)
    


if __name__ == "__main__":
    main()