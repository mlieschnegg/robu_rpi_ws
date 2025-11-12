from neopixel.common import neopixel_spidev as np

def createPixelStrip(led_num:int=3) -> np.NeoPixelSpiDev:
    # LED-Konfiguration
    led_count = 3  # 0=rot, 1=gelb, 2=grün
    spi_bus = 0
    spi_dev = 0

    # SK6812 -> 4 Bytes (RGBW)
    pixels = np.NeoPixelSpiDev(
        bus=spi_bus,
        dev=spi_dev,
        n=led_count,
        pixel_order=np.GRB,
        bpp=3,
        auto_write=False
    )
    return pixels

if __name__ == "__main__":
    pixels = createPixelStrip()
    pixels[0] = (255, 0, 0)
    pixels[1] = (0, 255, 0)
    pixels[2] = (255, 255, 0)
    pixels.show()
