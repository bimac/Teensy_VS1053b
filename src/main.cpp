#include <vs1053b.h>
#include "vs1053b-patches.plg"

#define SPI_MOSI       11
#define SPI_MISO       12
#define SPI_SCK        13

#define OLED_CS         6

#define VS1053_RST     21
#define VS1053_CS      19
#define VS1053_DCS     15
#define VS1053_DREQ    10
#define VS1053_SDCS    18

VS1053b vs(VS1053_RST, VS1053_CS, VS1053_DCS, VS1053_DREQ, VS1053_SDCS);

void setup() {
  Serial.begin(9600);
  while(!Serial) {;}

  SPI.setMOSI(SPI_MOSI);
  SPI.setMISO(SPI_MISO);
  SPI.setSCK(SPI_SCK);
  SPI.begin();

  pinMode(OLED_CS, OUTPUT);
  digitalWriteFast(OLED_CS, HIGH);

  #define SD_USE_CUSTOM_SPI
  SdSpiConfig SPIConfSD(VS1053_SDCS, SHARED_SPI, 50E6);
  SD.sdfs.begin(SPIConfSD);

  bool success = vs.begin(20E6);

  if (success == 0) {
    vs.loadPlugin(plugin, sizeof(plugin)/sizeof(plugin[0]));
    Serial.printf("patched: %d\n", vs.isPatched());
  } else {
    Serial.printf("failed: %d\n", success);
  }

  vs.pinMode(3, OUTPUT);
  vs.digitalWrite(3, HIGH);

}

void loop() {

}
