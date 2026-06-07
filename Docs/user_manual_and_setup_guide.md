# RTOS Flight Controller User Manual and Technical Setup Guide

Bu dokuman, `rtos_flight_controller` projesini ilk kez calistiracak bir kullaniciya ve projeyi gelistirmeye devam edecek bir ekip arkadasina ayni anda yardim etmek icin hazirlanmistir. Amac yalnizca "hangi komutu calistirayim?" sorusunu degil, sistemin hangi katmanlardan olustugunu, hangi verinin nereden gelip nereye gittigini ve demo sirasinda neyin nasil kontrol edilecegini de aciklamaktir.

> Mevcut repo hedefi: STM32 Nucleo-F411RE uzerinde FreeRTOS tabanli denge/ucus kontrol prototipi, MPU6050 IMU, BMP180 barometre, SG90 servo motorlar ve PC tarafinda canli telemetry dashboard.

---

## 1. Hızlı Başlangıç

### 1.1. Sadece dashboard'u calistirmak

Dashboard firmware olmadan da acilabilir; ancak normal calisma prensibi gelen telemetry paketlerini gorsellestirmektir.

```bash
cd Tools/TelemetryDashboard
npm install
npm run dev
```

Tarayicida ac:

```text
http://127.0.0.1:5173
```

### 1.2. STM32'den USB serial ile canli telemetry almak

1. STM32 kartini USB ile bilgisayara bagla.
2. Firmware'in UART telemetry cikisi verdiginden emin ol.
3. Dashboard'u ac.
4. Ust bardaki `Serial` butonuna bas.
5. Chrome Web Serial penceresinden STM32 Virtual COM Port'u sec.

Bu yol en basit kablolu demo yoludur:

```text
STM32 USART2 telemetry -> ST-LINK USB Virtual COM Port -> Chrome Web Serial -> Dashboard
```

### 1.3. Serial-to-WebSocket relay ile calismak

Bu yol ESP32 WiFi koprusunu bilgisayar ustunde taklit eder:

```bash
cd Tools/TelemetryDashboard
npm run serial:list
npm run serial:bridge -- --port=/dev/tty.usbmodemXXXX --baud=115200
```

Dashboard URL:

```text
http://127.0.0.1:5173/?ws=ws://127.0.0.1:8081/telemetry
```

Sonra dashboard'da `WiFi` butonuna bas.

---

## 2. Proje Amacı

Bu proje, roket benzeri dik duran bir platformun sensör verisiyle izlenmesi, yöneliminin hesaplanması, kontrol algoritmasıyla servo kanatcik komutlarinin uretilmesi ve tum bu sistemin telemetry dashboard uzerinde canli olarak gosterilmesi icin gelistirilmistir.

Juri/demo acisindan sistem su sorulara cevap verir:

- STM32 uzerinde RTOS task mimarisi nasil kuruldu?
- MPU6050 IMU verisi nasil okunuyor?
- BMP180 barometre driver'i sisteme nasil dahil edildi?
- Sensor verisi GDS yani Global Data Space uzerinden task'lere nasil dagitiliyor?
- Navigation katmani roll/pitch/yaw durumunu nasil yayinliyor?
- Flight Control katmani servo komutlarini nasil uretiyor?
- Actuator katmani servo driver'larini nasil komutluyor?
- Telemetry task binary paketleri nasil olusturuyor?
- Dashboard bu paketleri nasil parse edip 3D roket modeline ve veri panellerine yansitiyor?

---

## 3. Sistem Bileşenleri

### 3.1. Donanim

| Bilesen | Gorev |
| --- | --- |
| STM32 Nucleo-F411RE | Ana kontrol karti, RTOS task'leri, sensor/actuator/telemetry yazilimi |
| MPU6050 | IMU: ivmeolcer ve jiroskop verisi |
| BMP180 | Barometre: basinc, sicaklik, irtifa tahmini |
| SG90 servo motorlar | Kanatcik/fin hareketini temsil eden PWM kontrollu aktuatorler |
| Breadboard ve kablolar | Sensor, servo ve guc dagitimi |
| PC/Mac | Dashboard, serial/WebSocket testleri ve gelistirme ortami |
| ESP32 veya serial relay | Kablosuz telemetry koprusu veya bilgisayar ustu test koprusu |

### 3.2. Yazilim

| Katman | Dosya/Klasor | Gorev |
| --- | --- | --- |
| Core STM32 HAL | `Core`, `Drivers/STM32F4xx_HAL_Driver` | CubeMX tarafindan uretilen MCU baslatma, clock, GPIO, I2C, UART, DMA, TIM ayarlari |
| Device drivers | `Drivers/mpu6050`, `Drivers/bmp180`, `Drivers/servo` | Platformdan bagimsiz sensor/aktuator suruculeri ve STM32 port katmanlari |
| App managers | `App/SensorAcq`, `App/Actuator` | Driver'lari ortak app arayuzlerine baglayan yoneticiler |
| Navigation | `App/Navigation` | IMU verisinden arac durum kestirimi |
| Flight Management Control | `App/FlightManagementControl` | Mod yonetimi ve servo fin komutlari |
| Telemetry | `App/Telemetry` | GDS verilerini binary paketlere cevirme |
| Platform port | `Platform/STM32` | HAL, RTOS, driver ve app katmanlarini STM32 uzerinde birlestirme |
| Dashboard | `Tools/TelemetryDashboard` | Telemetry paketlerini parse edip juriye gosterilecek arayuzu calistirma |
| Host tests | `tests/host` | Driver ve app katmanlarini host ortaminda test etme |

---

## 4. Yazılım Mimarisi

### 4.1. Katmanli mimari

Proje, suruculerin dogrudan uygulama veya STM32 HAL ile karismamasi icin katmanli yazilmistir.

```text
Dashboard / MATLAB tools
        ^
        |
Telemetry packet stream
        ^
        |
App/Telemetry
        ^
        |
Global Data Space (GDS)
        ^
        |
SensorTask / NavTask / FlightControlTask / ActuatorTask
        ^
        |
Device Driver Interface adapters
        ^
        |
Platform independent drivers
        ^
        |
STM32 HAL port layer
        ^
        |
STM32 HAL / FreeRTOS / Hardware
```

Bu ayrim sayesinde:

- MPU6050, BMP180 ve servo core driver'lari STM32 HAL header'i bilmez.
- RTOS mutex, delay ve tick fonksiyonlari callback olarak enjekte edilir.
- SensorManager, driver'in ic detayini bilmeden `pfnReadImu`, `pfnProcess`, `pfnReadBarometer` gibi DDI fonksiyonlarini cagirir.
- TelemetryTask sensor driver'ina dogrudan gitmez; GDS'den yayinlanan son veriyi okur.
- Dashboard firmware icinde hesap yapmaz; sadece paketten gelen alani gosterir.

### 4.2. Veri akisi

```text
MPU6050
  -> mpu6050 STM32 HAL port
  -> MPU6050 core driver
  -> mpu6050_ddi_adapter
  -> SensorManager
  -> GDS RawImu topic
  -> NavigationTask
  -> GDS VehicleState topic
  -> FlightControlTask
  -> GDS ActuatorCmd / FlightStatus topics
  -> ActuatorTask / TelemetryTask
  -> UART DMA telemetry
  -> Dashboard
```

BMP180 barometre yolu:

```text
BMP180
  -> bmp180 STM32 HAL port
  -> BMP180 core driver
  -> bmp180_ddi_adapter
  -> SensorManager
  -> GDS Barometer topic
  -> Telemetry packet extension when firmware layout includes barometer fields
  -> Dashboard barometer and altitude panels
```

> Not: Mevcut `main` dalinda BMP180 driver ve DDI altyapisi repo icindedir. Platform baslatmasinda `AppPlatformPort_prvInitBmp180()` cagrisi yorum satirinda ise BMP180 fiziksel olarak okunmaz. Final barometre demosu icin bu yol firmware tarafinda aktif edilmeli ve telemetry paket layout'u barometre alanlarini icermelidir.

Servo aktuator yolu:

```text
FlightControlTask
  -> GDS ActuatorCmd topic
  -> ActuatorManager
  -> servo_ddi_adapter
  -> Servo core driver
  -> servo STM32 HAL port
  -> TIM1 PWM compare register
  -> SG90 servo
```

---

## 5. RTOS Task Yapısı

`Platform/STM32/app_platform_port.c` icinde statik task'ler olusturulur. Dinamik bellek kullanimi yerine statik task control block ve stack alanlari tercih edilir.

| Task | Periyot | Oncelik | Gorev |
| --- | --- | --- | --- |
| SensorTask | 10 ms | AboveNormal | IMU ve varsa barometre sensorlerini yonetir, GDS'ye sensor topic'leri yayinlar |
| NavTask | 10 ms | AboveNormal | Raw IMU verisini okuyup VehicleState topic'ini yayinlar |
| FlightControlTask | 10 ms | AboveNormal | Flight mode ve fin/servo komutlarini uretir |
| ActuatorTask | 10 ms | AboveNormal | GDS actuator komutlarini fiziksel servo suruculerine yazar |
| TelemetryTask | 50 ms | Normal | GDS verilerini binary UART DMA paketlerine cevirir |
| defaultTask | sistem baslatma | Normal | Platform init ve app task yaratma akisina giris yapar |

### 5.1. Event-driven ve DMA yaklaşımı

MPU6050 tarafinda veri hazir interrupt'i ve async DMA okuma yaklasimi kullanilir. Bu sayede IMU okuma yuku CPU'ya surekli polling olarak binmez.

Telemetry tarafinda UART TX DMA kullanilir:

- `TelemetryTask` paketi hazirlar.
- `AppPlatformPort_prvUartDmaSend()` paketi DMA buffer'a kopyalar.
- `HAL_UART_Transmit_DMA()` aktarimi baslatir.
- DMA tamamlaninca `HAL_UART_TxCpltCallback()` cagrilir.
- Platform port semaphore token'i geri birakir.

Bu yapinin amaci TelemetryTask'in UART byte'larini CPU ile tek tek basmak yerine DMA'ya devretmesidir.

### 5.2. Interrupt ve DMA kaynaklari

| Kaynak | Dosya | Gorev |
| --- | --- | --- |
| DMA1 Stream0 | `stm32f4xx_it.c` | I2C1 RX DMA, MPU6050 async read yolu |
| DMA1 Stream6 | `stm32f4xx_it.c` | USART2 TX DMA, telemetry paketi gonderimi |
| I2C1 EV/ER IRQ | `stm32f4xx_it.c` | I2C event/error interrupt'lari |
| USART2 IRQ | `stm32f4xx_it.c` | UART HAL interrupt yolu |
| TIM1_UP_TIM10 IRQ | `stm32f4xx_it.c` | TIM1/TIM10 interrupt; TIM10 HAL tick icin 1 ms zaman tabani |
| MPU6050 EXTI callback | `AppPlatformPort_OnExtiCallback()` | IMU data-ready interrupt'ini driver async worker'ina iletir |

---

## 6. Donanım Bağlantı Rehberi

### 6.1. Genel kurallar

- STM32, MPU6050, BMP180, ESP32 ve servo guc kaynaklari arasinda ortak `GND` kullan.
- MPU6050 ve BMP180 icin 3.3 V logic kullan.
- Servo motorlari STM32'nin 3.3 V pininden besleme. SG90 icin ayri 5 V kaynagi kullan; ama servo GND ile STM32 GND ortak olmali.
- I2C hatti uzun kablolarla bozulabilir. Kisa kablo kullan ve gerekli ise pull-up direnclerini kontrol et.
- RFD/ESP32 gibi harici haberlesme modullerinde TX-RX capraz baglanir: STM32 TX -> modul RX.

### 6.2. I2C sensor baglantilari

CubeMX/HAL ayarina gore I2C1 pinleri:

| STM32F411RE | Gorev | Sensor |
| --- | --- | --- |
| PB8 | I2C1 SCL | MPU6050 SCL, BMP180 SCL |
| PB9 | I2C1 SDA | MPU6050 SDA, BMP180 SDA |
| 3V3 | Sensor besleme | MPU6050 VCC, BMP180 VCC |
| GND | Ortak toprak | MPU6050 GND, BMP180 GND |

I2C adresleri:

| Sensor | Tipik 7-bit adres |
| --- | --- |
| MPU6050 AD0 low | `0x68` |
| BMP180 | `0x77` |

### 6.3. Servo PWM baglantilari

TIM1 PWM cikislari:

| STM32F411RE | TIM kanali | Servo |
| --- | --- | --- |
| PA8 | TIM1_CH1 | Fin/Servo A |
| PA9 | TIM1_CH2 | Fin/Servo B |
| PA10 | TIM1_CH3 | Fin/Servo C |
| PA11 | TIM1_CH4 | Fin/Servo D |

SG90 icin tipik sinyal:

- Frekans: 50 Hz
- Periyot: 20 ms
- Pulse: yaklasik 500 us - 2500 us
- Orta konum: yaklasik 1500 us

### 6.4. Telemetry UART

USART2 ayarlari:

| Alan | Deger |
| --- | --- |
| Baudrate | 115200 |
| Word length | 8 bit |
| Parity | None |
| Stop bit | 1 |
| Flow control | None |

Pinler:

| STM32F411RE | Gorev |
| --- | --- |
| PA2 | USART2_TX |
| PA3 | USART2_RX |

USB/ST-LINK Virtual COM Port ile calisirken bu UART hatti bilgisayara USB uzerinden gorunur. Harici ESP32/RFD baglamak icin ayni hatti kullanmak kart uzerindeki solder bridge ve kablolama durumuna bagli olabilir. En guvenli demo yolu, once USB serial ile dashboard'u dogrulamak, sonra ESP32/RFD yolunu ayri test etmektir.

---

## 7. Firmware Kurulum ve Derleme

### 7.1. Gerekli araclar

Minimum:

- Git
- CMake 3.22 veya ustu
- Ninja
- ARM GNU Toolchain (`arm-none-eabi-gcc`)
- STM32CubeIDE veya STM32CubeProgrammer
- Node.js ve npm

Opsiyonel:

- VS Code + clangd
- GoogleTest host testleri icin C/C++ compiler
- MATLAB live telemetry viewer icin MATLAB

### 7.2. Repo'yu alma

```bash
git clone --recurse-submodules https://github.com/AgahEnes/rtos_flight_controller.git
cd rtos_flight_controller
git submodule update --init --recursive
```

`Drivers/mpu6050` submodule oldugu icin `--recurse-submodules` veya sonradan `git submodule update` onemlidir.

### 7.3. CMake ile firmware build

```bash
cmake --preset Debug
cmake --build --preset Debug
```

Release build:

```bash
cmake --preset Release
cmake --build --preset Release
```

Build sonucunda `build/Debug` veya `build/Release` altinda firmware ciktilari olusur.

### 7.4. STM32CubeIDE ile acma

1. STM32CubeIDE'yi ac.
2. `File -> Open Projects from File System` sec.
3. Repo klasorunu sec.
4. `.ioc`, `CMakeLists.txt`, `Core`, `Drivers`, `App`, `Platform` klasorlerinin gorundugunu kontrol et.
5. Kart olarak Nucleo-F411RE bagli iken debug/flash islemini baslat.

### 7.5. STM32CubeProgrammer ile flash

Build edilen `.elf` veya `.bin` dosyasi STM32CubeProgrammer ile SWD uzerinden yuklenebilir.

Genel akış:

```text
Connect: ST-LINK / SWD
Open file: build/Debug/rtos_cmake_cubemx.elf
Download
Reset
```

---

## 8. Host Testleri

Host testleri, firmware'i karta yuklemeden app ve driver mantigini bilgisayar uzerinde test etmek icindir.

```bash
cmake -S tests/host -B build/host
cmake --build build/host
ctest --test-dir build/host --output-on-failure
```

Test kapsami:

- Sensor acquisition manager
- Navigation subsystem
- Flight control
- Actuator manager
- MPU6050 driver host testleri
- Servo driver testleri
- BMP180 smoke test

Host testleri donanim baglantisini dogrulamaz; algoritma, paketleme, state machine ve driver core mantigi icin kullanilir.

---

## 9. Dashboard Kullanım Kılavuzu

### 9.1. Dashboard'u baslatma

```bash
cd Tools/TelemetryDashboard
npm install
npm run dev
```

Tarayici:

```text
http://127.0.0.1:5173
```

LAN uzerinden baska cihazdan acmak:

```bash
npm run dev:lan
```

Sonra ayni agdaki cihazdan:

```text
http://<computer-ip>:5173
```

### 9.2. Dashboard ekranlari

| Bolum | Anlam |
| --- | --- |
| 3D Rocket View | Roll/pitch/yaw verisine gore roket modelinin yonelimi |
| Mission State | Flight mode, tilt, frame count, packet type |
| Sensor Stream | IMU accel/gyro verileri |
| Barometer | BMP180 sicaklik, basinc ve irtifa verileri; paket icinde varsa guncellenir |
| Estimation Altitude | Irtifa degerinin 3 m olceginde gorsel sunumu |
| Servo Commands | Fin/servo acilarinin gorsel ve sayisal sunumu; paket icinde varsa guncellenir |
| Trend grafikler | Attitude ve raw IMU trendlerinin zaman icinde akisi |

Dashboard su prensiple calisir:

- Paket icinde gelen degeri gosterir.
- Paket icinde olmayan degeri tahmin etmez.
- Ucus modu, servo acisi, barometre veya irtifa alanlari pakette yoksa baslangic degeri kalir.

### 9.3. Serial mod

1. Dashboard'u ac.
2. `Serial` butonuna bas.
3. STM32 portunu sec.
4. Link durumu ve frame sayisinin artmasini bekle.

Eger frame sayisi artmiyorsa:

- Dogru port secildi mi?
- Firmware calisiyor mu?
- UART baudrate 115200 mu?
- Browser Web Serial izni verildi mi?
- TelemetryTask TX DMA token'i kilitlenmis olabilir mi?

### 9.4. WiFi/WebSocket mod

WebSocket URL alanina su formatta adres yaz:

```text
ws://<ip>:<port>/telemetry
```

Ornekler:

```text
ws://127.0.0.1:8081/telemetry
ws://192.168.4.1:81/telemetry
```

Sonra `WiFi` butonuna bas.

### 9.5. Serial-to-WebSocket relay

Portlari listele:

```bash
npm run serial:list
```

Relay baslat:

```bash
npm run serial:bridge -- --port=/dev/tty.usbmodemXXXX --baud=115200
```

LAN binding:

```bash
npm run serial:bridge -- --port=/dev/tty.usbmodemXXXX --baud=115200 --host=0.0.0.0
```

Dashboard URL:

```text
http://127.0.0.1:5173/?ws=ws://127.0.0.1:8081/telemetry
```

---

## 10. Telemetry Paketleri

### 10.1. Ana state paketi

| Alan | Deger |
| --- | --- |
| Sync | `0xA5 0x5A` |
| Message ID | `0x12` |
| Uzunluk | 64 byte |
| CRC | CRC16/CCITT-FALSE |

Payload icerigi:

- Timestamp
- Accel X/Y/Z
- Gyro X/Y/Z
- IMU temperature
- Roll/pitch/yaw
- Roll-rate/pitch-rate/yaw-rate
- Estimator valid flag
- Flight mode byte

Bu paket `App/Telemetry/telemetry_task.c` icinde GDS'den okunup paketlenir.

### 10.2. Calibration event paketi

| Alan | Deger |
| --- | --- |
| Sync | `0xA5 0x5A` |
| Message ID | `0x81` |
| Uzunluk | 43 byte |
| CRC | CRC16/CCITT-FALSE |

Bu paket IMU kalibrasyon bilgisi guncellendiginde event slot uzerinden gonderilir.

### 10.3. Dashboard parser destekleri

Dashboard su protokolleri okuyabilir:

- Agah IMU/Vehicle state frame: `0xA5 0x5A`, msg `0x12`
- Agah calibration frame: `0xA5 0x5A`, msg `0x81`
- Legacy Akif IMU frame: `0xA5 0x5A`, msg `0x10`
- Mentor extended sensor frame: `0xAA 0x55`, msg `0x10`
- Gecici ASCII `RTOSFUS` frame

Final kural: Dashboard paket disinda bilgi uretmez. Yeni bir alan gosterilecekse once firmware paketine eklenmeli, sonra dashboard parser o alani okumali.

---

## 11. Demo Hazırlık Akışı

### 11.1. Fiziksel kontrol

- STM32 USB bagli mi?
- MPU6050 ve BMP180 3.3 V ile besleniyor mu?
- I2C SDA/SCL dogru mu?
- Servo GND ile STM32 GND ortak mi?
- Servo harici 5 V kaynaktan besleniyor mu?
- Servo sinyal pinleri PA8/PA9/PA10/PA11 uzerinde mi?
- Platform mekanigi serbest hareket ediyor mu?

### 11.2. Firmware kontrol

- Firmware build geciyor mu?
- Kart flash edildi mi?
- FreeRTOS task'leri basliyor mu?
- Telemetry frame sayisi artiyor mu?
- MPU6050 data-ready interrupt ve I2C DMA yolu calisiyor mu?
- UART TX DMA tamamlanma callback'i token'i geri birakiyor mu?

### 11.3. Dashboard kontrol

- Dashboard aciliyor mu?
- `Serial` veya `WiFi` baglantisi kuruluyor mu?
- Frame sayisi artiyor mu?
- Packet type beklenen deger mi?
- Roll/pitch hareketleri fiziksel platformla ayni yonde mi?
- Mission State modu beklenen sekilde degisiyor mu?
- Servo komutlari pakette varsa kanatciklar hareket ediyor mu?
- Barometre pakette varsa altitude gorseli degisiyor mu?

### 11.4. Juri anlatim sirasinda onerilen demo akisi

1. Katmanli mimariyi goster.
2. Kart ve sensor/servo fiziksel prototipini goster.
3. Dashboard'u ac ve telemetry baglantisini kur.
4. Platformu elle roll/pitch yonlerinde eg.
5. 3D modelin ayni hareketi yaptigini goster.
6. IMU stream ve trend grafiklerini anlat.
7. Flight mode ve telemetry paket mantigini anlat.
8. Servo kanatciklar paket icinde aktifse fin gorsellerinin ve fiziksel servolarin birlikte hareketini goster.
9. BMP180 aktifse altitude gorselini kullanarak barometrik yukseklik sunumunu yap.

---

## 12. ESP32 WiFi Köprüsü

Hedef mimari:

```text
STM32 UART binary telemetry
  -> ESP32 UART RX
  -> ESP32 WebSocket server
  -> Dashboard WiFi input
```

ESP32 kural:

- Paketleri parse etmeye calismamali.
- STM32'den gelen byte stream'i bozmadan WebSocket binary frame olarak dashboard'a forward etmeli.
- Dashboard parser zaten sync byte ve CRC ile frame'i cozer.

Tipik ESP32 dashboard URL:

```text
http://<dashboard-host>:5173/?ws=ws://192.168.4.1:81/telemetry
```

Kablolama:

| STM32 | ESP32 |
| --- | --- |
| UART TX | UART RX |
| GND | GND |

STM32 ve ESP32 3.3 V logic kullandigi icin level shifting gerekmez; ancak iki kartin GND'si ortak olmak zorundadir.

---

## 13. Sık Karşılaşılan Sorunlar

### 13.1. Dashboard "No Link" gosteriyor

Muhtemel nedenler:

- Serial port secilmedi.
- WebSocket URL yanlis.
- Relay veya ESP32 WebSocket server calismiyor.
- Browser izin vermedi.
- Firmware telemetry gondermiyor.

Cozum:

```bash
cd Tools/TelemetryDashboard
npm run serial:list
npm run serial:bridge -- --port=<dogru-port> --baud=115200
```

Sonra:

```text
http://127.0.0.1:5173/?ws=ws://127.0.0.1:8081/telemetry
```

### 13.2. Frame sayisi artiyor ama model hareket etmiyor

Muhtemel nedenler:

- Gelen paket sadece raw IMU iceriyor, attitude alanlari yok.
- Firmware `0x12` yerine eski `0x10` paketi gonderiyor.
- NavigationTask VehicleState yayinlamiyor.

Cozum:

- Packet type alanini kontrol et.
- Telemetry paket layout'unu `Docs/Telemetry/telemetry_dashboard_packet_plan.md` ile karsilastir.

### 13.3. Servo hareket etmiyor

Muhtemel nedenler:

- Servo harici 5 V ile beslenmiyor.
- GND ortak degil.
- TIM1 PWM baslamadi.
- ActuatorTask aktif komut almiyor.
- FlightControlTask GDS ActuatorCmd yayinlamiyor.

Cozum:

- PA8-PA11 pinlerini kontrol et.
- Servo beslemesini ayri kaynaktan ver.
- GDS ActuatorCmd topic'inin `bIsActive` ve `u32Sequence` alanlarini kontrol et.

### 13.4. I2C sensor okunmuyor

Muhtemel nedenler:

- SDA/SCL ters.
- Sensor 5 V ile beslendi ve zarar gordu.
- Pull-up yok veya bus kapasitansi yuksek.
- I2C adresi yanlis.
- Ortak GND yok.

Cozum:

- PB8/PB9 baglantisini kontrol et.
- I2C scanner veya driver health check ile adresleri dogrula.
- MPU6050 icin `0x68`, BMP180 icin `0x77` beklenir.

### 13.5. UART/RFD/ESP32 uzerinden veri gelmiyor

Muhtemel nedenler:

- STM32 TX ile modul RX capraz baglanmadi.
- GND ortak degil.
- USART2 pinleri ST-LINK VCP ile paylasiliyor.
- Baudrate farkli.
- Modul 5 V logic kullaniyor.

Cozum:

- Once USB serial ile telemetry'nin gercekten aktigini dogrula.
- Sonra ayni stream'i relay/ESP32/RFD yoluna tasima.
- Multimetre veya logic analyzer ile TX hattinda aktivite var mi kontrol et.

### 13.6. Build sirasinda `arm-none-eabi-gcc` bulunamiyor

Cozum:

- ARM GNU Toolchain kur.
- PATH icinde `arm-none-eabi-gcc` gorundugunu dogrula:

```bash
arm-none-eabi-gcc --version
```

### 13.7. Dashboard build hatasi

Cozum:

```bash
cd Tools/TelemetryDashboard
npm install
npm run build
```

Node surumu cok eskiyse guncelle.

---

## 14. Güvenlik ve Güvenilirlik Notları

Bu proje ders/demo prototipidir; yine de embedded security acisindan su onlemler tasarimda veya dokumantasyonda dikkate alinmalidir:

- Telemetry CRC hata tespiti icindir; kriptografik guvenlik saglamaz.
- WiFi/WebSocket uzerinden gelen paketler final sistemde HMAC/CMAC gibi bir mesaj dogrulama mekanizmasi ile korunmalidir.
- Replay saldirilarina karsi sequence/timestamp kontrolu uygulanmalidir.
- Actuator komutlari stale olursa servo safe/nötr pozisyona alinmalidir.
- Sensor verileri NaN/Inf, range ve rate-of-change kontrollerinden gecmelidir.
- Watchdog heartbeat ve stack watermark izleme final sistemde etkinlestirilmelidir.
- Demo disinda SWD/debug portu acik birakilmamalidir.
- ESP32 WiFi sifresi production secret gibi repo icinde tutulmamalidir.

---

## 15. Geliştirici Rehberi

### 15.1. Yeni sensor ekleme

1. `Drivers/<sensor>` altinda platform bagimsiz core driver yaz.
2. STM32 HAL port katmanini ayri dosyada tut.
3. Driver icinde HAL, FreeRTOS veya CMSIS header'i include etme.
4. App tarafina DDI adapter ekle.
5. SensorManager config'ine cihaz instance'ini ekle.
6. GDS'ye yeni topic veya mevcut topic publish et.
7. Telemetry packet layout'u gerekiyorsa guncelle.
8. Dashboard parser'i yalnizca yeni paket alanlarini okuyacak sekilde guncelle.

### 15.2. Yeni telemetry alani ekleme

Dogru sira:

```text
Producer task -> GDS topic -> TelemetryTask packet field -> Dashboard parser -> UI render
```

Yanlis sira:

```text
Dashboard icinde tahmin/uydurma veri -> UI render
```

Dashboard sadece gelen paketi gosterir.

### 15.3. Yeni actuator ekleme

1. Servo veya actuator driver core API'sini hazirla.
2. STM32 timer/GPIO port katmanini ayri tut.
3. DDI adapter ile `ActuatorManager` arayuzune bagla.
4. FlightControlTask'ten GDS ActuatorCmd yayinla.
5. ActuatorTask bu komutu fiziksel aktuatorlere uygulasin.

---

## 16. Mevcut Durum Kontrol Listesi

Bu checklist demo oncesi hizli kontrol icindir.

- [ ] Repo submodule'lari cekildi.
- [ ] Firmware Debug build geciyor.
- [ ] STM32 kart flash edildi.
- [ ] Dashboard `npm run dev` ile aciliyor.
- [ ] Web Serial veya WebSocket baglantisi kuruluyor.
- [ ] Frame count artiyor.
- [ ] Packet type beklenen protokol ile uyumlu.
- [ ] Roll/pitch/yaw fiziksel hareketle uyumlu.
- [ ] Servo gucu harici ve GND ortak.
- [ ] Telemetry UART DMA drop/error sayaclari demo sirasinda takip edilebilir.
- [ ] BMP180 aktif edilecekse platform init ve telemetry layout kontrol edildi.
- [ ] Demo icin yedek USB kablo, jumper, guc kaynagi ve breadboard hazir.

---

## 17. Önerilen Sunum Konuşma Akışı

1. "Bu proje, RTOS tabanli bir roket denge platformu prototipidir."
2. "STM32 uzerinde sensor acquisition, navigation, flight control, actuator ve telemetry task'lerini ayri ayri calistirdik."
3. "Sensor verileri dogrudan task'ler arasinda tasinmiyor; GDS dedigimiz publish/subscribe benzeri ortak veri alani uzerinden dagitiliyor."
4. "MPU6050 tarafinda event-driven DMA okuma ile CPU yukunu azalttik."
5. "Servo komutlari control task'ten cikiyor, GDS'ye yayinlaniyor, actuator task tarafindan fiziksel PWM'e cevriliyor."
6. "Telemetry task, GDS'deki son verileri binary paket haline getirip UART DMA ile dashboard'a gonderiyor."
7. "Dashboard herhangi bir kestirim yapmiyor; sadece gelen paketi parse edip roket modeli, sensor panelleri ve trend grafiklerinde gosteriyor."
8. "Bu sayede yazilim katmanlari ayrik, test edilebilir ve genisletilebilir bir hale geldi."

---

## 18. Referans Dosyalar

| Konu | Dosya |
| --- | --- |
| Telemetry dashboard | `Tools/TelemetryDashboard/README.md` |
| Telemetry packet plan | `Docs/Telemetry/telemetry_dashboard_packet_plan.md` |
| ESP32 bridge notlari | `Docs/Telemetry/esp32_wifi_bridge_notes.md` |
| GDS topic'leri | `App/SensorAcq/global_data_space.h` |
| Sensor manager | `App/SensorAcq/sensor_manager.c` |
| Telemetry task | `App/Telemetry/telemetry_task.c` |
| Platform RTOS binding | `Platform/STM32/app_platform_port.c` |
| MPU6050 driver | `Drivers/mpu6050` |
| BMP180 driver | `Drivers/bmp180` |
| Servo driver | `Drivers/servo` |

