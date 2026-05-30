# MATLAB Live Telemetry (Sifirdan)

Bu klasor, STM32 firmware tarafindaki `telemetry_task` cikisini seri hat uzerinden okuyup canli gostermek icin sifirdan yazilmis yapidir.

Eski script `Docs/Telemetry/gnc_telem_live_parser.m` bu akisin parcasi degildir.

## Paket formati (firmware ile birebir)

Kaynaklar:
- `App/Telemetry/telemetry_task.h`
- `App/Telemetry/telemetry_task.c`
- `Platform/STM32/app_platform_port.c`

Frame boyutu: **63 byte**

| Offset | Uzunluk | Alan |
|---|---:|---|
| 0 | 1 | Sync0 (`0xA5`) |
| 1 | 1 | Sync1 (`0x5A`) |
| 2 | 1 | MsgId (`0x12`) |
| 3 | 1 | Sequence |
| 4 | 4 | Timestamp (uint32 LE, ms) |
| 8 | 28 | IMU payload (7 x float32 LE) |
| 36 | 24 | Vehicle payload (6 x float32 LE) |
| 60 | 1 | isEstimated (`0/1`) |
| 61 | 2 | CRC16/CCITT-FALSE (LE), data araligi: bytes `0..60` |

UART: **115200 8N1**, telemetry period: **10 Hz**.

## Calistirma

MATLAB'da repo kokune gecip:

```matlab
run_live_telemetry
```

Isterseniz portu sabitleyin:

```matlab
run_live_telemetry("Port", "/dev/cu.usbserial-1410")
```

Isterseniz plot hizini/pençereyi degistirin:

```matlab
run_live_telemetry("PlotHz", 10, "WindowSec", 12, "MaxPlotPoints", 300)
```

## Performans optimizasyonlari

- Parse ve draw ayrik: parser tum frame'leri alir, cizim `PlotHz` ile throttled.
- Cizimde maksimum nokta sayisi sinirli (`MaxPlotPoints`) oldugu icin UI kasmasi azalir.
- Ring buffer kullanimiyla sabit bellek.
- `drawnow limitrate nocallbacks` ile GUI event maliyeti dusurulur.
- Seri hattan veri yoksa kisa `pause` ile CPU spin onlenir.

## Test scriptleri

`matlab/tests` altinda:
- `test_crc`
- `test_decode_frame`
- `test_parser_resync`

MATLAB'da:

```matlab
test_crc
test_decode_frame
test_parser_resync
```

## Sorun giderme

- Port gorunmuyorsa USB-UART driver ve kabloyu kontrol edin.
- Hic frame gelmiyorsa firmware tarafinda IMU verisinin `valid` oldugunu dogrulayin.
- CRC drop yuksekse baud, GND ortakligi ve kablo kalitesini kontrol edin.
