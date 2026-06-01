#ifndef BMP180_HAL_H_
#define BMP180_HAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Bu dosya BMP180 datasheet/register-map katmanıdır; STM32 HAL, FreeRTOS veya sürücü durum makinesi içermez. */
#define BMP180_HAL_API_VERSION_MAJOR              (1U)        /* HAL/register-map katmanı ana versiyonudur; datasheet sabitleri değişirse izlenebilirlik sağlar. */
#define BMP180_HAL_API_VERSION_MINOR              (0U)        /* HAL/register-map katmanı küçük versiyonudur; geriye uyumlu eklemeleri takip eder. */
#define BMP180_HAL_API_VERSION_PATCH              (0U)        /* HAL/register-map katmanı yama versiyonudur; yorum veya düzeltme seviyesini gösterir. */
#define BMP180_HAL_API_VERSION                    (0x0100U)   /* Platform bağımsız driver'ın okuyabileceği kompakt HAL API versiyon değeridir. */

/* I2C adres sabitleri BMP180 datasheet bilgisidir; STM32 port katmanı bu 7-bit adresi HAL formatına çevirir. */
#define BMP180_I2C_ADDR_DEFAULT                   (0x77U)     /* BMP180'in SDO piniyle seçilmeyen sabit 7-bit I2C adresidir. */

/* Register map sabitleri BMP180 datasheet kaynaklıdır; platform bağımsız driver bu adreslerle soyut bus callback'lerini çağırır. */
#define BMP180_REG_CALIB_START                    (0xAAU)     /* Kalibrasyon katsayılarının başladığı register adresidir; driver 22 baytı buradan blok olarak okur. */
#define BMP180_REG_CALIB_AC1_MSB                  (0xAAU)     /* AC1 katsayısının MSB adresidir; byte-byte big-endian parse için ayrı izlenebilirlik sağlar. */
#define BMP180_REG_CALIB_AC1_LSB                  (0xABU)     /* AC1 katsayısının LSB adresidir; sistem endian'ına güvenmeden parse edilir. */
#define BMP180_REG_CALIB_AC2_MSB                  (0xACU)     /* AC2 katsayısının MSB adresidir; sıcaklık/basınç telafisinde datasheet girdisidir. */
#define BMP180_REG_CALIB_AC2_LSB                  (0xADU)     /* AC2 katsayısının LSB adresidir; blok okuma içindeki offset'i belgelemek için tanımlıdır. */
#define BMP180_REG_CALIB_AC3_MSB                  (0xAEU)     /* AC3 katsayısının MSB adresidir; basınç hesap algoritmasında kullanılır. */
#define BMP180_REG_CALIB_AC3_LSB                  (0xAFU)     /* AC3 katsayısının LSB adresidir; kalibrasyon register kapsamasını tamamlar. */
#define BMP180_REG_CALIB_AC4_MSB                  (0xB0U)     /* AC4 katsayısının MSB adresidir; unsigned katsayı olduğu için özel parse edilir. */
#define BMP180_REG_CALIB_AC4_LSB                  (0xB1U)     /* AC4 katsayısının LSB adresidir; B4 ara değişkeninin temel girdisidir. */
#define BMP180_REG_CALIB_AC5_MSB                  (0xB2U)     /* AC5 katsayısının MSB adresidir; ham sıcaklıktan gerçek sıcaklığa geçişte kullanılır. */
#define BMP180_REG_CALIB_AC5_LSB                  (0xB3U)     /* AC5 katsayısının LSB adresidir; big-endian parse için açıkça tutulur. */
#define BMP180_REG_CALIB_AC6_MSB                  (0xB4U)     /* AC6 katsayısının MSB adresidir; UT sıcaklık ofsetinin datasheet katsayısıdır. */
#define BMP180_REG_CALIB_AC6_LSB                  (0xB5U)     /* AC6 katsayısının LSB adresidir; kalibrasyon bütünlüğü kontrolünde kullanılır. */
#define BMP180_REG_CALIB_B1_MSB                   (0xB6U)     /* B1 katsayısının MSB adresidir; basınç telafi denkleminde yer alır. */
#define BMP180_REG_CALIB_B1_LSB                   (0xB7U)     /* B1 katsayısının LSB adresidir; register kapsamasını eksiksiz tutar. */
#define BMP180_REG_CALIB_B2_MSB                   (0xB8U)     /* B2 katsayısının MSB adresidir; B3 ara değişkeni için datasheet girdisidir. */
#define BMP180_REG_CALIB_B2_LSB                   (0xB9U)     /* B2 katsayısının LSB adresidir; byte sıralamasını görünür kılar. */
#define BMP180_REG_CALIB_MB_MSB                   (0xBAU)     /* MB katsayısının MSB adresidir; datasheet kalibrasyon bloğunun parçasıdır. */
#define BMP180_REG_CALIB_MB_LSB                   (0xBBU)     /* MB katsayısının LSB adresidir; bazı örnek algoritmalarda kullanılmasa da saklanır. */
#define BMP180_REG_CALIB_MC_MSB                   (0xBCU)     /* MC katsayısının MSB adresidir; sıcaklık telafi denkleminde bölünen terimdir. */
#define BMP180_REG_CALIB_MC_LSB                   (0xBDU)     /* MC katsayısının LSB adresidir; kalibrasyon bloğu parse sırasını tamamlar. */
#define BMP180_REG_CALIB_MD_MSB                   (0xBEU)     /* MD katsayısının MSB adresidir; sıcaklık telafi denkleminde payda tarafında yer alır. */
#define BMP180_REG_CALIB_MD_LSB                   (0xBFU)     /* MD katsayısının LSB adresidir; sıfır payda riskini doğrulamak için saklanır. */
#define BMP180_REG_CHIP_ID                        (0xD0U)     /* CHIP_ID register adresidir; Class-A sensör health-check için driver tarafından okunur. */
#define BMP180_REG_VERSION                        (0xD1U)     /* VERSION register adresidir; debug/izlenebilirlik amaçlı register erişimine açılır. */
#define BMP180_REG_SOFT_RESET                     (0xE0U)     /* SOFT_RESET register adresidir; reset komutu yazıldığında sensör iç durumu sıfırlanır. */
#define BMP180_REG_CONTROL                        (0xF4U)     /* CONTROL register adresidir; sıcaklık veya basınç dönüşüm komutu buraya yazılır. */
#define BMP180_REG_OUT_MSB                        (0xF6U)     /* OUT_MSB output register adresidir; dönüşüm sonucunun en anlamlı baytını taşır. */
#define BMP180_REG_OUT_LSB                        (0xF7U)     /* OUT_LSB output register adresidir; dönüşüm sonucunun orta/düşük baytını taşır. */
#define BMP180_REG_OUT_XLSB                       (0xF8U)     /* OUT_XLSB output register adresidir; basınç ölçümünde oversampling'e bağlı ek bitleri taşır. */

/* Datasheet kimlik ve komut sabitleri platform bağımsız driver'ın sensörü doğru tanımasını sağlar. */
#define BMP180_CHIP_ID_EXPECTED                   (0x55U)     /* BMP180 CHIP_ID register'ından beklenen değerdir; yanlış cihaz veya bus hatasını ayırt eder. */
#define BMP180_CMD_READ_TEMP                      (0x2EU)     /* Sıcaklık dönüşüm komutudur; driver CONTROL register'a yazar ve 5 ms bekler. */
#define BMP180_CMD_READ_PRESSURE                  (0x34U)     /* Basınç dönüşüm temel komutudur; driver OSS bitlerini bu değere ekler. */
#define BMP180_CMD_SOFT_RESET                     (0xB6U)     /* Soft reset komutudur; driver SOFT_RESET register'a yazar ve kalibrasyonu tekrar yükler. */

/* Uzunluk sabitleri buffer taşmalarını önlemek ve register blok erişimini açık tutmak için kullanılır. */
#define BMP180_CALIB_DATA_LENGTH                  (22U)       /* AC1..MD arasındaki kalibrasyon bloğunun toplam bayt sayısıdır. */
#define BMP180_RAW_TEMP_DATA_LENGTH               (2U)        /* Ham sıcaklık sonucunun OUT_MSB ve OUT_LSB olarak iki bayt uzunluğudur. */
#define BMP180_RAW_PRESS_DATA_LENGTH              (3U)        /* Ham basınç sonucunun OUT_MSB, OUT_LSB ve OUT_XLSB olarak üç bayt uzunluğudur. */

/* Zamanlama sabitleri datasheet dönüşüm süreleridir; driver bu beklemeleri timing callback'i ile yapar. */
#define BMP180_TEMP_CONV_TIME_MS                  (5U)        /* Sıcaklık dönüşümü için gereken tipik/maksimum güvenli bekleme süresidir. */
#define BMP180_PRESS_CONV_TIME_OSS0_MS            (5U)        /* OSS0 ultra low power basınç dönüşümü için bekleme süresidir. */
#define BMP180_PRESS_CONV_TIME_OSS1_MS            (8U)        /* OSS1 standard basınç dönüşümü için bekleme süresidir. */
#define BMP180_PRESS_CONV_TIME_OSS2_MS            (14U)       /* OSS2 high resolution basınç dönüşümü için bekleme süresidir. */
#define BMP180_PRESS_CONV_TIME_OSS3_MS            (26U)       /* OSS3 ultra high resolution basınç dönüşümü için bekleme süresidir. */
#define BMP180_SOFT_RESET_DELAY_MS                (10U)       /* Soft reset sonrası kalibrasyon register'ları tekrar okunmadan önce verilen güvenli beklemedir. */
#define BMP180_POWER_ON_DELAY_MS                  (5U)        /* Open sırasında sensörün I2C cevaplarının kararlı olması için kısa başlangıç beklemesidir. */

/* Ölçekleme ve fizik sabitleri SI birimli çıktının driver içinde hesaplanmasını sağlar. */
#define BMP180_SEA_LEVEL_PRESSURE_PA              (101325.0f) /* Standart deniz seviyesi basıncıdır; altitude hesabı için varsayılan referanstır. */
#define BMP180_ALTITUDE_EXPONENT                  (0.19029495f) /* Barometrik yükseklik formülündeki 1/5.255 üssüdür; powf ile kullanılır. */
#define BMP180_ALTITUDE_SCALE_M                  (44330.0f)   /* Barometrik yükseklik formülündeki metre ölçek katsayısıdır. */

/* Oversampling alanı datasheet CONTROL komutunun 7:6 bitleridir; driver enum değerini komut baytına dönüştürür. */
#define BMP180_OSS_SHIFT                          (6U)        /* OSS bitlerinin CONTROL komut baytı içindeki sola kaydırma miktarıdır. */
#define BMP180_OSS_MASK                           (0x03U)     /* Geçerli oversampling değerlerini 0..3 ile sınırlar. */
#define BMP180_OSS_MIN                            (0U)        /* En düşük oversampling modudur; en kısa dönüşüm süresini verir. */
#define BMP180_OSS_MAX                            (3U)        /* En yüksek oversampling modudur; en uzun ama en düşük gürültülü dönüşümü verir. */
#define BMP180_BUILD_PRESSURE_CMD(oss_)           ((uint8_t)(BMP180_CMD_READ_PRESSURE + (((uint8_t)(oss_) & BMP180_OSS_MASK) << BMP180_OSS_SHIFT))) /* Basınç komutunu datasheet formülüne göre üretir. */

/* Plausibility limitleri Bmp180_Test için geniş güvenlik aralıklarıdır; uygulama kalibrasyonu yerine health-check amacı taşır. */
#define BMP180_TEMP_MIN_C                         (-40.0f)    /* BMP180 çalışma sıcaklığı alt sınırıdır; testte bariz hatalı ölçümü yakalar. */
#define BMP180_TEMP_MAX_C                         (85.0f)     /* BMP180 çalışma sıcaklığı üst sınırıdır; testte yanlış byte/kalibrasyon okumasını yakalar. */
#define BMP180_PRESSURE_MIN_PA                    (30000.0f)  /* Çok geniş basınç alt sınırıdır; normal atmosfer dışındaki hatalı hesapları ayıklar. */
#define BMP180_PRESSURE_MAX_PA                    (120000.0f) /* Çok geniş basınç üst sınırıdır; hatalı kalibrasyon veya byte sırası sorununu yakalar. */
#define BMP180_SEA_LEVEL_PRESSURE_MIN_PA          (30000.0f)  /* Kullanıcıdan alınan deniz seviyesi referansı için savunmacı alt sınırdır. */
#define BMP180_SEA_LEVEL_PRESSURE_MAX_PA          (120000.0f) /* Kullanıcıdan alınan deniz seviyesi referansı için savunmacı üst sınırdır. */

typedef struct
{
    uint8_t u8Command : 6;                               /* CONTROL register'ın komut alanıdır; sıcaklık veya basınç dönüşüm tipini taşır. */
    uint8_t u8Oss : 2;                                   /* CONTROL register'ın oversampling alanıdır; yalnızca basınç dönüşüm komutunda anlamlıdır. */
} ts_Bmp180_RegControlBits;

typedef union
{
    uint8_t u8Value;                                     /* Register'ın ham 8-bit değeridir; driver command byte üretirken doğrudan kullanabilir. */
    ts_Bmp180_RegControlBits sBits;                      /* Eğitim amaçlı bit alanı görünümüdür; datasheet komut byte yapısını okunur yapar. */
} tu_Bmp180_RegControl;

typedef struct
{
    uint8_t u8ResetCode : 8;                             /* SOFT_RESET register'a yazılan komut değeridir; sadece 0xB6 anlamlı kabul edilir. */
} ts_Bmp180_RegSoftResetBits;

typedef union
{
    uint8_t u8Value;                                     /* Soft reset register'ına yazılacak ham komut baytıdır. */
    ts_Bmp180_RegSoftResetBits sBits;                    /* Eğitim amaçlı tek alanlı bit modelidir; write-only register davranışını görünür kılar. */
} tu_Bmp180_RegSoftReset;

typedef struct
{
    uint8_t u8MlVersion : 4;                             /* VERSION register alt nibble alanıdır; datasheet revizyon bilgisini ham biçimde taşır. */
    uint8_t u8AlVersion : 4;                             /* VERSION register üst nibble alanıdır; debug amaçlı register modellemesini tamamlar. */
} ts_Bmp180_RegVersionBits;

typedef union
{
    uint8_t u8Value;                                     /* VERSION register'ın ham 8-bit değeridir; ioctl register okuma ile görülebilir. */
    ts_Bmp180_RegVersionBits sBits;                      /* VERSION register'ın nibble düzeyindeki eğitim modelidir. */
} tu_Bmp180_RegVersion;

#ifdef __cplusplus
}
#endif

#endif /* BMP180_HAL_H_ */
