#ifndef BMP180_DRIVER_H_
#define BMP180_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "driver_types.h"
#include "bmp180_hal.h"

#define BMP180_DRIVER_API_VERSION                 (0x0200U)   /* Public API versiyonudur; v2 bloklamasız ölçüm state machine desteğini bildirir. */

typedef enum
{
    BMP180_STATE_UNINIT = 0,                                /* Handle henüz Open ile başlatılmamıştır; public Read/Ioctl çoğu işlem reddedilir. */
    BMP180_STATE_READY,                                     /* Sensör kimliği ve kalibrasyonu doğrulanmıştır; Read çağrısı yapılabilir. */
    BMP180_STATE_SLEEP,                                     /* API tutarlılığı için ayrılmış uyku durumudur; BMP180'de ayrı sleep register yoktur. */
    BMP180_STATE_ERROR                                      /* Bus, chip-id veya kalibrasyon hatası sonrası deterministik hata durumudur. */
} te_Bmp180_State;

typedef enum
{
    BMP180_OSS0_ULTRA_LOW_POWER = 0,                        /* En hızlı ve en düşük güç tüketimli basınç ölçüm modudur. */
    BMP180_OSS1_STANDARD,                                   /* Standart basınç ölçüm modudur; hız/gürültü dengesi sağlar. */
    BMP180_OSS2_HIGH_RESOLUTION,                            /* Daha uzun dönüşüm süresiyle daha yüksek basınç çözünürlüğü sağlar. */
    BMP180_OSS3_ULTRA_HIGH_RESOLUTION                       /* En uzun dönüşüm süresiyle en düşük gürültülü basınç ölçüm modudur. */
} te_Bmp180_Oversampling;

typedef enum
{
    BMP180_MEASUREMENT_IDLE = 0,                            /* Aktif dönüşüm yoktur; Ioctl START_MEASUREMENT yeni sıcaklık dönüşümünü başlatabilir. */
    BMP180_MEASUREMENT_WAIT_TEMPERATURE,                    /* Sıcaklık dönüşümü donanım içinde sürmektedir; Process çağrısı deadline ve SCO bitini yoklar. */
    BMP180_MEASUREMENT_WAIT_PRESSURE                        /* Basınç dönüşümü donanım içinde sürmektedir; Process çağrısı hazır olduğunda telafiyi tamamlar. */
} te_Bmp180_MeasurementState;

typedef te_Driver_RetCode (*tpfn_Bmp180BusRead)(uint8_t u8DeviceAddr,
                                                 uint8_t u8RegisterAddr,
                                                 uint8_t *pu8ReadData,
                                                 uint16_t u16ReadLen,
                                                 uint32_t u32TimeoutMs,
                                                 void *vpCtx); /* Soyut bus okuma callback tipidir; driver STM32 HAL bilmeden register okur. */

typedef te_Driver_RetCode (*tpfn_Bmp180BusWrite)(uint8_t u8DeviceAddr,
                                                  uint8_t u8RegisterAddr,
                                                  const uint8_t *pu8WriteData,
                                                  uint16_t u16WriteLen,
                                                  uint32_t u32TimeoutMs,
                                                  void *vpCtx); /* Soyut bus yazma callback tipidir; port katmanı I2C/SPI ayrıntısını uygular. */

typedef te_Driver_RetCode (*tpfn_Bmp180DelayMs)(uint32_t u32DelayMs,
                                                 void *vpCtx); /* Open, soft-reset ve deployment test servisleri için platformdan enjekte edilen milisaniye gecikme callback'idir; runtime ölçümü bekletmez. */

typedef uint32_t (*tpfn_Bmp180GetTickMs)(void *vpCtx);        /* Ölçüm timestamp'i için platformdan enjekte edilen milisaniye sayaç callback'idir. */

typedef te_Driver_RetCode (*tpfn_Bmp180Lock)(uint32_t u32TimeoutMs,
                                              void *vpCtx);   /* Paylaşılan I2C bus'ı koruyan RTOS/bare-metal kilit callback tipidir. */

typedef te_Driver_RetCode (*tpfn_Bmp180Unlock)(void *vpCtx);  /* Paylaşılan bus kilidini bırakan callback tipidir; her transferden sonra çağrılır. */

typedef struct
{
    tpfn_Bmp180BusRead pfnRead;                              /* Platform bağımsız driver'ın register okumak için çağırdığı bus read callback'idir. */
    tpfn_Bmp180BusWrite pfnWrite;                            /* Platform bağımsız driver'ın register/komut yazmak için çağırdığı bus write callback'idir. */
    void *vpCtx;                                             /* Port katmanının I2C handle gibi platform bağımlı bağlamını taşıyan opaque pointer'dır. */
} ts_Bmp180_BusInterface;

typedef struct
{
    tpfn_Bmp180Lock pfnLock;                                 /* Her bus transferinden önce çağrılan opsiyonel kilit fonksiyonudur. */
    tpfn_Bmp180Unlock pfnUnlock;                             /* Her bus transferinden sonra çağrılan opsiyonel kilit bırakma fonksiyonudur. */
    void *vpCtx;                                             /* Mutex veya kritik bölüm nesnesini taşıyan port katmanı bağlam pointer'ıdır. */
} ts_Bmp180_LockInterface;

typedef struct
{
    tpfn_Bmp180DelayMs pfnDelayMs;                           /* Open, soft-reset ve deployment test servislerinde platform uyumlu bekleme yaptıran callback'tir; runtime ölçüm state machine'i kullanmaz. */
    tpfn_Bmp180GetTickMs pfnGetTickMs;                       /* Bloklamasız deadline takibi ve çıktı timestamp'i için zorunlu monotonik sistem tick callback'idir. */
    void *vpCtx;                                             /* Zamanlama callback'lerinin ihtiyaç duyabileceği platform bağlamıdır. */
} ts_Bmp180_TimingInterface;

typedef struct
{
    int16_t s16AC1;                                          /* BMP180 AC1 signed kalibrasyon katsayısıdır; basınç telafi algoritmasında kullanılır. */
    int16_t s16AC2;                                          /* BMP180 AC2 signed kalibrasyon katsayısıdır; B3 ara değişkenini etkiler. */
    int16_t s16AC3;                                          /* BMP180 AC3 signed kalibrasyon katsayısıdır; B4 ara değişkeni hesabında kullanılır. */
    uint16_t u16AC4;                                         /* BMP180 AC4 unsigned kalibrasyon katsayısıdır; basınç algoritmasında unsigned kalmalıdır. */
    uint16_t u16AC5;                                         /* BMP180 AC5 unsigned kalibrasyon katsayısıdır; sıcaklık telafisinin ölçek katsayısıdır. */
    uint16_t u16AC6;                                         /* BMP180 AC6 unsigned kalibrasyon katsayısıdır; sıcaklık telafisinin ofset katsayısıdır. */
    int16_t s16B1;                                           /* BMP180 B1 signed kalibrasyon katsayısıdır; basınç ikinci derece teriminde kullanılır. */
    int16_t s16B2;                                           /* BMP180 B2 signed kalibrasyon katsayısıdır; basınç ikinci derece teriminde kullanılır. */
    int16_t s16MB;                                           /* BMP180 MB signed kalibrasyon katsayısıdır; datasheet bloğunun tamamını saklamak için tutulur. */
    int16_t s16MC;                                           /* BMP180 MC signed kalibrasyon katsayısıdır; sıcaklık X2 teriminin payıdır. */
    int16_t s16MD;                                           /* BMP180 MD signed kalibrasyon katsayısıdır; sıcaklık X2 teriminin paydasında yer alır. */
} ts_Bmp180_CalibrationCoeffs;

typedef struct
{
    uint8_t u8I2cAddress;                                    /* BMP180 7-bit I2C adresidir; 0 verilirse driver varsayılan 0x77 adresini kullanır. */
    uint32_t u32BusTimeoutMs;                                /* Tek I2C transferi için milisaniye timeout değeridir; 0 verilirse driver varsayılanını seçer. */
    uint32_t u32BusLockTimeoutMs;                            /* Paylaşılan bus mutex'i için milisaniye bekleme süresidir; 0 verilirse varsayılan kullanılır. */
    te_Bmp180_Oversampling eOversampling;                    /* Basınç dönüşümünde kullanılacak BMP180 OSS modudur. */
    ts_Bmp180_BusInterface sBusInterface;                    /* STM32 HAL veya başka port katmanından enjekte edilen bus callback paketidir. */
    ts_Bmp180_LockInterface sLockInterface;                  /* FreeRTOS/CMSIS mutex veya bare-metal kritik bölüm callback paketidir. */
    ts_Bmp180_TimingInterface sTimingInterface;              /* Delay ve tick callback'lerini taşıyan platform zamanlama paketidir. */
    float f32SeaLevelPressurePa;                             /* Altitude hesabı için deniz seviyesi referans basıncıdır; 0 ise standart değer kullanılır. */
} ts_Bmp180_OpenConfig;

typedef struct
{
    float f32TemperatureC;                                   /* Ölçeklenmiş gerçek sıcaklıktır; uygulamaya Celsius biriminde sunulur. */
    float f32PressurePa;                                     /* Ölçeklenmiş gerçek basınçtır; uygulamaya Pascal SI biriminde sunulur. */
    float f32AltitudeM;                                      /* Deniz seviyesi referansına göre hesaplanan yaklaşık yüksekliktir; metre birimindedir. */
    uint32_t u32TimestampMs;                                 /* Ölçüm tamamlandığında timing callback'inden alınan sistem zamanıdır. */
    bool bValid;                                             /* Bu yapıdaki verilerin başarılı son Read çağrısından geldiğini gösteren bayraktır. */
} ts_Bmp180_Data;

typedef struct
{
    bool bBusOk;                                             /* CHIP_ID register'ı okunabildiyse true olur; bus transfer sağlığını gösterir. */
    bool bChipIdOk;                                          /* Okunan CHIP_ID beklenen 0x55 ise true olur; Class-A kimlik doğrulamasıdır. */
    uint8_t u8ChipIdValue;                                   /* Health-check sırasında okunan ham CHIP_ID değeridir. */
    bool bCalibrationOk;                                     /* Handle içindeki kalibrasyon katsayıları mantıklı görünüyorsa true olur. */
    te_Bmp180_State eState;                                  /* Health-check anındaki driver durum makinesi değeridir. */
} ts_Bmp180_HealthStatus;

typedef struct
{
    uint8_t u8RegisterAddr;                                  /* Tek register erişimi için başlangıç adresidir. */
    uint8_t u8Value;                                         /* Tek register okuma/yazma için veri baytıdır. */
} ts_Bmp180_RegisterAccess;

typedef struct
{
    uint8_t u8RegisterAddr;                                  /* Blok register erişimi için başlangıç adresidir. */
    uint8_t *pu8Buffer;                                      /* Blok okuma/yazma buffer pointer'ıdır; NULL olamaz. */
    uint16_t u16Length;                                      /* Blok erişim uzunluğudur; 0 olamaz ve caller buffer boyutuyla uyumlu olmalıdır. */
} ts_Bmp180_RegisterBlockAccess;

typedef struct
{
    te_Bmp180_State eState;                                  /* Instance durum makinesidir; global durum yerine handle içinde tutulur. */
    uint8_t u8I2cAddress;                                    /* Bu BMP180 instance'ının 7-bit I2C adresidir. */
    uint32_t u32BusTimeoutMs;                                /* Bus callback'lerine iletilen transfer timeout değeridir. */
    uint32_t u32BusLockTimeoutMs;                            /* Lock callback'ine iletilen mutex bekleme timeout değeridir. */
    te_Bmp180_Oversampling eOversampling;                    /* Bu instance için seçilmiş basınç oversampling modudur. */
    float f32SeaLevelPressurePa;                             /* Bu instance için altitude hesabında kullanılan basınç referansıdır. */
    ts_Bmp180_BusInterface sBusInterface;                    /* Bus erişimini soyutlayan callback paketidir; driver STM32 HAL bilmez. */
    ts_Bmp180_LockInterface sLockInterface;                  /* Paylaşılan bus erişimini koruyan callback paketidir. */
    ts_Bmp180_TimingInterface sTimingInterface;              /* Dönüşüm bekleme ve timestamp callback paketidir. */
    ts_Bmp180_CalibrationCoeffs sCalibrationCoeffs;          /* Open veya soft reset sonrası sensörden okunan kalibrasyon katsayılarıdır. */
    uint8_t u8LastChipId;                                    /* Son başarılı/başarısız health-check sırasında okunan CHIP_ID değeridir. */
    bool bLastBusOk;                                         /* Son sağlık kontrolünde bus okumasının başarılı olup olmadığını saklar. */
    bool bLastCalibrationOk;                                 /* Son kalibrasyon doğrulamasının sonucunu saklar. */
    te_Bmp180_MeasurementState eMeasurementState;            /* Bloklamasız ölçümün hangi datasheet aşamasında olduğunu handle içinde tutar. */
    uint32_t u32ConversionDeadlineMs;                         /* Process fonksiyonunun sensöre erken erişmesini önleyen minimum hazır olma zamanıdır. */
    uint32_t u32ConversionTimeoutMs;                          /* SCO biti beklenenden uzun süre aktif kalırsa deterministik timeout üretilecek zamandır. */
    int32_t s32PendingRawTemperature;                         /* Basınç telafisinde kullanılmak üzere tamamlanan son UT değeri state machine boyunca saklanır. */
    ts_Bmp180_Data sLatestData;                               /* Yalnız SensorTask bağlamında güncellenen son SI örneğidir; taskler arası atomik paylaşım GDS katmanında yapılır. */
} ts_Bmp180_Handle;

typedef enum
{
    BMP180_IOCTL_GET_VERSION = 0x00U,                        /* Argüman uint16_t*; public driver API versiyonunu döndürür. */
    BMP180_IOCTL_GET_STATE = 0x01U,                          /* Argüman te_Bmp180_State*; handle durum makinesini döndürür. */
    BMP180_IOCTL_CHECK_HEALTH = 0x10U,                       /* Argüman ts_Bmp180_HealthStatus*; bus, chip-id ve kalibrasyon sağlığını kontrol eder. */
    BMP180_IOCTL_SOFT_RESET = 0x11U,                         /* Argüman NULL; BMP180 soft reset komutu yollar ve kalibrasyonu yeniden okur. */
    BMP180_IOCTL_SET_OVERSAMPLING = 0x20U,                   /* Argüman te_Bmp180_Oversampling*; basınç OSS modunu ayarlar. */
    BMP180_IOCTL_GET_OVERSAMPLING = 0x21U,                   /* Argüman te_Bmp180_Oversampling*; mevcut OSS modunu döndürür. */
    BMP180_IOCTL_SET_SEA_LEVEL_PRESSURE = 0x22U,             /* Argüman float*; altitude hesabı için deniz seviyesi basıncını ayarlar. */
    BMP180_IOCTL_GET_SEA_LEVEL_PRESSURE = 0x23U,             /* Argüman float*; mevcut deniz seviyesi basınç referansını döndürür. */
    BMP180_IOCTL_START_MEASUREMENT = 0x24U,                  /* Argüman NULL; sıcaklık dönüşümünü başlatır ve beklemeden geri döner. */
    BMP180_IOCTL_PROCESS_MEASUREMENT = 0x25U,                /* Argüman NULL; bloklamasız state machine'i tek kısa adım ilerletir. */
    BMP180_IOCTL_GET_MEASUREMENT_STATE = 0x26U,              /* Argüman te_Bmp180_MeasurementState*; runtime ölçüm aşamasını döndürür. */
    BMP180_IOCTL_GET_CALIBRATION = 0x30U,                    /* Argüman ts_Bmp180_CalibrationCoeffs*; kalibrasyon katsayılarını kopyalar. */
    BMP180_IOCTL_REG_READ = 0x70U,                           /* Argüman ts_Bmp180_RegisterAccess*; tek register okur. */
    BMP180_IOCTL_REG_WRITE = 0x71U,                          /* Argüman ts_Bmp180_RegisterAccess*; tek register yazar. */
    BMP180_IOCTL_REG_READ_BLOCK = 0x72U,                     /* Argüman ts_Bmp180_RegisterBlockAccess*; blok register okur. */
    BMP180_IOCTL_REG_WRITE_BLOCK = 0x73U                     /* Argüman ts_Bmp180_RegisterBlockAccess*; blok register yazar. */
} te_Bmp180_IoctlCmd;

/**
 * @brief BMP180 instance'ını başlatır, CHIP_ID değerini doğrular ve kalibrasyon katsayılarını yükler.
 * @param psHandle Uygulamanın sahip olduğu BMP180 handle pointer'ıdır; NULL olamaz.
 * @param psConfig Bus, lock, timing ve ölçüm ayarlarını taşıyan open konfigürasyonudur; NULL olamaz.
 * @return DRIVER_OK başarıyı, diğer te_Driver_RetCode değerleri deterministik hata nedenini bildirir.
 */
te_Driver_RetCode Bmp180_Open(ts_Bmp180_Handle *psHandle, const ts_Bmp180_OpenConfig *psConfig);

/**
 * @brief BMP180 instance'ını uygulama açısından kapatır ve handle durumunu UNINIT yapar.
 * @param psHandle Uygulamanın sahip olduğu BMP180 handle pointer'ıdır.
 * @return DRIVER_OK başarıyı, hata kodları geçersiz pointer veya durum hatasını bildirir.
 */
te_Driver_RetCode Bmp180_Close(ts_Bmp180_Handle *psHandle);

/**
 * @brief Son tamamlanan bloklamasız ölçümü SI birimleriyle beklemeden kopyalar.
 * @param psHandle Open ile READY yapılmış BMP180 handle pointer'ıdır.
 * @param psOutData Ölçüm çıktısının yazılacağı uygulama buffer'ıdır; NULL olamaz.
 * @return DRIVER_OK başarıyı bildirir; henüz tamamlanmış örnek yoksa çıktı bValid=false olur.
 */
te_Driver_RetCode Bmp180_Read(ts_Bmp180_Handle *psHandle, ts_Bmp180_Data *psOutData);

/**
 * @brief POSIX ilhamlı API yüzeyi için ayrılmış genel write girişidir.
 * @param psHandle BMP180 handle pointer'ıdır.
 * @param vpInData BMP180 için kullanılmayan uygulama verisidir.
 * @return DRIVER_ERR_NOT_SUPPORTED çünkü BMP180 veri yazma modeli Ioctl ve dahili komutlarla yönetilir.
 */
te_Driver_RetCode Bmp180_Write(ts_Bmp180_Handle *psHandle, const void *vpInData);

/**
 * @brief Konfigürasyon, health-check, soft reset ve debug register erişim komutlarını yürütür.
 * @param psHandle BMP180 handle pointer'ıdır; GET_STATE dışında Open sonrası geçerli durumda olmalıdır.
 * @param eCmd Çalıştırılacak BMP180 ioctl komutudur.
 * @param vpArg Komutun beklediği tipe sahip argüman pointer'ıdır; komuta göre NULL olabilir veya olamaz.
 * @return DRIVER_OK başarıyı, diğer hata kodları komut sözleşmesi veya transfer hatasını bildirir.
 */
te_Driver_RetCode Bmp180_Ioctl(ts_Bmp180_Handle *psHandle, te_Bmp180_IoctlCmd eCmd, void *vpArg);

/**
 * @brief Health-check ve örnek ölçüm yaparak BMP180 instance'ının temel çalışma testini yürütür.
 * @param psHandle Open ile başlatılmış BMP180 handle pointer'ıdır.
 * @param u32TimeoutMs Deployment test polling döngüsünün bounded milisaniye bütçesidir; 0 ise güvenli varsayılan seçilir.
 * @return DRIVER_OK başarıyı, mantıksız ölçüm veya health-check hatası ilgili hata kodunu bildirir.
 */
te_Driver_RetCode Bmp180_Test(ts_Bmp180_Handle *psHandle, uint32_t u32TimeoutMs);

#ifdef __cplusplus
}
#endif

#endif /* BMP180_DRIVER_H_ */
