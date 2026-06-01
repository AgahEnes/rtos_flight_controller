#ifndef SERVO_DRIVER_H_
#define SERVO_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "servo_hal.h"

/* MPU6050 header'ı bu global dönüş tipini zaten tanımladıysa servo header tekrar tanımlamaz; amaç aynı proje içinde tek dönüş sözleşmesini paylaşmaktır. */
#if !defined(MPU6050_DRIVER_H_) && !defined(SERVO_DRIVER_RETCODE_DEFINED)
#define SERVO_DRIVER_RETCODE_DEFINED
typedef enum
{
    DRIVER_OK = 0,             /* İşlem başarılı; tüm sürücü katmanlarında ortak pozitif sonuç kodudur. */
    DRIVER_ERR_NULL_PTR,       /* NULL pointer hatası; MISRA benzeri savunmacı API kontrolü tarafından döndürülür. */
    DRIVER_ERR_INVALID_ARG,    /* Geçersiz değer hatası; açı, pulse veya ioctl argümanı sözleşmeye uymadığında kullanılır. */
    DRIVER_ERR_STATE,          /* Durum makinesi hatası; örneğin UNINIT veya ERROR durumunda komut verilirse kullanılır. */
    DRIVER_ERR_BUS,            /* Bus/peripheral erişim hatası; servo port katmanında timer/PWM erişim hatasını temsil eder. */
    DRIVER_ERR_TIMEOUT,        /* Kilit veya port işlemi zaman aşımına uğradı; RTOS kaynak yönetimi için kullanılır. */
    DRIVER_ERR_CONFIG,         /* Open konfigürasyonu eksik veya tutarsız; dependency injection doğrulaması için kullanılır. */
    DRIVER_ERR_NOT_SUPPORTED,  /* API yüzeyinde bulunan fakat bu servo sınıfında anlamlı olmayan işlem için kullanılır. */
    DRIVER_ERR_WHOAMI,         /* Class-A sensörlerden miras kalan ortak kod; servo Class-B olduğu için normalde kullanılmaz. */
    DRIVER_ERR_IO              /* Genel I/O hatası; port katmanı ayrıntıyı sınıflandıramadığında kullanılır. */
} te_Driver_RetCode;
#endif

typedef te_Driver_RetCode (*tpfn_ServoPulseWrite)(uint32_t u32PulseWidthUs, void *vpCtx);
/* PWM port callback'i; çekirdek driver'ın hesapladığı mikro-saniye darbesini STM32 timer CCR değerine çevirmek için port katmanına gider. */

typedef te_Driver_RetCode (*tpfn_ServoLock)(uint32_t u32TimeoutMs, void *vpCtx);
/* RTOS kilit callback'i; aynı timer veya actuator kaynağına birden fazla task eriştiğinde kritik bölgeyi dış katmandan korur. */

typedef te_Driver_RetCode (*tpfn_ServoUnlock)(void *vpCtx);
/* RTOS kilit bırakma callback'i; çekirdek driver lock nesnesini bilmeden kaynak korumasını tamamlar. */

typedef uint32_t (*tpfn_ServoGetTickMs)(void *vpCtx);
/* Zaman damgası callback'i; slew-rate sınırlaması için geçen süreyi platform bağımsız olarak ölçer. */

typedef struct
{
    tpfn_ServoPulseWrite pfnPulseWrite;  /* PWM yazma fonksiyonu; çekirdek driver'ın tek donanım çıkış kapısıdır. */
    void *vpCtx;                         /* PWM port bağlamı; STM32 tarafında timer handle, kanal ve opsiyonel mutex burada taşınır. */
} ts_Servo_PulseInterface;

typedef struct
{
    tpfn_ServoLock pfnLock;       /* Kaynak kilitleme fonksiyonu; RTOS mutex işlemini çekirdek driver'dan ayırır. */
    tpfn_ServoUnlock pfnUnlock;   /* Kaynak kilidi bırakma fonksiyonu; hata yolunda bile çağrılarak kaynak sızıntısını önler. */
    void *vpCtx;                  /* Kilit bağlamı; FreeRTOS/CMSIS nesnesi port katmanında bu pointer ile taşınır. */
} ts_Servo_LockInterface;

typedef struct
{
    tpfn_ServoGetTickMs pfnGetTickMs;  /* Sistem tick okuma fonksiyonu; özellikle slew-rate hesabında kullanılır. */
    void *vpCtx;                       /* Zamanlama bağlamı; STM32 HAL tick için genelde NULL bırakılır. */
} ts_Servo_TimingInterface;

typedef struct
{
    float f32MinAngleRad;                    /* Fiziksel minimum servo açısı; kontrol komutları bu radyan sınırına kelepçelenir. */
    float f32MaxAngleRad;                    /* Fiziksel maksimum servo açısı; mekanik bağlantı ve dişli koruması için zorunludur. */
    uint32_t u32MinPulseUs;                  /* Minimum PWM darbesi; fiziksel minimum açının timer portuna gönderilecek karşılığıdır. */
    uint32_t u32MaxPulseUs;                  /* Maksimum PWM darbesi; fiziksel maksimum açının timer portuna gönderilecek karşılığıdır. */
    float f32AngleOffsetRad;                 /* Kalibrasyon ofseti; mekanik sıfır ile kontrol sıfırı arasındaki farkı düzeltir. */
    uint32_t u32LockTimeoutMs;               /* Mutex bekleme süresi; actuator task'ın paylaşılan PWM kaynağında bloklanmasını sınırlar. */
    bool bEnableSlewRate;                    /* Slew-rate mekanizmasını açar; kanatçıkların bir çevrimde aşırı hızlı hareket etmesini önler. */
    float f32MaxSlewRateRadPerSec;           /* Maksimum açısal hız; yapısal yük ve akım çekişini sınırlamak için rad/s cinsindedir. */
    ts_Servo_PulseInterface sPulseInterface; /* Donanım PWM yazma arayüzü; STM32 HAL bağımlılığı bu callback'in arkasında saklanır. */
    ts_Servo_LockInterface sLockInterface;   /* Opsiyonel RTOS kaynak koruma arayüzü; çekirdek driver mutex tipi içermez. */
    ts_Servo_TimingInterface sTimingInterface; /* Opsiyonel zaman arayüzü; tick kaynağı platformdan enjekte edilir. */
} ts_Servo_OpenConfig;

typedef struct
{
    float f32MinAngleRad;    /* Yeni fiziksel minimum açı; Ioctl ile çalışma sırasında güvenli limit güncellemek için kullanılır. */
    float f32MaxAngleRad;    /* Yeni fiziksel maksimum açı; kontrol yüzeyinin izin verilen en büyük pozisyonunu tanımlar. */
    uint32_t u32MinPulseUs;  /* Yeni minimum darbe; kalibrasyon veya farklı servo modeli için PWM alt sınırını taşır. */
    uint32_t u32MaxPulseUs;  /* Yeni maksimum darbe; kalibrasyon veya farklı servo modeli için PWM üst sınırını taşır. */
} ts_Servo_Limits;

typedef struct
{
    bool bEnable;                    /* Slew-rate sınırlamasını açma/kapama bayrağı; kontrolcü testlerinde dinamik değiştirilebilir. */
    float f32MaxSlewRateRadPerSec;   /* Maksimum açısal hız; pozitif değilse konfigürasyon geçersiz sayılır. */
} ts_Servo_SlewRateConfig;

typedef struct
{
    uint32_t u32PulseWidthUs;  /* Ham PWM darbe genişliği; Ioctl override komutuyla doğrudan test amaçlı yazılır. */
} ts_Servo_RawPulseCommand;

typedef struct
{
    float f32Percent;  /* -100 ile +100 arası fiziksel limit yüzdesi; actuator testinde açı hesabı yerine kullanılabilir. */
} ts_Servo_PercentCommand;

typedef struct
{
    bool bPeripheralConfigured;  /* Port callback'leri ve limitler geçerliyse true olur; Class-B health-check için kullanılır. */
    bool bLastWriteOk;           /* Son PWM yazma denemesi başarılıysa true olur; actuator task arıza takibi için kullanılır. */
    te_Servo_State eState;       /* Güncel durum makinesi değeri; telemetry veya debug ekranında doğrudan gösterilebilir. */
    float f32CurrentAngleRad;    /* Sürücünün son uyguladığı kelepçelenmiş servo açısı; SI birimi radyandır. */
    float f32TargetAngleRad;     /* Uygulamanın en son istediği hedef açı; saturasyon öncesi komut izlenebilirliği sağlar. */
    uint32_t u32CurrentPulseUs;  /* Sürücünün son ürettiği PWM darbesi; port katmanı timer tick'e çevirmeden önceki değerdir. */
    uint32_t u32LastWriteTickMs; /* Son başarılı yazmanın zaman damgası; slew-rate ve telemetry teşhisi için kullanılır. */
} ts_Servo_Status;

typedef struct
{
    bool bPulseCallbackOk;       /* PWM yazma callback'i bağlıysa true olur; Class-B peripheral erişim kapısının varlığını gösterir. */
    bool bLimitConfigOk;         /* Açı/pulse limitleri tutarlıysa true olur; mekanik koruma konfigürasyonunun sağlıklı olduğunu gösterir. */
    bool bSlewConfigOk;          /* Slew-rate konfigürasyonu kullanılabilir durumdaysa true olur; hız sınırlama güvenliğini raporlar. */
    bool bLastWriteOk;           /* Son port yazması başarılıysa true olur; runtime actuator komut yolunun güncel durumunu gösterir. */
    te_Servo_State eState;       /* Güncel state machine değeri; health-check sonucunun hangi çalışma durumunda alındığını belirtir. */
} ts_Servo_HealthStatus;

typedef struct
{
    te_Servo_State eState;                   /* Sürücü durum makinesi; her servo instance'ı kendi handle'ında durum taşır. */
    float f32MinAngleRad;                    /* Kopyalanmış minimum açı; Open sonrası config ömrüne bağımlı kalmamak için handle'da tutulur. */
    float f32MaxAngleRad;                    /* Kopyalanmış maksimum açı; saturation hesabının instance'a özgü parametresidir. */
    uint32_t u32MinPulseUs;                  /* Kopyalanmış minimum pulse; angle-to-PWM eşlemesinin alt noktasıdır. */
    uint32_t u32MaxPulseUs;                  /* Kopyalanmış maksimum pulse; angle-to-PWM eşlemesinin üst noktasıdır. */
    float f32AngleOffsetRad;                 /* Kopyalanmış mekanik kalibrasyon ofseti; her komutta hedef açıya uygulanır. */
    uint32_t u32LockTimeoutMs;               /* Kopyalanmış kilit timeout değeri; lock callback çağrısına her yazmada verilir. */
    bool bEnableSlewRate;                    /* Aktif slew-rate durumu; Ioctl ile çalışma zamanında değiştirilebilir. */
    float f32MaxSlewRateRadPerSec;           /* Aktif maksimum açısal hız; hedef açının çevrimden çevrime sınırlanmasını sağlar. */
    float f32CurrentAngleRad;                /* Son uygulanan kelepçelenmiş açı; bir sonraki slew-rate hesabının başlangıç noktasıdır. */
    float f32TargetAngleRad;                 /* Son istenen komut açısı; telemetry ve hata ayıklama için saklanır. */
    uint32_t u32CurrentPulseUs;              /* Son hesaplanan mikro-saniye darbesi; ham PWM gözlemi için saklanır. */
    uint32_t u32LastWriteTickMs;             /* Son başarılı yazma zamanı; tick wrap-around'a uygun unsigned fark ile kullanılır. */
    bool bHasLastWriteTick;                  /* İlk yazmada slew-rate hesabının geçersiz zaman farkı kullanmasını engeller. */
    bool bLastWriteOk;                       /* Son port callback sonucunu saklar; Class-B health-check durumuna katkı verir. */
    ts_Servo_PulseInterface sPulseInterface; /* Kopyalanmış PWM port arayüzü; çekirdek driver STM32 HAL'i doğrudan include etmez. */
    ts_Servo_LockInterface sLockInterface;   /* Kopyalanmış lock arayüzü; çoklu task erişimi için dışarıdan enjekte edilir. */
    ts_Servo_TimingInterface sTimingInterface; /* Kopyalanmış zaman arayüzü; slew-rate ve status timestamp'i için kullanılır. */
} ts_Servo_Handle;

typedef enum
{
    SERVO_IOCTL_GET_VERSION = 0x00U,        /* API sürümünü uint16_t olarak döndürür; entegrasyon uyumluluğu kontrolü içindir. */
    SERVO_IOCTL_GET_STATE = 0x01U,          /* Güncel te_Servo_State değerini döndürür; durum makinesi gözlemi içindir. */
    SERVO_IOCTL_GET_STATUS = 0x02U,         /* ts_Servo_Status doldurur; telemetry/debug için bütüncül snapshot sağlar. */
    SERVO_IOCTL_CHECK_HEALTH = 0x10U,       /* Class-B sanity-check yapar; callback ve konfigürasyon geçerliliğini raporlar. */
    SERVO_IOCTL_SET_LIMITS = 0x20U,         /* ts_Servo_Limits ile açı ve pulse limitlerini günceller. */
    SERVO_IOCTL_GET_LIMITS = 0x21U,         /* ts_Servo_Limits ile aktif limitleri döndürür. */
    SERVO_IOCTL_SET_SLEW_RATE = 0x30U,      /* ts_Servo_SlewRateConfig ile açısal hız sınırlamasını ayarlar. */
    SERVO_IOCTL_GET_SLEW_RATE = 0x31U,      /* ts_Servo_SlewRateConfig ile aktif slew-rate ayarını döndürür. */
    SERVO_IOCTL_OVERRIDE_RAW_PULSE = 0x40U, /* ts_Servo_RawPulseCommand ile ham PWM darbesi yazar; yine güvenli pulse sınırlarına kelepçelenir. */
    SERVO_IOCTL_WRITE_PERCENT = 0x41U       /* ts_Servo_PercentCommand ile -100/+100 fiziksel limit yüzdesini açı komutuna çevirir. */
} te_Servo_IoctlCmd;

/**
 * @brief Servo instance'ını açar, konfigürasyonu handle'a kopyalar ve PWM callback sözleşmesini doğrular.
 * @param psHandle Servo instance handle pointer'ı; NULL olamaz ve sürücü durumunu taşır.
 * @param psConfig Platformdan enjekte edilen PWM/lock/timing arayüzlerini ve fiziksel limitleri taşır.
 * @return DRIVER_OK başarılı kurulumda, aksi halde ortak te_Driver_RetCode hata kodu.
 */
te_Driver_RetCode Servo_Open(ts_Servo_Handle *psHandle, const ts_Servo_OpenConfig *psConfig);

/**
 * @brief Servo instance'ını güvenli şekilde kapatır ve state'i UNINIT yapar.
 * @param psHandle Servo instance handle pointer'ı; NULL olamaz.
 * @return DRIVER_OK başarılı kapanışta, aksi halde ortak te_Driver_RetCode hata kodu.
 */
te_Driver_RetCode Servo_Close(ts_Servo_Handle *psHandle);

/**
 * @brief Servo durum snapshot'ını okur; Class-B driver'da fiziksel sensör okuması yerine status döndürür.
 * @param psHandle Servo instance handle pointer'ı; NULL olamaz.
 * @param psOutStatus Çıkış status yapısı; ts_Servo_Status tipinde olmalıdır.
 * @return DRIVER_OK başarılı okumada, aksi halde ortak te_Driver_RetCode hata kodu.
 */
te_Driver_RetCode Servo_Read(ts_Servo_Handle *psHandle, ts_Servo_Status *psOutStatus);

/**
 * @brief Hedef açıyı radyan cinsinden alır, güvenli sınırlardan geçirir ve PWM port callback'ine yazar.
 * @param psHandle Servo instance handle pointer'ı; NULL olamaz.
 * @param vpInData float hedef açı pointer'ı; uygulama/actuator task SI birimi radyan gönderir.
 * @return DRIVER_OK başarılı PWM yazmada, aksi halde ortak te_Driver_RetCode hata kodu.
 */
te_Driver_RetCode Servo_Write(ts_Servo_Handle *psHandle, const void *vpInData);

/**
 * @brief Servo konfigürasyonu, health-check ve test komutları için komut tabanlı servis girişidir.
 * @param psHandle Servo instance handle pointer'ı; NULL olamaz.
 * @param eCmd Servo Ioctl komutu; argüman tipi komuta göre değişir.
 * @param vpArg Komut argümanı; NULL gerektiren komutlar dışında geçerli pointer olmalıdır.
 * @return DRIVER_OK başarılı komutta, aksi halde ortak te_Driver_RetCode hata kodu.
 */
te_Driver_RetCode Servo_Ioctl(ts_Servo_Handle *psHandle, te_Servo_IoctlCmd eCmd, void *vpArg);

/**
 * @brief Deployment öncesi yazılım süpürme testi yapar; min/center/max/center komutlarının port callback'e ulaşmasını doğrular.
 * @param psHandle Servo instance handle pointer'ı; READY durumda olmalıdır.
 * @param u32TimeoutMs Testin tick bazlı üst zaman bütçesi; 0 ise varsayılan değer kullanılır.
 * @return DRIVER_OK başarılı testte, aksi halde ortak te_Driver_RetCode hata kodu.
 */
te_Driver_RetCode Servo_Test(ts_Servo_Handle *psHandle, uint32_t u32TimeoutMs);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_DRIVER_H_ */
