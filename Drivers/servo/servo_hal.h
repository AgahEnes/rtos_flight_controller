#ifndef SERVO_HAL_H_
#define SERVO_HAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Servo sürücüsü API sürümü; uygulama/telemetri katmanı ile driver sözleşmesini izlenebilir yapmak için HAL-tanım katmanında tutulur. */
#define SERVO_DRIVER_API_VERSION                  (0x0100U)

/* SG90 ve benzeri analog servo motorlar için tipik PWM frekansı; bu değer datasheet/ürün bilgisi kaynaklı fiziksel zamanlama varsayımıdır. */
#define SERVO_DEFAULT_PWM_FREQUENCY_HZ            (50U)

/* 50 Hz PWM sinyalinin mikro-saniye cinsinden periyodu; port katmanı timer period ayarını doğrularken bu değeri referans alabilir. */
#define SERVO_DEFAULT_PERIOD_US                   (20000U)

/* SG90 için tipik minimum darbe genişliği; çekirdek driver -90 derece sınırını PWM komutuna çevirirken kullanır. */
#define SERVO_SG90_MIN_PULSE_US                   (500U)

/* SG90 için tipik orta darbe genişliği; test ve kalibrasyon akışlarında mekanik merkez referansı olarak kullanılır. */
#define SERVO_SG90_CENTER_PULSE_US                (1500U)

/* SG90 için tipik maksimum darbe genişliği; çekirdek driver +90 derece sınırını PWM komutuna çevirirken kullanır. */
#define SERVO_SG90_MAX_PULSE_US                   (2500U)

/* Servo sinyali için güvenli alt mikro-saniye sınırı; hatalı konfigürasyonun mekanik zorlamaya dönüşmesini engellemek için kullanılır. */
#define SERVO_SAFE_MIN_PULSE_US                   (100U)

/* Servo sinyali için güvenli üst mikro-saniye sınırı; port katmanına fiziksel olarak anlamsız darbe gönderilmesini engeller. */
#define SERVO_SAFE_MAX_PULSE_US                   (3000U)

/* Float karşılaştırmalarında sıfıra çok yakın değerleri ayırt etmek için kullanılan küçük eşik; konfigürasyon doğrulama katmanı içindir. */
#define SERVO_FLOAT_EPSILON                       (0.000001F)

/* Pi sabiti; derece/radyan dönüşümleri ve SG90 açı sınırı makroları için platform bağımsız matematik sabitidir. */
#define SERVO_PI_F                                (3.14159265358979323846F)

/* Dereceyi radyana çevirmek için katsayı; uygulama isterse derece tabanlı komutu güvenli biçimde radyan API'sine çevirebilir. */
#define SERVO_DEG_TO_RAD_F                        (SERVO_PI_F / 180.0F)

/* Radyanı dereceye çevirmek için katsayı; telemetri veya debug çıktılarında SI iç temsili okunabilir dereceye çevirmek için kullanılır. */
#define SERVO_RAD_TO_DEG_F                        (180.0F / SERVO_PI_F)

/* SG90 için tipik minimum mekanik açı; çekirdek driver giriş komutunu bu fiziksel alt sınıra kelepçeler. */
#define SERVO_SG90_MIN_ANGLE_RAD                  (-90.0F * SERVO_DEG_TO_RAD_F)

/* SG90 için tipik merkez mekanik açı; test süpürmesi sonunda servoyu güvenli merkeze döndürmek için kullanılır. */
#define SERVO_SG90_CENTER_ANGLE_RAD               (0.0F)

/* SG90 için tipik maksimum mekanik açı; çekirdek driver giriş komutunu bu fiziksel üst sınıra kelepçeler. */
#define SERVO_SG90_MAX_ANGLE_RAD                  (90.0F * SERVO_DEG_TO_RAD_F)

/* Genel servo konfigürasyonu için izin verilen mutlak açı büyüklüğü; 180/270/360 derece servo ailelerini desteklerken saçma değerleri reddeder. */
#define SERVO_ABSOLUTE_MAX_ANGLE_RAD              (2.0F * SERVO_PI_F)

/* Yüzde tabanlı komutların alt sınırı; -100 yüzde mekanik minimum açıya karşılık gelir. */
#define SERVO_PERCENT_MIN                         (-100.0F)

/* Yüzde tabanlı komutların merkez değeri; 0 yüzde mekanik merkez anlamına gelir. */
#define SERVO_PERCENT_CENTER                      (0.0F)

/* Yüzde tabanlı komutların üst sınırı; +100 yüzde mekanik maksimum açıya karşılık gelir. */
#define SERVO_PERCENT_MAX                         (100.0F)

/* Varsayılan mutex bekleme süresi; RTOS port katmanı kilit sağladığında actuator task'ın sonsuza kadar beklememesi için kullanılır. */
#define SERVO_DEFAULT_LOCK_TIMEOUT_MS             (20U)

/* Test fonksiyonunun servo komutlarını hızlıca doğrularken kullandığı varsayılan zaman bütçesi; gerçek bekleme çekirdek driver'da yapılmaz. */
#define SERVO_DEFAULT_TEST_TIMEOUT_MS             (100U)

typedef enum
{
    SERVO_STATE_UNINIT = 0,  /* Sürücü henüz açılmadı; çekirdek driver bu durumda PWM komutu üretmez. */
    SERVO_STATE_READY,       /* Sürücü konfigüre edildi; uygulama/actuator task güvenli şekilde açı komutu yazabilir. */
    SERVO_STATE_BUSY,        /* Sürücü kısa süreli PWM yazma işlemi içinde; durum makinesi eşzamanlı erişimi görünür kılar. */
    SERVO_STATE_ERROR        /* Konfigürasyon veya port callback hatası oluştu; hata sonrası kontrollü toparlanma için Close/Open beklenir. */
} te_Servo_State;

#ifdef __cplusplus
}
#endif

#endif /* SERVO_HAL_H_ */
