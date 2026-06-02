# Hungarian Notation Glossary

Bu dokuman projede kullanilan Macar notasyonu (Hungarian notation) kisaltmalarini standartlastirir.

## Type Prefixes

- `u8`   : `uint8_t`
- `u16`  : `uint16_t`
- `u32`  : `uint32_t`
- `u64`  : `uint64_t`
- `s8`   : `int8_t`
- `s16`  : `int16_t`
- `s32`  : `int32_t`
- `s64`  : `int64_t`
- `f32`  : `float`
- `f64`  : `double`
- `b`    : `bool`
- `c`    : `char`

## Pointer and Container Prefixes

- `p`    : pointer (generic)
- `pfn`  : pointer to function (`p` + `fn`)
- `tpfn` : typedef pointer-to-function (`t` + `pfn`)
- `px`   : pointer to external handle/object
- `ps`   : pointer to struct
- `pu8`  : pointer to `uint8_t`
- `pu16` : pointer to `uint16_t`
- `pu32` : pointer to `uint32_t`
- `pc`   : pointer to char/string
- `pp`   : pointer-to-pointer
- `pv`   : pointer to variable/opaque payload
- `vp`   : void pointer

## Object Kind Prefixes

- `fn`   : function (role marker; typically with `p` as `pfn`)
- `x`    : RTOS object/handle (task, mutex, queue, semaphore)
- `e`    : enum value
- `t`    : typedef type
- `s`    : struct instance
- `a`    : array
- `au8`  : array of `uint8_t`
- `as32` : array of `int32_t`

## Scope Prefixes

- `g`    : global variable
- `gs`   : global struct instance
- `gx`   : global RTOS object
- `prv`  : private/internal (module-local helper)
- `k`    : compile-time constant (tercih edildiginde)
- `ks`   : const struct instance (`k` + `s`)

## Naming Rules

1. Prefix sirasini `[scope][type/role][name]` olarak koru.  
   Ornek: `gsMpuHandle`, `gxI2cBusMutex`, `pu8TxBuffer`
2. `bool` degiskenler `b` ile baslar.  
   Ornek: `bIsInitialized`, `bDataReady`
3. Input/output pointer'lar adindan anlasilir olmalidir.  
   Ornek: `const uint8_t *pu8Data`, `uint8_t *pu8OutBuffer`
4. RTOS handle degiskenleri `x` ile baslar.
5. Donanim/HAL bagimli pointer'larda `px`/`ps` kullanimi tutarli tutulur.

## Notes

- Bu sozluk, aviyonik kod tabaninda okunabilirlik ve izlenebilirlik icin referanstir.
- Kural ihlallerinde once isimlendirme netligi, sonra kisalik tercih edilir.

---

## Turkce Surum (Birebir)

Bu bolum, yukaridaki sozlugun Turkce karsiligidir.

### Tur On Ekleri (Type Prefixes)

- `u8`   : `uint8_t` (isaretsiz 8-bit tamsayi)
- `u16`  : `uint16_t` (isaretsiz 16-bit tamsayi)
- `u32`  : `uint32_t` (isaretsiz 32-bit tamsayi)
- `u64`  : `uint64_t` (isaretsiz 64-bit tamsayi)
- `s8`   : `int8_t` (isaretli 8-bit tamsayi)
- `s16`  : `int16_t` (isaretli 16-bit tamsayi)
- `s32`  : `int32_t` (isaretli 32-bit tamsayi)
- `s64`  : `int64_t` (isaretli 64-bit tamsayi)
- `f32`  : `float` (32-bit kayan nokta)
- `f64`  : `double` (64-bit kayan nokta)
- `b`    : `bool` (mantiksal deger)
- `c`    : `char` (karakter)

### Isaretci ve Kapsayici On Ekleri (Pointer and Container Prefixes)

- `p`    : genel isaretci
- `pfn`  : fonksiyon isaretcisi (`p` + `fn`)
- `tpfn` : typedef fonksiyon isaretcisi (`t` + `pfn`)
- `px`   : harici sahiplikte handle/nesne isaretcisi
- `ps`   : struct isaretcisi
- `pu8`  : `uint8_t` isaretcisi
- `pu16` : `uint16_t` isaretcisi
- `pu32` : `uint32_t` isaretcisi
- `pc`   : karakter/dizi (string) isaretcisi
- `pp`   : isaretcinin isaretcisi
- `pv`   : degisken/opaque payload isaretcisi
- `vp`   : `void *` isaretci

### Nesne Turu On Ekleri (Object Kind Prefixes)

- `fn`   : fonksiyon (rol; genelde `pfn` icinde `p` ile birlikte)
- `x`    : RTOS nesnesi/handle (`task`, `mutex`, `queue`, `semaphore`)
- `e`    : enum degeri
- `t`    : typedef tipi
- `s`    : struct ornegi
- `a`    : dizi
- `au8`  : `uint8_t` dizisi
- `as32` : `int32_t` dizisi

### Kapsam On Ekleri (Scope Prefixes)

- `g`    : global degisken
- `gs`   : global struct ornegi
- `gx`   : global RTOS nesnesi
- `prv`  : private/internal (modul-ici yardimci)
- `k`    : derleme zamani sabiti (kullanilan yerlerde)
- `ks`   : const struct ornegi (`k` + `s`)

### Adlandirma Kurallari (Naming Rules)

1. On ek sirasi `[scope][type/role][name]` olmalidir.  
   Ornek: `gsMpuHandle`, `gxI2cBusMutex`, `pu8TxBuffer`

2. `bool` degiskenler `b` ile baslamalidir.  
   Ornek: `bIsInitialized`, `bDataReady`, `bTxCompleted`

3. Girdi/cikti isaretcileri isimden anlasilmalidir.  
   Ornek: `const uint8_t *pu8Data`, `uint8_t *pu8OutBuffer`, `const ts_Config *psConfig`

4. RTOS handle degiskenleri `x` ile baslamalidir.  
   Ornek: `xSensorTaskHandle`, `xI2cBusMutex`, `xDmaSemaphore`

5. Donanim/HAL bagimli isaretcilerde `px`/`ps` kullanimi tutarli olmalidir.  
   Ornek: `I2C_HandleTypeDef *pxI2cHandle`, `UART_HandleTypeDef *pxUartHandle`, `ts_BusContext *psBusContext`

### Notlar

- Bu sozluk, aviyonik kod tabaninda okunabilirlik ve izlenebilirlik icin referanstir.
- Kural ihlalinde once isimlendirme netligi, sonra kisalik tercih edilir.
