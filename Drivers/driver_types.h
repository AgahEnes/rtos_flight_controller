#ifndef DRIVER_TYPES_H_
#define DRIVER_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    DRIVER_OK = 0,
    DRIVER_ERR_NULL_PTR,
    DRIVER_ERR_INVALID_ARG,
    DRIVER_ERR_STATE,
    DRIVER_ERR_BUS,
    DRIVER_ERR_TIMEOUT,
    DRIVER_ERR_CONFIG,
    DRIVER_ERR_NOT_SUPPORTED,
    DRIVER_ERR_WHOAMI,
    DRIVER_ERR_IO
} te_Driver_RetCode;

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_TYPES_H_ */
