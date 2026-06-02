#ifndef APP_ACTUATOR_ACTUATOR_DRIVER_INTERFACE_H_
#define APP_ACTUATOR_ACTUATOR_DRIVER_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    ACTUATOR_DRIVER_OK = 0,
    ACTUATOR_DRIVER_ERR_ARG,
    ACTUATOR_DRIVER_ERR_STATE,
    ACTUATOR_DRIVER_ERR_IO
} te_ActuatorDriverRetCode;

typedef te_ActuatorDriverRetCode (*tpfn_ActuatorDriverInit)(void *vpContext);
typedef te_ActuatorDriverRetCode (*tpfn_ActuatorDriverWriteAngle)(void *vpContext, float f32AngleRad);

typedef struct
{
    tpfn_ActuatorDriverInit pfnInit;
    tpfn_ActuatorDriverWriteAngle pfnWriteAngle;
} ts_ActuatorDriverVTable;

typedef struct
{
    const ts_ActuatorDriverVTable *psVTable;
    void *vpContext;
} ts_ActuatorDevice;

#ifdef __cplusplus
}
#endif

#endif /* APP_ACTUATOR_ACTUATOR_DRIVER_INTERFACE_H_ */
