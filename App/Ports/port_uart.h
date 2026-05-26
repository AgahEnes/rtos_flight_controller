#ifndef APP_PORTS_PORT_UART_H_
#define APP_PORTS_PORT_UART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Sends a binary payload over a transport endpoint.
 * @param pu8Data Input payload pointer.
 * @param u16Length Payload length in bytes.
 * @param vpContext Opaque context pointer.
 * @return true when payload is accepted by transport, false otherwise.
 */
typedef bool (*tpfn_PortUartSend)(const uint8_t *pu8Data, uint16_t u16Length, void *vpContext);

#ifdef __cplusplus
}
#endif

#endif /* APP_PORTS_PORT_UART_H_ */
