#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>

static const char *ksWifiSsid = "RTOS-FLIGHT-TELEM";
static const char *ksWifiPassword = "flight1234";
static const uint32_t ku32Stm32BaudRate = 115200U;
static const uint8_t ku8Esp32RxPin = 16U;
static const uint8_t ku8Esp32TxPin = 17U;
static const uint16_t ku16WebSocketPort = 81U;
static const char *ksWebSocketPath = "/telemetry";
static const uint16_t ku16ChunkLimit = 128U;

static WebSocketsServer gsWebSocketServer(ku16WebSocketPort);
static uint8_t gau8RxChunk[ku16ChunkLimit];

static void WebSocketEvent(uint8_t u8ClientId, WStype_t eType, uint8_t *pu8Payload, size_t xLength)
{
    (void)pu8Payload;
    (void)xLength;

    if (eType == WStype_CONNECTED)
    {
        IPAddress xClientIp = gsWebSocketServer.remoteIP(u8ClientId);
        Serial.printf("ws client %u connected from %s\n", u8ClientId, xClientIp.toString().c_str());
    }
    else if (eType == WStype_DISCONNECTED)
    {
        Serial.printf("ws client %u disconnected\n", u8ClientId);
    }
}

static void ForwardStm32UartToWebSocket(void)
{
    size_t xCount = 0U;

    while ((Serial2.available() > 0) && (xCount < sizeof(gau8RxChunk)))
    {
        gau8RxChunk[xCount] = (uint8_t)Serial2.read();
        xCount++;
    }

    if (xCount > 0U)
    {
        gsWebSocketServer.broadcastBIN(gau8RxChunk, xCount);
    }
}

void setup()
{
    Serial.begin(115200);
    Serial2.begin(ku32Stm32BaudRate, SERIAL_8N1, ku8Esp32RxPin, ku8Esp32TxPin);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(ksWifiSsid, ksWifiPassword);

    Serial.print("ESP32 AP IP: ");
    Serial.println(WiFi.softAPIP());
    Serial.print("Dashboard WebSocket URL: ws://");
    Serial.print(WiFi.softAPIP());
    Serial.print(":");
    Serial.print(ku16WebSocketPort);
    Serial.println(ksWebSocketPath);

    gsWebSocketServer.begin();
    gsWebSocketServer.onEvent(WebSocketEvent);
}

void loop()
{
    gsWebSocketServer.loop();
    ForwardStm32UartToWebSocket();
}
