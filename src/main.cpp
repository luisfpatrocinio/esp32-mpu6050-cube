#include <Arduino.h>
#include "gyro.h"
#include <ArduinoOTA.h>

// CONFIG
#define BAUDRATE 115200         // Serial baud rate
#define LED_BUILTIN 2           // Built-in LED pin for ESP32
#define HEARTBEAT_INTERVAL 2000 // 2 seconds between heartbeats
#define MAX_MISSED_HEARTBEATS 3 // Number of missed heartbeats before considering connection lost

// Pins
#define LED_PIN 17 // Pin for external LED

// Wi-fi config
#include <WiFi.h>
#include <WiFiUdp.h>
#define WIFI_SSID "patro"
#define WIFI_PASSWORD "cafecombiscoito"

// Defina o IP, gateway e máscara desejados:
// IPAddress local_IP(192, 168, 137, 15); // IP fixo desejado
// IPAddress gateway(192, 168, 137, 1);   // Gateway da sua rede
// IPAddress subnet(255, 255, 255, 0);    // Máscara de sub-rede
// IPAddress primaryDNS(8, 8, 8, 8);      // (opcional) DNS primário
// IPAddress secondaryDNS(8, 8, 4, 4);    // (opcional) DNS secundário

// UDP config
unsigned int localUdpPort = 4469;         // ESP32 UDP port
const char *udpAddress = "192.168.137.1"; // Destiny IP
const int udpPort = 5000;                 // Destiny UDP port

// UDP communication manager object
WiFiUDP udp;

// Buffer for incoming packets
char incomingPacket[255];

// struct to hold sensor data
MPU6050_data_t sensor_data;

// Game variables
bool connectedToGame = 0;

// Timer handle
hw_timer_t *heartbeatTimer = NULL;

// Heartbeat tracking
volatile uint8_t missedHeartbeats = 0;
volatile bool waitingForHeartbeatAck = false;
volatile bool heartbeatFlag = false; // Flag to trigger heartbeat in main loop

// Function prototypes
void checkUDPPackages();
bool initUDP();
void sendUDP(const char *message);
void setupWiFi();
void IRAM_ATTR onHeartbeatTimer();

// Timer callback function
void IRAM_ATTR onHeartbeatTimer()
{
    if (connectedToGame)
    {
        if (waitingForHeartbeatAck)
        {
            missedHeartbeats++;
            if (missedHeartbeats >= MAX_MISSED_HEARTBEATS)
            {
                connectedToGame = false;
                missedHeartbeats = 0;
                waitingForHeartbeatAck = false;
            }
        }
        else
        {
            heartbeatFlag = true; // Set flag instead of sending UDP directly
        }
    }
}

/**
 * @brief Initializes UDP communication and binds to the specified port
 * @return true if initialization was successful, false otherwise
 */
bool initUDP()
{
    Serial.printf("[UDP] Attempting to bind to port %d\n", localUdpPort);

    if (!udp.begin(localUdpPort))
    {
        Serial.printf("[UDP] Failed to bind to port %d\n", localUdpPort);
        return false;
    }

    Serial.printf("[UDP] Successfully bound to port %d\n", localUdpPort);
    return true;
}

void checkUDPPackages()
{
    int packageSize = udp.parsePacket();
    if (packageSize > 0)
    {
        Serial.printf("\n>>> Pacote recebido! Tamanho: %d bytes, do IP: %s, Porta: %d\n",
                      packageSize,
                      udp.remoteIP().toString().c_str(),
                      udp.remotePort());

        // Cria buffer
        char buffer[255];
        int len = udp.read(buffer, 255);
        if (len > 0)
        {
            buffer[len] = '\0'; // Null-terminate the string
        }

        Serial.printf("Package contents: %s\n", buffer);

        // Check if the received data is a handshake message
        if (strcmp(buffer, "udp_handshake") == 0)
        {
            Serial.println("[CONNECTION] Game connection established! Sending acknowledgment...");
            // Send an acknowledgment back to the sender
            sendUDP("udp_handshake_ack");
            connectedToGame = true;
            missedHeartbeats = 0;
            waitingForHeartbeatAck = false;

            // Atualizar udpAddress
            // udpAddress = udp.remoteIP().toString().c_str(); // Update the UDP address to the sender's IP
        }

        // Check for heartbeat response
        else if (strcmp(buffer, "heartbeat_ack") == 0)
        {
            Serial.println("[HEARTBEAT] Heartbeat acknowledged by game server.");
            connectedToGame = true;
            missedHeartbeats = 0;
            waitingForHeartbeatAck = false;
        }
    }
}

/**
 * @brief Envia um pacote UDP com a mensagem especificada.
 * @param message Mensagem a ser enviada
 */
void sendUDP(const char *message)
{
    Serial.printf("<<< Enviando pacote para %s:%d -> '%s'\n", udpAddress, udpPort, message);

    // Envia o pacote
    udp.beginPacket(udpAddress, udpPort);
    udp.print(message);
    udp.endPacket();
}

void setup()
{
    Serial.begin(BAUDRATE);
    pinMode(LED_BUILTIN, OUTPUT);

    // Setup LED
    pinMode(LED_PIN, OUTPUT);

    while (!Serial)
    {
        delay(10);
    }

    // Init MPU6050
    Serial.println("Initializing MPU6050...");
    initMPU6050();
    initOrientation(&sensor_data);
    Serial.println("MPU6050 initialized!");

    // // Config Static IP before connect
    // if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS))
    // {
    //     Serial.println("Falha ao configurar IP estático");
    // }

    // Init Wi-fi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.println("Conectando...");
    while (WiFi.status() != WL_CONNECTED)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        digitalWrite(LED_PIN, HIGH);
        delay(250);
        digitalWrite(LED_BUILTIN, LOW);
        digitalWrite(LED_PIN, LOW);
        delay(250);
        Serial.print(".");
    }

    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(LED_PIN, LOW);
    Serial.println("Conectado ao Wi-Fi!");
    Serial.print("Endereço IP: ");
    Serial.println(WiFi.localIP());

    // Initialize UDP
    if (!initUDP())
    {
        Serial.println("Failed to initialize UDP. Restarting...");
        ESP.restart();
    }

    // Setup OTA
    ArduinoOTA.begin();

    // Initialize heartbeat timer
    heartbeatTimer = timerBegin(0, 80, true);                         // Timer 0, prescaler 80 (80MHz/80 = 1MHz), count up
    timerAttachInterrupt(heartbeatTimer, &onHeartbeatTimer, true);    // Attach callback
    timerAlarmWrite(heartbeatTimer, HEARTBEAT_INTERVAL * 1000, true); // Set alarm value and auto-reload
    timerAlarmEnable(heartbeatTimer);                                 // Enable timer
}

void loop()
{
    // Handle OTA updates
    ArduinoOTA.handle();

    // Check and handle heartbeat flag
    if (heartbeatFlag)
    {
        heartbeatFlag = false;
        sendUDP("heartbeat");
        waitingForHeartbeatAck = true;
    }

    digitalWrite(LED_BUILTIN, connectedToGame); // Turn on LED if connected to game
    digitalWrite(LED_PIN, connectedToGame);     // Turn on LED if connected to game

    // Update sensor data
    updateOrientation(&sensor_data);
    calculateInclinationAngles(&sensor_data);

    // Get current face
    current_face = getCubeFace(sensor_data.roll, sensor_data.pitch);

    // Print orientation data
    Serial.print("Roll: ");
    Serial.print(sensor_data.roll);
    Serial.print("\t Pitch: ");
    Serial.print(sensor_data.pitch);
    Serial.print("\t Face: ");

    // Print face name
    switch (current_face)
    {
    case FACE_Z_POS:
        Serial.println("Z+ (Top)");
        break;
    case FACE_Z_NEG:
        Serial.println("Z- (Bottom)");
        break;
    case FACE_X_POS:
        Serial.println("X+ (Front)");
        break;
    case FACE_X_NEG:
        Serial.println("X- (Back)");
        break;
    case FACE_Y_POS:
        Serial.println("Y+ (Right)");
        break;
    case FACE_Y_NEG:
        Serial.println("Y- (Left)");
        break;
    default:
        Serial.println("Unknown");
        break;
    }

    // Checar recebimento
    checkUDPPackages();

    // Envio de pacote UDP:
    char message[32];
    snprintf(message, sizeof(message), "C|%d", (int)current_face);
    udp.print(message);
    sendUDP(message);

    // Printar numeros inteiros de acordo com os valores de roll e pitch,
    // simulando um dado:
    int roll_int = (int)(sensor_data.roll / 90 * MAX_ROLL);   // Mapeia roll
    int pitch_int = (int)(sensor_data.pitch / 90 * MAX_ROLL); // Mapeia pitch
    int yaw_int = (int)(sensor_data.yaw / 90 * MAX_ROLL);     // Mapeia yaw

    // Get and send Roll and Pitch
    char roll_pitch_str[32];
    snprintf(roll_pitch_str, sizeof(roll_pitch_str), "R|%d|%d|%d", roll_int, pitch_int, yaw_int);
    sendUDP(roll_pitch_str);

    delay(30); // Small delay
}
