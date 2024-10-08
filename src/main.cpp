#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include "esp_camera.h"
#include "Freenove_4WD_Car_WiFi.h"
#include "Freenove_4WD_Car_Emotion.h"
#include "Freenove_4WD_Car_WS2812.h"
#include "Freenove_4WD_Car_For_ESP32.h"
#include "Freenove_4WD_Websocket.h"
#include "Freenove_4WD_Race_Data.h"
#include <PubSubClient.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <limits.h>


#define STREAM_CONTENT_BOUNDARY "123456789000000000000987654321"
#define OBSTACLE_DISTANCE      40
#define OBSTACLE_DISTANCE_LOW  20

// Ressources
// https://randomnerdtutorials.com/esp32-websocket-server-arduino/#1
// https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/
// https://randomnerdtutorials.com/esp32-static-fixed-ip-address-arduino-ide/
// https://github.com/Freenove/Freenove_4WD_Car_Kit_for_ESP32/tree/master

char *ssid_wifi = "****"; // Le nom du réseau WiFi
char *password_wifi = "****"; // Le password du WiFi

const char *mqtt_server = "0.0.0.0"; // L'IP de votre broker MQTT (ta machine, ifconfig | grep 192 )
const int mqtt_interval_ms = 5000;          // L'interval en ms entre deux envois de données

IPAddress localIP(0, 0, 0, 0);  // l'IP que vous voulez donner à votre voiture (faire attention a ce que les 3 premieres partie de l'ip soit toujours identique à celle de votre machine)

IPAddress localGateway(0, 0, 0, 0);  // L'IP de la gateway de votre réseau "route -n get default"
IPAddress localSubnet(255, 255, 255, 0);   // Le masque de sous réseau

IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

AsyncWebServer server(80);
AsyncWebSocket ws("/ws"); // Changez le nom de ce point d'accès pour "sécuriser" l'accès à votre voiture

WiFiClient espClient;
PubSubClient client(espClient);

// WiFiServer server_Cmd(4000);
WiFiServer server_Camera(7000);
bool videoFlag = 0;

long last_message = 0;

int distance[4];          // Storage of ultrasonic data
char buff[6];             // Buffer to store the battery voltage data
char ultrasonic_buff[10]; // Buffer to store the Ultrasonic data
char average_speed_buff[32]; // Buffer to store the average speed data
char distance_covered_buff[128]; // Buffer to store the distance covered data and the race id
char collision_duration_buff[32]; // Buffer to store the number of collisions data
char out_of_parcours_buff[32]; // Buffer to store the number of out of parcours data


long startTime = 0;
long endTime = 0;
float constantSpeed = 0.0;
long commandDuration = 0;
bool newRaceMessage = false;
bool newDistanceMessage = false;
float distanceCar = 0.0;

// variable used to create vehicule history API side
bool servoMoveCommand = false;
bool headAngleCommand = false;
bool ledAnimationCommand = false;
bool ledPrincipalColorCommand = false;
bool buzzerAlarmCommand = false;
bool buzzerVariableCommand = false;
bool videoCommand = false;
bool autoModeCommand = false;
bool raceStartingModeCommand = false;

// put function declarations here:
void WiFi_Init();
void loopTask_Camera(void *pvParameters);
void notifyClients();
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void initWebSocket();
void reconnect();

void WiFi_Init()
{
    ssid_Router = ssid_wifi;         // Modify according to your router name
    password_Router = password_wifi; // Modify according to your router password
    ssid_AP = "Sunshine";            // ESP32 turns on an AP and calls it Sunshine
    password_AP = "Sunshine";        // Set your AP password for ESP32 to Sunshine
    frame_size = FRAMESIZE_CIF;      // 400*296
}

void setup()
{
    Serial.begin(115200);
    Serial.setDebugOutput(true);

    WiFi_Init();

    if (!WiFi.config(localIP, localGateway, localSubnet, primaryDNS, secondaryDNS))
    {
        Serial.println("STA Failed to configure");
    }

    Buzzer_Setup(); // Buzzer initialization
    WiFi_Init();    // WiFi paramters initialization
    WiFi_Setup(0);  // Start AP Mode. If you want to connect to a router, change 1 to 0.
    // server_Cmd.begin(4000);    // Start the command server
    server_Camera.begin(7000); // Turn on the camera server

    cameraSetup(); // Camera initialization
    camera_vflip(true);
    camera_hmirror(true);
    Emotion_Setup();    // Emotion initialization
    WS2812_Setup();     // WS2812 initialization
    PCA9685_Setup();    // PCA9685 initialization
    Light_Setup();      // Light initialization
    Track_Setup();      // Track initialization
    Ultrasonic_Setup(); // Initialize the ultrasonic module

    // Cette section serait peut être à virer...
    disableCore0WDT(); // Turn off the watchdog function in kernel 0
    xTaskCreateUniversal(loopTask_Camera, "loopTask_Camera", 8192, NULL, 0, NULL, 0);
    xTaskCreateUniversal(loopTask_WTD, "loopTask_WTD", 8192, NULL, 0, NULL, 0);

    client.setServer(mqtt_server, 1883);

    initWebSocket();

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { 
                camera_fb_t *fb = NULL;
                fb = esp_camera_fb_get();
                        if (fb != NULL)
                        {
                            uint8_t slen[4];
                            slen[0] = fb->len >> 0;
                            slen[1] = fb->len >> 8;
                            slen[2] = fb->len >> 16;
                            slen[3] = fb->len >> 24;
                            AsyncResponseStream *response = request->beginResponseStream("image");
                            response->write(slen, 4);
                            response->write(fb->buf, fb->len);
                            request->send(response);
                            client.write(slen, 4);
                            client.write(fb->buf, fb->len);
                            request->send_P(200, "application/octet-stream", fb->buf, fb->len);
                            Serial.println("Camera send");
                            esp_camera_fb_return(fb);
                            fb = NULL;
                        } });

    server.begin();

    // Init the state of the car
    Emotion_SetMode(1);
    WS2812_SetMode(1);
}

void loop()
{
    // put your main code here, to run repeatedly:
    ws.cleanupClients();

    Emotion_Show(emotion_task_mode); // Led matrix display function
    WS2812_Show(ws2812_task_mode);   // Car color lights display function

    Track_Car(carFlag);

    //The MQTT part
     if (!client.connected())
     {
        reconnect();
     }
     client.loop();

    if (race_mode) {
        updateAverageSpeed();
        updateDistanceCovered();
        updateOutOfParcours();
        updateCollisionDurationTime();
    }

    if (race_id != 0 || previous_race_id != 0) {
        char message_buffer[20];
        int active_race_id = (race_id != 0) ? race_id : previous_race_id;

        if (servoMoveCommand) {
            const char *message = "Mouvement de la tête";
            client.publish(generateRaceTopic(message_buffer, sizeof(message_buffer), active_race_id, "message"), message);
            servoMoveCommand = false;
        }

        if (headAngleCommand) {
            const char *message = "Changement d'angle de la tête";
            client.publish(generateRaceTopic(message_buffer, sizeof(message_buffer), active_race_id, "message"), message);
            headAngleCommand = false;
        }

        if (ledAnimationCommand) {
            const char *message = "Animation des leds";
            client.publish(generateRaceTopic(message_buffer, sizeof(message_buffer), active_race_id, "message"), message);
            ledAnimationCommand = false;
        }

        if (ledPrincipalColorCommand) {
            const char *message = "Changement de couleur des leds";
            client.publish(generateRaceTopic(message_buffer, sizeof(message_buffer), active_race_id, "message"), message);
            ledPrincipalColorCommand = false;   
        }

        if (buzzerAlarmCommand) {
            const char *message = "Changement de l'état de l'alarme";
            client.publish(generateRaceTopic(message_buffer, sizeof(message_buffer), active_race_id, "message"), message);
            buzzerAlarmCommand = false;
        }

        if (buzzerVariableCommand) {
            const char *message = "Changement des variables de l'alarme";
            client.publish(generateRaceTopic(message_buffer, sizeof(message_buffer), active_race_id, "message"), message);
            buzzerVariableCommand = false;
        }
        
        if (videoCommand) {
            const char *message = "Activation/Désactivation de la vidéo";
            client.publish(generateRaceTopic(message_buffer, sizeof(message_buffer), active_race_id, "message"), message);
            videoCommand = false;
        }
        
        if (autoModeCommand) {
            const char *message = "Activate/Désactivation du mode auto";
            client.publish(generateRaceTopic(message_buffer, sizeof(message_buffer), active_race_id, "message"), message);
            autoModeCommand = false;
        }

        if (raceStartingModeCommand) {
            const char *message = "Début de course";
            client.publish(generateRaceTopic(message_buffer, sizeof(message_buffer), active_race_id, "message"), message);
            raceStartingModeCommand = false;
        }
    }

    if (newRaceMessage)
    {
        char message[20];

        ltoa(commandDuration, message, 10);

        const char* topic = "esp32/course_time";

        client.publish(topic, message);

        newRaceMessage = false;
    }

    if (newDistanceMessage)
    {
        char message[20];

        dtostrf(distanceCar, 5, 2, message);

        const char* topic = "esp32/distance";

        client.publish(topic, message);

        newDistanceMessage = false;
    }
    
    long now = millis();

    if (now - last_message > mqtt_interval_ms)
    {
        last_message = now;

        // Les led et la batteries sont branchés tous les deux sur le pin 32
        // du coup, lire la valeur de batterie fait freeze la batterie
        // Battery level
        // dtostrf(Get_Battery_Voltage(), 5, 2, buff);
        // client.publish("esp32/battery", buff);

        if (race_mode && race_id != 0) {

            char topic_buffer[64];

            // -> distance parcourue
            if (distance_covered != distance_previously_covered) {
                dtostrf(distance_covered, 5, 2, distance_covered_buff);

                client.publish(
                    generateRaceTopic(topic_buffer, sizeof(topic_buffer), race_id, "distance_covered"), 
                    distance_covered_buff
                );

                // Pour empêcher les publications superflues
                distance_previously_covered = distance_covered;
            }

            // -> vitesse moyenne
            dtostrf(average_speed, 5, 2, average_speed_buff);

            client.publish(
                generateRaceTopic(topic_buffer, sizeof(topic_buffer), race_id, "average_speed"),
                average_speed_buff
            );
           
            // -> sortie de route
            if (out_of_parcours != 0) {
                dtostrf(out_of_parcours, 5, 2, out_of_parcours_buff);

                client.publish(
                    generateRaceTopic(topic_buffer, sizeof(topic_buffer), race_id, "out_of_parcours"),
                    out_of_parcours_buff
                );
            }

            // -> nombre de colisions
            if (collision_duration != 0) {
                dtostrf(collision_duration, 5, 2, collision_duration_buff);

                client.publish(
                    generateRaceTopic(topic_buffer, sizeof(topic_buffer), race_id, "collision_duration"),
                    collision_duration_buff
                );
            }
        }
    
        // Condition pour terminer la course
        if (must_end_race_api_signal && previous_race_id != 0) {
            updateDistanceCovered();
            updateAverageSpeed();     // Mettre à jour la vitesse moyenne après la dernière distance ajoutée

            char message_buffer[64];

            const char *payload = "completed";

            client.publish(
                generateRaceTopic(message_buffer, sizeof(message_buffer), previous_race_id, "status"),
                payload
            );

            const char *message = "Fin de course";
            client.publish(generateRaceTopic(message_buffer, sizeof(message_buffer), previous_race_id, "message"), message);


            must_end_race_api_signal = false;
        }
    }
}

// put function definitions here:
void notifyClients()
{
    ws.textAll("ok");
}
TaskHandle_t AutoMoveTask;

//////////////////////////////////////////////////////////////////////////////////////////

void AutoMoveTaskCode(void *pvParameters)
{
    const int consecutiveReadings = 2;  // Nombre de lectures consécutives nécessaires pour confirmer une absence d'obstacle
    int noObstacleCounter = 0;  // Compteur de lectures consécutives sans obstacle

    while (true)
    {
        // Vérification des obstacles
        float distance = Get_Sonar();

        if (distance < 25.0)
        {
            Car_SetMode(0); // arrete le vehicule si obstacle detecté
            Serial.print("Obstacle detected! Distance: ");
            Serial.print(distance);
            Serial.println(" cm. Stopping the vehicle.");
            Motor_Move(0, 0, 0, 0);  // Stop the vehicle
            noObstacleCounter = 0;  // Réinitialiser le compteur car un obstacle est détecté
        }
        else
        {
            noObstacleCounter++;
            if (noObstacleCounter >= consecutiveReadings)
            {
                // Si aucun obstacle n'est détecté, activer le suivi de ligne
                Serial.println("No obstacle detected. Activating line tracking.");
                Car_SetMode(1);
                //Track_Car(1);

                
                noObstacleCounter = 0;  // Réinitialiser le compteur après avoir bougé
            }
        }

        delay(200);  // Shorter delay to check the distance more frequently
    }
}


// Déclarations globales


//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////


void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
    AwsFrameInfo *info = (AwsFrameInfo *)arg;

    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
    {
        data[len] = 0;

        StaticJsonDocument<200> doc;

        DeserializationError error = deserializeJson(doc, (char *)data);

        if (error)
        {
            Serial.print("deserializeJson() failed: ");
            Serial.println(error.c_str());
            return;
        }

        int cmd = doc["cmd"];

        if (cmd == 1)
        {
            JsonArray data = doc["data"];
            int data_0 = data[0];
            int data_1 = data[1];
            int data_2 = data[2];
            int data_3 = data[3];

            Motor_Move(data_0, data_1, data_2, data_3);
        }
        else if (2 == cmd)
        {
            int data = doc["data"];
            Emotion_SetMode(data);
            servoMoveCommand = true;
        }
        else if (3 == cmd)
        {
            JsonArray angles = doc["data"];
            int angle_0 = angles[0];
            int angle_1 = angles[1];
            Servo_1_Angle(angle_0); // Set the Angle value of servo 1 to 0 to 180°
            Servo_2_Angle(angle_1);
            headAngleCommand = true;
        }
        else if (4 == cmd)
        {
            int led_mode = doc["data"];
            WS2812_SetMode(led_mode);
            ledAnimationCommand = true;
        }
        else if (5 == cmd)
        {
            JsonArray led_color = doc["data"];
            int led_color_0 = led_color[0];
            int led_color_1 = led_color[1];
            int led_color_2 = led_color[2];
            int led_color_3 = led_color[3];

            WS2812_Set_Color_1(led_color_0, led_color_1, led_color_2, led_color_3);
            ledPrincipalColorCommand = true;
        }
        else if (6 == cmd)
        {
            JsonArray led_color_2 = doc["data"];
            int led_color_2_0 = led_color_2[0];
            int led_color_2_1 = led_color_2[1];
            int led_color_2_2 = led_color_2[2];
            int led_color_2_3 = led_color_2[3];

            WS2812_Set_Color_2(led_color_2_0, led_color_2_1, led_color_2_2, led_color_2_3);
        }
        else if (7 == cmd)
        {
            bool alarm = doc["data"] == 1;
            Buzzer_Alarm(alarm);
            buzzerAlarmCommand = true;
        }
        else if (8 == cmd)
        {
            JsonArray buzzer_data = doc["data"];
            int alarm_on = buzzer_data[0] == 1;
            int frequency_hz = buzzer_data[1];
            Buzzer_Variable(alarm_on, frequency_hz);
            buzzerVariableCommand = true;
        }
        else if (9 == cmd)
        {
            bool video_activation = doc["data"] == 1;
            videoFlag = video_activation;
            videoCommand = true;
        } 
        else if (10 == cmd) 
        {
            bool enable_auto_mode = doc["data"] == 1;

            if (enable_auto_mode) {
                xTaskCreatePinnedToCore(
                    AutoMoveTaskCode,   // Function that implements the task.
                    "AutoMoveTask",     // Name of the task.
                    10000,              // Stack size in words.
                    NULL,               // Task input parameter.
                    1,                  // Priority of the task.
                    &AutoMoveTask,      // Task handle.
                    0
                );                 // Core where the task should run.
            } else {
                vTaskDelete(AutoMoveTask);
                Car_SetMode(0);
                // Motor_Move(0, 0, 0, 0);  // Stop the vehicle
            }

            autoModeCommand = true;
        }
        else if (11 == cmd)
        {
            int car_mode = doc["data"] == 1;
            Car_SetMode(car_mode);
        }
        else if (12 == cmd) {
            // Permet d'activer le mode course de la voiture
            // Lorsque le mode course est activé, la voiture enregistre et génère des données de course
            // Ces données sont ensuite envoyées au un serveur MQTT
            // body : [race_id, race_mode]

            JsonArray data = doc["data"];

            bool wasInRaceMode = race_mode == true;

            race_id = data[0];

            race_mode = data[1] == 1;

            if (! race_mode && wasInRaceMode) {

                resetRaceDataToDefault();

                must_end_race_api_signal = true;
            }

            if (race_mode && race_id != 0) {
                raceStartingModeCommand = true;

                race_start_time = millis();
            }
        }
        else if (cmd == 20) // cmd start race
        {
            startTime = millis();

            JsonArray data = doc["data"];
            
            int data_0 = data[0];
            
            int data_1 = data[1];
            
            int data_2 = data[2];
            
            int data_3 = data[3];

            Motor_Move(data_0, data_1, data_2, data_3);

            constantSpeed = getConstantSpeed();

            //topic pour la vitesse constante
            char message[20];
            dtostrf(constantSpeed, 5, 2, message);
            client.publish("esp32/speed", message); 

            newRaceMessage = true;
        }
        // cmd stop (mais à changer pour changer la logique du stop)
        else if (cmd == 21)
        {
            endTime = millis();

            Motor_Move(0, 0, 0, 0);

            // Temps de la course en ms
            commandDuration = endTime - startTime;

            // Distance parcourue par la voiture en mètre
            distanceCar = constantSpeed * (commandDuration / 1000.0);

            newRaceMessage = true;

            newDistanceMessage = true;
        }

        notifyClients();
    }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    switch (type)
    {
    case WS_EVT_CONNECT:
        HandleWebSocketConnectionSuccess(client->id(), client->remoteIP().toString());
        break;
    case WS_EVT_DISCONNECT:
        HandleWebSocketDisconnection(client->id());
        break;
    case WS_EVT_DATA:
        handleWebSocketMessage(arg, data, len);
        break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
        break;
    }
}

void initWebSocket()
{
    ws.onEvent(onEvent);
    server.addHandler(&ws);
}

void reconnect()
{
    // Loop until we're reconnected
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect("ESP8266Client"))
        {
            Serial.println("connected");
            // Subscribe
            client.subscribe("esp32/output");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

void loopTask_Camera(void *pvParameters)
{
    while (1)
    {
        char size_buf[12];
        WiFiClient wf_client = server_Camera.available(); // listen for incoming clients
        if (wf_client)
        { // if you get a client
            Serial.println("Camera_Server connected to a client.");
            if (wf_client.connected())
            {
                camera_fb_t *fb = NULL;
                wf_client.write("HTTP/1.1 200 OK\r\nAccess-Control-Allow-Origin: *\r\nContent-Type: multipart/x-mixed-replace; boundary=" STREAM_CONTENT_BOUNDARY "\r\n");
                while (wf_client.connected())
                { // loop while the client's connected
                    if (videoFlag == 1)
                    {
                        fb = esp_camera_fb_get();
                        if (fb != NULL)
                        {
                            wf_client.write("\r\n--" STREAM_CONTENT_BOUNDARY "\r\n");
                            wf_client.write("Content-Type: image/jpeg\r\nContent-Length: ");
                            sprintf(size_buf, "%d\r\n\r\n", fb->len);
                            wf_client.write(size_buf);
                            wf_client.write(fb->buf, fb->len);

                            uint8_t slen[4];
                            slen[0] = fb->len >> 0;
                            slen[1] = fb->len >> 8;
                            slen[2] = fb->len >> 16;
                            slen[3] = fb->len >> 24;
                            wf_client.write(slen, 4);
                            wf_client.write(fb->buf, fb->len);
                            Serial.println("Camera send");
                            esp_camera_fb_return(fb);
                        }
                    }
                }
                // close the connection:
                wf_client.stop();
                Serial.println("Camera Client Disconnected.");
                ESP.restart();
            }
        }
    }
}

// void printWiFiStatus() {
//     Serial.print("SSID: ");
//     Serial.println(WiFi.SSID());

//     long rssi = WiFi.RSSI();
//     Serial.print("Signal strength (RSSI): ");
//     Serial.print(rssi);
//     Serial.println(" dBm");

//     Serial.print("IP Address: ");
//     Serial.println(WiFi.localIP());

//     Serial.print("Subnet Mask: ");
//     Serial.println(WiFi.subnetMask());

//     Serial.print("Gateway IP: ");
//     Serial.println(WiFi.gatewayIP());

//     Serial.print("DNS IP 1: ");
//     Serial.println(WiFi.dnsIP(0));

//     Serial.print("DNS IP 2: ");
//     Serial.println(WiFi.dnsIP(1));
// }

// void scanNetworks() {
//     int n = WiFi.scanNetworks();
//     Serial.println("Scan done");
//     if (n == 0) {
//         Serial.println("No networks found");
//     } else {
//         Serial.print(n);
//         Serial.println(" networks found");
//         for (int i = 0; i < n; ++i) {
//             Serial.print(i + 1);
//             Serial.print(": ");
//             Serial.print(WiFi.SSID(i));
//             Serial.print(" (");
//             Serial.print(WiFi.RSSI(i));
//             Serial.print(")");
//             Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
//             delay(10);
//         }
//     }
//     Serial.println("");
// }

// void WiFi_Init() {
//     Serial.println("Initializing WiFi...");

//     WiFi.mode(WIFI_STA); // Set ESP32 to station mode

//     if (!WiFi.config(localIP, localGateway, localSubnet, primaryDNS, secondaryDNS)) {
//         Serial.println("Failed to configure static IP");
//     }

//     //scanNetworks(); // Scan for available networks

//     WiFi.begin(ssid_wifi, password_wifi);
//     Serial.print("Connecting to WiFi");

//     unsigned long startAttemptTime = millis();

//     // Wait for connection with a timeout
//     while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
//         delay(500);
//         Serial.print(".");
//     }

//     if (WiFi.status() != WL_CONNECTED) {
//         Serial.println("\nFailed to connect to WiFi");
//         Serial.print("SSID: ");
//         Serial.print(ssid_wifi);
//         Serial.print(", Password: ");
//         Serial.println(password_wifi);

//         // Detailed error information
//         Serial.print("WiFi status: ");
//         Serial.println(WiFi.status());
//         switch (WiFi.status()) {
//             case WL_IDLE_STATUS:
//                 Serial.println("WiFi is in idle status.");
//                 break;
//             case WL_NO_SSID_AVAIL:
//                 Serial.println("No SSID available.");
//                 break;
//             case WL_SCAN_COMPLETED:
//                 Serial.println("Scan completed.");
//                 break;
//             case WL_CONNECTED:
//                 Serial.println("Successfully connected.");
//                 break;
//             case WL_CONNECT_FAILED:
//                 Serial.println("Failed to connect.");
//                 break;
//             case WL_CONNECTION_LOST:
//                 Serial.println("Connection lost.");
//                 break;
//             case WL_DISCONNECTED:
//                 Serial.println("Disconnected.");
//                 break;
//             default:
//                 Serial.println("Unknown status.");
//                 break;
//         }
//     } else {
//         Serial.println("\nConnected to WiFi");
//         //printWiFiStatus();
//     }
// }