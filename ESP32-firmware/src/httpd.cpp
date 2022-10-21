#include "httpd.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
//#include <LITTLEFS.h>
#include <LittleFS.h>
#include <WiFi.h>

#include "ESPAsyncWebServer.h"
#include "RF4463.h"
#include "decoder.h"
#include "encoder.h"
#include "jwifi.h"
#include "pagerkeepalive.h"
#include "settings.h"
#include "statustracker.h"

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
bool HTTPrestartNeeded = false;

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    sendStatus(STATUS_WIFI_ACTIVITY);
    switch (type) {
        case WS_EVT_CONNECT:
            //client connected
            ets_printf("ws[%s][%u] connect\n", server->url(), client->id());
            //client->ping();
            sendStatus(STATUS_NEW_WS_CONNECTION);
            break;
        case WS_EVT_DISCONNECT:
            //client disconnected
            ets_printf("ws[%s][%u] disconnect: %u\n", server->url(), client->id());
            break;
        case WS_EVT_ERROR:
            //error was received from the other end
            ets_printf("WS Error received :(\n\n");
            //ets_printf("ws[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t *)arg), (char *)data);
            break;
        case WS_EVT_PONG:
            //pong message was received (in response to a ping request maybe)
            ets_printf("ws[%s][%u] pong[%u]: %s\n", server->url(), client->id(), len, (len) ? (char *)data : "");
            break;
        case WS_EVT_DATA:
            //data packet
            AwsFrameInfo *info = (AwsFrameInfo *)arg;
            if (info->final && info->index == 0 && info->len == len) {
                //the whole message is in a single frame and we got all of it's data
                ets_printf("ws[%s][%u] %s-message[%llu]: ", server->url(), client->id(), (info->opcode == WS_TEXT) ? "text" : "binary", info->len);
                if (info->opcode == WS_TEXT) {
                    data[len] = 0;
                    ets_printf("%s\n", (char *)data);
                } else {
                    for (size_t i = 0; i < info->len; i++) {
                        ets_printf("%02x ", data[i]);
                    }
                    ets_printf("\n");
                }
                if (info->opcode == WS_TEXT)
                    client->text("{\"status\":\"received\"}");
                else
                    client->binary("{\"status\":\"received\"}");
            } else {
                //message is comprised of multiple frames or the frame is split into multiple packets
                if (info->index == 0) {
                    if (info->num == 0)
                        ets_printf("ws[%s][%u] %s-message start\n", server->url(), client->id(), (info->message_opcode == WS_TEXT) ? "text" : "binary");
                    ets_printf("ws[%s][%u] frame[%u] start[%llu]\n", server->url(), client->id(), info->num, info->len);
                }

                ets_printf("ws[%s][%u] frame[%u] %s[%llu - %llu]: ", server->url(), client->id(), info->num, (info->message_opcode == WS_TEXT) ? "text" : "binary", info->index, info->index + len);
                if (info->message_opcode == WS_TEXT) {
                    data[len] = 0;
                    ets_printf("%s\n", (char *)data);
                } else {
                    for (size_t i = 0; i < len; i++) {
                        ets_printf("%02x ", data[i]);
                    }
                    ets_printf("\n");
                }

                if ((info->index + len) == info->len) {
                    ets_printf("ws[%s][%u] frame[%u] end[%llu]\n", server->url(), client->id(), info->num, info->len);
                    if (info->final) {
                        ets_printf("ws[%s][%u] %s-message end\n", server->url(), client->id(), (info->message_opcode == WS_TEXT) ? "text" : "binary");
                        if (info->message_opcode == WS_TEXT)
                            client->text("{\"status\":\"received\"}");
                        else
                            client->binary("{\"status\":\"received\"}");
                    }
                }
            }
            break;
    }
}

void webSocketSendProcess(void *parameter) {
    uint32_t ulNotificationValue;
    status.websocketUpdater = xTaskGetCurrentTaskHandle();
    while (true) {
        ulNotificationValue = ulTaskNotifyTake(pdTRUE, 1000 / portTICK_RATE_MS);
        if (ulNotificationValue == 0) {  // timeout, so every 1s
            ws.cleanupClients();
        } else {
            if (ws.count())
                sendStatus(STATUS_WIFI_ACTIVITY);
            DynamicJsonDocument doc(1500);
            if (ulNotificationValue & WS_SEND_MODE_STATUS) {
                doc["rxActive"] = status.rxActive;
                doc["txActive"] = status.txActive;
                doc["freq"] = status.freq;
                doc["txMode"] = status.currentmode;
            }
            if (ulNotificationValue & WS_SEND_SIGNAL) {
                doc["rxSignal"] = status.rxSignal;
            }
            if (ulNotificationValue & WS_SEND_MODMODE) {
                switch (settings.current.mode) {
                    case 0:
                        doc["mode"] = "MODE_FLEX";
                        break;
                    case 1:
                        doc["mode"] = "MODE_POCSAG";
                        break;
                    case 2:
                        doc["mode"] = "MODE_TX_ONLY";
                        break;
                    case 3:
                        doc["mode"] = "MODE_FLEX_SYNCED";
                        break;
                }
                doc["baud"] = settings.current.baud;
            }
            if (ulNotificationValue & WS_SEND_FRAME_STATUS) {  // flex frame counter update
                JsonArray statusframes = doc.createNestedArray("frames");
                for (uint8_t c = 0; c < STATUSFRAMELISTSIZE; c++) {
                    if (statusframearr[c]) {
                        JsonObject statusframe = statusframes.createNestedObject();
                        statusframe["frame"] = statusframearr[c]->frameno;
                        statusframe["isTX"] = statusframearr[c]->isTX;
                        statusframe["freq"] = statusframearr[c]->freq;
                        statusframe["txSkipped"] = statusframearr[c]->txCancelled;
                        switch (statusframearr[c]->rxtype) {
                            case flexsynctype::SYNC_FLEX_1600:
                                statusframe["rxType"] = "FLEX_1600";
                                break;
                            case flexsynctype::SYNC_FLEX_3200_2:
                                statusframe["rxType"] = "FLEX_3200_2";
                                break;
                            case flexsynctype::SYNC_FLEX_3200_4:
                                statusframe["rxType"] = "FLEX_3200_4";
                                break;
                            case flexsynctype::SYNC_FLEX_6400:
                                statusframe["rxType"] = "FLEX_3200_4";
                                break;
                            default:
                                break;
                        }
                        switch (statusframearr[c]->txformat) {
                            case txframe::FORMAT_FLEX:
                                statusframe["txType"] = "FLEX";
                                break;
                            case txframe::FORMAT_POCSAG:
                                statusframe["txType"] = "POCSAG";
                                break;
                            case txframe::FORMAT_IDLE:
                                statusframe["txType"] = "IDLE";
                                break;
                            case txframe::FORMAT_BLOCKED:
                                statusframe["txType"] = "BLOCKED";
                                break;
                            default:
                                break;
                        }
                    }
                }
            }
            size_t len = measureJson(doc);
            xSemaphoreTake(wsMutex, portMAX_DELAY);
            auto buffer = std::make_shared<std::vector<uint8_t>>(len);
            serializeJson(doc, buffer->data(), len);
            ws.textAll(buffer);
            xSemaphoreGive(wsMutex);
        }
    }
}

void httpdProcess(void *parameter) {
    xTaskCreate(webSocketSendProcess, "webSocketSendProcess", 10000, NULL, 5, NULL);
    wsMutex = xSemaphoreCreateMutex();
    bool daemonRunning = false;
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    while (true) {
        if (HTTPrestartNeeded) {
            if (daemonRunning) {
                server.end();
                daemonRunning = false;
            }
            initHTTPD();
            daemonRunning = true;
            HTTPrestartNeeded = false;
        }
        if (wifiConnected == true && !daemonRunning) {
            initHTTPD();
            daemonRunning = true;
        } else if (!wifiConnected && daemonRunning) {
            server.end();
        }
        //
        vTaskDelay(20000 / portTICK_PERIOD_MS);
    }
}

void notFound(AsyncWebServerRequest *request) {
    sendStatus(STATUS_WIFI_ACTIVITY);
    //List all parameters
    int params = request->params();
    for (int i = 0; i < params; i++) {
        AsyncWebParameter *p = request->getParam(i);
        if (p->isFile()) {  //p->isPost() is also true
            Serial.printf("FILE[%s]: %s, size: %u\n", p->name().c_str(), p->value().c_str(), p->size());
        } else if (p->isPost()) {
            Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
        } else {
            Serial.printf("GET[%s]: %s\n", p->name().c_str(), p->value().c_str());
        }
    }

    AsyncResponseStream *response = request->beginResponseStream("text/html");
    response->print("<!DOCTYPE html><html><body><h1>Send Message</h1><p></p><form action=\"/sendpocsag\" method=\"POST\">");
    response->print("Frequency: <input placeholder=\"Frequency\" name=\"freq\" size=20/><br/>");
    response->print("Speed: <select name=\"speed\"><option value=\"512\">512</option><option value=\"1200\">1200</option><option value=\"2400\">2400</option></select><br/>");
    response->print("RIC: <input placeholder=\"RIC\" name=\"ric\" size=12/>Function:<select name=\"func\"><option value=\"0\">A</option><option value=\"1\">B</option>");
    response->print("<option value=\"2\">C</option><option value=\"3\">D</option></select><br/>");
    response->print("Type: <select name=\"type\"><option value=\"0\">Numeric</option><option value=\"1\">TONE ONLY</option><option value=\"3\">Alpha</option></select><br/>");
    response->print("<textarea name=\"text\" placeholder=\"Hello world\" cols=36 rows=8></textarea><br/><input type=\"submit\" value=\"Send\"></form>");

    response->print("<br/><br/><form action=\"/sendflex\" method=\"POST\">");
    response->print("Frequency: <input placeholder=\"Frequency\" name=\"freq\" size=20/><br/>");
    response->print("RIC: <input placeholder=\"E\" value=\"E\" name=\"pre\" size=1/><input placeholder=\"RIC\" name=\"ric\" size=12/><input type=\"checkbox\" name=\"isnumeric\">Numeric<br/>");
    response->print("<textarea name=\"text\" placeholder=\"Hello world\" cols=36 rows=8></textarea><br/><input type=\"submit\" value=\"Send\"></form>");

    //response->print("<!DOCTYPE html><html><head><title>Captive Portal</title></head><body>");
    //response->print("<p>This is out captive portal front page.</p>");
    //response->printf("<p>You were trying to reach: http://%s%s</p>", request->host().c_str(), request->url().c_str());
    //response->printf("<p>Try opening <a href='http://%s'>this link</a> instead</p>", WiFi.softAPIP().toString().c_str());
    response->print("</body></html>");
    request->send(response);
}

void sendPOCSAG(AsyncWebServerRequest *request) {
    String message;
    StaticJsonDocument<100> data;
    message = request->getParam("text", true)->value();
    request->getParam("text", true)->value();
    uint32_t ric = request->getParam("ric", true)->value().toInt();
    uint16_t baud = request->getParam("speed", true)->value().toInt();
    uint8_t func = request->getParam("func", true)->value().toInt();
    uint32_t freq = request->getParam("freq", true)->value().toInt();
    uint8_t type = request->getParam("type", true)->value().toInt();
    enum pocsagaddr::type sendtype = pocsagaddr::TONE;
    enum pocsagaddr::func sendfunc = pocsagaddr::A;
    switch (type) {
        case 0:
            sendtype = pocsagaddr::NUMERIC;
            break;
        case 1:
            sendtype = pocsagaddr::TONE;
            break;
        case 3:
            sendtype = pocsagaddr::ALPHA;
            break;
    }
    switch (func) {
        case 0:
            sendfunc = pocsagaddr::A;
            break;
        case 1:
            sendfunc = pocsagaddr::B;
            break;
        case 2:
            sendfunc = pocsagaddr::C;
            break;
        case 3:
            sendfunc = pocsagaddr::D;
            break;
    }

    if (RF4463::checkAvail(freq, RF4463::rftype::POCSAG_2400_TX)) {
        pocsagaddr adr(ric, sendtype, sendfunc);
        int frame = addPOCSAG(&adr, freq, baud, (char *)message.c_str());
        if (frame == -1) {
            data["error"] = "NO_SLOTS_AVAILABLE";
        } else {
            data["status"] = "QUEUED";
            data["transmissionBeginFrame"] = frame;
        }
    } else {
        data["error"] = "FREQUENCY_NOT_AVAIL";
    }
    String response;
    serializeJson(data, response);
    request->send(200, "application/json", response);
}

void sendFLEX(AsyncWebServerRequest *request) {
    String message;
    StaticJsonDocument<100> data;
    message = request->getParam("text", true)->value();
    request->getParam("text", true)->value();
    uint32_t ric = request->getParam("ric", true)->value().toInt();
    uint32_t freq = request->getParam("freq", true)->value().toInt();
    bool isNumeric = false;
    if(request->hasParam("isnumeric",true)){
        isNumeric = true;
    }
    if (RF4463::checkAvail(freq, RF4463::rftype::FLEX_6400_TX)) {
        int frame = addFLEX('E', ric, freq, (char *)message.c_str(), isNumeric);
        if (frame == -1) {
            data["error"] = "NO_SLOTS_AVAILABLE";
        } else {
            data["status"] = "QUEUED";
            data["transmissionBeginFrame"] = frame;
        }
    } else {
        data["error"] = "FREQUENCY_NOT_AVAIL";
    }
    String response;
    serializeJson(data, response);
    request->send(200, "application/json", response);
}

void resetwifi(AsyncWebServerRequest *request) {
    wifiResetConfig();
}

void initHTTPD() {
    Serial.print("Starting HTTPD... ");
    //WiFi.softAP("esp-captive");
    //dnsServer.start(53, "*", WiFi.softAPIP());
    //server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);//only when requested from AP
    //more handlers...
    server.onNotFound(notFound);
    ws.onEvent(onEvent);
    server.addHandler(&ws);
    server.on("/sendpocsag", HTTP_POST, sendPOCSAG);
    server.on("/sendflex", HTTP_POST, sendFLEX);
    server.on("/settings", HTTP_GET, getSettings);
    server.on("/forgetwifi", HTTP_GET, resetwifi);
    server.on("/settings", HTTP_POST, saveSettingsComplete, NULL, saveSettings);
    server.on("/rxmode", HTTP_GET, getMode);
    server.on("/rxmode", HTTP_POST, getMode, NULL, setMode);
    server.on("/idlelist", HTTP_GET, getIdleList);
    server.on("/idlelist", HTTP_POST, getIdleList, NULL, delFromIdleList);
    server.serveStatic("/", LittleFS, "/www/");
    //server.onRequestBody(saveSettings);
    server.begin();
    Serial.print("Running on ");
    Serial.print(WiFi.localIP());
    Serial.println("");
}
