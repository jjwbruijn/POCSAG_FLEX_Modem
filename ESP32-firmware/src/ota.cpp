#include <Update.h>
#include <WiFiClientSecure.h>
extern const char* rootCACertificate;
//extern const char* API_KEY;
volatile int contentLength = 0;
volatile bool isValidContentType = false;

const char* rootCACertificate =
    "-----BEGIN CERTIFICATE-----\n"
    "-----END CERTIFICATE-----\n";

inline String getHeaderValue(String header, String headerName) {
    return header.substring(strlen(headerName.c_str()));
}

void processOTAUpdate(void) {
    WiFiClientSecure client;  // = new WiFiClientSecure;
    client.setCACert(rootCACertificate);
    if (!client.connect("OTAHOST", 443)) {
        Serial.println("Cannot connect");
        return;
    }

    bool redirect = true;
    while (redirect) {
        client.print(String("GET ") + "/getfirmware HTTP/1.1\r\n");
        client.print(String("Host: OTAHOST\r\n"));
        client.print("Cache-Control: no-cache\r\n");
        client.print("User-Agent: ESP32-http-Update\r\n");
        client.print("x-ESP32-Chip-ID: " + String(ESP.getChipRevision())+"\r\n");
        client.print("x-ESP32-STA-MAC: " + WiFi.macAddress()+"\r\n");
        client.print("x-ESP32-AP-MAC: " + WiFi.softAPmacAddress()+"\r\n");
        client.print("x-ESP32-free-space: " + String(ESP.getFreeSketchSpace())+"\r\n");
        client.print("x-ESP32-sketch-size: " + String(ESP.getSketchSize())+"\r\n");
        client.print("x-ESP32-sketch-md5: " + String(ESP.getSketchMD5())+"\r\n");
        client.print("x-ESP32-chip-size: " + String(ESP.getFlashChipSize())+"\r\n");
        client.print("x-ESP32-sdk-version: " + String(ESP.getSdkVersion())+"\r\n");
        client.print("Connection: close\r\n\r\n");

        unsigned long timeout = millis();
        while (client.available() == 0) {
            if (millis() - timeout > 5000) {
                Serial.println("Client Timeout !");
                client.stop();
                return;
            }
        }

        while (client.available()) {
            String line = client.readStringUntil('\n');
            // Check if the line is end of headers by removing space symbol
            line.trim();
            // if the the line is empty, this is the end of the headers
            if (!line.length()) {
                break;  // proceed to OTA update
            }

            // Check allowed HTTP responses
            if (line.startsWith("HTTP/1.1")) {
                if (line.indexOf("200") > 0) {
                    //Serial.println("Got 200 status code from server. Proceeding to firmware flashing");
                    redirect = false;
                } else if (line.indexOf("302") > 0) {
                    //Serial.println("Got 302 status code from server. Redirecting to the new address");
                    redirect = true;
                } else if (line.indexOf("304") > 0) {
                    Serial.println("No update available...");
                    client.flush();
                    return;
                } else {
                    //Serial.println("Could not get a valid firmware url");
                    //Unexptected HTTP response. Retry or skip update?
                    redirect = false;
                }
            }
            Serial.println(line);
            // Checking headers
            if (line.startsWith("Content-Length: ")) {
                contentLength = atoi((getHeaderValue(line, "Content-Length: ")).c_str());
                Serial.println("Got " + String(contentLength) + " bytes from server");
            }

            if (line.startsWith("Content-Type: ")) {
                String contentType = getHeaderValue(line, "Content-Type: ");
                //Serial.println("Got " + contentType + " payload.");
                if (contentType == "application/octet-stream") {
                    isValidContentType = true;
                }
            }
        }

        // check whether we have everything for OTA update
        if (contentLength && isValidContentType) {
            if (Update.begin(contentLength)) {
                //killThreads();
                Serial.println("Starting Over-The-Air update. This may take some time to complete ...");
                size_t written = Update.writeStream(client);
                if (written == contentLength) {
                    Serial.println("Written : " + String(written) + " successfully");
                } else {
                    Serial.println("Written only : " + String(written) + "/" + String(contentLength) + ". Retry?");
                    // Retry??
                }

                if (Update.end()) {
                    if (Update.isFinished()) {
                        Serial.println("OTA update has successfully completed. Rebooting ...");
                        ESP.restart();
                    } else {
                        Serial.println("Something went wrong! OTA update hasn't been finished properly.");
                    }
                } else {
                    Serial.println("An error Occurred. Error #: " + String(Update.getError()));
                }
            } else {
                Serial.println("There isn't enough space to start OTA update");
                client.flush();
            }
        } else {
            Serial.println("There was no valid content in the response from the OTA server!");
            client.flush();
        }
    }
}