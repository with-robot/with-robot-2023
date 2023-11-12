#include <string>
#include <wifi.h>

#define LED_PIN 4

#ifndef StreamIO_H
#define StreamIO_H

class StreamIO
{
private:
    // command
    byte CMD_CFG = 0x01;
    byte NOTI_CAM = 0x81;
    // error
    byte REQUEST = 0x00;
    byte RES_OK = 0x01;
    byte UNK_CMD = 0xEE;

    WiFiClient tcpClient;

    byte read_buf[256] = {
        0xFF,
        0,
    };
    byte send_buf[65536] = {
        0xFF,
        0,
    };

public:
    StreamIO() = default;

    StreamIO(WiFiClient client)
    {
        tcpClient = client;
    }

    // Read the request line. The string from the JavaScript client ends with a newline.
    std::string readRequest(WiFiClient *client)
    {
        std::string request = "";

        // Loop while the client is connected.
        while (client->connected())
        {
            // Read available bytes.
            while (client->available())
            {
                // Read a byte.
                char c = client->read();

                // Print the value (for debugging).
                Serial.write(c);

                // Exit loop if end of line.
                if ('\n' == c)
                {
                    return request;
                }

                // Add byte to request line.
                request += c;
            }
        }
        return request;
    }

    void sendTCP(byte cmd, byte seq, byte rsp, byte *buf, int len)
    {
        send_buf[0] = 0xFF;
        send_buf[1] = cmd;
        send_buf[2] = seq;
        send_buf[3] = rsp;
        for (int i = 0; i < 4; i++)
        {
            send_buf[i + 4] = 0xFF & (len >> i * 8);
        }
        memcpy(&send_buf[8], buf, len);
        int checksum = 0;
        for (int i = 0; i < len + 8; i++)
        {
            checksum += send_buf[i];
        }
        send_buf[8 + len] = checksum & 0xFF;
        tcpClient.write(send_buf, len + 9);
    }

    int readTCP()
    {
        // recv command
        tcpClient.read(read_buf, 8);

        if (read_buf[0] == 0xFF)
        {
            int len = 0;
            for (int i = 0; i < 4; i++)
            {
                len += (read_buf[i + 4] << i * 8);
            }
            tcpClient.read(&read_buf[8], len + 1);
            int checksum = 0;
            for (int i = 0; i < len + 8; i++)
            {
                checksum += read_buf[i];
            }
            if ((0xFF & checksum) == read_buf[8 + len])
            {
                if (read_buf[1] == CMD_CFG)
                {
                    setConfig(len);
                }
                else
                {
                    memcpy(send_buf, read_buf, len + 8);
                    send_buf[3] = UNK_CMD;
                }
                // make response and send
                len = 0;
                for (int i = 0; i < 4; i++)
                {
                    len += (send_buf[i + 4] << i * 8);
                }

                return len;
            }
        }
    }

    void respTCP(int len)
    {
        int checksum = 0;
        for (int i = 0; i < len + 8; i++)
        {
            checksum += send_buf[i];
        }
        send_buf[8 + len] = checksum & 0xFF;
        tcpClient.write(send_buf, len + 9);
    }

    void setConfig(int len)
    {
        memcpy(send_buf, read_buf, len + 8);
        send_buf[3] = RES_OK;
    }

    void executeRequest(WiFiClient *client, String *request)
    {
        char command = readCommand(request);
        int n = readParam(request);
        if ('O' == command)
        {
            pinMode(n, OUTPUT);
        }
        else if ('I' == command)
        {
            pinMode(n, INPUT);
        }
        else if ('L' == command)
        {
            digitalWrite(n, LOW);
        }
        else if ('H' == command)
        {
            digitalWrite(n, HIGH);
        }
        else if ('R' == command)
        {
            sendResponse(client, String(digitalRead(n)));
        }
        else if ('A' == command)
        {
            sendResponse(client, String(analogRead(n)));
        }
    }

    // Read the command from the request string.
    char readCommand(String *request)
    {
        String commandString = request->substring(0, 1);
        return commandString.charAt(0);
    }

    // Read the parameter from the request string.
    int readParam(String *request)
    {
        // This handles a hex digit 0 to F (0 to 15).
        char buffer[2];
        buffer[0] = request->charAt(1);
        buffer[1] = 0;
        return (int)strtol(buffer, NULL, 16);
    }

    void sendResponse(WiFiClient *client, String response)
    {
        // Send response to client.
        client->println(response);

        // Debug print.
        Serial.println("sendResponse:");
        Serial.println(response);
    }

    void printWifiStatus()
    {
        Serial.println("WiFi status");

        // Print network name.
        Serial.print("  SSID: ");
        Serial.println(WiFi.SSID());

        // Print WiFi shield IP address.
        IPAddress ip = WiFi.localIP();
        Serial.print("  IP Address: ");
        Serial.println(ip);

        // Print the signal strength.
        long rssi = WiFi.RSSI();
        Serial.print("  Signal strength (RSSI):");
        Serial.print(rssi);
        Serial.println(" dBm");
    }
};

#endif