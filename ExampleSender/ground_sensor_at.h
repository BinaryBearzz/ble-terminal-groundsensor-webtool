#ifndef GROUND_SENSOR_AT_H
#define GROUND_SENSOR_AT_H

#include <Arduino.h>

// GroundSensor AT-command helper library.
// Wraps sending/receiving AT commands over a Stream (HardwareSerial or
// SoftwareSerial) and classifying the GroundSensor's response, per
// uart-command-spec.html sections 4/5/6.

// Result of a GroundSensor response.
enum class GsStatus
{
    NONE,   // no response received before timeout
    OK,     // a line starting with "OK" was seen (may still be followed by more lines)
    ERROR,  // a line starting with "ERROR" was seen
    TIMEOUT // final/terminator line never arrived within timeout
};

struct GsResponse
{
    GsStatus status = GsStatus::NONE;
    String raw;      // full raw text received (all lines, newline separated)
    String lastLine; // last non-empty line received (often the final status line)
};

class GroundSensor
{
public:
    // How long to wait for a single "AT" ping response before retrying.
    static const unsigned long PING_TIMEOUT_MS = 500;
    // How many "AT" pings to attempt before giving up on the connection check.
    static const int PING_MAX_ATTEMPTS = 20;

    // Bind the stream used to talk to the GroundSensor (e.g. &Serial1).
    void begin(Stream *stream) { _stream = stream; }

    bool isConnected() const { return _connected; }

    // Repeatedly send a bare "AT" ping and wait for "OK" so we know the
    // GroundSensor is actually alive on the UART before issuing real commands.
    // Returns true once "OK" is received, false if it never responds.
    bool waitForReady()
    {
        if (!_stream)
            return false;

        for (int attempt = 1; attempt <= PING_MAX_ATTEMPTS; ++attempt)
        {
            // Flush any stale bytes left over from wiring noise/boot garbage.
            while (_stream->available())
                _stream->read();

            _stream->println("AT");
            Serial.println("TX -> AT (ping " + String(attempt) + "/" + String(PING_MAX_ATTEMPTS) + ")");

            GsResponse resp = readResponse(PING_TIMEOUT_MS);
            logResponse(resp);

            if (resp.status == GsStatus::OK)
            {
                Serial.println("GroundSensor connected.");
                _connected = true;
                return true;
            }
            delay(200);
        }

        Serial.println("GroundSensor did not respond to AT - check wiring (TX/RX crossed), baud rate, and power.");
        _connected = false;
        return false;
    }

    // Send raw line to ground sensor, print to Serial monitor, then wait for
    // and handle the device's response (OK / ERROR / TIMEOUT).
    GsResponse send(const String &cmdLine, unsigned long respTimeoutMs = 2000)
    {
        if (!_stream)
            return GsResponse();

        // Flush any pending input that might be leftover from previous
        // operations so we don't misattribute an earlier reply to this
        // command.
        while (_stream->available())
            _stream->read();

        _stream->println(cmdLine);
        Serial.println("TX -> " + cmdLine);

        // Small pause to give the device time to start replying on slower
        // processors or when it must process the request.
        delay(30);

        GsResponse resp = readResponse(respTimeoutMs);
        return logResponse(resp);
    }

    // Wrapper for AT+SEND with channel and project id. Returns the parsed
    // response so callers can branch on GsStatus::OK / ERROR / TIMEOUT.
    GsResponse sendAT_SEND(int ch, int projectId, const String &payloadCsv)
    {
        String cmd = "AT+SEND=" + String(ch) + "," + String(projectId) + "," + payloadCsv;
        // AT+SEND can take longer to process and produce multi-line progress,
        // so use a larger timeout by default.
        return send(cmd, 5000);
    }

    // Wrapper for AT+SYNCTIME with unix parameter.
    GsResponse sendAT_SYNCTIME(long unixTime)
    {
        String cmd = "AT+SYNCTIME=" + String(unixTime);
        // Give the device more time to accept and confirm time sync.
        return send(cmd, 5000);
    }

private:
    Stream *_stream = nullptr;
    bool _connected = false;

    // Read one line (until '\n') from the stream, blocking up to timeoutMs.
    // Returns empty string on timeout with no data.
    String readLine(unsigned long timeoutMs)
    {
        String line;
        unsigned long start = millis();
        while (millis() - start < timeoutMs)
        {
            while (_stream->available())
            {
                char c = (char)_stream->read();
                if (c == '\n')
                {
                    line.trim();
                    return line;
                }
                if (c != '\r')
                    line += c;
                start = millis(); // reset timeout while data keeps arriving
            }
            delay(2);
        }
        line.trim();
        return line;
    }

    // Line patterns that terminate a multi-line GroundSensor reply (see spec sections 4/5).
    static bool isFinalLine(const String &line)
    {
        if (line.length() == 0)
            return false;
        if (line.startsWith("ERROR"))
            return true; // any ERROR,... / ERROR:...
        if (line.indexOf("FINISH") >= 0)
            return true; // SEND/POST/SATE final line
        if (line.indexOf("FINISHED") >= 0)
            return true; // INFO/HELP terminator
        if (line == "OK")
            return true; // plain AT ping
        if (line.startsWith("OK,SETTIME") || line.startsWith("OK,SYNCTIME"))
            return true;
        if (line.startsWith("OK,RSTTIME") || line.startsWith("OK,SETDID"))
            return true;
        if (line.startsWith("OK:INITSEND")){
            return true;
        }
        if (line.startsWith("OK,KEY") || line.startsWith("OK,RSTKEY"))
            return true;
        if (line.startsWith("OK,NEWSECUREKEY") || line.startsWith("OK,NEWAESKEY"))
            return true;
        return false;
    }

    // Read the GroundSensor's response until a recognized terminator line is
    // seen or the overall timeout elapses. Classifies the result as
    // OK / ERROR / TIMEOUT.
    GsResponse readResponse(unsigned long timeoutMs)
    {
        GsResponse result;
        if (!_stream)
            return result;

        unsigned long start = millis();
        bool gotAnyLine = false;

        while (millis() - start < timeoutMs)
        {
            unsigned long remaining = timeoutMs - (millis() - start);
            String line = readLine(remaining);
            if (line.length() == 0)
                continue; // timed out waiting for this line, loop checks overall timeout

            gotAnyLine = true;
            if (result.raw.length())
                result.raw += '\n';
            result.raw += line;
            result.lastLine = line;

            if (line.startsWith("ERROR"))
            {
                result.status = GsStatus::ERROR;
                return result;
            }
            if (isFinalLine(line))
            {
                result.status = GsStatus::OK;
                return result;
            }
            // Otherwise keep looping: multi-line response (e.g. AT+INFO, AT+HELP, AT+POST progress)
        }

        result.status = gotAnyLine ? GsStatus::TIMEOUT : GsStatus::NONE;
        return result;
    }

    // Print a GsResponse to the Serial monitor and return it for further handling.
    static GsResponse logResponse(const GsResponse &resp)
    {
        switch (resp.status)
        {
        case GsStatus::OK:
            Serial.println("RX <- [OK] " + resp.lastLine);
            break;
        case GsStatus::ERROR:
            Serial.println("RX <- [ERROR] " + resp.lastLine);
            break;
        case GsStatus::TIMEOUT:
            Serial.println("RX <- [TIMEOUT] partial: " + resp.raw);
            break;
        case GsStatus::NONE:
        default:
            Serial.println("RX <- [NONE] no response");
            break;
        }
        return resp;
    }
};

#endif // GROUND_SENSOR_AT_H
