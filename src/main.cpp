#include "Arduino.h"
#include <WiFi.h>
#include "driver/rtc_io.h"
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include "HardwareSerial.h"
#include <esp_sleep.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>

#define POWER_GPIO 25
#define BUTTON_GPIO 32
#define LED_GPIO 20

#define BAT_GPIO 37

#define SPO2_UART_RX_PIN 22
#define SPO2_UART_TX_PIN 21

#define SDA_GPIO 8
#define SCL_GPIO 7

#define spo2_pin 26
#define UART_BAUDRATE 115200

#define SPO2_UART Serial2

const char* ssid = "Varun";
const char* password = "9884755901";
const char* serverURL = "https://ls.towerstay.com/api/wearable/bpapi.php";

const adc1_channel_t BATTERY_ADC_CHANNEL = ADC1_CHANNEL_6;
const float R1_VALUE = 2700.0;  // From Battery (+) to ADC input
const float R2_VALUE = 10000.0; // From ADC input to GND
// --- ADC Configuration ---
const adc_atten_t ADC_ATTENUATION = ADC_ATTEN_DB_12; // 0-3.9V range (approx. 0-3.3V usable)
const adc_bits_width_t ADC_RESOLUTION = ADC_WIDTH_BIT_12; // 12-bit resolution (0-4095)
const int NO_OF_SAMPLES = 64; // Multisampling for stability
const uint32_t DEFAULT_VREF = 1100; // Default Vref for calibration (mV)
// ADC calibration characteristics structure
static esp_adc_cal_characteristics_t *adc_chars;

// --- Battery Voltage Thresholds (for a typical 3.7V LiPo battery in mV) ---
// Adjust these values based on your specific battery's discharge curve.
const uint32_t BATTERY_FULL_MV = 4200;
const uint32_t BATTERY_EMPTY_MV = 3000; // Consider this as 0%zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzjz

int epocheTime;

int toTime = 1;

static int i = 1000;
int address;
static unsigned char inp_arr[7];
int j;
static unsigned int perf;
static int ncnt = 1000;
uint8_t o2r = 0, o2r_timer = 0, rgain = 0, igain = 0, signalLevel = 0, gbeep = 0;
static int plethCnt = 0;
static int spo2PacketRecvd = 0;
static char rev[5];

volatile int buttonPressCount = 0;
unsigned short buttontimeout = 0;
unsigned long lastButtonPressTime = 0;
const unsigned long debounceDelay = 50;

bool spo2result = false;
bool piresult = false;
bool hrresult = false;
bool powerhigh;

// Remove SensorBuffer struct and use simple int variables for last valid values
int lastValidSpO2 = 0;
int lastValidHR = 0;
int lastValidPI = 0;
int lastValidStatus = 0;   // New: last valid status
int lastValidIrGain = 0;   // New: last valid IR gain
int lastValidO2R = 0;      // New: last valid O2R
int lastValidRGain = 0;    // New: last valid R gain
bool jsonSent = false; // Flag to ensure JSON is sent only once per session

RTC_DATA_ATTR int seq_num = 1;

void Parse(uint8_t a);

struct {
    uint8_t spo2Val;
    uint8_t hrVal;
    uint8_t status;
    uint16_t pi;
    uint32_t plethData[3600]; // Increased from 256 to 3600
} spo2ResultValues;

void sensorJson();

void IRAM_ATTR handleButtonInterrupt(void) {
    unsigned long currentTime = millis();
    if ((currentTime - lastButtonPressTime) > debounceDelay) {
        buttonPressCount++;
        buttontimeout = 0;
        lastButtonPressTime = currentTime;
    }
}

void validSensorReading() {
    digitalWrite(LED_GPIO, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(LED_GPIO, LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(LED_GPIO, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(LED_GPIO, LOW);
    vTaskDelay(pdMS_TO_TICKS(2000));
    digitalWrite(LED_GPIO, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(LED_GPIO, LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(LED_GPIO, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(LED_GPIO, LOW);
}

void invalidSensorReading() {
    digitalWrite(LED_GPIO, HIGH);
    vTaskDelay(pdMS_TO_TICKS(1000));
    digitalWrite(LED_GPIO, LOW);
    vTaskDelay(pdMS_TO_TICKS(1000));
    digitalWrite(LED_GPIO, HIGH);
    vTaskDelay(pdMS_TO_TICKS(1000));
    digitalWrite(LED_GPIO, LOW);
    vTaskDelay(pdMS_TO_TICKS(2000));
    digitalWrite(LED_GPIO, HIGH);
    vTaskDelay(pdMS_TO_TICKS(1000));
    digitalWrite(LED_GPIO, LOW);
    vTaskDelay(pdMS_TO_TICKS(1000));
    digitalWrite(LED_GPIO, HIGH);
    vTaskDelay(pdMS_TO_TICKS(1000));
    digitalWrite(LED_GPIO, LOW);
}

void deviceoff() {
    digitalWrite(LED_GPIO, HIGH);
    vTaskDelay(pdMS_TO_TICKS(200));
    digitalWrite(LED_GPIO, LOW);
    digitalWrite(LED_GPIO, HIGH);
    vTaskDelay(pdMS_TO_TICKS(200));
    digitalWrite(LED_GPIO, LOW);
    // Release power hold and enter deep sleep
    Serial.println("Device entering deep sleep (ordered off)");
    rtc_gpio_hold_dis((gpio_num_t)POWER_GPIO);
    digitalWrite(POWER_GPIO, LOW);
    vTaskDelay(pdMS_TO_TICKS(50));

}

void handleButtonActions(void *pvParameters) {
    while (1) {
        if (buttonPressCount >= 5) {
            Serial.println("Button pressed 5 times, Turned Off...");
            deviceoff();
            //rtc_gpio_hold_dis((gpio_num_t)POWER_GPIO);
            //digitalWrite(POWER_GPIO, LOW);
            buttonPressCount = 0;
        } else if (buttontimeout >= 2) {
            Serial.println("Button pressed less than 5 times, no action taken.");
            buttonPressCount = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void wificonnect() {
    WiFi.begin(ssid, password);
    Serial.print("WIfi connection begin...\n");
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(1000);
        Serial.print(".");
    }
    Serial.printf("Connected to %s, IP address: %s\n", ssid, WiFi.localIP().toString().c_str());
}

void InternetTime() {
    if (WiFi.status() != WL_CONNECTED) {
        wificonnect();
        Serial.println("Connected to WiFi for time sync.\n");
    }
    if (epocheTime == 0) {
        Serial.print("Connected to internet for time syncc\n");
        configTime(19800, 0, "pool.ntp.org", "time.google.com");
        struct tm timeinfo;
        if (getLocalTime(&timeinfo, 10000)) {
            epocheTime = (int)time(NULL);
        } else {
            epocheTime = 0;
        }
    }
}

void deepsleep() {
    Serial.println("Entering deep sleep, waiting for button press or timer to wake up...");
    digitalWrite(LED_GPIO, LOW);
    digitalWrite(spo2_pin, LOW);
    rtc_gpio_hold_en((gpio_num_t)spo2_pin); // Hold the spo2 pin to minimize leakage
    vTaskDelay(pdMS_TO_TICKS(100)); // Allow time for the pin state
    esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_GPIO, 1);
    uint64_t timer_us = (uint64_t)toTime * 60ULL * 1000000ULL;
    Serial.printf("[deepsleep] Setting timer wakeup for %d minutes (%llu us)\n", toTime, timer_us);
    esp_sleep_enable_timer_wakeup(timer_us);
    rtc_gpio_hold_en((gpio_num_t)POWER_GPIO);
    Serial.printf("spo2 pin status: %d\n", digitalRead(spo2_pin));
    Serial.printf("BUTTON_GPIO state before sleep: %d\n", digitalRead(BUTTON_GPIO));
    vTaskDelay(pdMS_TO_TICKS(100));
    Serial.println("[deepsleep] Calling esp_deep_sleep_start() now");
    esp_deep_sleep_start();
    Serial.println("entered deepsleep\n");
}

struct SensorPacket {
    int spo2 = 0;
    int hr = 0;
    int pi = 0;
    int pleth = 0;
};
SensorPacket latestPacket;

void processSensorData() {
    // Update last valid values if new valid data is available
    if (latestPacket.spo2 > 70 && latestPacket.spo2 <= 100) {
        lastValidSpO2 = latestPacket.spo2;
        Serial.printf("SpO2: %d\n", lastValidSpO2);
    } else if (latestPacket.spo2 != 0) {
        jsonSent = false;
    }
    if (latestPacket.hr > 30 && latestPacket.hr < 220) {
        lastValidHR = latestPacket.hr;
        Serial.printf("HR: %d\n", lastValidHR);
    } else if (latestPacket.hr != 0) {
        jsonSent = false;
    }
    if (latestPacket.pi > 0 && latestPacket.pi != 0xFFFF) {
        lastValidPI = latestPacket.pi;
        Serial.printf("PI: %d\n", lastValidPI);
    }
    // Only send JSON and sleep if both are valid and not already sent
    if (lastValidSpO2 > 70 && lastValidSpO2 <= 100 && lastValidHR > 30 && lastValidHR < 220 && !jsonSent) {
        validSensorReading();
        digitalWrite(spo2_pin, LOW);
        sensorJson();
        jsonSent = true;
        Serial.println("Going to deep sleep after JSON sent.");
        vTaskDelay(pdMS_TO_TICKS(100));
        deepsleep();
    }
    // Reset latestPacket after processing
    latestPacket = SensorPacket();
}

void Parse(uint8_t a) {
    if (a & 0x80) i = 0;
    if (i < 7) inp_arr[i++] = a;
    if (i == 7) {
        Serial.print("Packet: ");
        for (int dbg = 0; dbg < 7; ++dbg) {
            Serial.printf("%02X ", inp_arr[dbg]);
        }
        Serial.println();
        if (inp_arr[0] & 0x40) {
            rev[0] = inp_arr[1];
            rev[1] = '.';
            rev[2] = inp_arr[2];
            rev[3] = inp_arr[3];
            rev[4] = 0;
            Serial.print("SW Rev: ");
            Serial.println(rev);
        } else {
            if (inp_arr[0] & 1) gbeep = true;
            address = inp_arr[0] & 0xe;
            Serial.printf("Address: %d\n", address);
            switch (address) {
                case 0:
                    spo2ResultValues.spo2Val = inp_arr[3];
                    spo2PacketRecvd = 1;
                    Serial.printf("Raw SpO2: %d\n", spo2ResultValues.spo2Val);
                    latestPacket.spo2 = spo2ResultValues.spo2Val;
                    lastValidStatus = inp_arr[2]; // Example: status in byte 2
                    break;
                case 2:
                    spo2ResultValues.hrVal = inp_arr[3];
                    if (inp_arr[2] & 0x40)
                        spo2ResultValues.hrVal |= 0x80;
                    Serial.printf("Raw HR: %d\n", spo2ResultValues.hrVal);
                    latestPacket.hr = spo2ResultValues.hrVal;
                    lastValidIrGain = inp_arr[2]; // Example: ir gain in byte 2
                    break;
                case 14:
                    switch (inp_arr[3] & 0x30) {
                        case 0x00:
                            perf = 0; // <-- Reset perf at start
                            perf = inp_arr[3] & 0xf;
                            ncnt = 1;
                            break;
                        case 0x10:
                            perf |= (uint16_t)((inp_arr[3] & 0xf) << 4);
                            ncnt++;
                            break;
                        case 0x20:
                            perf |= (uint16_t)((inp_arr[3] & 0xf) << 8);
                            ncnt++;
                            break;
                        case 0x30:
                            perf |= (uint16_t)((inp_arr[3] & 0xf) << 12);
                            if (ncnt == 3) {
                                spo2ResultValues.pi = perf;
                                Serial.printf("Raw PI: %d\n", spo2ResultValues.pi);
                                latestPacket.pi = spo2ResultValues.pi;
                                lastValidO2R = inp_arr[2];
                                lastValidRGain = inp_arr[4];
                            }
                            ncnt = 0;
                            perf = 0; // <-- Reset perf after use
                            break;
                    }
                    break;
                default:
                    break;
            }
        }
        if (plethCnt < 3600) {
            spo2ResultValues.plethData[plethCnt++] = (uint32_t(inp_arr[6]) << 14) | (uint32_t(inp_arr[5]) << 7) | (uint32_t(inp_arr[4]));
            Serial.printf("Pleth: %lu\n", spo2ResultValues.plethData[plethCnt-1]);
            latestPacket.pleth = spo2ResultValues.plethData[plethCnt-1];
        } else {
            plethCnt = 0;
        }
        i = 1000;
    }
}

void sensorReading() {
    Serial.print("Waiting for valid SpO2 and HR values before starting pleth collection...\n");
    if(digitalRead(spo2_pin) == LOW) {
        digitalWrite(spo2_pin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    int validSpO2 = 0;
    int validHR = 0;
    int validPI = 0;
    unsigned long startMillis = millis();
    const unsigned long maxDuration = 15000; // 15 seconds total for the whole function
    plethCnt = 0;
    int validPlethCnt = 0;
    while ((millis() - startMillis) < maxDuration) {
        while (SPO2_UART.available()) {
            uint8_t incomingByte = SPO2_UART.read();
            Parse(incomingByte);
            if (latestPacket.spo2 > 70 && latestPacket.spo2 <= 100) {
                validSpO2 = latestPacket.spo2;
            }
            if (latestPacket.hr > 30 && latestPacket.hr < 220) {
                validHR = latestPacket.hr;
            }
            if (plethCnt > 0 && spo2ResultValues.plethData[plethCnt-1] > 0) {
                validPlethCnt++;
            } else {
                if (plethCnt > 0) plethCnt--;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    lastValidSpO2 = validSpO2;
    lastValidHR = validHR;
    Serial.printf("Got valid SpO2: %d, HR: %d. Pleth collection done. %d valid samples.\n", lastValidSpO2, lastValidHR, validPlethCnt);
    digitalWrite(spo2_pin, LOW);
    if (validPlethCnt > 0) {
        validSensorReading();
        if (epocheTime == 0) {
            Serial.println("epocheTime is 0, calling InternetTime() to get current time...");
            InternetTime();
        }
        sensorJson();
        jsonSent = true;
        Serial.println("Going to deep sleep after JSON sent.");
        vTaskDelay(pdMS_TO_TICKS(100));
    } else {
        Serial.println("No valid pleth samples, not sending JSON.");
        invalidSensorReading();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


// --- Battery Monitoring Functions ---
uint32_t readBatteryVoltageRaw() {
    uint32_t adc_reading = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        adc_reading += adc1_get_raw(BATTERY_ADC_CHANNEL);
    }
    adc_reading /= NO_OF_SAMPLES;
    return adc_reading;
}

uint32_t getBatteryVoltageMV() {
    uint32_t raw = readBatteryVoltageRaw();
    // Use ESP-IDF calibration API for accurate voltage
    uint32_t voltage = esp_adc_cal_raw_to_voltage(raw, adc_chars);
    // Compensate for voltage divider
    float vbat = voltage * ((R1_VALUE + R2_VALUE) / R2_VALUE);
    return (uint32_t)vbat;
}

uint8_t getBatteryPercent(uint32_t vbat_mv) {
    if (vbat_mv >= BATTERY_FULL_MV) return 100;
    if (vbat_mv <= BATTERY_EMPTY_MV) return 0;
    return (uint8_t)(((float)(vbat_mv - BATTERY_EMPTY_MV) / (BATTERY_FULL_MV - BATTERY_EMPTY_MV)) * 100.0f);
}

void sensorJson() {
    // Only call InternetTime if epocheTime is 0
    if (epocheTime == 0) {
        InternetTime();
    }
    Serial.print("Creating JSON data in new format...\n");
    JsonDocument doc;
    doc["device_name"] = "NISO-301";
    doc["device_id"] = "LS19e2a8";
    doc["packet_epoch"] = epocheTime;
    // Format time as string (IST)
    char timeStr[32];
    time_t rawtime = epocheTime;
    struct tm * timeinfo = localtime(&rawtime);
    snprintf(timeStr, sizeof(timeStr), "%02d-%s-%04d %02d:%02d:%02d IST",
        timeinfo->tm_mday,
        (const char*[]){"Jan","Feb","Mar","Apr","May","June","July","Aug","Sep","Oct","Nov","Dec"}[timeinfo->tm_mon],
        timeinfo->tm_year + 1900,
        timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
    doc["formatted_time"] = timeStr;
    doc["sequence_number"] = seq_num;

    // --- Battery Monitoring ---
    uint32_t vbat_mv = getBatteryVoltageMV();
    uint8_t batt_percent = getBatteryPercent(vbat_mv);
    doc["battery_voltage_mv"] = vbat_mv;
    doc["battery_soc"] = batt_percent;
    doc["battery_current"] = 0; // Placeholder

    doc["accel_x"] = 0;
    doc["accel_y"] = 0;
    doc["accel_z"] = 0;
    doc["temperature"] = 0.0;
    doc["accel_status"] = 0;
    doc["status"] = 2;
    doc["Pr"] = lastValidHR;
    doc["spo2"] = lastValidSpO2;

    // Arrays (fill with available data or zeros)
    JsonArray spo2Arr = doc["spo2_array"].to<JsonArray>();
    JsonArray prArr = doc["pr_array"].to<JsonArray>();
    JsonArray piArr = doc["pi_array"].to<JsonArray>();
    JsonArray statusArr = doc["status_array"].to<JsonArray>();
    JsonArray irGainArr = doc["ir_gain"].to<JsonArray>();
    JsonArray rGainArr = doc["r_gain"].to<JsonArray>();
    JsonArray o2rArr = doc["o2r"].to<JsonArray>();
    JsonArray o2rTimerArr = doc["o2r_timer"].to<JsonArray>();
    JsonArray signalArr = doc["signal"].to<JsonArray>();
    JsonArray gbeepArr = doc["gbeep"].to<JsonArray>();
    JsonArray plethDataArr = doc["pleth_data"].to<JsonArray>();
    JsonArray plethWaveArr = doc["pleth_wave"].to<JsonArray>();

    for (int i = 0; i < 10; ++i) {
        spo2Arr.add(lastValidSpO2); // Replace with real array if available
        prArr.add(lastValidHR);     // Replace with real array if available
        piArr.add(lastValidPI);     // Replace with real array if available
        statusArr.add(lastValidStatus); // Use last valid status
        irGainArr.add(lastValidIrGain); // Use last valid ir gain
        rGainArr.add(lastValidRGain);   // Use last valid r gain
        o2rArr.add(lastValidO2R);       // Use last valid o2r
        o2rTimerArr.add(0);
        signalArr.add(0);
        gbeepArr.add(0);
    }
    // Pleth Data: output as array of integers (raw) and as wave (low 8 bits)
    int plethLen = plethCnt > 0 ? plethCnt : 2700;
    for (int i = 0; i < plethLen; ++i) {
        uint32_t plethRaw = (i < plethCnt) ? spo2ResultValues.plethData[i] : 40800;
        plethDataArr.add(plethRaw); // Raw pleth value
        plethWaveArr.add(plethRaw & 0xFF); // Only the lowest 8 bits as wave
    }

    String jsonString;
    serializeJson(doc, jsonString);
    Serial.println(jsonString);
    // Optionally send as hex as before
    String hexString = "";
    for (size_t i = 0; i < jsonString.length(); i++) {
        char buf[3];
        sprintf(buf, "%02X", (uint8_t)jsonString[i]);
        hexString += buf;
    }
    HTTPClient http;
    http.begin(serverURL);
    http.addHeader("Content-Type", "text/plain");
    int httpResponseCode = http.POST(hexString);
    Serial.printf("HTTP Response code: %d\n", httpResponseCode);
    http.end();
    spo2result = false;
    seq_num++; // increment after sending
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println("WiFi disconnected after sending JSON.");
}

void setup() {
    setCpuFrequencyMhz(80);

    // --- ADC/Battery Monitoring Init ---
    adc1_config_width(ADC_RESOLUTION);
    adc1_config_channel_atten(BATTERY_ADC_CHANNEL, ADC_ATTENUATION);
    // Allocate memory for calibration characteristics
    static esp_adc_cal_characteristics_t adc_chars_storage;
    adc_chars = &adc_chars_storage;
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTENUATION, ADC_RESOLUTION, DEFAULT_VREF, adc_chars);
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        Serial.println("eFuse Vref used for ADC calibration");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        Serial.println("Two Point calibration used for ADC");
    } else {
        Serial.println("Default Vref used for ADC calibration");
    }

    rtc_gpio_init((gpio_num_t)BUTTON_GPIO);
    rtc_gpio_init((gpio_num_t)spo2_pin);
    rtc_gpio_set_direction((gpio_num_t)BUTTON_GPIO, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_en((gpio_num_t)BUTTON_GPIO);
    rtc_gpio_hold_dis((gpio_num_t)POWER_GPIO); // Release hold so you can control it during normal operation
    rtc_gpio_hold_dis((gpio_num_t)spo2_pin); // Release hold on spo2 pin
    pinMode(BUTTON_GPIO, INPUT_PULLUP); // For normal operation
    pinMode(spo2_pin, OUTPUT);
    digitalWrite(spo2_pin, HIGH);

    rtc_gpio_init((gpio_num_t)POWER_GPIO);
    pinMode(POWER_GPIO, OUTPUT);
    pinMode(LED_GPIO, OUTPUT);
    
    digitalWrite(POWER_GPIO, HIGH);

    Serial.begin(115200, SERIAL_8N1, GPIO_NUM_3, GPIO_NUM_1);
    SPO2_UART.begin(57600, SERIAL_8N1, SPO2_UART_RX_PIN, SPO2_UART_TX_PIN);

    attachInterrupt(digitalPinToInterrupt(BUTTON_GPIO), handleButtonInterrupt, RISING);
    xTaskCreate(handleButtonActions, "ButtonInterrupt", 2048, NULL, 1, NULL);
    while (true) {
        sensorReading();
        deepsleep();
    }
}

void loop() {
    
}