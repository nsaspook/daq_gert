/*
 * Demo code for driver testing, a simple console display of data inputs and voltage
 *
 * This file may be freely modified, distributed, and combined with
 * other software, as long as proper attribution is given in the
 * source code.
 */

#include <stdlib.h>
#include <stdio.h> /* for printf() */
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <comedilib.h>
#include "bmc/daq.h"
#include <cjson/cJSON.h>
#include "matesocketcan/mqtt_pub.h"
#include "matesocketcan/pge.h"
#include "MQTTClient.h"

#define LOG_VERSION            "v00.9"

#define ADDRESS     "tcp://10.1.1.172:1883"
#define CLIENTID    "Comedi_Mqtt_HA"
#define TOPIC_P     "comedi/data/p8055/get"
#define TOPIC_S     "comedi/data/p8055/set"
#define PAYLOAD     ""
#define QOS         1
#define TIMEOUT     10000L

volatile MQTTClient_deliveryToken deliveredtoken, receivedtoken = false;

void delivered(void *context, MQTTClient_deliveryToken dt) {
    //    printf("Message with token value %d delivery confirmed\n", dt);
    deliveredtoken = dt;
}

int msgarrvd(void *context, char *topicName, int topicLen, MQTTClient_message *message) {
    int i;
    char* payloadptr;
    char buffer[1024];
    char chann[DAQ_STR];

    printf("Message arrived\n");
    payloadptr = message->payload;
    for (i = 0; i < message->payloadlen; i++) {
        buffer[i] = *payloadptr++;
    }
    buffer[i] = 0; // make C string

    // parse the JSON data 
    cJSON *json = cJSON_ParseWithLength(buffer, message->payloadlen);
    if (json == NULL) {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL) {
            printf("Error: %s\n", error_ptr);
        }
        cJSON_Delete(json);
        return 1;
    }

    for (int i = 0; i < channels_do; i++) {
        snprintf(chann, DAQ_STR_M, "DO%d", i);

        // access the JSON data 
        cJSON *name = cJSON_GetObjectItemCaseSensitive(json, chann);
        if (cJSON_IsString(name) && (name->valuestring != NULL)) {
            printf("Name: %s\n", name->valuestring);
        }

        if (cJSON_IsNumber(name)) {
            printf("%s Value: %i\n", chann, name->valueint);
            put_dio_bit(i, name->valueint);
        }
    }

    for (int i = 0; i < channels_ao; i++) {
        snprintf(chann, DAQ_STR_M, "DAC%d", i);

        // access the JSON data 
        cJSON *name = cJSON_GetObjectItemCaseSensitive(json, chann);
        if (cJSON_IsString(name) && (name->valuestring != NULL)) {
            printf("Name: %s\n", name->valuestring);
        }

        if (cJSON_IsNumber(name)) {
            printf("%s Value: %f\n", chann, name->valuedouble);
            set_dac_volts(i, name->valuedouble);
        }
    }

    // delete the JSON object 
    cJSON_Delete(json);

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    receivedtoken = true;

    return 1;
}

void connlost(void *context, char *cause) {

    printf("\nConnection lost\n");
    printf("     cause: %s\n", cause);
}

void led_lightshow(int);

volatile struct bmcdata bmc; /* DIO buffer */

/* ripped from http://aquaticus.info/pwm-sine-wave */

uint8_t sine_wave[256] = {
    0x80, 0x83, 0x86, 0x89, 0x8C, 0x90, 0x93, 0x96,
    0x99, 0x9C, 0x9F, 0xA2, 0xA5, 0xA8, 0xAB, 0xAE,
    0xB1, 0xB3, 0xB6, 0xB9, 0xBC, 0xBF, 0xC1, 0xC4,
    0xC7, 0xC9, 0xCC, 0xCE, 0xD1, 0xD3, 0xD5, 0xD8,
    0xDA, 0xDC, 0xDE, 0xE0, 0xE2, 0xE4, 0xE6, 0xE8,
    0xEA, 0xEB, 0xED, 0xEF, 0xF0, 0xF1, 0xF3, 0xF4,
    0xF5, 0xF6, 0xF8, 0xF9, 0xFA, 0xFA, 0xFB, 0xFC,
    0xFD, 0xFD, 0xFE, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFE, 0xFE, 0xFD,
    0xFD, 0xFC, 0xFB, 0xFA, 0xFA, 0xF9, 0xF8, 0xF6,
    0xF5, 0xF4, 0xF3, 0xF1, 0xF0, 0xEF, 0xED, 0xEB,
    0xEA, 0xE8, 0xE6, 0xE4, 0xE2, 0xE0, 0xDE, 0xDC,
    0xDA, 0xD8, 0xD5, 0xD3, 0xD1, 0xCE, 0xCC, 0xC9,
    0xC7, 0xC4, 0xC1, 0xBF, 0xBC, 0xB9, 0xB6, 0xB3,
    0xB1, 0xAE, 0xAB, 0xA8, 0xA5, 0xA2, 0x9F, 0x9C,
    0x99, 0x96, 0x93, 0x90, 0x8C, 0x89, 0x86, 0x83,
    0x80, 0x7D, 0x7A, 0x77, 0x74, 0x70, 0x6D, 0x6A,
    0x67, 0x64, 0x61, 0x5E, 0x5B, 0x58, 0x55, 0x52,
    0x4F, 0x4D, 0x4A, 0x47, 0x44, 0x41, 0x3F, 0x3C,
    0x39, 0x37, 0x34, 0x32, 0x2F, 0x2D, 0x2B, 0x28,
    0x26, 0x24, 0x22, 0x20, 0x1E, 0x1C, 0x1A, 0x18,
    0x16, 0x15, 0x13, 0x11, 0x10, 0x0F, 0x0D, 0x0C,
    0x0B, 0x0A, 0x08, 0x07, 0x06, 0x06, 0x05, 0x04,
    0x03, 0x03, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x03,
    0x03, 0x04, 0x05, 0x06, 0x06, 0x07, 0x08, 0x0A,
    0x0B, 0x0C, 0x0D, 0x0F, 0x10, 0x11, 0x13, 0x15,
    0x16, 0x18, 0x1A, 0x1C, 0x1E, 0x20, 0x22, 0x24,
    0x26, 0x28, 0x2B, 0x2D, 0x2F, 0x32, 0x34, 0x37,
    0x39, 0x3C, 0x3F, 0x41, 0x44, 0x47, 0x4A, 0x4D,
    0x4F, 0x52, 0x55, 0x58, 0x5B, 0x5E, 0x61, 0x64,
    0x67, 0x6A, 0x6D, 0x70, 0x74, 0x77, 0x7A, 0x7D
};

char *token;
cJSON *json;

void led_lightshow(int speed) {
    static int j = 0;
    static uint8_t cylon = 0xff;
    static int alive_led = 0;
    static bool LED_UP = true;

    if (j++ >= speed) { // delay a bit ok
        if (0) { // screen status feedback
            bmc.dataout.dio_buf = ~cylon; // roll leds cylon style
        } else {
            bmc.dataout.dio_buf = cylon; // roll leds cylon style (inverted)
        }

        if (LED_UP && (alive_led != 0)) {
            alive_led = alive_led * 2;
            cylon = cylon << 1;
        } else {
            if (alive_led != 0) alive_led = alive_led / 2;
            cylon = cylon >> 1;
        }
        if (alive_led < 2) {
            alive_led = 2;
            LED_UP = true;
        } else {
            if (alive_led > 128) {

                alive_led = 128;
                LED_UP = false;
            }
        }
        j = 0;
    }
}

int main(int argc, char *argv[]) {
    int do_ao_only = false;
    uint8_t i = 0, j = 75;
    uint32_t speed_go = 0, sequence = 0;

    MQTTClient client;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    MQTTClient_deliveryToken token;
    int rc;

    MQTTClient_create(&client, ADDRESS, CLIENTID,
            MQTTCLIENT_PERSISTENCE_NONE, NULL);
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;


    MQTTClient_setCallbacks(client, NULL, connlost, msgarrvd, delivered);
    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS) {
        printf("Failed to connect, return code %d\n", rc);
        exit(EXIT_FAILURE);
    }

    MQTTClient_subscribe(client, TOPIC_S, QOS);

    printf("\r\n log version %s : mqtt version %s\r\n", LOG_VERSION, MQTT_VERSION);

    if (do_ao_only) {
        if (init_dac(0.0, 25.0, false) < 0) {
            printf("Missing Analog AO subdevice\n");
            return -1;
        }


        while (true) {
            set_dac_raw(0, sine_wave[255 - i++] << 4);
            set_dac_raw(1, sine_wave[255 - j++] << 4);
        }
    } else {

        if (init_daq(0.0, 25.0, false) < 0) {
            printf("Missing Analog subdevice(s)\n");
            return -1;
        }
        if (init_dio() < 0) {
            printf("Missing Digital subdevice(s)\n");
            return -1;
        }

        set_dac_raw(0, 255); // show max Voltage

        while (1) {
            get_data_sample();
            if (!bmc.datain.D0) {
                led_lightshow(4);
            }

            if (speed_go++ > 500) {
                char chann[DAQ_STR];

                speed_go = 0;
                json = cJSON_CreateObject();
                cJSON_AddStringToObject(json, "Name", "HA_comedi");
                cJSON_AddNumberToObject(json, "Sequence", sequence++);
                for (int i = 0; i < channels_ai; i++) {
                    snprintf(chann, DAQ_STR_M, "ADC%d", i);
                    cJSON_AddNumberToObject(json, chann, get_adc_volts(i));
                }
                for (int i = 0; i < channels_di; i++) {
                    snprintf(chann, DAQ_STR_M, "DI%d", i);
                    cJSON_AddNumberToObject(json, chann, get_dio_bit(i));
                }
                cJSON_AddStringToObject(json, "System", "K8055/VM110");
                // convert the cJSON object to a JSON string 
                char *json_str = cJSON_Print(json);

                pubmsg.payload = json_str;
                pubmsg.payloadlen = strlen(json_str);
                pubmsg.qos = QOS;
                pubmsg.retained = 0;
                deliveredtoken = 0;
                MQTTClient_publishMessage(client, TOPIC_P, &pubmsg, &token);
                //                printf("Waiting for publication of %s\n"
                //                        "on topic %s for client with ClientID: %s\n",
                //                        PAYLOAD, TOPIC_P, CLIENTID);
                while (deliveredtoken != token);

                cJSON_free(json_str);
                cJSON_Delete(json);
            }
        }
    }
    return 0;
}


