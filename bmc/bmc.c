/*
 * Demo code for Comedi to MQTT JSON format data
 * asynchronous mode using threads
 *
 * This file may be freely modified, distributed, and combined with
 * other software, as long as proper attribution is given in the
 * source code.
 */
#define _DEFAULT_SOURCE
#include "bmc/bmc.h"
#include "bmc/mqtt_rec.h"
#include "bmc/sine_table.h"

#define LOG_VERSION     "v0.22"
#define MQTT_VERSION    "V3.11"
#define ADDRESS         "tcp://10.1.1.172:1883"
#define CLIENTID        "Comedi_Mqtt_HA"
#define TOPIC_P         "comedi/data/p8055/get"
#define TOPIC_S         "comedi/data/p8055/set"
#define QOS             1
#define TIMEOUT         10000L
#define SPACING_USEC    500 * 1000

//volatile MQTTClient_deliveryToken deliveredtoken, receivedtoken = false;
//volatile bool runner = false;

/*
 * for the publish and subscribe topic pair
 * passed as a context variable
 */
struct ha_flag_type ha_flag_vars = {
    .runner = false,
    .receivedtoken = false,
    .deliveredtoken = false,
    .rec_ok = false,
};

char *token;
const char *board_name = "NO_BOARD", *driver_name = "NO_DRIVER";
cJSON *json;

/*
 * Async processing threads
 */

/*
 * Comedi data update timer flag
 */
void timer_callback(int32_t signum) {
    signal(signum, timer_callback);
    ha_flag_vars.runner = true;
}

/*
 * Broker errors
 */
void connlost(void *context, char *cause) {
    printf("\nConnection lost\n");
    printf("     cause: %s\n", cause);
    exit(EXIT_FAILURE);
}

/*
 * Use MQTT to send/receive DAQ updates to a Comedi hardware device
 */
int main(int argc, char *argv[]) {
    int32_t do_ao_only = false;
    uint8_t i = 0, j = 75;
    uint32_t speed_go = 0, sequence = 0, rc;
    char chann[DAQ_STR];
    struct itimerval new_timer = {
        .it_value.tv_sec = 1,
        .it_value.tv_usec = 0,
        .it_interval.tv_sec = 0,
        .it_interval.tv_usec = SPACING_USEC,
    };
    struct itimerval old_timer;

    MQTTClient client;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    MQTTClient_deliveryToken token;

    printf("\r\n LOG Version %s : MQTT Version %s\r\n", LOG_VERSION, MQTT_VERSION);
    /*
     * set the timer for MQTT publishing sample speed
     */
    setitimer(ITIMER_REAL, &new_timer, &old_timer);
    signal(SIGALRM, timer_callback);

    MQTTClient_create(&client, ADDRESS, CLIENTID,
            MQTTCLIENT_PERSISTENCE_NONE, NULL);
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;

    MQTTClient_setCallbacks(client, &ha_flag_vars, connlost, msgarrvd, delivered);
    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS) {
        printf("Failed to connect, return code %d\n", rc);
        exit(EXIT_FAILURE);
    }

    /*
     * on topic received data will trigger the msgarrvd function
     */
    MQTTClient_subscribe(client, TOPIC_S, QOS);

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
        // setup the DAQ hardware
        if (init_daq(0.0, 25.0, false) < 0) {
            printf("Missing Analog subdevice(s)\n");
            return -1;
        }
        if (init_dio() < 0) {
            printf("Missing Digital subdevice(s)\n");
            return -1;
        }

        // parse the remote JSON names for the DAQ outputs
        printf("\r\n ANALOG OUT channel names for Board: %s, Driver: %s", board_name, driver_name);
        for (int i = 0; i < channels_ao; i++) {
            snprintf(chann, DAQ_STR_M, "DAC%d", i);
            printf("\r\n%s", chann);
        }
        printf("\r\n DIGITAL OUT channel names");
        for (int i = 0; i < channels_do; i++) {
            snprintf(chann, DAQ_STR_M, "DO%d", i);
            printf("\r\n%s", chann);
        }
        printf("\r\n Use these channel names in JSON formatted data\r\n");

        while (true) {
#ifndef NO_CYLON
            get_data_sample();
            /*
             * testing inputs and outputs
             */

            if (!bmc.datain.D0) {
                led_lightshow(4);
            }
#else
            usleep(100);
#endif

            if (ha_flag_vars.runner || speed_go++ > 1500) {
                speed_go = 0;
                ha_flag_vars.runner = false;
                json = cJSON_CreateObject();
                cJSON_AddStringToObject(json, "Name", "HA_comedi_get");
                cJSON_AddNumberToObject(json, "Sequence", sequence++);
                // parse the remote JSON names for DAQ inputs
                for (int i = 0; i < channels_ai; i++) {
                    snprintf(chann, DAQ_STR_M, "ADC%d", i);
                    cJSON_AddNumberToObject(json, chann, get_adc_volts(i));
                }
                for (int i = 0; i < channels_di; i++) {
                    snprintf(chann, DAQ_STR_M, "DI%d", i);
                    cJSON_AddNumberToObject(json, chann, get_dio_bit(i));
                }
                cJSON_AddStringToObject(json, "System", board_name);
                // convert the cJSON object to a JSON string
                char *json_str = cJSON_Print(json);

                pubmsg.payload = json_str;
                pubmsg.payloadlen = strlen(json_str);
                pubmsg.qos = QOS;
                pubmsg.retained = 0;
                ha_flag_vars.deliveredtoken = 0;
                MQTTClient_publishMessage(client, TOPIC_P, &pubmsg, &token);
                // a busy, wait loop for the async delivery thread to complete
                {
                    uint32_t waiting = 0;
                    while (ha_flag_vars.deliveredtoken != token) {
                        usleep(100);
                        if (waiting++ > MQTT_TIMEOUT) {
                            printf("\r\nStill Waiting, timeout");
                            break;
                        }
                    };
                }

                cJSON_free(json_str);
                cJSON_Delete(json);
            } else {
                if (ha_flag_vars.receivedtoken) {
                    ha_flag_vars.receivedtoken = false;
                }
            }
        }
    }
    return 0;
}

/*
 * idle test pattern of Comedi DO leds
 */
void led_lightshow(int32_t speed) {
    static int32_t j = 0;
    static uint8_t cylon = 0xff;
    static int32_t alive_led = 0;
    static bool LED_UP = true;

    if (j++ >= speed) { // delay a bit ok
        if (false) { // screen status feedback
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


