/*
 * Demo code for driver testing, a simple console display of data inputs and voltage
 *
 * This file may be freely modified, distributed, and combined with
 * other software, as long as proper attribution is given in the
 * source code.
 */
#define _DEFAULT_SOURCE

#include <stdlib.h>
#include <stdio.h> /* for printf() */
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <comedilib.h>
#include <signal.h>
#include <time.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/time.h>
#include <errno.h>
#include <cjson/cJSON.h>
#include "MQTTClient.h"

#define LOG_VERSION            "v00.91"

#define ADDRESS     "tcp://10.1.1.172:1883"
#define CLIENTID    "Comedi_MQTT_HA_SET"
#define TOPIC_P     "comedi/data/p8055/set"
#define TOPIC_S     "comedi/data/p8055/foo"
#define PAYLOAD     ""
#define QOS         1
#define TIMEOUT     10000L

#define DAQ_STR 32
#define DAQ_STR_M DAQ_STR-1
#define MQTT_TIMEOUT    150

#define MQTT_VERSION  "v0.3"

volatile MQTTClient_deliveryToken deliveredtoken, receivedtoken = false;
volatile bool runner = false;
uint8_t led_buf = 0;

void timer_callback(int sig);
void delivered(void *, MQTTClient_deliveryToken);
int msgarrvd(void *, char *, int, MQTTClient_message *);
void connlost(void *, char *);

void led_lightshow(int32_t);

/*
 * async control threads
 */

/*
 * Comedi data update timer flag
 */
void timer_callback(int signum)
{
	signal(signum, timer_callback);
	runner = true;
}

void delivered(void *context, MQTTClient_deliveryToken dt)
{
	//    printf("Message with token value %d delivery confirmed\n", dt);
	deliveredtoken = dt;
}

int msgarrvd(void *context, char *topicName, int topicLen, MQTTClient_message *message)
{
	MQTTClient_freeMessage(&message);
	MQTTClient_free(topicName);
	receivedtoken = true;

	return 1;
}

void connlost(void *context, char *cause)
{

	printf("\nConnection lost\n");
	printf("     cause: %s\n", cause);
	exit(EXIT_FAILURE);
}

char *token;
cJSON *json;

int main(int argc, char *argv[])
{
	uint32_t sequence = 0;

	struct itimerval new_timer = {
		.it_value.tv_sec = 1,
		.it_value.tv_usec = 0,
		.it_interval.tv_sec = 0,
		.it_interval.tv_usec = 100000,
	}, old_timer;

	setitimer(ITIMER_REAL, &new_timer, &old_timer);
	signal(SIGALRM, timer_callback);

	MQTTClient client;
	MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
	MQTTClient_message pubmsg = MQTTClient_message_initializer;
	MQTTClient_deliveryToken token;
	int32_t rc;

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

	while (true) {
		if (runner) {
			runner = false;
			led_lightshow(1); // roll remote LED bits from led_buf
			json = cJSON_CreateObject();

			cJSON_AddStringToObject(json, "Name", "HA_comedi_set");
			cJSON_AddNumberToObject(json, "Sequence_set", sequence++);
			cJSON_AddNumberToObject(json, "DAC0", 1.234);
			cJSON_AddNumberToObject(json, "DAC1", 2.345);
			cJSON_AddNumberToObject(json, "DO0", led_buf >> 0 & 1);
			cJSON_AddNumberToObject(json, "DO1", led_buf >> 1 & 1);
			cJSON_AddNumberToObject(json, "DO2", led_buf >> 2 & 1);
			cJSON_AddNumberToObject(json, "DO3", led_buf >> 3 & 1);
			cJSON_AddNumberToObject(json, "DO4", led_buf >> 4 & 1);
			cJSON_AddNumberToObject(json, "DO5", led_buf >> 5 & 1);
			cJSON_AddNumberToObject(json, "DO6", led_buf >> 6 & 1);
			cJSON_AddNumberToObject(json, "DO7", led_buf >> 7 & 1);
			cJSON_AddStringToObject(json, "System", "K8055/VM110");
			// convert the cJSON object to a JSON string
			char *json_str = cJSON_Print(json);

			pubmsg.payload = json_str;
			pubmsg.payloadlen = strlen(json_str);
			pubmsg.qos = QOS;
			pubmsg.retained = 0;
			deliveredtoken = 0;
			MQTTClient_publishMessage(client, TOPIC_P, &pubmsg, &token);
			{
				uint32_t waiting = 0;
				while (deliveredtoken != token) {
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
			usleep(100);
		}
	}
	return 0;
}

void led_lightshow(int32_t speed)
{
	static int32_t j = 0;
	static uint8_t cylon = 0xff;
	static int32_t alive_led = 0;
	static bool LED_UP = true;

	if (++j >= speed) { // delay a bit ok
		if (false) { // screen status feedback
			led_buf = ~cylon; // roll leds cylon style
		} else {
			led_buf = cylon; // roll leds cylon style (inverted)
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


