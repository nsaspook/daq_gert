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
#define MQTT_TIMEOUT    100

#define MQTT_VERSION  "v0.2"

volatile MQTTClient_deliveryToken deliveredtoken, receivedtoken = false;
volatile bool runner = false;

void timer_callback(int sig);
void delivered(void *, MQTTClient_deliveryToken);
int msgarrvd(void *, char *, int, MQTTClient_message *);
void connlost(void *, char *);

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
}

char *token;
cJSON *json;

int main(int argc, char *argv[])
{
	uint8_t j = 75;
	uint32_t sequence = 0;

	struct itimerval new_timer;
	struct itimerval old_timer;

	new_timer.it_value.tv_sec = 1;
	new_timer.it_value.tv_usec = 0;
	new_timer.it_interval.tv_sec = 1;
	new_timer.it_interval.tv_usec = 0;

	setitimer(ITIMER_REAL, &new_timer, &old_timer);
	signal(SIGALRM, timer_callback);

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

	while (true) {
		if (runner) {
			runner = false;
			json = cJSON_CreateObject();

			cJSON_AddStringToObject(json, "Name", "HA_comedi_set");
			cJSON_AddNumberToObject(json, "Sequence_set", sequence++);
			cJSON_AddNumberToObject(json, "DAC0", 1.234);
			cJSON_AddNumberToObject(json, "DAC1", 2.345);
			cJSON_AddNumberToObject(json, "DO0", 1);
			cJSON_AddNumberToObject(json, "DO5", 0);
			cJSON_AddNumberToObject(json, "DO7", j++&0x1);
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


