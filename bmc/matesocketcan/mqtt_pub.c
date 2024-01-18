/*
 * MQTT routines for mateQ84 solar energy monitor
 */
#define _DEFAULT_SOURCE

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include "templates/posix_sockets.h"
#include "mqtt_pub.h"

const char* addr;
const char* port;
const char* topic_p;
const char* topic_s;
int sockfd;

struct mqtt_client client;
uint8_t sendbuf[2048]; /* sendbuf should be large enough to hold multiple whole mqtt messages */
uint8_t recvbuf[2048]; /* recvbuf should be large enough any whole mqtt message expected to be received */
const char* client_id;
uint8_t connect_flags;
pthread_t client_daemon;

/*
 * init the socket connection to the broker server and start client daemon
 */
int mqtt_socket(void) {
    //	addr = "hp8.sma2.rain.com"; // internal house server/broker

    addr = ADDR_MQTT; // cloud testing server
    port = "1883";
    topic_p = DATA_MQTT_COMEDI_P;
    topic_s = DATA_MQTT_COMEDI_S;

    /* open the non-blocking TCP socket (connecting to the broker) */
    sockfd = open_nb_socket_mqtt(addr, port);

    if (sockfd == -1) {
        return sockfd;
    }

    /* setup a client */
    mqtt_init(&client, sockfd, sendbuf, sizeof (sendbuf), recvbuf, sizeof (recvbuf), publish_callback);
    /* Create an anonymous session */
    client_id = NULL;
    /* Ensure we have a clean session */
    connect_flags = MQTT_CONNECT_CLEAN_SESSION;
    /* Send connection request to the broker. */
    mqtt_connect(&client, client_id, NULL, NULL, 0, NULL, NULL, connect_flags, 400);

    /* check that we don't have any errors */
    if (client.error != MQTT_OK) {
        return client.error;
    }

    /* start a thread to refresh the client (handle egress and ingree client traffic) */
    if (pthread_create(&client_daemon, NULL, client_refresher, &client)) {
        return 1;
    }
    
    mqtt_subscribe(&client, topic_s, 0);

    return 0;

}

/*
 * send client data to the broker server
 */
int mqtt_check(uint8_t * application_message) {
    if (strlen((const char *) application_message) < 3) {
        return -1;
    }
    /* publish the logging data */
    mqtt_publish(&client, topic_p, (const char *) application_message, strlen((const char *) application_message), MQTT_PUBLISH_QOS_1);

    /* check for errors */
    if (client.error != MQTT_OK) {
        return client.error;
    }

//    mqtt_subscribe(&client, topic_s, 0);
    return 0;
}

/*
 * cleanup before exit
 */
void mqtt_exit(void) {
    /* exit */
    exit_example(EXIT_SUCCESS, sockfd, &client_daemon);
}

void exit_example(int status, int sockfd, pthread_t *client_daemon) {
    if (sockfd != -1) close(sockfd);
    if (client_daemon != NULL) pthread_cancel(*client_daemon);
    exit(status);
}

void publish_callback(void** unused, struct mqtt_response_publish *published) {
    /* not used in this example */
}

void* client_refresher(void* client) {
    while (1) {
        mqtt_sync((struct mqtt_client*) client);
        usleep(100000U);
    }
    return NULL;
}