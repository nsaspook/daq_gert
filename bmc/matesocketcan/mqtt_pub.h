/*
 * File:   mqtt_pub.h
 * Author: root
 *
 * Created on November 3, 2023, 8:52 AM
 */

#ifndef MQTT_PUB_H
#define MQTT_PUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>

#include "mqtt.h"

#define MQTT_VERSION            "v05.0"
#define DATA_MQTT_COMEDI_P      "comedi/data/p8055/get"
#define DATA_MQTT_COMEDI_S      "comedi/data/p8055/set"
//#define ADDR_MQTT               "test.mosquitto.org" // testing broker
#define ADDR_MQTT               "10.1.1.172"  // local broker
        
        int mqtt_socket(void);
        int mqtt_check(uint8_t *);
        void mqtt_exit(void);

        /**
         * @brief The function that would be called whenever a PUBLISH is received.
         *
         * @note This function is not used in this example.
         */
        void publish_callback(void** unused, struct mqtt_response_publish *published);

        /**
         * @brief The client's refresher. This function triggers back-end routines to
         *        handle ingress/egress traffic to the broker.
         *
         * @note All this function needs to do is call \ref __mqtt_recv and
         *       \ref __mqtt_send every so often. I've picked 100 ms meaning that
         *       client ingress/egress traffic will be handled every 100 ms.
         */
        void* client_refresher(void* client);

        /**
         * @brief Safelty closes the \p sockfd and cancels the \p client_daemon before \c exit.
         */
        void exit_example(int status, int sockfd, pthread_t *client_daemon);


#ifdef __cplusplus
}
#endif

#endif /* MQTT_PUB_H */

