/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/cFiles/file.c to edit this template
 */
#include "mqtt_rec.h"

/*
 * data received on topic from the broker
 */
int32_t msgarrvd(void *context, char *topicName, int topicLen, MQTTClient_message *message) {
    int32_t i, ret = 1;
    char* payloadptr;
    char buffer[1024];
    char chann[DAQ_STR];
    struct ha_flag_type *ha_flag = context;

    // bug-out if no context variables passed to callback
    if (context == NULL) {
        ret = -1;
        ha_flag->rec_ok = false;
        goto null_exit;
    }

#ifdef DEBUG_REC
    printf("Message arrived\n");
#endif
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
        ret = -1;
        ha_flag->rec_ok = false;
        goto error_exit;
        return ret;
    }

    for (int32_t i = 0; i < channels_do; i++) {
        snprintf(chann, DAQ_STR_M, "DO%d", i);

        // access the JSON data
        cJSON *name = cJSON_GetObjectItemCaseSensitive(json, chann);
        if (cJSON_IsString(name) && (name->valuestring != NULL)) {
#ifdef DEBUG_REC
            printf("Name: %s\n", name->valuestring);
#endif
        }

        if (cJSON_IsNumber(name)) {
#ifdef DEBUG_REC
            printf("%s Value: %i\n", chann, name->valueint);
#endif
            put_dio_bit(i, name->valueint);
        }
    }

    for (int32_t i = 0; i < channels_ao; i++) {
        snprintf(chann, DAQ_STR_M, "DAC%d", i);

        // access the JSON data
        cJSON *name = cJSON_GetObjectItemCaseSensitive(json, chann);
        if (cJSON_IsString(name) && (name->valuestring != NULL)) {
#ifdef DEBUG_REC
            printf("Name: %s\n", name->valuestring);
#endif
        }

        if (cJSON_IsNumber(name)) {
#ifdef DEBUG_REC
            printf("%s Value: %f\n", chann, name->valuedouble);
#endif
            set_dac_volts(i, name->valuedouble);
        }
    }

    ha_flag->receivedtoken = true;
    ha_flag->rec_ok = true;
error_exit:
    // delete the JSON object
    cJSON_Delete(json);
null_exit:
    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return ret;
}

/*
 * set the broker has message token
 */
void delivered(void *context, MQTTClient_deliveryToken dt) {
    struct ha_flag_type *ha_flag = context;

    // bug-out if no context variables passed to callback
    if (context == NULL) {
        return;
    }
    ha_flag->deliveredtoken = dt;
}
