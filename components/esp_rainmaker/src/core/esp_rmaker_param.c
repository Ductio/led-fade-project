// Copyright 2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <sdkconfig.h>
#include <string.h>
#include <esp_log.h>
#include <esp_err.h>
#include <nvs.h>

#include <json_parser.h>
#include <json_generator.h>

#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_mqtt.h>

#include "esp_rmaker_internal.h"

#define NODE_PARAMS_LOCAL_TOPIC_SUFFIX          "params/local"
#define NODE_PARAMS_LOCAL_INIT_TOPIC_SUFFIX     "params/local/init"
#define NODE_PARAMS_REMOTE_TOPIC_SUFFIX         "params/remote"
#define TIME_SERIES_DATA_TOPIC_SUFFIX           "params/ts_data"

#define ESP_RMAKER_NVS_PART_NAME        "nvs"
#define MAX_PUBLISH_TOPIC_LEN           64

static char publish_payload[CONFIG_ESP_RMAKER_MAX_PARAM_DATA_SIZE];
static char publish_topic[MAX_PUBLISH_TOPIC_LEN];

static const char *TAG = "esp_rmaker_param";

esp_rmaker_param_val_t esp_rmaker_bool(bool val)
{
    esp_rmaker_param_val_t param_val = {
        .type = RMAKER_VAL_TYPE_BOOLEAN,
        .val.b = val
    };
    return param_val;
}

esp_rmaker_param_val_t esp_rmaker_int(int val)
{
    esp_rmaker_param_val_t param_val = {
        .type = RMAKER_VAL_TYPE_INTEGER,
        .val.i = val
    };
    return param_val;
}

esp_rmaker_param_val_t esp_rmaker_float(float val)
{
    esp_rmaker_param_val_t param_val = {
        .type = RMAKER_VAL_TYPE_FLOAT,
        .val.f = val
    };
    return param_val;
}

esp_rmaker_param_val_t esp_rmaker_str(const char *val)
{
    esp_rmaker_param_val_t param_val = {
        .type = RMAKER_VAL_TYPE_STRING,
        .val.s = (char *)val
    };
    return param_val;
}

static esp_err_t esp_rmaker_report_params(uint8_t flags, bool init)
{
    json_gen_str_t jstr;
    json_gen_str_start(&jstr, publish_payload, sizeof(publish_payload), NULL, NULL);
    json_gen_start_object(&jstr);
    _esp_rmaker_device_t *device = esp_rmaker_node_get_first_device(esp_rmaker_get_node());
    while (device) {
        bool device_added = false;
        _esp_rmaker_param_t *param = device->params;
        while (param) {
            if (!flags || (param->flags & flags)) {
                if (!device_added) {
                    json_gen_push_object(&jstr, device->name);
                    device_added = true;
                }
                esp_rmaker_report_value(&param->val, param->name, &jstr);
                param->flags &= ~flags;
            }
            param = param->next;
        }
        if (device_added) {
            json_gen_pop_object(&jstr);
        }
        device = device->next;
    }
    if (json_gen_end_object(&jstr) < 0) {
        ESP_LOGE(TAG, "Buffer size %d not sufficient for Node Params.\n"
                "Please increase CONFIG_ESP_RMAKER_MAX_PARAM_DATA_SIZE",
                CONFIG_ESP_RMAKER_MAX_PARAM_DATA_SIZE);
    }
    json_gen_str_end(&jstr);
    /* Just checking if there is any data to send by comparing with a decent enough
     * length as even the smallest possible data, Eg. '{"d":{"p":0}}' will be > 10 bytes
     */
    if (strlen(publish_payload) > 10) {
        snprintf(publish_topic, sizeof(publish_topic), "node/%s/%s", esp_rmaker_get_node_id(),
                init ? NODE_PARAMS_LOCAL_INIT_TOPIC_SUFFIX : NODE_PARAMS_LOCAL_TOPIC_SUFFIX);
        ESP_LOGI(TAG, "Reporting params: %s", publish_payload);
        esp_rmaker_mqtt_publish(publish_topic, publish_payload, strlen(publish_payload));
    }
    return ESP_OK;
}

esp_err_t esp_rmaker_report_param_internal(void)
{
    return esp_rmaker_report_params(RMAKER_PARAM_FLAG_VALUE_CHANGE, false);
}

esp_err_t esp_rmaker_report_node_state(void)
{
    return esp_rmaker_report_params(0, true);
}

static esp_err_t esp_rmaker_handle_get_params(_esp_rmaker_device_t *device, jparse_ctx_t *jptr)
{
    _esp_rmaker_param_t *param = device->params;
    while (param) {
        esp_rmaker_param_val_t new_val = {0};
        bool param_found = false;
        switch(param->val.type) {
            case RMAKER_VAL_TYPE_BOOLEAN:
                if (json_obj_get_bool(jptr, param->name, &new_val.val.b) == 0) {
                    new_val.type = RMAKER_VAL_TYPE_BOOLEAN;
                    param_found = true;
                }
                break;
            case RMAKER_VAL_TYPE_INTEGER:
                if (json_obj_get_int(jptr, param->name, &new_val.val.i) == 0) {
                    new_val.type = RMAKER_VAL_TYPE_INTEGER;
                    param_found = true;
                }
                break;
            case RMAKER_VAL_TYPE_FLOAT:
                if (json_obj_get_float(jptr, param->name, &new_val.val.f) == 0) {
                    new_val.type = RMAKER_VAL_TYPE_FLOAT;
                    param_found = true;
                }
                break;
            case RMAKER_VAL_TYPE_STRING: {
                int val_size = 0;
                if (json_obj_get_strlen(jptr, param->name, &val_size) == 0) {
                    val_size++; /* For NULL termination */
                    new_val.val.s = calloc(1, val_size);
                    json_obj_get_string(jptr, param->name, new_val.val.s, val_size);
                    new_val.type = RMAKER_VAL_TYPE_STRING;
                    param_found = true;
                }
                break;
            }
            default:
                break;
        }
        if (param_found) {
            /* Special handling for ESP_RMAKER_PARAM_NAME. Just update the name instead
             * of calling the registered callback.
             */
            if (param->type && (strcmp(param->type, ESP_RMAKER_PARAM_NAME) == 0)) {
                esp_rmaker_param_update_and_report((esp_rmaker_param_t *)param, new_val);
            } else if (device->write_cb) {
                esp_rmaker_write_ctx_t ctx = {
                    .src = ESP_RMAKER_REQ_SRC_CLOUD,
                };
                if (device->write_cb((esp_rmaker_device_t *)device, (esp_rmaker_param_t *)param, new_val, device->priv_data, &ctx) != ESP_OK) {
                    ESP_LOGE(TAG, "Remote update to param %s - %s failed", device->name, param->name);
                }
            }
            if (new_val.type == RMAKER_VAL_TYPE_STRING) {
                if (new_val.val.s) {
                    free(new_val.val.s);
                }
            }
        }
        param = param->next;
    }
    return ESP_OK;
}

static void esp_rmaker_set_params_callback(const char *topic, void *payload, size_t payload_len, void *priv_data)
{
    jparse_ctx_t jctx;
    ESP_LOGI(TAG, "Received params: %.*s", payload_len, (char *)payload);
    if (json_parse_start(&jctx, (char *)payload, payload_len) != 0) {
        return;
    }
    _esp_rmaker_device_t *device = esp_rmaker_node_get_first_device(esp_rmaker_get_node());
    while (device) {
        if (json_obj_get_object(&jctx, device->name) == 0) {
            esp_rmaker_handle_get_params(device, &jctx);
            json_obj_leave_object(&jctx);
        }
        device = device->next;
    }

    json_parse_end(&jctx);
}

esp_err_t esp_rmaker_register_for_set_params(void)
{
    char subscribe_topic[100];
    snprintf(subscribe_topic, sizeof(subscribe_topic), "node/%s/%s",
                esp_rmaker_get_node_id(), NODE_PARAMS_REMOTE_TOPIC_SUFFIX);
    esp_err_t err = esp_rmaker_mqtt_subscribe(subscribe_topic, esp_rmaker_set_params_callback, NULL);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to subscribe to %s. Error %d", subscribe_topic, err);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t esp_rmaker_param_get_stored_value(_esp_rmaker_param_t *param, esp_rmaker_param_val_t *val)
{
    if (!param || !param->parent || !val) {
        return ESP_FAIL;
    }
    nvs_handle handle;
    esp_err_t err = nvs_open_from_partition(ESP_RMAKER_NVS_PART_NAME, param->parent->name, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return err;
    }
    if (param->val.type == RMAKER_VAL_TYPE_STRING) {
        size_t len = 0;
        if ((err = nvs_get_str(handle, param->name, NULL, &len)) == ESP_OK) {
            char *s_val = calloc(1, len);
            if (!s_val) {
                err = ESP_ERR_NO_MEM;
            } else {
                nvs_get_str(handle, param->name, s_val, &len);
                val->type = param->val.type;
                val->val.s = s_val;
            }
        }
    } else {
        size_t len = sizeof(esp_rmaker_param_val_t);
        err = nvs_get_blob(handle, param->name, val, &len);
    }
    nvs_close(handle);
    return err;
}

esp_err_t esp_rmaker_param_store_value(_esp_rmaker_param_t *param)
{
    if (!param || !param->parent) {
        return ESP_FAIL;
    }
    nvs_handle handle;
    esp_err_t err = nvs_open_from_partition(ESP_RMAKER_NVS_PART_NAME, param->parent->name, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    if (param->val.type == RMAKER_VAL_TYPE_STRING) {
        /* Store only if value is not NULL */
        if (param->val.val.s) {
            err = nvs_set_str(handle, param->name, param->val.val.s);
            nvs_commit(handle);
        } else {
            err = ESP_OK;
        }
    } else {
        err = nvs_set_blob(handle, param->name, &param->val, sizeof(esp_rmaker_param_val_t));
        nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

esp_err_t esp_rmaker_param_delete(const esp_rmaker_param_t *param)
{
    _esp_rmaker_param_t *_param = (_esp_rmaker_param_t *)param;
    if (_param) {
        if (_param->name) {
            free(_param->name);
        }
        if (_param->type) {
            free(_param->type);
        }
        if (_param->ui_type) {
            free(_param->ui_type);
        }
        free(_param);
        return ESP_OK;
    }
    return ESP_ERR_INVALID_ARG;
}

esp_rmaker_param_t *esp_rmaker_param_create(const char *param_name, const char *type,
        esp_rmaker_param_val_t val, uint8_t properties)
{
    if (!param_name) {
        ESP_LOGE(TAG, "Param name is mandatory");
    }
    _esp_rmaker_param_t *param = calloc(1, sizeof(_esp_rmaker_param_t));
    if (!param) {
        ESP_LOGE(TAG, "Failed to allocate memory for param %s", param_name);
        return NULL;
    }
    param->name = strdup(param_name);
    if (!param->name) {
        ESP_LOGE(TAG, "Failed to allocate memory for name for param %s.", param_name);
        goto param_create_err;
    }
    if (type) {
        param->type = strdup(type);
        if (!param->type) {
            ESP_LOGE(TAG, "Failed to allocate memory for type for param %s.", param_name);
            goto param_create_err;
        }
    }
    param->val.type = val.type;
    param->prop_flags = properties;
    if (val.type == RMAKER_VAL_TYPE_STRING) {
        if (val.val.s) {
             param->val.val.s = strdup(val.val.s);
             if (!param->val.val.s) {
                 ESP_LOGE(TAG, "Failed to allocate memory for the value of param %s.", param_name);
             }
        }
    } else {
        param->val.val = val.val;
    }
    return (esp_rmaker_param_t *)param;

param_create_err:
    esp_rmaker_param_delete((esp_rmaker_param_t *)param);
    return NULL;
}

esp_err_t esp_rmaker_param_add_bounds(const esp_rmaker_param_t *param,
    esp_rmaker_param_val_t min, esp_rmaker_param_val_t max, esp_rmaker_param_val_t step)
{
    if (!param) {
        ESP_LOGE(TAG, "Param handle cannot be NULL.");
        return ESP_ERR_INVALID_ARG;
    }
    _esp_rmaker_param_t *_param = (_esp_rmaker_param_t *)param;
    if ((_param->val.type != RMAKER_VAL_TYPE_INTEGER) && (_param->val.type != RMAKER_VAL_TYPE_FLOAT)) {
        ESP_LOGE(TAG, "Only integer and float params can have bounds.");
        return ESP_ERR_INVALID_ARG;
    }
    if ((min.type != _param->val.type) || (max.type != _param->val.type) || (step.type != _param->val.type)) {
        ESP_LOGE(TAG, "Cannot set bounds for %s because of value type mismatch.", _param->name);
        return ESP_ERR_INVALID_ARG;
    }
    _param->min = min;
    _param->max = max;
    _param->step = step;
    return ESP_OK;
}

esp_err_t esp_rmaker_param_add_ui_type(const esp_rmaker_param_t *param, const char *ui_type)
{
    if (!param || !ui_type) {
        ESP_LOGE(TAG, "Param handle or UI type cannot be NULL.");
        return ESP_ERR_INVALID_ARG;
    }
    _esp_rmaker_param_t *_param = (_esp_rmaker_param_t *)param;
    if (_param->ui_type) {
        free(_param->ui_type);
    }
    if ((_param->ui_type = strdup(ui_type)) != NULL ){
        return ESP_OK;
    } else {
        return ESP_ERR_NO_MEM;
    }
}

esp_err_t esp_rmaker_param_update(const esp_rmaker_param_t *param, esp_rmaker_param_val_t val)
{
    if (!param) {
        ESP_LOGE(TAG, "Param handle cannot be NULL.");
        return ESP_ERR_INVALID_ARG;
    }
    _esp_rmaker_param_t *_param = (_esp_rmaker_param_t *)param;
    if (_param->val.type != val.type) {
        ESP_LOGE(TAG, "New param value type not same as the existing one.");
        return ESP_ERR_INVALID_ARG;
    }
    switch (_param->val.type) {
        case RMAKER_VAL_TYPE_STRING: {
            char *new_val = NULL;
            if (val.val.s) {
                new_val = strdup(val.val.s);
                if (!new_val) {
                    return ESP_FAIL;
                }
            }
            if (_param->val.val.s) {
                free(_param->val.val.s);
            }
            _param->val.val.s = new_val;
            break;
        }
        case RMAKER_VAL_TYPE_BOOLEAN:
        case RMAKER_VAL_TYPE_INTEGER:
        case RMAKER_VAL_TYPE_FLOAT:
            _param->val.val = val.val;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    if (_param->prop_flags & PROP_FLAG_PERSIST) {
        esp_rmaker_param_store_value(_param);
    }
    return ESP_OK;
}

esp_err_t esp_rmaker_param_report(const esp_rmaker_param_t *param)
{
    if (!param) {
        ESP_LOGE(TAG, "Param handle cannot be NULL.");
        return ESP_ERR_INVALID_ARG;
    }
    ((_esp_rmaker_param_t *)param)->flags |= RMAKER_PARAM_FLAG_VALUE_CHANGE;
    return esp_rmaker_report_param_internal();
}

esp_err_t esp_rmaker_param_update_and_report(const esp_rmaker_param_t *param, esp_rmaker_param_val_t val)
{
    esp_err_t err = esp_rmaker_param_update(param, val);
    if (err == ESP_OK) {
        err = esp_rmaker_param_report(param);
    }
    return err;
}

char *esp_rmaker_param_get_name(const esp_rmaker_param_t *param)
{
    if (!param) {
        ESP_LOGE(TAG, "Param handle cannot be NULL.");
        return NULL;
    }
    return ((_esp_rmaker_param_t *)param)->name;
}

char *esp_rmaker_param_get_type(const esp_rmaker_param_t *param)
{
    if (!param) {
        ESP_LOGE(TAG, "Param handle cannot be NULL.");
        return NULL;
    }
    return ((_esp_rmaker_param_t *)param)->type;
}
