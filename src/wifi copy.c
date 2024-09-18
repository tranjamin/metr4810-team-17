#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "dhcpserver.h"
#include "dnsserver.h"

#include "FreeRTOS.h"
#include "task.h"

#include "wifi_helpers.h"

#include "diagnostics.h"
#include "delivery.h"
#include "extraction.h"
#include "motors.h"

#define VDELAY 100

// webpage paths
#define LED_PATH "/ledtest"
#define DIAGNOSTICS_PATH "/diagnostics"
#define LOG_PATH "/log"
#define CONTROL_PATH "/control"
#define LOCALISATION_PATH "/localisation"

// params
#define LED_PARAM "led=%d"
#define CONTROL_PARAM "command=%d"
#define LOCALISATION_LHS "lhs=%d"
#define LOCALISATION_RHS "rhs=%d"
#define LOCALISATION_PARAM "lhs=%d&rhs=%d"
#define LED_GPIO 0

// HTTP formats
#define LED_BODY "<html><body><h1>Hello from Pico W.</h1><p>Led is %s</p><p><a href=\"?led=%d\">Turn led %s</a></body></html>"
#define CONTROL_BODY "\
<html><body>\
<a href=\"?command=0\">Start Delivery</a><br>\
<a href=\"?command=1\">Set Left Forward</a><br>\
<a href=\"?command=2\">Set Left Stopped</a><br>\
<a href=\"?command=3\">Set Left Back</a><br>\
<a href=\"?command=4\">Set Right Forward</a><br>\
<a href=\"?command=5\">Set Right Stopped</a><br>\
<a href=\"?command=6\">Set Right Back</a><br>\
<a href=\"?command=7\">Set Extraction Forward</a><br>\
<a href=\"?command=8\">Set Extraction Stopped</a><br>\
<a href=\"?command=9\">Set Extraction Back</a><br>\
</body></html>"

// Handles response
int generate_response(const char*, const char*, char*, size_t);

// Function prototypes
void vWifiInit();
void vWifiTask();

// Send content as a response to a request
int generate_response(const char *request, const char *params, char *result, size_t max_result_len) {
    int len = 0; // size of the response

    // LED PAGE
    if (strncmp(request, LED_PATH, sizeof(LED_PATH) - 1) == 0) {
        bool value;
        cyw43_gpio_get(&cyw43_state, LED_GPIO, &value);
        int led_state = value;

        // Update led if necessary
        if (params) {
            int led_param = sscanf(params, LED_PARAM, &led_state);
            if (led_param == 1) {
                cyw43_gpio_set(&cyw43_state, LED_GPIO, led_state ? true : false);
            }
        }

        // Generate result
        len = snprintf(result, max_result_len, LED_BODY, led_state ? "ON" : "OFF", led_state ? 0 : 1, led_state ? "OFF" : "ON");
    } 
    
    // DIAGNOSTICS PAGE
    else if (strncmp(request, DIAGNOSTICS_PATH, sizeof(DIAGNOSTICS_PATH) - 1) == 0) {
        DiagnosticMessage msg;
        if (xGetDiagnosticMessage(&msg) == pdTRUE) {

        } else {
            snprintf(msg.message, DIAGNOSTICS_MAX_SIZE, "No Diagnostics Available\n");
        }
        len = snprintf(result, max_result_len, msg.message);
    } 
    
    // DEBUG LOG PAGE
    else if (strncmp(request, LOG_PATH, sizeof(LOG_PATH) - 1) == 0) {
        char msg[LOG_MAX_LENGTH];
        if (xGetDebugLog(msg) == pdTRUE) {

        } else {
            snprintf(msg, DIAGNOSTICS_MAX_SIZE, "Log is Empty\n");
        }
        len = snprintf(result, max_result_len, msg);
    }

    // CONTROL PAGE
    else if (strncmp(request, CONTROL_PATH, sizeof(CONTROL_PATH) - 1) == 0) {
        int param;
        if (params) {
            int control_number = sscanf(params, CONTROL_PARAM, &param);
            if (control_number) {
                switch (param) {
                    case 0:
                        vStartDelivery();
                        break;
                    case 1:
                        SET_TRAVERSAL_LHS_FORWARD();
                        break;
                    case 2:
                        SET_TRAVERSAL_LHS_STOPPED();
                        break;
                    case 3:
                        SET_TRAVERSAL_LHS_BACKWARD();
                        break;
                    case 4:
                        SET_TRAVERSAL_RHS_FORWARD();
                        break;
                    case 5:
                        SET_TRAVERSAL_RHS_STOPPED();
                        break;
                    case 6:
                        SET_TRAVERSAL_RHS_BACKWARD();
                        break;
                    case 7:
                        SET_EXTRACTION_FORWARD();
                        break;
                    case 8:
                        SET_EXTRACTION_STOPPED();
                        break;
                    case 9:
                        SET_EXTRACTION_BACKWARD();
                        break;

                }   
            }
        }
        len = snprintf(result, max_result_len, CONTROL_BODY);
    }

    // LOCALISATION PAGE
    else if (strncmp(request, LOCALISATION_PATH, sizeof(LOCALISATION_PATH) - 1) == 0) {
        int lhs_param, rhs_param;
        if (params) {
            sscanf(params, LOCALISATION_PARAM, &lhs_param, &rhs_param);
            len = snprintf(result, max_result_len, "Params 1: %d Params 2: %d", lhs_param, rhs_param);

            if (lhs_param) {
                if (lhs_param > 0) {
                    SET_TRAVERSAL_LHS_FORWARD();
                    setTraversalDuty_LHS(lhs_param);
                } else if (lhs_param < 0) {
                    SET_TRAVERSAL_LHS_BACKWARD();
                    setTraversalDuty_LHS(-lhs_param);
                } else {
                    SET_TRAVERSAL_LHS_STOPPED();
                    setTraversalDuty_LHS(0);
                }
            }
            if (rhs_param) {
                if (rhs_param > 0) {
                    SET_TRAVERSAL_RHS_FORWARD();
                    setTraversalDuty_RHS(rhs_param);
                } else if (rhs_param < 0) {
                    SET_TRAVERSAL_RHS_BACKWARD();
                    setTraversalDuty_RHS(-rhs_param);
                } else {
                    SET_TRAVERSAL_RHS_STOPPED();
                    setTraversalDuty_RHS(0);
                }
            }
        }
    }

    return len;
}

void vWifiInit() {
    // stdio_init_all();
}

void vWifiTask() {
    for (;;) {
        // Enable server
        TCP_SERVER_T *state = calloc(1, sizeof(TCP_SERVER_T));
        const char *ap_name = "METR4810 Team 17";
        const char *password = "password";
        cyw43_arch_enable_ap_mode(ap_name, password, CYW43_AUTH_WPA2_AES_PSK);

        // Set address
        ip4_addr_t mask;
        IP4_ADDR(ip_2_ip4(&state->gw), 192, 168, 4, 1);
        IP4_ADDR(ip_2_ip4(&mask), 255, 255, 255, 0);

        // Start the dhcp server
        dhcp_server_t dhcp_server;
        dhcp_server_init(&dhcp_server, &state->gw, &mask);

        // Start the dns server
        dns_server_t dns_server;
        dns_server_init(&dns_server, &state->gw);

        // Open server
        if (!tcp_server_open(state, ap_name)) {
            // vDebugLog("failed to open server\n");
            return;
        }
        
        // Wait for completion (in background)
        state->complete = false;
        while(!state->complete) {
            // cyw43_arch_poll();
            // vTaskDelay(pdMS_TO_TICKS(VDELAY));
        }

        // If an error occurs, close server
        tcp_server_close(state);
        dns_server_deinit(&dns_server);
        dhcp_server_deinit(&dhcp_server);
        cyw43_arch_deinit();
    }
    return;
}