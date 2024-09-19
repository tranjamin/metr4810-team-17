#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "pico/cyw43_arch.h"

#include "pico/stdlib.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "dhcpserver.h"
#include "FreeRTOS.h"
#include "task.h"

#include "diagnostics.h"
#include "delivery.h"
#include "extraction.h"
#include "motors.h"
#include "rgb.h"
#include "wifi_helpers.h"

#define VDELAY 100

// webpage paths
#define DIAGNOSTICS_PATH "/diagnostics"
#define LOG_PATH "/log"
#define CONTROL_PATH "/control"
#define LOCALISATION_PATH "/localisation"

// params
#define CONTROL_PARAM "command=%d"
#define LOCALISATION_LHS "lhs=%d"
#define LOCALISATION_RHS "rhs=%d"
#define LOCALISATION_PARAM "lhs=%d&rhs=%d"

// HTTP formats
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

#define STR_MATCH(str, CONST) (strncmp(str, CONST, sizeof(CONST) - 1) == 0)

// Handles response
int generate_response(const char*, const char*, char*, size_t);

// Function prototypes
void vWifiInit();
void vWifiTask();

// Send content as a response to a request
int generate_response(const char *request, const char *params, char *result, size_t max_result_len) {
    int len = 0; // size of the response

    // DIAGNOSTICS PAGE
    if (STR_MATCH(request, DIAGNOSTICS_PATH)) {
        DiagnosticMessage msg;
        if (xGetDiagnosticMessage(&msg) == pdTRUE) {

        } else {
            snprintf(msg.message, DIAGNOSTICS_MAX_SIZE, "No Diagnostics Available\n");
        }
        len = snprintf(result, max_result_len, msg.message);
    } 
    
    // DEBUG LOG PAGE
    else if (STR_MATCH(request, LOG_PATH)) {
        char msg[LOG_MAX_LENGTH];
        if (xGetDebugLog(msg) == pdTRUE) {

        } else {
            snprintf(msg, DIAGNOSTICS_MAX_SIZE, "Log is Empty\n");
        }
        len = snprintf(result, max_result_len, msg);
    }

    // CONTROL PAGE
    else if (STR_MATCH(request, CONTROL_PATH)) {
        int param;
        if (params) {
            // get command number
            int control_param = atoi(params + 8);

            // execute commands
            switch (control_param) {
                case 0:
                    vStartDelivery();
                    setRGB_COLOUR_RED();
                    break;
                case 1:
                    SET_TRAVERSAL_LHS_FORWARD();
                    setRGB_COLOUR_GREEN();
                    break;
                case 2:
                    SET_TRAVERSAL_LHS_STOPPED();
                    setRGB_COLOUR_BLUE();
                    break;
                case 3:
                    SET_TRAVERSAL_LHS_BACKWARD();
                    setRGB_COLOUR_PURPLE();
                    break;
                case 4:
                    SET_TRAVERSAL_RHS_FORWARD();
                    setRGB_COLOUR_CYAN();
                    break;
                case 5:
                    SET_TRAVERSAL_RHS_STOPPED();
                    setRGB_COLOUR_YELLOW();
                    break;
                case 6:
                    SET_TRAVERSAL_RHS_BACKWARD();
                    setRGB_COLOUR_WHITE();
                    break;
                case 7:
                    SET_EXTRACTION_FORWARD();
                    setRGB_COLOUR_DARK_RED();
                    break;
                case 8:
                    SET_EXTRACTION_STOPPED();
                    setRGB_COLOUR_DARK_GREEN();
                    break;
                case 9:
                    SET_EXTRACTION_BACKWARD();
                    setRGB_COLOUR_DARK_BLUE();
                    break;

            }   
        }
        len = snprintf(result, max_result_len, CONTROL_BODY);
    }

    // LOCALISATION PAGE
    else if (STR_MATCH(request, LOCALISATION_PATH)) {
        float lhs_param, rhs_param = 101;
        if (params) {
            // copy params from ROM
            char params_copy[23];
            strncpy(params_copy, params, 23);
            
            // split params and convert to floats
            params_copy[11] = '\0';
            lhs_param = strtof(params_copy + 4, NULL);
            rhs_param = strtof(params_copy + 16, NULL);

            // format repsonse string
            len = snprintf(result, max_result_len, "Params 1: %.3f Params 2: %.3f", lhs_param, rhs_param);
            
            // suppress any response
            len = 0;

            // send motor controls
            if (lhs_param != 101) {
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
            if (rhs_param != 101) {
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

        // Open server
        if (!tcp_server_open(state, ap_name)) {
            // vDebugLog("failed to open server\n");
            return;
        }
        
        // Wait for completion (in background)
        state->complete = false;
        while(!state->complete) {
            // cyw43_arch_poll();
            vTaskDelay(VDELAY);
        }


        setRGB_COLOUR_PURPLE();
        // If an error occurs, close server
        tcp_server_close(state);
        dhcp_server_deinit(&dhcp_server);
        // cyw43_arch_deinit();
    }
    return;
}