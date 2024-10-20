// Adapted with attribution to pico-examples 

// RTOS INCLUDES
#include "FreeRTOS.h"
#include "task.h"

// PICO INCLUDES
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

// LWIP INCLUDES
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "dhcpserver.h"

// STD INCLUDES
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// DRIVER INCLUDES
#include "diagnostics.h"
#include "extraction.h"
#include "delivery.h"
#include "motors.h"
#include "rgb.h"

#define VDELAY 100 // RTOS blocking time

#define TCP_PORT 80 // the port to host the wifi on
#define UDP_PORT 80 // the port to receive udp messages on
#define WIFI_CHANNEL 11 // the wifi channel to create the server on

// webpage paths
#define DIAGNOSTICS_PATH "/diagnostics"
#define LOG_PATH "/log"
#define CONTROL_PATH "/control"
#define LOCALISATION_PATH "/localisation"
#define UDP_PATH "/udp"

// request string formats
#define HTTP_GET "GET" // GET Request
#define CONTROL_PARAM "command=%d"
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
<a href=\"?command=9\">Set Extraction Procedure</a><br>\
</body></html>"
#define HTTP_RESPONSE_HEADERS "HTTP/1.1 %d OK\nContent-Length: %d\nContent-Type: text/html; charset=utf-8\nConnection: close\n\n"
#define MAX_RESULT_SIZE 1200 // maximum size of a response 

// Warn if buffer sizes don't match
#ifdef DIAGNOSTICS_H
    #if MAX_RESULT_SIZE < DIAGNOSTICS_MAX_SIZE
        #warning "MAX_RESULT_SIZE must be larger than DIAGNOSTICS_MAX_SIZE"
    #endif
    #if MAX_RESULT_SIZE < LOG_MAX_LENGTH
        #warning "MAX_RESULT_SIZE must be larger than LOG_MAX_LENGTH"
    #endif
#endif

// helper macro to test string equality
#define STR_MATCH(str, CONST) (strncmp(str, CONST, sizeof(CONST) - 1) == 0)

volatile int udp_count = 0; // number of udp messages received
static volatile bool enable_udp = true; // flag to enable udp or not

// Structure to represent the tcp server
typedef struct TCP_SERVER_T_ {
    struct tcp_pcb *server_pcb;
    bool complete;
    ip_addr_t gw;
    async_context_t *context;
} TCP_SERVER_T;

// Structure to represent the udp server
typedef struct UDP_SERVER_T_ {
    struct udp_pcb *server_pcb;
    bool complete;
    ip_addr_t gw;
    async_context_t *context;
} UDP_SERVER_T;

// Structure to represent the connection state
typedef struct TCP_CONNECT_STATE_T_ {
    struct tcp_pcb *pcb;
    int sent_len;
    char headers[128];
    char result[MAX_RESULT_SIZE];
    int header_len;
    int result_len;
    ip_addr_t *gw;
} TCP_CONNECT_STATE_T;

// Function Definitions for TCP
err_t tcp_server_recv(void*, struct tcp_pcb*, struct pbuf*, err_t);
err_t tcp_server_poll(void*, struct tcp_pcb*);
void tcp_server_err(void*, err_t);
err_t tcp_server_accept(void*, struct tcp_pcb*, err_t);
bool tcp_server_open(void*, const char*);
err_t tcp_close_client_connection(TCP_CONNECT_STATE_T*, struct tcp_pcb*, err_t);
void tcp_server_close(TCP_SERVER_T*);
err_t tcp_server_sent(void*, struct tcp_pcb*, u16_t);

// Function Definitions for UDP
bool udp_server_open(void *arg, const char *ap_name);
void udp_server_recv(void*, struct udp_pcb*, struct pbuf*, const ip_addr_t*, u16_t);

// Function prototypes
void vWifiInit();
void vWifiTask();
void vWifiUDPTask();
int generate_response(const char*, const char*, char*, size_t);

/**
Enables UDP message handling
 */
void enableUDP() {
    enable_udp = true;
}

/**
Disables UDP message handling, provided UDP_ALWAYS_ON is not set.
Still processes messages through the buffer, but ignores them
 */
void disableUDP() {
    #ifndef UDP_ALWAYS_ON
        enable_udp = false;
    #endif
}

/**
Handles a request from a TCP client.

Parameters:
    arg: the connection state
    pcb: the protocol control block for the TCP server
    p: the packet buffer
    err: the error status for the process
 */
err_t tcp_server_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
    if (!p) {
        return tcp_close_client_connection(con_state, pcb, ERR_OK);
    }
    
    if (p->tot_len > 0) {
        // Copy the request into the buffer
        pbuf_copy_partial(p, con_state->headers, p->tot_len > sizeof(con_state->headers) - 1 ? sizeof(con_state->headers) - 1 : p->tot_len, 0);

        // Handle GET request
        if (strncmp(HTTP_GET, con_state->headers, sizeof(HTTP_GET) - 1) == 0) {
            char *request = con_state->headers + sizeof(HTTP_GET);
            char *params = strchr(request, '?');
            if (params) {
                if (*params) {
                    char *space = strchr(request, ' ');
                    *params++ = 0;
                    if (space) {
                        *space = 0;
                    }
                } else {
                    params = NULL;
                }
            }

            // Generate content
            con_state->result_len = generate_response(request, params, con_state->result, sizeof(con_state->result));

            // Check we had enough buffer space
            if (con_state->result_len > sizeof(con_state->result) - 1) {
                return tcp_close_client_connection(con_state, pcb, ERR_CLSD);
            }

            // Generate web page
            if (con_state->result_len > 0) {
                con_state->header_len = snprintf(con_state->headers, sizeof(con_state->headers),
                    HTTP_RESPONSE_HEADERS, 200, con_state->result_len);
                if (con_state->header_len > sizeof(con_state->headers) - 1) {
                    return tcp_close_client_connection(con_state, pcb, ERR_CLSD);
                }
            }

            // Send the headers to the client
            con_state->sent_len = 0;
            err_t err = tcp_write(pcb, con_state->headers, con_state->header_len, 0);
            if (err != ERR_OK) {
                return tcp_close_client_connection(con_state, pcb, err);
            }

            // Send the body to the client
            if (con_state->result_len) {
                err = tcp_write(pcb, con_state->result, con_state->result_len, 0);
                if (err != ERR_OK) {
                    return tcp_close_client_connection(con_state, pcb, err);
                }
            }
        }
        tcp_recved(pcb, p->tot_len);
    }
    pbuf_free(p);
    return ERR_OK;
}

/**
Handles a request from a UDP client.

Parameters:
    arg: the connection state
    pcb: the protocol control block for the UDP server
    p: the packet buffer
    addr: the incoming address of the packet
    port: the incoming port of the packet
 */
void udp_server_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t* addr, u16_t port) {
    UDP_SERVER_T *state = (UDP_SERVER_T*) arg;
    if (!p) {
        return;
    }
    if (p->tot_len > 0 && enable_udp) {
        // format is "Llhs=XXX.XXX&rhs=XXX.XXX"
        if (*((char*) p->payload) == 'L') {
            float lhs_param, rhs_param;
            udp_count++;

            // copy params from ROM
            char params_copy[24];
            strncpy(params_copy, p->payload, 24);
            
            // split params and convert to floats
            params_copy[12] = '\0';
            lhs_param = strtof(params_copy + 5, NULL);
            rhs_param = strtof(params_copy + 17, NULL);

            // send motor controls
            if (enable_udp) {
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

        // format is "Ccommand=X"
        else if (*((char*) p->payload) == 'C') {
            udp_count++;

            // copy params from ROM
            char params_copy[12];
            strncpy(params_copy, p->payload, 12);
            int control_param = atoi(params_copy + 9);

            switch (control_param) {
                case 0: // delivery
                    vStartDelivery();
                    setRGB_COLOUR_RED();
                    break;
                case 1: // forward
                    SET_TRAVERSAL_LHS_FORWARD();
                    SET_TRAVERSAL_RHS_FORWARD();
                    setTraversalDuty_LHS(100.0);
                    setTraversalDuty_RHS(100.0);
                    setRGB_COLOUR_GREEN();
                    break;
                case 2: // backward
                    SET_TRAVERSAL_LHS_BACKWARD();
                    SET_TRAVERSAL_RHS_BACKWARD();
                    setTraversalDuty_LHS(100.0);
                    setTraversalDuty_RHS(100.0);
                    setRGB_COLOUR_BLUE();
                    break;
                case 3: // stopped
                    SET_TRAVERSAL_LHS_STOPPED();
                    SET_TRAVERSAL_RHS_STOPPED();
                    setRGB_COLOUR_PURPLE();
                    break;
                case 4: 
                    break;
                case 5:
                    break;
                case 6:
                    break;
                case 7: // extract
                    extractionManualStart();
                    setRGB_COLOUR_DARK_RED();
                    break;
                case 8: // stop extract
                    extractionManualStop();
                    setRGB_COLOUR_DARK_GREEN();
                    break;
                case 9: // extract method
                    extractionProcedureSignalStart();
                    setRGB_COLOUR_DARK_BLUE();
                    break;
                }
            }   
    }            

    
    pbuf_free(p);
    return;
}

/**
Poll current activate connections. Does nothing.

Parameters:
    arg: the connection state
    pcb: the protocol control block for the TCP server
 */
err_t tcp_server_poll(void *arg, struct tcp_pcb *pcb) {
    TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
    // return tcp_close_client_connection(con_state, pcb, ERR_OK); // Just disconnect clent?
    return ERR_OK;
}

/**
Check for client errors. Close connections on fatal error.

Parameters:
    arg: the connection state
    err: the error status from the client
 */
void tcp_server_err(void *arg, err_t err) {
    TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
    if (err != ERR_ABRT) {
        tcp_close_client_connection(con_state, con_state->pcb, err);
    }
}

/**
Accept a TCP connection.

Parameters:
    arg: the connection state
    clinet_pcb: the protocol control block for the TCP server
    err: the error status

Returns:
    an error code
 */
err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (err != ERR_OK || client_pcb == NULL) {
        return ERR_VAL;
    }

    // Create the state for the connection
    TCP_CONNECT_STATE_T *con_state = calloc(1, sizeof(TCP_CONNECT_STATE_T));
    if (!con_state) {
        return ERR_MEM;
    }

    con_state->pcb = client_pcb; // for checking
    con_state->gw = &state->gw;

    // setup connection to client
    tcp_arg(client_pcb, con_state);
    tcp_sent(client_pcb, tcp_server_sent);
    tcp_recv(client_pcb, tcp_server_recv);
    tcp_err(client_pcb, tcp_server_err);

    return ERR_OK;
}

/**
Open the TCP server.

Parameters:
    arg: the connection state
    ap_name: the access point name

Returns:
    true on success, false on failure
 */
bool tcp_server_open(void *arg, const char *ap_name) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;

    // create server control block
    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        return false;
    }

    // bind to the server
    err_t err = tcp_bind(pcb, IP_ANY_TYPE, TCP_PORT);
    if (err) {
        return false;
    }

    // listen on the server
    state->server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (!state->server_pcb) {
        if (pcb) {
            tcp_close(pcb);
        }
        return false;
    }

    tcp_arg(state->server_pcb, state);
    tcp_accept(state->server_pcb, tcp_server_accept);

    return true;
}

/**
Open a UDP server

Parameters:
    arg: the connection state
    pcb: the access point name
 */
bool udp_server_open(void *arg, const char *ap_name) {
    UDP_SERVER_T* state = (UDP_SERVER_T*)arg; // create server
    struct udp_pcb* pcb = udp_new(); // create server state

    // if could not allocate memory for PCB, return
    if (!pcb) {
        return false;
    }

    // bind on the UDP port, and return if failure
    err_t err = udp_bind(pcb, IP_ADDR_ANY, UDP_PORT);
    if (err) {
        return false;
    }

    // set up callback to receive messages
    udp_recv(pcb, udp_server_recv, state);
    state->server_pcb = pcb;
    return true;
}

/**
Close a TCP client connection.

Parameters:
    con_state: the connection state
    client_pcb: the protocol control block for the TCP server
    close_err: the error status from the client

Returns:
    an error code
 */
err_t tcp_close_client_connection(TCP_CONNECT_STATE_T *con_state, struct tcp_pcb *client_pcb, err_t close_err) {
    if (client_pcb) {
        // remove all callbacks
        tcp_arg(client_pcb, NULL);
        tcp_poll(client_pcb, NULL, 0);
        tcp_sent(client_pcb, NULL);
        tcp_recv(client_pcb, NULL);
        tcp_err(client_pcb, NULL);

        // close connection
        err_t err = tcp_close(client_pcb);

        // abort on failure to close
        if (err != ERR_OK) {
            tcp_abort(client_pcb);
            close_err = ERR_ABRT;
        }

        // free memory
        if (con_state) {
            free(con_state);
        }
    }
    return close_err;
}

/**
Closes the TCP server

Parameters:
    state: the tcp server
 */
void tcp_server_close(TCP_SERVER_T *state) {
    if (state->server_pcb) {
        tcp_arg(state->server_pcb, NULL);
        tcp_close(state->server_pcb);
        state->server_pcb = NULL;
    }
}

/**
Send a data packet via TCP.

Parameters:
    arg: the connection state
    pcb: the protocol control block for the TCP server
    len: the length of response

Returns:
    an error code
 */
err_t tcp_server_sent(void *arg, struct tcp_pcb *pcb, u16_t len) {
    TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg; // get connection state
    con_state->sent_len += len;

    // close connection if too much data sent.
    if (con_state->sent_len >= con_state->header_len + con_state->result_len) {
        return tcp_close_client_connection(con_state, pcb, ERR_OK);
    }
    return ERR_OK;
}

/**
Generate a response to TCP

Parameters:
    request: the request message string
    params: the HTTP query parameters
    result: the buffer to fill the result with
    max_result_len: the length of the result buffer
 */
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

    // UDP PAGE
    if (STR_MATCH(request, UDP_PATH)) {
        len = snprintf(result, max_result_len, "UDP Message Count: %d", udp_count);
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
                    extractionManualStart();
                    setRGB_COLOUR_DARK_RED();
                    break;
                case 8:
                    extractionManualStop();
                    setRGB_COLOUR_DARK_GREEN();
                    break;
                case 9:
                    extractionProcedureSignalStart();
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

/**
Initialise hardware for the wifi task.
 */
void vWifiInit() {
}

/**
The main loop for the TCP wifi. Sets up TCP wifi to receive data.
 */
void vWifiTask() {
    for (;;) {
        // Enable server
        TCP_SERVER_T *state = calloc(1, sizeof(TCP_SERVER_T));
        
        const char *ap_name = "METR4810 Team 17";
        const char *password = "password";
        
        cyw43_wifi_ap_set_channel(&cyw43_state, WIFI_CHANNEL); // custom set channel
        cyw43_arch_enable_ap_mode(ap_name, password, CYW43_AUTH_WPA2_AES_PSK); // enable access point
        cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM); // set pico in performance mode

        // Set address
        ip4_addr_t mask;
        IP4_ADDR(ip_2_ip4(&state->gw), 192, 168, 4, 1);
        IP4_ADDR(ip_2_ip4(&mask), 255, 255, 255, 0);

        // Start the dhcp server
        dhcp_server_t dhcp_server;
        dhcp_server_init(&dhcp_server, &state->gw, &mask);

        // Open server
        if (!tcp_server_open(state, ap_name)) {
            continue;
        }
        
        // Wait for completion (in background)
        while(true) {
            vTaskDelay(VDELAY);
        }

        // if an error occurs, close the server and restart
        setRGB_COLOUR_PURPLE();
        tcp_server_close(state);
        dhcp_server_deinit(&dhcp_server);
    }
    return;
}

/**
The main loop for the UDP wifi. Sets up wifi to receive messages.
 */
void vWifiUDPTask() {
    for (;;) {
        // Enable server
        UDP_SERVER_T *udp_state = calloc(1, sizeof(UDP_SERVER_T));        
        
        const char *ap_name = "METR4810 Team 17";
        const char *password = "password";
        
        if (!udp_server_open(udp_state, ap_name)) {
            continue;
        }
        
        // Wait for completion (in background)
        while(true) {
            vTaskDelay(VDELAY);
        }

        // if error occurs, restart the server
        setRGB_COLOUR_PURPLE();
    }
    return;
}