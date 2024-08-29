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

#include "diagnostics.h"

#define VDELAY 100

#define TCP_PORT 80 // the port to host the wifi on
#define POLL_TIME_S 5 // the polling time of the wifi server
#define HTTP_GET "GET" // GET Request

// webpage paths
#define LED_PATH "/ledtest"
#define DIAGNOSTICS_PATH "/diagnostics"
#define LOG_PATH "/log"

// HTTP formats
#define DEFAULT_BODY "<html><body>%s</body></html>"
#define LED_BODY "<html><body><h1>Hello from Pico W.</h1><p>Led is %s</p><p><a href=\"?led=%d\">Turn led %s</a></body></html>"
#define HTTP_RESPONSE_REDIRECT "HTTP/1.1 302 Redirect\nLocation: http://%s" LED_PATH "\n\n"
#define HTTP_RESPONSE_HEADERS "HTTP/1.1 %d OK\nContent-Length: %d\nContent-Type: text/html; charset=utf-8\nConnection: close\n\n"

// LED params
#define LED_PARAM "led=%d"
#define LED_GPIO 0

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

// Structure to represent the server
typedef struct TCP_SERVER_T_ {
    struct tcp_pcb *server_pcb;
    bool complete;
    ip_addr_t gw;
    async_context_t *context;
} TCP_SERVER_T;

// Structure to represent the connection packet
typedef struct TCP_CONNECT_STATE_T_ {
    struct tcp_pcb *pcb;
    int sent_len;
    char headers[128];
    char result[MAX_RESULT_SIZE];
    int header_len;
    int result_len;
    ip_addr_t *gw;
} TCP_CONNECT_STATE_T;

// Esoteric TCP handlers
err_t tcp_server_recv(void*, struct tcp_pcb*, struct pbuf*, err_t);
static err_t tcp_server_poll(void*, struct tcp_pcb*);
static void tcp_server_err(void*, err_t);
static err_t tcp_server_accept(void*, struct tcp_pcb*, err_t);
static bool tcp_server_open(void*, const char*);
static err_t tcp_close_client_connection(TCP_CONNECT_STATE_T*, struct tcp_pcb*, err_t);
static void tcp_server_close(TCP_SERVER_T*);
static err_t tcp_server_sent(void*, struct tcp_pcb*, u16_t);

// Handles response
static int generate_response(const char*, const char*, char*, size_t);

// Function prototypes
void vWifiInit();
void vWifiTask();

// Handle request from client
err_t tcp_server_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
    if (!p) {
        vDebugLog("connection closed\n");
        return tcp_close_client_connection(con_state, pcb, ERR_OK);
    }
    assert(con_state && con_state->pcb == pcb);
    if (p->tot_len > 0) {
        vDebugLog("tcp_server_recv %d err %d\n", p->tot_len, err);

        // Copy the request into the buffer
        pbuf_copy_partial(p, con_state->headers, p->tot_len > sizeof(con_state->headers) - 1 ? sizeof(con_state->headers) - 1 : p->tot_len, 0);

        // Handle GET request
        if (strncmp(HTTP_GET, con_state->headers, sizeof(HTTP_GET) - 1) == 0) {
            char *request = con_state->headers + sizeof(HTTP_GET); // + space
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
            vDebugLog("Request: %s?%s\n", request, params);
            vDebugLog("Result: %d\n", con_state->result_len);

            // Check we had enough buffer space
            if (con_state->result_len > sizeof(con_state->result) - 1) {
                vDebugLog("Too much result data %d\n", con_state->result_len);
                return tcp_close_client_connection(con_state, pcb, ERR_CLSD);
            }

            // Generate web page
            if (con_state->result_len > 0) {
                con_state->header_len = snprintf(con_state->headers, sizeof(con_state->headers), HTTP_RESPONSE_HEADERS,
                    200, con_state->result_len);
                if (con_state->header_len > sizeof(con_state->headers) - 1) {
                    vDebugLog("Too much header data %d\n", con_state->header_len);
                    return tcp_close_client_connection(con_state, pcb, ERR_CLSD);
                }
            } else {
                // Send redirect
                con_state->header_len = snprintf(con_state->headers, sizeof(con_state->headers), HTTP_RESPONSE_REDIRECT,
                    ipaddr_ntoa(con_state->gw));
                vDebugLog("Sending redirect %s", con_state->headers);
            }

            // Send the headers to the client
            con_state->sent_len = 0;
            err_t err = tcp_write(pcb, con_state->headers, con_state->header_len, 0);
            if (err != ERR_OK) {
                vDebugLog("failed to write header data %d\n", err);
                return tcp_close_client_connection(con_state, pcb, err);
            }

            // Send the body to the client
            if (con_state->result_len) {
                err = tcp_write(pcb, con_state->result, con_state->result_len, 0);
                if (err != ERR_OK) {
                    vDebugLog("failed to write result data %d\n", err);
                    return tcp_close_client_connection(con_state, pcb, err);
                }
            }
        }
        tcp_recved(pcb, p->tot_len);
    }
    pbuf_free(p);
    return ERR_OK;
}

// Poll server for client connections
static err_t tcp_server_poll(void *arg, struct tcp_pcb *pcb) {
    TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
    vDebugLog("tcp_server_poll_fn\n");
    return tcp_close_client_connection(con_state, pcb, ERR_OK); // Just disconnect clent?
}

// Check client errors
static void tcp_server_err(void *arg, err_t err) {
    TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
    if (err != ERR_ABRT) {
        vDebugLog("tcp_client_err_fn %d\n", err);
        tcp_close_client_connection(con_state, con_state->pcb, err);
    }
}

// Accept client connection
static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (err != ERR_OK || client_pcb == NULL) {
        vDebugLog("failure in accept\n");
        return ERR_VAL;
    }
    vDebugLog("client connected\n");

    // Create the state for the connection
    TCP_CONNECT_STATE_T *con_state = calloc(1, sizeof(TCP_CONNECT_STATE_T));
    if (!con_state) {
        vDebugLog("failed to allocate connect state\n");
        return ERR_MEM;
    }
    con_state->pcb = client_pcb; // for checking
    con_state->gw = &state->gw;

    // setup connection to client
    tcp_arg(client_pcb, con_state);
    tcp_sent(client_pcb, tcp_server_sent);
    tcp_recv(client_pcb, tcp_server_recv);
    tcp_poll(client_pcb, tcp_server_poll, POLL_TIME_S * 2);
    tcp_err(client_pcb, tcp_server_err);

    return ERR_OK;
}

// Open server
static bool tcp_server_open(void *arg, const char *ap_name) {
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    vDebugLog("starting server on port %d\n", TCP_PORT);

    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        vDebugLog("failed to create pcb\n");
        return false;
    }

    err_t err = tcp_bind(pcb, IP_ANY_TYPE, TCP_PORT);
    if (err) {
        vDebugLog("failed to bind to port %d\n",TCP_PORT);
        return false;
    }

    state->server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (!state->server_pcb) {
        vDebugLog("failed to listen\n");
        if (pcb) {
            tcp_close(pcb);
        }
        return false;
    }

    tcp_arg(state->server_pcb, state);
    tcp_accept(state->server_pcb, tcp_server_accept);

    vDebugLog("Try connecting to '%s'\n", ap_name);
    return true;
}

// Close a connection to client
static err_t tcp_close_client_connection(TCP_CONNECT_STATE_T *con_state, struct tcp_pcb *client_pcb, err_t close_err) {
    if (client_pcb) {
        assert(con_state && con_state->pcb == client_pcb);
        tcp_arg(client_pcb, NULL);
        tcp_poll(client_pcb, NULL, 0);
        tcp_sent(client_pcb, NULL);
        tcp_recv(client_pcb, NULL);
        tcp_err(client_pcb, NULL);
        err_t err = tcp_close(client_pcb);
        if (err != ERR_OK) {
            vDebugLog("close failed %d, calling abort\n", err);
            tcp_abort(client_pcb);
            close_err = ERR_ABRT;
        }
        if (con_state) {
            free(con_state);
        }
    }
    return close_err;
}

// Close the server
static void tcp_server_close(TCP_SERVER_T *state) {
    if (state->server_pcb) {
        tcp_arg(state->server_pcb, NULL);
        tcp_close(state->server_pcb);
        state->server_pcb = NULL;
    }
}

// Send data packet
static err_t tcp_server_sent(void *arg, struct tcp_pcb *pcb, u16_t len) {
    TCP_CONNECT_STATE_T *con_state = (TCP_CONNECT_STATE_T*)arg;
    vDebugLog("tcp_server_sent %u\n", len);
    con_state->sent_len += len;
    if (con_state->sent_len >= con_state->header_len + con_state->result_len) {
        vDebugLog("all done\n");
        return tcp_close_client_connection(con_state, pcb, ERR_OK);
    }
    return ERR_OK;
}

// Send content as a response to a request
static int generate_response(const char *request, const char *params, char *result, size_t max_result_len) {
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
            vDebugLog("failed to open server\n");
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