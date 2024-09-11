#ifndef WIFI_HELPERS_H
#define WIFI_HELPERS_H


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
err_t tcp_server_poll(void*, struct tcp_pcb*);
void tcp_server_err(void*, err_t);
err_t tcp_server_accept(void*, struct tcp_pcb*, err_t);
bool tcp_server_open(void*, const char*);
err_t tcp_close_client_connection(TCP_CONNECT_STATE_T*, struct tcp_pcb*, err_t);
void tcp_server_close(TCP_SERVER_T*);
err_t tcp_server_sent(void*, struct tcp_pcb*, u16_t);


#endif