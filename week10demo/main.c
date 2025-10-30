#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/ip_addr.h"
#include "lwip/tcp.h"
#include "config.h"         // Your Wi-Fi config file (WIFI_SSID, WIFI_PASSWORD)

// --- NEW: Include your unified hardware configuration ---
#include "robot_config.h"

// =========================================================================
// W6 Telemetry/Web Server Code (mostly unchanged from your uploaded main.c)
// =========================================================================

typedef struct {
    uint32_t seq;
    uint32_t ms;                       // monotonic milliseconds since boot
    char     ip[16];                   // "a.b.c.d"
    char     type[MAX_TYPE_LEN+1];
    char     msg[MAX_MSG_LEN+1];
} Row;

static struct tcp_pcb *listen_pcb;
static Row rows[MAX_ROWS];
static uint32_t rows_count = 0;        // number ever added
static uint32_t rows_head  = 0;        // next write index
// ... (Your other web server static variables and functions like http_write, http_404, etc.)

static uint32_t now_ms(void){ return to_ms_since_boot(get_absolute_time()); }

// --- IMPORTANT: This is the function you call to log data to the web page ---
static void add_row(const ip_addr_t *src, const char *type, const char *msg) {
    Row *r = &rows[rows_head];
    r->seq = ++rows_count;
    r->ms  = now_ms();
    snprintf(r->ip,   sizeof(r->ip),   "%s", ipaddr_ntoa(src));
    strncpy(r->type, type, sizeof(r->type));
    strncpy(r->msg,  msg,  sizeof(r->msg));
    rows_head = (rows_head + 1) % MAX_ROWS;
}
// ... (Keep the rest of your web server implementation: clear_rows, on_recv, on_accept, etc.)


// =========================================================================
// Week 10 Integration Logic
// =========================================================================

// Helper function to log a motor/encoder state
void log_motion_telemetry(const ip_addr_t *src) {
    int16_t ax, ay, az;
    imu_read_raw(&ax, &ay, &az);

    uint32_t enc1_pw = encoder_pulse_width_us(1);
    uint32_t enc2_pw = encoder_pulse_width_us(2);
    int32_t distance_us = measure_echo_us();

    // Log IMU data
    char imu_msg[64];
    snprintf(imu_msg, sizeof(imu_msg), "AX=%d AY=%d AZ=%d", ax, ay, az);
    add_row(src, "IMU", imu_msg);

    // Log Encoder/Motion data
    char motion_msg[64];
    snprintf(motion_msg, sizeof(motion_msg), "M1_PW=%lu M2_PW=%lu", (unsigned long)enc1_pw, (unsigned long)enc2_pw);
    add_row(src, "Encoder", motion_msg);

    // Log Ultrasonic data (useful for Demo 3)
    char dist_msg[32];
    snprintf(dist_msg, sizeof(dist_msg), "%ld us", (long)distance_us);
    add_row(src, "Distance", dist_msg);
}


int main() {
    stdio_init_all(); sleep_ms(1200);

    // 1. Initialize Wi-Fi and Web Server
    if (cyw43_arch_init()) return -1;
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Failed to connect to Wi-Fi.\n");
        return -1;
    }
    printf("Wi-Fi connected! IP address: %s\n", ip4addr_ntoa(cyw43_arch_get_ip_info(CYW43_ITF_STA, NULL)->ip));
    // ... (rest of your web server setup)

    // 2. Initialize All Integrated Hardware Components
    motors_and_encoders_init();
    imu_init();
    ultrasonic_init();
    ir_sensors_init();
    add_row(NULL, "SYSTEM", "All hardware initialized.");
    motors_stop(); // Safety stop

    // 3. Week 10 Demo Control Loop (Example: Basic Motion)
    add_row(NULL, "STATUS", "Starting Demo 1: Move Forward...");
    
    // Drive forward at 30% speed
    motor_set(0.3f, 0.3f);
    
    // Log data every 100ms
    uint32_t last_log_ms = now_ms();
    const uint32_t LOG_PERIOD_MS = 100;
    const uint32_t RUN_TIME_MS = 5000;
    uint32_t start_time_ms = now_ms();

    while (true) {
        // Run the main Wi-Fi scheduler tick
        cyw43_arch_poll(); 

        // Check if it's time to log telemetry
        if (now_ms() - last_log_ms >= LOG_PERIOD_MS) {
            log_motion_telemetry(NULL); 
            last_log_ms = now_ms();
        }

        // Stop the demo after a set time
        if (now_ms() - start_time_ms >= RUN_TIME_MS) {
            motors_stop();
            add_row(NULL, "STATUS", "Demo 1 Complete. Motors stopped.");
            while(true) { cyw43_arch_poll(); sleep_ms(100); } // Loop and keep web server alive
        }

        // Short sleep to prevent busy-waiting
        sleep_ms(5); 
    }
    
    cyw43_arch_deinit();
    return 0;
}

// ---------------- TCP callbacks ----------------
static err_t on_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (!p){ tcp_close(tpcb); return ERR_OK; }
    tcp_recved(tpcb, p->tot_len);

    // parse first request line: "GET /path?qs HTTP/1.1"
    char line[256]={0}; int n = p->tot_len < sizeof(line)-1 ? p->tot_len : sizeof(line)-1;
    memcpy(line, p->payload, n); char *crlf = strstr(line,"\r\n"); if(crlf) *crlf=0;

    if (!strncmp(line,"GET ",4)) {
        char *path = line+4; char *sp=strchr(path,' '); if(sp)*sp=0;

        if (!strcmp(path,"/")) {
            serve_index(tpcb);
        } else if (!strcmp(path,"/favicon.ico")) {
            http_404(tpcb);
        } else if (!strncmp(path,"/api/send",9)) {
            // read params
            char tbuf[MAX_TYPE_LEN+1]={0}, mbuf[MAX_MSG_LEN+1]={0};
            get_qs_param(path,"type",tbuf,sizeof(tbuf));
            get_qs_param(path,"msg", mbuf,sizeof(mbuf));
            // record
            ip_addr_t rip = ((struct tcp_pcb*)tpcb)->remote_ip;
            add_row(&rip, tbuf, mbuf);
            http_write(tpcb, "{\"ok\":true}", "application/json");
        } else if (!strcmp(path,"/api/all")) {
            // dump JSON (latest first)
            char *out = (char*)malloc( MAX_ROWS*(MAX_MSG_LEN/2 + 64) + 64 );
            if (!out){ http_write(tpcb, "{\"err\":\"oom\"}", "application/json"); }
            else {
                int idx = rows_head, have = (rows_count<MAX_ROWS)?rows_count:MAX_ROWS;
                int len = sprintf(out, "{\"rows\":[");
                for (int i=0;i<have;i++){
                    const Row *r = &rows[(idx - 1 - i + MAX_ROWS) % MAX_ROWS];
                    len += snprintf(out+len, 65535, "%s{\"seq\":%u,\"ms\":%u,\"ip\":\"%s\",\"type\":\"%s\",\"msg\":\"%s\"}",
                                    (i?",":""), r->seq, r->ms, r->ip, r->type, r->msg);
                }
                len += sprintf(out+len, "]}");
                out[len]=0;
                http_write(tpcb, out, "application/json");
                free(out);
            }
        } else if (!strcmp(path,"/api/clear")) {
            clear_rows();
            http_write(tpcb, "{\"ok\":true}", "application/json");
        } else {
            http_404(tpcb);
        }
    } else {
        http_404(tpcb);
    }

    pbuf_free(p);
    tcp_close(tpcb); // simple: close per request
    return ERR_OK;
}

static err_t on_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
    tcp_recv(newpcb, on_recv);
    return ERR_OK;
}
