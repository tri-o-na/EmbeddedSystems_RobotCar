#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/ip_addr.h"
#include "lwip/tcp.h"
#include "config.h"

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

// ---------------- helpers ----------------
static uint32_t now_ms(void){ return to_ms_since_boot(get_absolute_time()); }

static void add_row(const ip_addr_t *src, const char *type, const char *msg) {
    Row *r = &rows[rows_head];
    r->seq = ++rows_count;
    r->ms  = now_ms();
    snprintf(r->ip,   sizeof(r->ip),   "%s", ipaddr_ntoa(src));
    snprintf(r->type, sizeof(r->type), "%.*s", MAX_TYPE_LEN,  type?type:"");
    snprintf(r->msg,  sizeof(r->msg),  "%.*s", MAX_MSG_LEN,   msg?msg:"");
    rows_head = (rows_head + 1) % MAX_ROWS;
}

static void clear_rows(void){ rows_count = 0; rows_head = 0; memset(rows,0,sizeof(rows)); }

// very small URL decoder: handles %HH and '+' -> space
static void url_decode(char *s) {
    char *r = s, *w = s;
    while (*r) {
        if (*r=='%' && isxdigit((unsigned char)r[1]) && isxdigit((unsigned char)r[2])) {
            int hi = (r[1]<='9'?r[1]-'0':(tolower(r[1])-'a'+10));
            int lo = (r[2]<='9'?r[2]-'0':(tolower(r[2])-'a'+10));
            *w++ = (char)((hi<<4)|lo); r+=3;
        } else if (*r=='+') { *w++=' '; r++; }
        else { *w++=*r++; }
    }
    *w=0;
}

static int get_qs_param(const char *path_qs, const char *key, char *out, int outsz) {
    const char *q = strchr(path_qs, '?'); if(!q) return 0; q++;
    size_t klen = strlen(key);
    while (*q) {
        if (!strncmp(q, key, klen) && q[klen]=='=') {
            q += klen+1;
            int i=0;
            while (*q && *q!='&' && *q!=' ') { if (i<outsz-1) out[i++]=*q; q++; }
            out[i]=0; url_decode(out); return 1;
        }
        const char *amp = strchr(q,'&'); if(!amp) break; q = amp+1;
    }
    return 0;
}

static void http_write(struct tcp_pcb *tpcb, const char *body, const char *ctype) {
    char hdr[256]; int blen=(int)strlen(body);
    int hlen = snprintf(hdr,sizeof(hdr),
        "HTTP/1.1 200 OK\r\nContent-Type: %s; charset=utf-8\r\n"
        "Connection: close\r\nContent-Length: %d\r\n\r\n", ctype, blen);
    tcp_write(tpcb, hdr, hlen, TCP_WRITE_FLAG_COPY);
    tcp_write(tpcb, body, blen, TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);
}

static void http_404(struct tcp_pcb *tpcb){
    const char* s="HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n";
    tcp_write(tpcb, s, strlen(s), TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);
}

static void serve_index(struct tcp_pcb *tpcb) {
    // The page fetches /api/all every second and renders a table.
    const char *page =
        "<!doctype html><html><head><meta name=viewport content='width=device-width, initial-scale=1'/>"
        "<title>Pico Telemetry</title>"
        "<style>body{font-family:system-ui,Arial;margin:24px;max-width:900px}"
        "table{border-collapse:collapse;width:100%}th,td{border:1px solid #ddd;padding:6px}"
        "th{background:#f3f3f3;text-align:left}code{background:#f6f6f6;padding:2px 4px;border-radius:4px}"
        "input,button{font-size:16px;padding:6px;margin-right:6px}</style></head><body>"
        "<h2>Pico W Telemetry ✅</h2>"
        "<div style='margin:8px 0'>"
        "<input id=type placeholder='type (e.g., temp, log)' style='width:160px'>"
        "<input id=msg  placeholder='message' style='width:420px'>"
        "<button onclick='send()'>Send</button>"
        "<button onclick='clr()'>Clear</button>"
        "</div>"
        "<table id=tbl><thead><tr><th>#</th><th>ms</th><th>ip</th><th>type</th><th>message</th></tr></thead><tbody></tbody></table>"
        "<script>"
        "async function refresh(){let r=await fetch('/api/all');let j=await r.json();"
        "let tb=document.querySelector('#tbl tbody');tb.innerHTML='';"
        "for(const row of j.rows){let tr=document.createElement('tr');"
        "tr.innerHTML=`<td>${row.seq}</td><td>${row.ms}</td><td>${row.ip}</td><td>${row.type}</td><td>${row.msg}</td>`;tb.appendChild(tr);} }"
        "async function send(){const t=document.getElementById('type').value||'';"
        "const m=document.getElementById('msg').value||'';"
        "await fetch('/api/send?type='+encodeURIComponent(t)+'&msg='+encodeURIComponent(m));"
        "document.getElementById('msg').value=''; refresh(); }"
        "async function clr(){await fetch('/api/clear'); refresh();}"
        "setInterval(refresh,1000); refresh();"
        "</script></body></html>";
    http_write(tpcb, page, "text/html");
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


// ===== NEW: bidirectional TCP server =====
static err_t bidir_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (!p) { tcp_close(tpcb); return ERR_OK; }
    tcp_recved(tpcb, p->tot_len);

    printf("PC -> %.*s\n", p->tot_len, (char*)p->payload);

    // Reply message back to PC
    const char *reply = "I am pico and i am sending back to PC";
    tcp_write(tpcb, reply, strlen(reply), TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);

    pbuf_free(p);
    return ERR_OK;
}

static err_t bidir_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
    printf("Bidirectional client connected!\n");
    tcp_recv(newpcb, bidir_recv);
    return ERR_OK;
}

static void start_bidir_server(void) {
    struct tcp_pcb *pcb = tcp_new();
    tcp_bind(pcb, IP_ADDR_ANY, 12345);       // listen on TCP port 12345
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, bidir_accept);
    printf("Bidirectional TCP server on port 12345\n");
}



int main() {
    stdio_init_all(); sleep_ms(1200);
    if (cyw43_arch_init()) return -1;
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) return -1;

    const ip4_addr_t *ip = netif_ip4_addr(netif_default);
    printf("Open http://%s/  (telemetry table)\n", ip4addr_ntoa(ip));

    listen_pcb = tcp_new();
    tcp_bind(listen_pcb, IP_ADDR_ANY, TCP_PORT);
    listen_pcb = tcp_listen(listen_pcb);
    tcp_accept(listen_pcb, on_accept);


    // ===== NEW =====
    start_bidir_server();
    // ===== END NEW =====

    while (true) {
        // blink heartbeat
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1); sleep_ms(60);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0); sleep_ms(940);
    }
}
