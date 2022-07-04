#include "app_http.h"
#include "app_debug.h"

#include "lwip/tcp.h"
#include "lwip/tcpip.h"
#include "lwip/apps/http_client.h"
#include "lwip/dns.h"
#include "lwip/debug.h"
#include "lwip/mem.h"
//#include "lwip/altcp_tls.h"
#include "lwip/init.h"
#include "string.h"
#include "fatfs.h"
#include "semphr.h"
#include "utilities.h"

#define HTTP_DOWNLOAD_BUFFER_SIZE 1024

static app_http_config_t m_http_cfg;
static uint32_t m_total_bytes_recv = 0;
static uint32_t m_content_length = 0;
static char m_http_cmd_buffer[256];

static uint32_t m_real_data_rev = 0;

static httpc_connection_t m_conn_settings_try;
static err_t httpc_file_recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static httpc_state_t *m_http_connection_state;
void http_get_body(uint8_t *data);

//static uint8_t post_body[1024];
static uint8_t *data_carry;
static uint8_t data_rev[128];
static uint8_t data_to_get[128];
static uint32_t block = 0;
app_http_config_t *app_http_get_config(void)
{
    return &m_http_cfg;
}
// extern bool send_offline_file;
void app_http_cleanup(void)
{
    m_total_bytes_recv = 0;
    m_content_length = 0;
    m_real_data_rev = 0;
    memset(&m_http_cmd_buffer, 0, sizeof(m_http_cmd_buffer));
    memset(&m_http_cfg, 0, sizeof(m_http_cfg));
}

static uint32_t m_begin = 0;
bool m_http_free = true;
bool app_http_is_idle(void)
{
    return m_http_free;
}
/**
 * @brief Result transfer done callback
 */
static void httpc_result_callback(void *arg, httpc_result_t httpc_result, u32_t rx_content_len, u32_t srv_res, err_t err)
{
    DEBUG_INFO("result: %d, content len: %d, status code: %d, mem %u\r\n", httpc_result, rx_content_len, srv_res, xPortGetFreeHeapSize());
    if (data_carry)
    {
        vPortFree(data_carry);
        data_carry = NULL;
    }

    if (srv_res != 200)
    {
        block = 0;
    }
    switch (httpc_result)
    {
    case HTTPC_RESULT_OK: /** File successfully received */
    {
        DEBUG_INFO("HTTPC_RESULT_OK, speed %uKB/s\r\n", rx_content_len / (sys_get_ms() - m_begin));
        if (m_http_cfg.on_event_cb)
        {
            m_http_cfg.on_event_cb(APP_HTTP_EVENT_FINISH_SUCCESS, &rx_content_len);
        }
    }
    break;

    case HTTPC_RESULT_ERR_UNKNOWN:     /** Unknown error */
                                       // break;
    case HTTPC_RESULT_ERR_CONNECT:     /** Connection to server failed */
                                       // break;
    case HTTPC_RESULT_ERR_HOSTNAME:    /** Failed to resolve server hostname */
                                       // break;
    case HTTPC_RESULT_ERR_CLOSED:      /** Connection unexpectedly closed by remote server */
                                       // break;
    case HTTPC_RESULT_ERR_TIMEOUT:     /** Connection timed out (server didn't respond in time) */
                                       // break;
    case HTTPC_RESULT_ERR_SVR_RESP:    /** Server responded with an error code */
                                       // break;
    case HTTPC_RESULT_ERR_MEM:         /** Local memory error */
                                       // break;
    case HTTPC_RESULT_LOCAL_ABORT:     /** Local abort */
                                       // break;
    case HTTPC_RESULT_ERR_CONTENT_LEN: /** Content length mismatch */
        DEBUG_ERROR("HTTP error %d, r = %d\r\n", httpc_result, err);
        if (m_http_cfg.on_event_cb)
        {
            m_http_cfg.on_event_cb(APP_HTTP_EVENT_FINISH_FAILED, &httpc_result);
        }
        break;

    default:
        DEBUG_INFO("httpc_result_callback error %d\r\n", err);
        break;
    }

    m_http_free = true;
}

//void trans_content_to_body(uint8_t *databuff, uint16_t len)
//{
//    for (uint16_t i = 0; i < len; i++)
//    {
//        post_body[i] = databuff[i];
//    }
//    body_len = len;
//}

/**
 * @brief Result transfer done callback
 */
static err_t http_post_make_body(httpc_state_t *connection, void *arg, uint8_t **buffer, uint16_t *len)
{
    DEBUG_VERBOSE("HTTP post body\r\n");
//    *buffer = (uint8_t *)post_body;
//    *len = body_len;
    app_http_data_t require_data =
	{
		.data = NULL,
		.length = 0
	};
    if (m_http_cfg.on_event_cb)
    {
    	m_http_cfg.on_event_cb(APP_HTTP_EVENT_DATA, &require_data);
    }
    *buffer = require_data.data;
    *len = require_data.length;

    return ERR_OK;
}

static err_t http_post_make_body_from_file(httpc_state_t *connection, void *arg, uint8_t **buffer, uint16_t *len)
{
	DEBUG_VERBOSE("HTTP post body\r\n");
    static uint32_t last_pos = 0;
    uint32_t byte_read;
    uint32_t size_of_file = fatfs_get_file_size(m_http_cfg.local_file);

    if (!data_carry)
    {
        data_carry = (uint8_t *)pvPortMalloc(513 * sizeof(uint8_t));
    }

    if (!data_carry)
    {
    	return ERR_MEM;
    }

    if (size_of_file)
    {
        memset(data_carry, 0, 513);
        byte_read = fatfs_read_file_at_pos(m_http_cfg.local_file, data_carry, (uint32_t)512, last_pos);
        //			DEBUG_INFO ("DATA READ: %s \r\n", data);
        if (byte_read)
        {
            //				memcpy(data_carry, data, byte_read);
            if (block * 512 < size_of_file)
            {
                block++;
                if (size_of_file < 512)
                {
                	*len = byte_read;
                }
                else
                {
                	*len = 512;
                }
            }
            else
            {
                *len = size_of_file - (512 * block) + 2;
                block = 0;
                DEBUG_INFO("All data was send to server\r\n");
            }
//            DEBUG_RAW("%s", data_carry, *len);

            *buffer = (uint8_t *)data_carry;
            last_pos += byte_read;
        }
        else
        {
        	*buffer = NULL;
            *len = 0;
            last_pos = 0;
            block = 0;
            DEBUG_INFO("No more data\r\n");
        }
        DEBUG_VERBOSE("LAST POS NOW IS %u\r\n", last_pos);
    }

    //    	end:
    return ERR_OK;
}
/**
 * @brief Header received done callback
 */
err_t httpc_headers_done_callback(httpc_state_t *connection, void *arg, struct pbuf *hdr, u16_t hdr_len, u32_t content_len)
{
    DEBUG_VERBOSE("httpc_headers_callback, content length %d\r\n", content_len);
    char *p = pvPortMalloc(hdr_len + 1);
    if (p)
    {
		snprintf(p, hdr_len, "HEADER: %s", (char *)hdr->payload);
		DEBUG_INFO("%s\r\n", p);
		vPortFree(p);
    }

    m_content_length = content_len;
    if (content_len == 0xFFFFFFFF)
    {
        DEBUG_INFO("Invalid content length\r\n");
        if (m_http_cfg.on_event_cb)
            m_http_cfg.on_event_cb(APP_HTTP_EVENT_FINISH_FAILED, &m_content_length);
    }
    else
    {
        if (m_http_cfg.on_event_cb)
            m_http_cfg.on_event_cb(APP_HTTP_EVENT_CONNTECTED, &m_content_length);
    }

    return ERR_OK;
}

/**
 * @brief Handle data connection incoming data
 * @param pointer to lwftp session data
 * @param pointer to PCB
 * @param pointer to incoming pbuf
 * @param state of incoming process
 */
uint32_t last_tick = 0;

static err_t httpc_file_recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    // DEBUG_INFO("lwip http event cb\r\n");
    static uint32_t len = 0;
    static uint16_t offset = 0;
    if (p)
    {
        struct pbuf *q;
        for (q = p; q; q = q->next)
        {
            memcpy(&data_rev[m_total_bytes_recv], q->payload, q->len); // RECIEVE ALL DATA AFTER SOME CALLBACK
            m_total_bytes_recv += q->len;
            DEBUG_VERBOSE("HTTP data %s\r\n", data_rev);
        }
        {
            m_real_data_rev = m_total_bytes_recv; // TAM GAN SO BYTE NHAN
            memcpy(data_to_get, data_rev, 128);   // COPY DATA NHAN DC TRONG MOI TRUONG HOP
        }

        // if content contain this it's encoded content
        if (strstr((char *)data_rev, "\r\n0\r\n\r\n")) // KIEM TRA CHUNKED ENCODE ??? NEU PHAT HIEN END CHUNK THI XU LI
        {
            m_real_data_rev = 0;
            memset(data_to_get, '\0', sizeof(data_to_get));
            len = utilities_get_number_from_hex_string(0, (char *)data_rev);
            char *content = strstr((char *)data_rev, "\r\n") + 2;
            memcpy(data_to_get, content, len);
            while (len)
            {
                m_real_data_rev += len;
                content += len;
                content = strstr(content, "\r\n");
                content += 2;
                len = utilities_get_number_from_hex_string(0, content);
                memcpy(&data_to_get[offset], content, len);
                offset += len;
            }
            len = 0;
            DEBUG_VERBOSE("HTTP chunked data \r\n%s\r\n", data_to_get);
        }
        // MUST used 2 commands!
        tcp_recved(tpcb, p->tot_len);
        pbuf_free(p);
    }
    else
    {
        DEBUG_WARN("tcp_close\r\n");
        tcp_close(tpcb);
        return ERR_ABRT;
    }

    // DEBUG_INFO("Done\r\n");
    return ERR_OK;
}

void http_get_body(uint8_t *data)
{
    DEBUG_VERBOSE("REAL DATA GET %d\r\n", m_real_data_rev);
    DEBUG_VERBOSE("DATA GET: %s\r\n", data_to_get);
    for (uint8_t i = 0; i < m_real_data_rev; i++)
    {
        data[i] = data_to_get[i];
    }
}

bool app_http_start(app_http_config_t *config)
{
    // ASSERT(config);

    app_http_cleanup();
    sprintf(m_http_cfg.url, "%s", config->url);
    m_http_cfg.port = config->port;
    sprintf(m_http_cfg.file, "%s", config->file);
    m_http_cfg.on_event_cb = config->on_event_cb;
    m_http_cfg.len = config->len;
    m_http_cfg.local_file = config->local_file;

    /* Init Http connection params */
    m_conn_settings_try.use_proxy = 0;
    m_conn_settings_try.headers_done_fn = httpc_headers_done_callback;
    m_conn_settings_try.result_fn = httpc_result_callback;

    DEBUG_INFO("HTTP url %s:%u%s\r\n", m_http_cfg.url, m_http_cfg.port, m_http_cfg.file);
    err_t error;

    if ((config->transfile) == TRANS_STRING)
    {
    	DEBUG_INFO("HTTP method post string\r\n");
        m_conn_settings_try.on_post_body_cb = http_post_make_body;
    }
    else
    {
        DEBUG_INFO("HTTP method post\r\n");
        m_conn_settings_try.on_post_body_cb = http_post_make_body_from_file;
    }

    if (config->method == APP_HTTP_GET)
    {
        m_conn_settings_try.method = HTTP_METHOD_GET;
        error = httpc_get_file_dns((const char *)m_http_cfg.url,
                                   m_http_cfg.port,
                                   m_http_cfg.file,
                                   &m_conn_settings_try,
                                   httpc_file_recv_callback,
                                   NULL,
                                   &m_http_connection_state);
    }
    else // post
    {
        m_conn_settings_try.method = HTTP_METHOD_POST;
        DEBUG_INFO("Post body size :%d\r\n", m_http_cfg.len);
        error = httpc_post_file_dns((const char *)m_http_cfg.url,
                                    m_http_cfg.port,
                                    m_http_cfg.file,
                                    &m_conn_settings_try,
                                    httpc_file_recv_callback,
                                    NULL,
                                    &m_http_connection_state,
									config->len);
    }

    m_conn_settings_try.headers_done_fn = httpc_headers_done_callback;
    m_conn_settings_try.result_fn = httpc_result_callback;

    if (error != ERR_OK)
    {
        DEBUG_INFO("Cannot connect HTTP server, error %d\r\n", error);
        return false;
    }

    m_begin = sys_get_ms();

    if (m_http_cfg.on_event_cb)
        m_http_cfg.on_event_cb(APP_HTTP_EVENT_START, (void *)0);
    m_http_free = false;
    return true;
}
