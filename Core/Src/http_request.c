#include "http_request.h"
#include <stdint.h>
#include <string.h>

static int equal(const char* p, size_t n, const char* expected)
{
    if(n != strlen(expected)) { return 0; }
    for(size_t i = 0; i < n; ++i)
    {
        char c = p[i];
        if(c >= 'A' && c <= 'Z') { c = (char)(c - 'A' + 'a'); }
        if(c != expected[i]) { return 0; }
    }
    return 1;
}
static int copy(char* out, size_t capacity, const char* p, size_t n)
{
    if(n == 0U || n >= capacity) { return 0; }
    memcpy(out, p, n); out[n] = 0;
    return 1;
}
int HttpRequest_Parse(const char* data, size_t length, HttpRequest* r)
{
    if(data == NULL || r == NULL || length > 3071U || memchr(data, 0, length) != NULL) { return -1; }
    /* Caller maintains a NUL terminator outside length. All header searches are
     * bounded here so the parser is also safe on nonterminated host fixtures. */
    size_t header = 0;
    for(size_t i = 0; i + 3U < length; ++i)
    {
        if(memcmp(data + i, "\r\n\r\n", 4) == 0) { header = i + 4U; break; }
    }
    if(header == 0U) { return length > 1536U ? -1 : 0; }
    if(header > 1536U) { return -1; }
    memset(r, 0, sizeof(*r));
    const char* end = data + header;
    const char* line = data;
    const char* eol = line;
    while(eol + 1 < end && !(eol[0] == '\r' && eol[1] == '\n')) { ++eol; }
    size_t prefix;
    if(eol - line >= 5 && memcmp(line, "GET ", 4) == 0) { prefix = 4; }
    else if(eol - line >= 6 && memcmp(line, "POST ", 5) == 0) { prefix = 5; r->post = 1; }
    else { return -1; }
    const char* space = memchr(line + prefix, ' ', (size_t)(eol - line) - prefix);
    if(space == NULL || eol - space != 9 ||
       (memcmp(space + 1, "HTTP/1.1", 8) != 0 && memcmp(space + 1, "HTTP/1.0", 8) != 0) ||
       !copy(r->path, sizeof(r->path), line + prefix, (size_t)(space - line) - prefix) || r->path[0] != '/') { return -1; }
    for(const char* c = line; c < end; ++c)
    {
        if((unsigned char)*c < 32U && *c != '\r' && *c != '\n' && *c != '\t') { return -1; }
        if((unsigned char)*c >= 127U) { return -1; }
    }
    uint32_t seen = 0;
    size_t content_length = 0;
    line = eol + 2;
    while(line + 2 < end)
    {
        eol = line;
        while(eol + 1 < end && !(eol[0] == '\r' && eol[1] == '\n')) { ++eol; }
        const char* colon = memchr(line, ':', (size_t)(eol - line));
        if(colon == NULL || colon == line) { return -1; }
        const char* value = colon + 1;
        while(value < eol && (*value == ' ' || *value == '\t')) { ++value; }
        const char* last = eol;
        while(last > value && (last[-1] == ' ' || last[-1] == '\t')) { --last; }
        const size_t key_length = (size_t)(colon - line), n = (size_t)(last - value);
        unsigned int bit = 0;
        if(equal(line, key_length, "host")) { bit = 1; if(!copy(r->host, sizeof(r->host), value, n)) { return -1; } }
        else if(equal(line, key_length, "origin")) { bit = 2; if(!copy(r->origin, sizeof(r->origin), value, n)) { return -1; } }
        else if(equal(line, key_length, "content-length"))
        {
            bit = 4;
            if(n == 0U || n > 4U) { return -1; }
            for(size_t i = 0; i < n; ++i)
            {
                if(value[i] < '0' || value[i] > '9') { return -1; }
                content_length = content_length * 10U + (size_t)(value[i] - '0');
            }
            if(content_length > 1536U) { return -1; }
        }
        else if(equal(line, key_length, "transfer-encoding")) { return -1; }
        else if(equal(line, key_length, "x-air-config")) { bit = 8; r->write_token = n == 1U && *value == '1'; }
        else if(equal(line, key_length, "content-type"))
        { bit = 16; r->form_content_type = equal(value, n, "application/x-www-form-urlencoded"); }
        if(bit && (seen & bit)) { return -1; }
        seen |= bit;
        line = eol + 2;
    }
    if((seen & 1U) == 0U || (!r->post && content_length != 0U) || (r->post && (seen & 4U) == 0U)) { return -1; }
    if(length < header + content_length) { return 0; }
    if(length != header + content_length) { return -1; } /* no pipelining */
    r->body = data + header;
    r->body_length = content_length;
    return 1;
}

int HttpRequest_AllowedHost(const HttpRequest* r, const char* hostname)
{
    char local[40];
    const size_t n = strlen(hostname);
    if(n == 0U || n > 31U) { return 0; }
    memcpy(local, hostname, n);
    memcpy(local + n, ".local", 7);
    char host[64];
    const size_t host_length = strlen(r->host);
    memcpy(host, r->host, host_length + 1U);
    if(host_length > 3U && strcmp(host + host_length - 3U, ":80") == 0) { host[host_length - 3U] = 0; }
    for(size_t i = 0; host[i]; ++i)
    { if(host[i] >= 'A' && host[i] <= 'Z') { host[i] = (char)(host[i] - 'A' + 'a'); } }
    if(strcmp(host, "192.168.7.1") != 0 && strcmp(host, local) != 0) { return 0; }
    /* Browsers cannot send this custom header cross-origin without a preflight,
     * which this server rejects. Also check Origin and Host against rebinding. */
    if(r->post)
    {
        if(!r->write_token || !r->form_content_type) { return 0; }
        if(r->origin[0] != 0 && (strncmp(r->origin, "http://", 7) != 0 || strcmp(r->origin + 7, r->host) != 0)) { return 0; }
    }
    return 1;
}
