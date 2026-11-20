#ifndef HTTP_REQUEST_H
#define HTTP_REQUEST_H
#include <stddef.h>
typedef struct
{
    int post;
    char path[96], host[64], origin[96];
    const char* body;
    size_t body_length;
    int write_token;
    int form_content_type;
} HttpRequest;
/* 0: more bytes needed, 1: complete, -1: malformed/unsupported framing. */
int HttpRequest_Parse(const char* data, size_t length, HttpRequest* output);
int HttpRequest_AllowedHost(const HttpRequest* request, const char* hostname);
#endif
