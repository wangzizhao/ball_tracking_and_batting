// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#include <string.h>
#include "camera_coord_t.h"

static int __camera_coord_t_hash_computed;
static int64_t __camera_coord_t_hash;

int64_t __camera_coord_t_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __camera_coord_t_get_hash)
            return 0;

    __lcm_hash_ptr cp;
    cp.parent =  p;
    cp.v = (void*)__camera_coord_t_get_hash;
    (void) cp;

    int64_t hash = (int64_t)0xe7a63d3acc61be77LL
         + __double_hash_recursive(&cp)
         + __double_hash_recursive(&cp)
         + __double_hash_recursive(&cp)
         + __double_hash_recursive(&cp)
         + __double_hash_recursive(&cp)
         + __double_hash_recursive(&cp)
         + __int64_t_hash_recursive(&cp)
         + __int64_t_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __camera_coord_t_get_hash(void)
{
    if (!__camera_coord_t_hash_computed) {
        __camera_coord_t_hash = __camera_coord_t_hash_recursive(NULL);
        __camera_coord_t_hash_computed = 1;
    }

    return __camera_coord_t_hash;
}

int __camera_coord_t_encode_array(void *buf, int offset, int maxlen, const camera_coord_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &(p[element].x), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &(p[element].y), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &(p[element].z), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &(p[element].v_x), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &(p[element].v_y), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_encode_array(buf, offset + pos, maxlen - pos, &(p[element].v_z), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].mode), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].utime), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int camera_coord_t_encode(void *buf, int offset, int maxlen, const camera_coord_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __camera_coord_t_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __camera_coord_t_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int __camera_coord_t_encoded_array_size(const camera_coord_t *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {

        size += __double_encoded_array_size(&(p[element].x), 1);

        size += __double_encoded_array_size(&(p[element].y), 1);

        size += __double_encoded_array_size(&(p[element].z), 1);

        size += __double_encoded_array_size(&(p[element].v_x), 1);

        size += __double_encoded_array_size(&(p[element].v_y), 1);

        size += __double_encoded_array_size(&(p[element].v_z), 1);

        size += __int64_t_encoded_array_size(&(p[element].mode), 1);

        size += __int64_t_encoded_array_size(&(p[element].utime), 1);

    }
    return size;
}

int camera_coord_t_encoded_size(const camera_coord_t *p)
{
    return 8 + __camera_coord_t_encoded_array_size(p, 1);
}

size_t camera_coord_t_struct_size(void)
{
    return sizeof(camera_coord_t);
}

int camera_coord_t_num_fields(void)
{
    return 8;
}

int camera_coord_t_get_field(const camera_coord_t *p, int i, lcm_field_t *f)
{
    if (0 > i || i >= camera_coord_t_num_fields())
        return 1;
    
    switch (i) {
    
        case 0: {
            f->name = "x";
            f->type = LCM_FIELD_DOUBLE;
            f->typestr = "double";
            f->num_dim = 0;
            f->data = (void *) &p->x;
            return 0;
        }
        
        case 1: {
            f->name = "y";
            f->type = LCM_FIELD_DOUBLE;
            f->typestr = "double";
            f->num_dim = 0;
            f->data = (void *) &p->y;
            return 0;
        }
        
        case 2: {
            f->name = "z";
            f->type = LCM_FIELD_DOUBLE;
            f->typestr = "double";
            f->num_dim = 0;
            f->data = (void *) &p->z;
            return 0;
        }
        
        case 3: {
            f->name = "v_x";
            f->type = LCM_FIELD_DOUBLE;
            f->typestr = "double";
            f->num_dim = 0;
            f->data = (void *) &p->v_x;
            return 0;
        }
        
        case 4: {
            f->name = "v_y";
            f->type = LCM_FIELD_DOUBLE;
            f->typestr = "double";
            f->num_dim = 0;
            f->data = (void *) &p->v_y;
            return 0;
        }
        
        case 5: {
            f->name = "v_z";
            f->type = LCM_FIELD_DOUBLE;
            f->typestr = "double";
            f->num_dim = 0;
            f->data = (void *) &p->v_z;
            return 0;
        }
        
        case 6: {
            f->name = "mode";
            f->type = LCM_FIELD_INT64_T;
            f->typestr = "int64_t";
            f->num_dim = 0;
            f->data = (void *) &p->mode;
            return 0;
        }
        
        case 7: {
            f->name = "utime";
            f->type = LCM_FIELD_INT64_T;
            f->typestr = "int64_t";
            f->num_dim = 0;
            f->data = (void *) &p->utime;
            return 0;
        }
        
        default:
            return 1;
    }
}

const lcm_type_info_t *camera_coord_t_get_type_info(void)
{
    static int init = 0;
    static lcm_type_info_t typeinfo;
    if (!init) {
        typeinfo.encode         = (lcm_encode_t) camera_coord_t_encode;
        typeinfo.decode         = (lcm_decode_t) camera_coord_t_decode;
        typeinfo.decode_cleanup = (lcm_decode_cleanup_t) camera_coord_t_decode_cleanup;
        typeinfo.encoded_size   = (lcm_encoded_size_t) camera_coord_t_encoded_size;
        typeinfo.struct_size    = (lcm_struct_size_t)  camera_coord_t_struct_size;
        typeinfo.num_fields     = (lcm_num_fields_t) camera_coord_t_num_fields;
        typeinfo.get_field      = (lcm_get_field_t) camera_coord_t_get_field;
        typeinfo.get_hash       = (lcm_get_hash_t) __camera_coord_t_get_hash;
    }
    
    return &typeinfo;
}
int __camera_coord_t_decode_array(const void *buf, int offset, int maxlen, camera_coord_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &(p[element].x), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &(p[element].y), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &(p[element].z), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &(p[element].v_x), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &(p[element].v_y), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __double_decode_array(buf, offset + pos, maxlen - pos, &(p[element].v_z), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].mode), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].utime), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __camera_coord_t_decode_array_cleanup(camera_coord_t *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __double_decode_array_cleanup(&(p[element].x), 1);

        __double_decode_array_cleanup(&(p[element].y), 1);

        __double_decode_array_cleanup(&(p[element].z), 1);

        __double_decode_array_cleanup(&(p[element].v_x), 1);

        __double_decode_array_cleanup(&(p[element].v_y), 1);

        __double_decode_array_cleanup(&(p[element].v_z), 1);

        __int64_t_decode_array_cleanup(&(p[element].mode), 1);

        __int64_t_decode_array_cleanup(&(p[element].utime), 1);

    }
    return 0;
}

int camera_coord_t_decode(const void *buf, int offset, int maxlen, camera_coord_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __camera_coord_t_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __camera_coord_t_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int camera_coord_t_decode_cleanup(camera_coord_t *p)
{
    return __camera_coord_t_decode_array_cleanup(p, 1);
}

int __camera_coord_t_clone_array(const camera_coord_t *p, camera_coord_t *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __double_clone_array(&(p[element].x), &(q[element].x), 1);

        __double_clone_array(&(p[element].y), &(q[element].y), 1);

        __double_clone_array(&(p[element].z), &(q[element].z), 1);

        __double_clone_array(&(p[element].v_x), &(q[element].v_x), 1);

        __double_clone_array(&(p[element].v_y), &(q[element].v_y), 1);

        __double_clone_array(&(p[element].v_z), &(q[element].v_z), 1);

        __int64_t_clone_array(&(p[element].mode), &(q[element].mode), 1);

        __int64_t_clone_array(&(p[element].utime), &(q[element].utime), 1);

    }
    return 0;
}

camera_coord_t *camera_coord_t_copy(const camera_coord_t *p)
{
    camera_coord_t *q = (camera_coord_t*) malloc(sizeof(camera_coord_t));
    __camera_coord_t_clone_array(p, q, 1);
    return q;
}

void camera_coord_t_destroy(camera_coord_t *p)
{
    __camera_coord_t_decode_array_cleanup(p, 1);
    free(p);
}

int camera_coord_t_publish(lcm_t *lc, const char *channel, const camera_coord_t *p)
{
      int max_data_size = camera_coord_t_encoded_size (p);
      uint8_t *buf = (uint8_t*) malloc (max_data_size);
      if (!buf) return -1;
      int data_size = camera_coord_t_encode (buf, 0, max_data_size, p);
      if (data_size < 0) {
          free (buf);
          return data_size;
      }
      int status = lcm_publish (lc, channel, buf, data_size);
      free (buf);
      return status;
}

struct _camera_coord_t_subscription_t {
    camera_coord_t_handler_t user_handler;
    void *userdata;
    lcm_subscription_t *lc_h;
};
static
void camera_coord_t_handler_stub (const lcm_recv_buf_t *rbuf,
                            const char *channel, void *userdata)
{
    int status;
    camera_coord_t p;
    memset(&p, 0, sizeof(camera_coord_t));
    status = camera_coord_t_decode (rbuf->data, 0, rbuf->data_size, &p);
    if (status < 0) {
        fprintf (stderr, "error %d decoding camera_coord_t!!!\n", status);
        return;
    }

    camera_coord_t_subscription_t *h = (camera_coord_t_subscription_t*) userdata;
    h->user_handler (rbuf, channel, &p, h->userdata);

    camera_coord_t_decode_cleanup (&p);
}

camera_coord_t_subscription_t* camera_coord_t_subscribe (lcm_t *lcm,
                    const char *channel,
                    camera_coord_t_handler_t f, void *userdata)
{
    camera_coord_t_subscription_t *n = (camera_coord_t_subscription_t*)
                       malloc(sizeof(camera_coord_t_subscription_t));
    n->user_handler = f;
    n->userdata = userdata;
    n->lc_h = lcm_subscribe (lcm, channel,
                                 camera_coord_t_handler_stub, n);
    if (n->lc_h == NULL) {
        fprintf (stderr,"couldn't reg camera_coord_t LCM handler!\n");
        free (n);
        return NULL;
    }
    return n;
}

int camera_coord_t_subscription_set_queue_capacity (camera_coord_t_subscription_t* subs,
                              int num_messages)
{
    return lcm_subscription_set_queue_capacity (subs->lc_h, num_messages);
}

int camera_coord_t_unsubscribe(lcm_t *lcm, camera_coord_t_subscription_t* hid)
{
    int status = lcm_unsubscribe (lcm, hid->lc_h);
    if (0 != status) {
        fprintf(stderr,
           "couldn't unsubscribe camera_coord_t_handler %p!\n", hid);
        return -1;
    }
    free (hid);
    return 0;
}

