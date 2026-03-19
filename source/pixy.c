/* pixy.c - Pixy2 I2C (LPI2C + eDMA) mini-driver for MCUXpresso/NXP SDK */

#include "pixy.h"
#include <string.h>
#include <stdbool.h>

/* Pixy2 protocol sync words (little-endian when read as uint16_t):
 * bytes: AE C1 -> 0xC1AE (no checksum)
 * bytes: AF C1 -> 0xC1AF (checksum present)
 */
#define PIXY_SYNC_NOCHK   (0xC1AEu)
#define PIXY_SYNC_CHK     (0xC1AFu)

/* Command types we use */
#define PIXY_CMD_GET_VERSION  (0x0Eu)
#define PIXY_CMD_SET_LED      (0x14u)  /* 20 */
#define PIXY_CMD_SET_LAMP     (0x16u)  /* 22 */
#define PIXY_CMD_GET_FEATURES (0x30u)  /* 48 - line tracking "get main features" */

/* Feature type for vectors (line tracking) */
#define PIXY_FEAT_VECTORS     (0x01u)

/* Internal transfer completion flags */
static volatile bool     s_transferDone;
static volatile status_t s_transferStatus;

static void pixy_edma_cb(LPI2C_Type *base,
                         lpi2c_master_edma_handle_t *handle,
                         status_t status,
                         void *userData)
{
    (void)base; (void)handle; (void)userData;
    s_transferStatus = status;
    s_transferDone = true; /* IMPORTANT: also true on error */
}

/* Busy-wait with cycle countdown (simple + deterministic) */
static status_t pixy_wait_done(uint32_t timeoutCycles)
{
    while (!s_transferDone)
    {
        if (timeoutCycles-- == 0)
            return kStatus_Timeout;
    }
    return s_transferStatus;
}

void pixy_init(pixy_t *cam,
               LPI2C_Type *inst,
               uint8_t addr7,
               edma_handle_t *rxHandle,
               edma_handle_t *txHandle)
{
    cam->instance = inst;
    cam->address  = addr7;

    LPI2C_MasterCreateEDMAHandle(cam->instance,
                                &cam->edmaHandle,
                                rxHandle,
                                txHandle,
                                pixy_edma_cb,
                                NULL);
}

/* Low-level EDMA send/recv (kept similar to your style) */
static status_t pixy_send(pixy_t *cam, const uint8_t *cmd, size_t len)
{
    s_transferDone = false;
    s_transferStatus = kStatus_Fail;

    lpi2c_master_transfer_t xfer = {
        .slaveAddress   = cam->address,
        .direction      = kLPI2C_Write,
        .subaddress     = 0,
        .subaddressSize = 0,
        .data           = (uint8_t *)cmd,
        .dataSize       = len,
        .flags          = kLPI2C_TransferDefaultFlag,
    };

    status_t s = LPI2C_MasterTransferEDMA(cam->instance, &cam->edmaHandle, &xfer);
    if (s != kStatus_Success) return s;

    return pixy_wait_done(2000000u);
}

static status_t pixy_recv(pixy_t *cam, uint8_t *buf, size_t len)
{
    s_transferDone = false;
    s_transferStatus = kStatus_Fail;

    lpi2c_master_transfer_t xfer = {
        .slaveAddress   = cam->address,
        .direction      = kLPI2C_Read,
        .subaddress     = 0,
        .subaddressSize = 0,
        .data           = buf,
        .dataSize       = len,
        .flags          = kLPI2C_TransferDefaultFlag,
    };

    status_t s = LPI2C_MasterTransferEDMA(cam->instance, &cam->edmaHandle, &xfer);
    if (s != kStatus_Success) return s;

    return pixy_wait_done(2000000u);
}

/* Send Pixy request: [AE C1] [type] [len] [payload...] */
static status_t pixy_send_req(pixy_t *cam, uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t buf[4 + 32];
    if (len > 32) return kStatus_OutOfRange;

    buf[0] = 0xAE;
    buf[1] = 0xC1;
    buf[2] = type;
    buf[3] = len;

    if (len && payload) memcpy(&buf[4], payload, len);

    return pixy_send(cam, buf, 4u + (size_t)len);
}

/* Read one Pixy response packet:
 * - reads 4-byte header (sync,type,len)
 * - if sync indicates checksum, reads 2-byte checksum (ignored here)
 * - reads payload (len bytes)
 */
static status_t pixy_read_packet(pixy_t *cam, uint8_t *outType, uint8_t *payload, uint16_t payloadMax, uint16_t *outLen)
{
    status_t s;
    uint8_t h4[4];

    s = pixy_recv(cam, h4, 4);
    if (s != kStatus_Success) return s;

    uint16_t sync = (uint16_t)h4[0] | ((uint16_t)h4[1] << 8);
    uint8_t  type = h4[2];
    uint16_t len  = h4[3];

    if (sync == PIXY_SYNC_CHK)
    {
        /* checksum (2 bytes) */
        uint8_t chk[2];
        s = pixy_recv(cam, chk, 2);
        if (s != kStatus_Success) return s;
    }
    else if (sync != PIXY_SYNC_NOCHK)
    {
        return kStatus_Fail;
    }

    if (outType) *outType = type;

    if (len > payloadMax)
    {
        /* Drain remaining bytes so stream stays aligned */
        uint16_t remaining = len;
        while (remaining)
        {
            uint8_t tmp[32];
            uint16_t chunk = (remaining > (uint16_t)sizeof(tmp)) ? (uint16_t)sizeof(tmp) : remaining;
            s = pixy_recv(cam, tmp, chunk);
            if (s != kStatus_Success) return s;
            remaining -= chunk;
        }
        return kStatus_OutOfRange;
    }

    if (len && payload)
    {
        s = pixy_recv(cam, payload, len);
        if (s != kStatus_Success) return s;
    }
    else if (len && !payload)
    {
        /* Drain if caller doesn't provide buffer */
        uint16_t remaining = len;
        while (remaining)
        {
            uint8_t tmp[32];
            uint16_t chunk = (remaining > (uint16_t)sizeof(tmp)) ? (uint16_t)sizeof(tmp) : remaining;
            s = pixy_recv(cam, tmp, chunk);
            if (s != kStatus_Success) return s;
            remaining -= chunk;
        }
    }

    if (outLen) *outLen = len;
    return kStatus_Success;
}

/* ---------------- Public API ---------------- */

status_t pixy_set_led(pixy_t *cam, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t payload[3] = { r, g, b };
    status_t s = pixy_send_req(cam, PIXY_CMD_SET_LED, payload, sizeof(payload));
    if (s != kStatus_Success) return s;

    /* Response exists; we can just read & ignore packet */
    uint8_t t;
    uint16_t len;
    uint8_t buf[16];
    return pixy_read_packet(cam, &t, buf, sizeof(buf), &len);
}

status_t pixy_set_lamp(pixy_t *cam, uint8_t on, uint8_t upper)
{
    uint8_t payload[2] = { on, upper };
    status_t s = pixy_send_req(cam, PIXY_CMD_SET_LAMP, payload, sizeof(payload));
    if (s != kStatus_Success) return s;

    uint8_t t;
    uint16_t len;
    uint8_t buf[16];
    return pixy_read_packet(cam, &t, buf, sizeof(buf), &len);
}

status_t pixy_get_version(pixy_t *cam, pixy_version_t *out)
{
    if (!out) return kStatus_InvalidArgument;
    memset(out, 0, sizeof(*out));

    status_t s = pixy_send_req(cam, PIXY_CMD_GET_VERSION, NULL, 0);
    if (s != kStatus_Success) return s;

    uint8_t type;
    uint16_t len;
    uint8_t payload[64];

    s = pixy_read_packet(cam, &type, payload, sizeof(payload), &len);
    if (s != kStatus_Success) return s;

    /* Payload format (Pixy2): major(1), minor(1), build(2), hw ver str (10), fw ver str (10)
       Some firmwares may vary; we parse best-effort. */
    if (len < 4) return kStatus_Fail;

    out->major = payload[0];
    out->minor = payload[1];
    out->build = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);

    if (len >= 4 + 10)
    {
        memcpy(out->hardware, &payload[4], 10);
        out->hardware[10] = '\0';
    }
    if (len >= 4 + 10 + 10)
    {
        memcpy(out->firmware, &payload[14], 10);
        out->firmware[10] = '\0';
    }

    return kStatus_Success;
}

status_t pixy_get_vectors(pixy_t *cam,
                          pixy_vector_t *out,
                          size_t max_vectors,
                          size_t *num_vectors)
{
    if (!out || !num_vectors) return kStatus_InvalidArgument;
    *num_vectors = 0;

    /* GET_MAIN_FEATURES (line tracking):
       payload: [features, wait]
       features bit0 = vectors
       wait=1 -> block until new data available (nice for loop)
     */
    uint8_t payloadReq[2] = { 0x01u, 0x01u };

    status_t s = pixy_send_req(cam, PIXY_CMD_GET_FEATURES, payloadReq, sizeof(payloadReq));
    if (s != kStatus_Success) return s;

    uint8_t type;
    uint16_t len;
    uint8_t payload[128];

    s = pixy_read_packet(cam, &type, payload, sizeof(payload), &len);
    if (s != kStatus_Success) return s;

    /* Payload is a list of features:
       [featType][featLen][featData...], repeated
       Vectors featType = 0x01
       Each vector = 6 bytes: x0,y0,x1,y1,index,flags
     */
    size_t count = 0;
    uint16_t i = 0;

    while (i + 2u <= len)
    {
        uint8_t featType = payload[i + 0u];
        uint8_t featLen  = payload[i + 1u];
        i += 2u;

        if (i + (uint16_t)featLen > len) break;

        if (featType == PIXY_FEAT_VECTORS)
        {
            size_t n = (size_t)featLen / 6u;
            for (size_t k = 0; k < n && count < max_vectors; k++)
            {
                uint16_t off = i + (uint16_t)(k * 6u);
                out[count].x0    = payload[off + 0u];
                out[count].y0    = payload[off + 1u];
                out[count].x1    = payload[off + 2u];
                out[count].y1    = payload[off + 3u];
                out[count].index = payload[off + 4u];
                out[count].flags = payload[off + 5u];
                count++;
            }
        }

        i += featLen;
    }

    *num_vectors = count;
    return kStatus_Success;
}

status_t pixy_get_vectors_u16(pixy_t *cam,
                              uint16_t *vectors,
                              size_t max_vectors,
                              size_t *num_vectors)
{
    if (!vectors || !num_vectors) return kStatus_InvalidArgument;

    pixy_vector_t tmp[16];
    size_t nmax = (max_vectors > 16u) ? 16u : max_vectors;

    status_t s = pixy_get_vectors(cam, tmp, nmax, num_vectors);
    if (s != kStatus_Success) return s;

    for (size_t i = 0; i < *num_vectors; i++)
    {
        vectors[4u*i + 0u] = tmp[i].x0;
        vectors[4u*i + 1u] = tmp[i].y0;
        vectors[4u*i + 2u] = tmp[i].x1;
        vectors[4u*i + 3u] = tmp[i].y1;
    }

    return kStatus_Success;
}
