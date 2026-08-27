// link_comms.cpp — 数传链路 UART 收发 (控制指令 + 离散指令 + JPEG)
#include "link_comms.h"

static uint32_t g_telem_cnt   = 0;
static uint32_t g_cmd_cnt     = 0;
static uint32_t g_last_cmd_us = 0;
static uint32_t g_timeout_cnt = 0;
static uint32_t g_crc_errs    = 0;
static link_cmd_t g_last_cmd  = {0};
static uint16_t   g_jpeg_seq  = 0;

// CRC8 (Dallas/Maxim)
static uint8_t crc8(const uint8_t* data, int len) {
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        uint8_t b = data[i];
        for (int j = 0; j < 8; j++) {
            if ((crc ^ b) & 1) crc = (crc >> 1) ^ 0x8C;
            else               crc >>= 1;
            b >>= 1;
        }
    }
    return crc;
}

void link_begin() {
    // 显式指定 RX/TX 引脚 (避开摄像头/SD, 见 config.h 引脚表)
    LINK_SERIAL.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
    Serial.printf("[LINK] UART%d @ %d baud (RX=%d TX=%d) 就绪\n",
                  LINK_UART_NUM, LINK_BAUD, LINK_RX_PIN, LINK_TX_PIN);
}

// ========================== 遥测发送 ===========================================
void link_send_telem(const link_telem_t* pkt) {
    uint8_t buf[sizeof(link_telem_t)];
    memcpy(buf, pkt, sizeof(link_telem_t));
    buf[0] = TELEM_SYNC;
    buf[sizeof(link_telem_t) - 1] = crc8(buf, sizeof(link_telem_t) - 1);
    LINK_SERIAL.write(buf, sizeof(link_telem_t));
    g_telem_cnt++;
}

// ========================== 控制指令接收 =======================================
// 注意: 清扫循环必须保留其他协议的同步字节 (0xEF/0xCD/0xFC),
//       否则会吞掉离散指令/遥测帧导致丢包 (原实现 bug)。
bool link_recv_cmd(link_cmd_t* cmd) {
    while (LINK_SERIAL.available() > 0) {
        uint8_t b = LINK_SERIAL.peek();
        if (b == CMD_SYNC || b == DISC_SYNC || b == TELEM_SYNC || b == JPEG_SYNC) break;
        LINK_SERIAL.read();
    }
    if (LINK_SERIAL.available() < (int)sizeof(link_cmd_t)) return false;

    uint8_t buf[sizeof(link_cmd_t)];
    LINK_SERIAL.readBytes(buf, sizeof(link_cmd_t));

    if (buf[0] != CMD_SYNC) return false;
    if (crc8(buf, sizeof(link_cmd_t) - 1) != buf[sizeof(link_cmd_t) - 1]) {
        g_crc_errs++;
        return false;
    }

    memcpy(cmd, buf, sizeof(link_cmd_t));
    g_last_cmd = *cmd;
    g_last_cmd_us = micros();
    g_cmd_cnt++;
    return true;
}

// ========================== 离散指令接收 (点火/阀门/降落伞) =====================
bool link_recv_disc_cmd(link_disc_cmd_t* cmd) {
    // 只处理 DISC_SYNC 开头的 3 字节包; 其他协议字节留给对应解析器
    if (LINK_SERIAL.available() < 3) return false;
    if (LINK_SERIAL.peek() != DISC_SYNC) return false;

    uint8_t buf[3];
    LINK_SERIAL.readBytes(buf, 3);

    if (buf[0] != DISC_SYNC) return false;
    if (crc8(buf, 2) != buf[2]) {
        g_crc_errs++;
        return false;
    }

    cmd->sync = buf[0];
    cmd->cmd  = buf[1];
    cmd->crc8 = buf[2];
    Serial.printf("[LINK] 离散指令: 0x%02X\n", cmd->cmd);
    return true;
}

bool link_cmd_timeout() {
    if (g_last_cmd_us == 0) return false;
    if (micros() - g_last_cmd_us > CMD_TIMEOUT_US) {
        g_timeout_cnt++;
        return true;
    }
    return false;
}

// ========================== JPEG 帧发送 (分块) =================================
// 性能优化: 不再逐块 flush() 阻塞等待 (57600 波特下 220B ≈ 38ms/块,
// 一帧 15~37 块会冻结 50Hz 控制循环 0.6~1.4s)。
// 改为依赖 UART 硬件/软件发送缓冲自然节流, 整帧完成后只 flush 一次,
// 由调用方控制发送频率 (rocket_flight.ino 中 0.5fps)。
void link_send_jpeg(const uint8_t* jpeg_buf, uint16_t jpeg_len) {
    if (jpeg_len == 0) return;

    uint8_t total_chunks = (jpeg_len + JPEG_MAX_CHUNK - 1) / JPEG_MAX_CHUNK;
    uint16_t seq = g_jpeg_seq++;

    for (uint8_t i = 0; i < total_chunks; i++) {
        uint16_t offset = i * JPEG_MAX_CHUNK;
        uint16_t chunk_len = (i == total_chunks - 1)
                           ? (jpeg_len - offset) : JPEG_MAX_CHUNK;

        // 构建数据包: sync + header(7B) + data(chunk_len) + crc8(1B)
        uint8_t pkt[7 + JPEG_MAX_CHUNK + 1];
        pkt[0] = JPEG_SYNC;
        pkt[1] = seq & 0xFF;
        pkt[2] = (seq >> 8) & 0xFF;
        pkt[3] = i;
        pkt[4] = total_chunks;
        pkt[5] = jpeg_len & 0xFF;
        pkt[6] = (jpeg_len >> 8) & 0xFF;
        memcpy(pkt + 7, jpeg_buf + offset, chunk_len);
        pkt[7 + chunk_len] = crc8(pkt, 7 + chunk_len);

        LINK_SERIAL.write(pkt, 7 + chunk_len + 1);
        // 块间让出 CPU (保留控制帧时序), 不再逐块 flush
    }
    LINK_SERIAL.flush();  // 整帧完成后确保发送完毕
}

void link_status_print() {
    Serial.printf("[LINK] TX=%lu RX=%lu CRCerr=%lu TO=%lu\n",
                  g_telem_cnt, g_cmd_cnt, g_crc_errs, g_timeout_cnt);
}
