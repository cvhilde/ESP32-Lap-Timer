#include <ble.h>
#include <display.h>
#include <cstring>

BLEServer *pServer = nullptr;
BLECharacteristic *pTxChar = nullptr;

static const char b64tbl[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
static String rxBuf;

constexpr size_t kCmdArgSize = 256;
constexpr uint16_t kDesiredBleMtu = 247;
constexpr uint16_t kDefaultPeerMtu = 23;
constexpr size_t kLegacyRawChunkSize = 180;
constexpr uint8_t kBinaryFrameMarker = 0xA5;
constexpr uint8_t kBinaryFrameTypeData = 0x01;
constexpr uint8_t kBinaryFrameTypePutData = 0x02;
constexpr size_t kBinaryFrameHeaderSize = 8;
constexpr uint32_t kLegacyTxWindow = 4;
constexpr uint32_t kFastTxWindow = 8;
constexpr uint32_t kFastPutAckInterval = 4;

bool bleConnected = false;
bool bleAdvertising = false;

int totalFiles = 0;
int currentFileNumber = 0;
bool currentlySending = false;
static uint16_t bleConnId = 0;
static uint16_t blePeerMtu = kDefaultPeerMtu;

QueueHandle_t cmdQ;
enum class TxState {
    IDLE,
    SENDING,
    PUT_RX
};

enum class TransferMode {
    LEGACY_BASE64,
    FAST_BINARY
};

struct Cmd {
    enum Type {
        LIST,
        GET,
        GET_FAST,
        ACK,
        RESEND,
        PURGE,
        PUT_BEGIN,
        PUT_BEGIN_FAST,
        PUT_DATA,
        PUT_END
    } type;
    char arg[kCmdArgSize] = {0};
    uint32_t n = 0;
    uint16_t len = 0;
};

static std::vector<uint8_t> putBuf;
static uint32_t putSeq = 0;
static uint32_t putExpectedSize = 0;
static uint32_t putExpectedCrc = 0;
static TransferMode putMode = TransferMode::LEGACY_BASE64;
static uint16_t putChunkSize = kLegacyRawChunkSize;

static void resetPutState() {
    putBuf.clear();
    putSeq = 0;
    putExpectedSize = 0;
    putExpectedCrc = 0;
    putMode = TransferMode::LEGACY_BASE64;
    putChunkSize = kLegacyRawChunkSize;
}

static void setCmdArg(Cmd& cmd, const String& value) {
    value.toCharArray(cmd.arg, sizeof(cmd.arg));
    cmd.len = strlen(cmd.arg);
}

static void setCmdPayload(Cmd& cmd, const uint8_t* data, size_t len) {
    if (len > sizeof(cmd.arg)) {
        len = sizeof(cmd.arg);
    }

    memcpy(cmd.arg, data, len);
    cmd.len = len;
}

static void writeLE16(uint8_t* dst, uint16_t value) {
    dst[0] = value & 0xFF;
    dst[1] = (value >> 8) & 0xFF;
}

static void writeLE32(uint8_t* dst, uint32_t value) {
    dst[0] = value & 0xFF;
    dst[1] = (value >> 8) & 0xFF;
    dst[2] = (value >> 16) & 0xFF;
    dst[3] = (value >> 24) & 0xFF;
}

static uint16_t readLE16(const uint8_t* src) {
    return static_cast<uint16_t>(src[0]) |
           (static_cast<uint16_t>(src[1]) << 8);
}

static uint32_t readLE32(const uint8_t* src) {
    return static_cast<uint32_t>(src[0]) |
           (static_cast<uint32_t>(src[1]) << 8) |
           (static_cast<uint32_t>(src[2]) << 16) |
           (static_cast<uint32_t>(src[3]) << 24);
}

static uint16_t getBinaryPayloadCapacity(uint16_t mtu = blePeerMtu) {
    size_t attPayload = mtu > 3 ? mtu - 3 : 20;
    if (attPayload <= kBinaryFrameHeaderSize) {
        return 1;
    }

    size_t payload = attPayload - kBinaryFrameHeaderSize;
    if (payload > kCmdArgSize) {
        payload = kCmdArgSize;
    }

    return static_cast<uint16_t>(payload);
}

static uint32_t getChunkCount(uint32_t size, uint16_t chunkSize) {
    if (chunkSize == 0) {
        return 0;
    }

    return (size + chunkSize - 1) / chunkSize;
}

static bool decodeBinaryFrame(
    const uint8_t* data,
    size_t len,
    uint8_t& type,
    uint32_t& seq,
    const uint8_t*& payload,
    uint16_t& payloadLen
) {
    if (len < kBinaryFrameHeaderSize || data[0] != kBinaryFrameMarker) {
        return false;
    }

    type = data[1];
    seq = readLE32(data + 2);
    payloadLen = readLE16(data + 6);
    if (payloadLen > kCmdArgSize || (kBinaryFrameHeaderSize + payloadLen) != len) {
        return false;
    }

    payload = data + kBinaryFrameHeaderSize;
    return true;
}

static struct {
    File file;
    String fname;
    uint32_t size = 0;
    uint32_t crc = 0;
    uint32_t nextSeq = 0;
    uint32_t windowBase = 0;
    uint32_t totalChunks = 0;
    uint32_t windowSize = 1;
    uint16_t chunkSize = kLegacyRawChunkSize;
    TransferMode mode = TransferMode::LEGACY_BASE64;
    TxState st = TxState::IDLE;
} tx;

static void resetTxState() {
    if (tx.file) {
        tx.file.close();
    }

    tx.fname = "";
    tx.size = 0;
    tx.crc = 0;
    tx.nextSeq = 0;
    tx.windowBase = 0;
    tx.totalChunks = 0;
    tx.windowSize = 1;
    tx.chunkSize = kLegacyRawChunkSize;
    tx.mode = TransferMode::LEGACY_BASE64;
    tx.st = TxState::IDLE;
    currentlySending = false;
}

static void queueCommand(const Cmd& cmd) {
    xQueueSend(cmdQ, &cmd, 0);
}

static void queueTextCommand(const String& line) {
    Cmd cmd = {};
    if (line == "LIST") {
        cmd.type = Cmd::LIST;
    } else if (line.startsWith("GET_FAST,")) {
        cmd.type = Cmd::GET_FAST;
        setCmdArg(cmd, line.substring(9));
    } else if (line.startsWith("GET,")) {
        cmd.type = Cmd::GET;
        setCmdArg(cmd, line.substring(4));
    } else if (line.startsWith("ACK,")) {
        cmd.type = Cmd::ACK;
        cmd.n = line.substring(4).toInt();
    } else if (line.startsWith("RESEND,")) {
        cmd.type = Cmd::RESEND;
        cmd.n = line.substring(7).toInt();
    } else if (line == "PURGE") {
        cmd.type = Cmd::PURGE;
    } else if (line.startsWith("PUT_BEGIN_FAST,")) {
        cmd.type = Cmd::PUT_BEGIN_FAST;
        setCmdArg(cmd, line.substring(15));
    } else if (line.startsWith("PUT_BEGIN,")) {
        cmd.type = Cmd::PUT_BEGIN;
        setCmdArg(cmd, line.substring(10));
    } else if (line.startsWith("PUT_DATA,")) {
        int comma = line.indexOf(',', 9);
        if (comma < 0) {
            return;
        }
        cmd.type = Cmd::PUT_DATA;
        cmd.n = line.substring(9, comma).toInt();
        setCmdArg(cmd, line.substring(comma + 1));
    } else if (line == "PUT_END") {
        cmd.type = Cmd::PUT_END;
    } else {
        return;
    }

    queueCommand(cmd);
}

class MyServerCB : public BLEServerCallbacks {
    void onConnect(BLEServer* server) override {
        bleConnected = true;
        bleAdvertising = false;
        blePeerMtu = BLEDevice::getMTU();
        Serial.println("BLE connected");
    }

    void onConnect(BLEServer* server, esp_ble_gatts_cb_param_t* param) override {
        onConnect(server);
        if (param != nullptr) {
            bleConnId = param->connect.conn_id;
            blePeerMtu = server->getPeerMTU(bleConnId);
            Serial.printf("BLE MTU negotiated: %u\n", blePeerMtu);
        }
    }

    void onDisconnect(BLEServer*) override {
        bleConnected = false;
        bleConnId = 0;
        blePeerMtu = kDefaultPeerMtu;
        resetTxState();
        resetPutState();
        Serial.println("BLE disconnected");
    }

    void onMtuChanged(BLEServer*, esp_ble_gatts_cb_param_t* param) override {
        if (param != nullptr) {
            blePeerMtu = param->mtu.mtu;
            Serial.printf("BLE MTU updated: %u\n", blePeerMtu);
        }
    }
};

class MyRxCB : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *c, esp_ble_gatts_cb_param_t* param) override {
        const uint8_t* data = param != nullptr
            ? param->write.value
            : reinterpret_cast<const uint8_t*>(c->getData());
        size_t len = param != nullptr ? param->write.len : c->getLength();
        if (len == 0 || data == nullptr) {
            return;
        }

        if (data[0] == kBinaryFrameMarker) {
            uint8_t type = 0;
            uint32_t seq = 0;
            const uint8_t* payload = nullptr;
            uint16_t payloadLen = 0;
            if (!decodeBinaryFrame(data, len, type, seq, payload, payloadLen)) {
                Serial.println("Ignoring malformed binary BLE frame");
                return;
            }

            if (type == kBinaryFrameTypePutData) {
                Cmd cmd = {};
                cmd.type = Cmd::PUT_DATA;
                cmd.n = seq;
                setCmdPayload(cmd, payload, payloadLen);
                queueCommand(cmd);
            }
            return;
        }

        for (size_t i = 0; i < len; i++) {
            rxBuf += static_cast<char>(data[i]);
        }

        while (true) {
            int nl = rxBuf.indexOf('\n');
            if (nl < 0) {
                break;
            }

            String line = rxBuf.substring(0, nl);
            rxBuf.remove(0, nl + 1);
            line.trim();
            if (!line.isEmpty()) {
                queueTextCommand(line);
            }
        }
    }
};

uint32_t crc32_file(File &f) {
    uint32_t crc = 0xFFFFFFFF;
    uint8_t buf[256];
    while (f.available()) {
        int read = f.read(buf, sizeof(buf));
        crc = esp_rom_crc32_le(crc, buf, read);
    }
    return crc ^ 0xFFFFFFFF;
}

int base64_decode(uint8_t* out, const char* in, size_t len) {
    static uint8_t lut[256];
    static bool init=false; if(!init){
        const char* p=b64tbl; for(int i=0;i<64;i++) lut[(uint8_t)p[i]]=i; init=true;
    }
    int i=0,o=0; while(i<len){
        uint32_t v = lut[(uint8_t)in[i++]]<<18 |
                     lut[(uint8_t)in[i++]]<<12 |
                     lut[(uint8_t)in[i++]]<< 6 |
                     lut[(uint8_t)in[i++]];
        out[o++] = (v>>16)&0xFF;
        if(in[i-2]!='=') out[o++] = (v>>8)&0xFF;
        if(in[i-1]!='=') out[o++] =  v     &0xFF;
    }
    return o;
}

String base64_encode(const uint8_t *in, size_t len) {
    String out; out.reserve((len + 2) / 3 * 4);
    for (size_t i = 0; i < len; i += 3) {
        uint32_t v = in[i] << 16 | (i + 1 < len ? in[i + 1] << 8 : 0) | (i + 2 < len ? in[i + 2] : 0);
        out += b64tbl[(v >> 18) & 0x3F];
        out += b64tbl[(v >> 12) & 0x3F];
        out += (i + 1 < len) ? b64tbl[(v >> 6) & 0x3F] : '=';
        out += (i + 2 < len) ? b64tbl[ v & 0x3F] : '=';
    }
    return out;
}

static void appendBase64Chunk(std::vector<uint8_t>& buf, const String& b64) {
    int rawLen = (b64.length() * 3) / 4;
    size_t idx = buf.size();
    buf.resize(idx + rawLen);
    int actual = base64_decode(&buf[idx], b64.c_str(), b64.length());
    buf.resize(idx + actual);
}

static void appendRawChunk(std::vector<uint8_t>& buf, const uint8_t* data, size_t len) {
    size_t idx = buf.size();
    buf.resize(idx + len);
    memcpy(&buf[idx], data, len);
}

void txLine(const String &s) {
    pTxChar->setValue((uint8_t*)s.c_str(), s.length());
    pTxChar->notify();
}

static void txBinaryFrame(uint8_t type, uint32_t seq, const uint8_t* payload, uint16_t payloadLen) {
    std::vector<uint8_t> frame(kBinaryFrameHeaderSize + payloadLen);
    frame[0] = kBinaryFrameMarker;
    frame[1] = type;
    writeLE32(frame.data() + 2, seq);
    writeLE16(frame.data() + 6, payloadLen);

    if (payloadLen > 0) {
        memcpy(frame.data() + kBinaryFrameHeaderSize, payload, payloadLen);
    }

    pTxChar->setValue(frame.data(), frame.size());
    pTxChar->notify();
}

static void handleList() {
    totalFiles = 0;

    File manifest = SPIFFS.open("/sessions.txt", "r");
    if (!manifest) {
        txLine("LIST,0\nEND\n");
        return;
    }

    txLine("LIST, 0\n");
    while (manifest.available()) {
        String ts = manifest.readStringUntil('\n');
        ts.trim();

        if (ts.isEmpty()) {
            continue;
        }

        if (SPIFFS.exists("/log_" + ts + ".csv")) {
            txLine("/log_" + ts + ".csv\n");
            vTaskDelay(1);
            txLine("/timestamps_" + ts + ".csv\n");
            vTaskDelay(1);
            totalFiles += 2;
            if (SPIFFS.exists("/summary_" + ts + ".csv")) {
                txLine("/summary_" + ts + ".csv\n");
                vTaskDelay(1);
                totalFiles += 1;
            }
        }

        else if (SPIFFS.exists("/route_" + ts + ".csv")) {
            txLine("/route_" + ts + ".csv\n");
            vTaskDelay(1);
            totalFiles += 1;
            if (SPIFFS.exists("/summary_" + ts + ".csv")) {
                txLine("/summary_" + ts + ".csv\n");
                vTaskDelay(1);
                totalFiles += 1;
            }
        }
    }

    manifest.close();
    txLine("END\n");
}

static void finishTransfer() {
    txLine("DONE," + tx.fname + "\n");
    resetTxState();
}

static uint32_t sendChunk(uint32_t seq) {
    if (tx.st != TxState::SENDING || seq >= tx.totalChunks) {
        return 0;
    }

    const uint32_t offset = seq * tx.chunkSize;
    if (!tx.file.seek(offset)) {
        Serial.printf("BLE seek failed for chunk %lu\n", static_cast<unsigned long>(seq));
        return 0;
    }

    uint8_t buf[kCmdArgSize];
    int read = tx.file.read(buf, tx.chunkSize);
    if (read <= 0) {
        return 0;
    }

    if (tx.mode == TransferMode::FAST_BINARY) {
        txBinaryFrame(kBinaryFrameTypeData, seq, buf, read);
    } else {
        String b64 = base64_encode(buf, read);
        txLine("DATA," + String(seq) + "," + b64 + "\n");
    }

    return read;
}

static void sendAvailableChunks() {
    while (tx.nextSeq < tx.totalChunks && (tx.nextSeq - tx.windowBase) < tx.windowSize) {
        if (sendChunk(tx.nextSeq) == 0) {
            break;
        }
        tx.nextSeq++;
    }
}

static void handleGet(const String &raw, TransferMode mode) {
    String fname = raw;
    fname.trim();
    fname.replace("\r", "");
    fname.replace("\n", "");
    if (!fname.startsWith("/")) {
        fname = "/" + fname;
    }

    if (tx.st != TxState::IDLE) {
        return;
    }

    tx.file = SPIFFS.open(fname, "r");
    if (!tx.file) {
        txLine("ERR,NOFILE\n");
        return;
    }

    currentFileNumber++;
    currentlySending = true;

    tx.fname = fname;
    tx.size = tx.file.size();
    tx.crc = crc32_file(tx.file);
    tx.mode = mode;
    tx.chunkSize = mode == TransferMode::FAST_BINARY ? getBinaryPayloadCapacity() : kLegacyRawChunkSize;
    tx.windowSize = mode == TransferMode::FAST_BINARY ? kFastTxWindow : kLegacyTxWindow;
    tx.totalChunks = getChunkCount(tx.size, tx.chunkSize);
    tx.windowBase = 0;
    tx.nextSeq = 0;
    tx.st = TxState::SENDING;

    if (mode == TransferMode::FAST_BINARY) {
        txLine(
            "SIZE_FAST," + String(tx.size) + "," + String(tx.crc, HEX) + "," +
            String(tx.chunkSize) + "," + String(tx.windowSize) + "\n"
        );
    } else {
        txLine("SIZE," + String(tx.size) + "," + String(tx.crc, HEX) + "\n");
    }

    if (tx.totalChunks == 0) {
        finishTransfer();
        return;
    }

    if (mode == TransferMode::FAST_BINARY) {
        sendAvailableChunks();
    } else {
        if (sendChunk(tx.nextSeq) > 0) {
            tx.nextSeq++;
        }
    }
}

static void handleAck(uint32_t seq) {
    if (tx.st != TxState::SENDING || seq >= tx.totalChunks) {
        return;
    }

    uint32_t ackBase = seq + 1;
    if (ackBase <= tx.windowBase) {
        return;
    }

    tx.windowBase = ackBase;
    if (tx.windowBase >= tx.totalChunks) {
        finishTransfer();
        return;
    }

    sendAvailableChunks();
}

static void handleResend(uint32_t seq) {
    if (tx.st != TxState::SENDING || seq >= tx.totalChunks) {
        return;
    }
    sendChunk(seq);
}

static void handlePurge() {
    std::vector<uint8_t> backup;
    if (SPIFFS.exists(waypointsFile)) {
        File file = SPIFFS.open(waypointsFile, "r");
        backup.resize(file.size());
        file.readBytes((char*)backup.data(), backup.size());
        file.close();
    }

    displayPurgingMessage();
    startBlink(250);
    if (SPIFFS.format()) {
        Serial.println("SPIFFS partition erased");
        if (!backup.empty()) {
            File file = SPIFFS.open(waypointsFile, "w");
            file.write(backup.data(), backup.size());
            file.close();
            loadWaypoints();
            Serial.println("Waypoints restored after purge");
        }

        totalFiles = 0;
        currentFileNumber = 0;
        resetTxState();
        txLine("PURGED\n");
    } else {
        Serial.println("SPIFFS erase failed");
    }

    clearPurgingMessage();
    stopBlink();
}

static void handlePutBegin(const String& meta, TransferMode mode) {
    int comma = meta.indexOf(',');
    if (comma < 0) {
        txLine("ERR,BAD_BEGIN\n");
        return;
    }

    uint32_t size = strtoul(meta.substring(0, comma).c_str(), nullptr, 10);
    uint32_t crc = strtoul(meta.substring(comma + 1).c_str(), nullptr, 16);

    resetPutState();
    putExpectedSize = size;
    putExpectedCrc = crc;
    putMode = mode;
    putChunkSize = mode == TransferMode::FAST_BINARY ? getBinaryPayloadCapacity() : kLegacyRawChunkSize;
    tx.st = TxState::PUT_RX;
    if (mode == TransferMode::FAST_BINARY) {
        txLine("READY_FAST," + String(putChunkSize) + "," + String(kFastPutAckInterval) + "\n");
    } else {
        txLine("READY\n");
    }
    displayGettingMessage();
}

static void handlePutData(const Cmd& cmd) {
    if (tx.st != TxState::PUT_RX) {
        txLine("ERR,NO_PUT\n");
        return;
    }

    if (cmd.n != putSeq) {
        txLine("RESEND," + String(putSeq) + "\n");
        return;
    }

    if (putMode == TransferMode::FAST_BINARY) {
        appendRawChunk(putBuf, reinterpret_cast<const uint8_t*>(cmd.arg), cmd.len);
    } else {
        appendBase64Chunk(putBuf, String(cmd.arg));
    }

    if (putBuf.size() > putExpectedSize) {
        resetPutState();
        tx.st = TxState::IDLE;
        txLine("ERR,SIZE\n");
        return;
    }

    putSeq++;
    if (putMode == TransferMode::LEGACY_BASE64) {
        txLine("ACK," + String(cmd.n) + "\n");
    } else if ((putSeq % kFastPutAckInterval) == 0 || putBuf.size() == putExpectedSize) {
        txLine("ACK," + String(putSeq - 1) + "\n");
    }
}

static void handlePutEnd() {
    if (tx.st != TxState::PUT_RX) {
        txLine("ERR,NO_PUT\n");
        return;
    }

    if (putBuf.size() != putExpectedSize) {
        resetPutState();
        tx.st = TxState::IDLE;
        txLine("ERR,SIZE\n");
        return;
    }

    uint32_t crcCalc = esp_rom_crc32_le(0xFFFFFFFF, putBuf.data(), putBuf.size()) ^ 0xFFFFFFFF;

    if (crcCalc == putExpectedCrc) {
        writeWaypointsFile(putBuf.data(), putBuf.size());
        resetPutState();
        tx.st = TxState::IDLE;
        txLine("STORED,/waypoints.json\n");
    } else {
        resetPutState();
        tx.st = TxState::IDLE;
        txLine("BADCRC\n");
    }

    clearGettingMessage();
}

static void bleWorker(void*) {
    for (Cmd cmd;;) {
        if (xQueueReceive(cmdQ, &cmd, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        switch(cmd.type) {
            case Cmd::LIST:handleList();
                break;
            case Cmd::GET:handleGet(cmd.arg, TransferMode::LEGACY_BASE64);
                break;
            case Cmd::GET_FAST:handleGet(cmd.arg, TransferMode::FAST_BINARY);
                break;
            case Cmd::ACK:handleAck(cmd.n);
                break;
            case Cmd::RESEND:handleResend(cmd.n);
                break;
            case Cmd::PURGE:handlePurge();
                break;
            case Cmd::PUT_BEGIN:handlePutBegin(cmd.arg, TransferMode::LEGACY_BASE64);
                break;
            case Cmd::PUT_BEGIN_FAST:handlePutBegin(cmd.arg, TransferMode::FAST_BINARY);
                break;
            case Cmd::PUT_DATA:handlePutData(cmd);
                break;
            case Cmd::PUT_END:handlePutEnd();
                break;
        }
    }
}

void initBLE() {
    BLEDevice::init("ESP32_LapTimer");
    BLEDevice::setMTU(kDesiredBleMtu);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCB());
    BLEService *svc = pServer->createService(SERVICE_UUID);

    pTxChar = svc->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
    pTxChar->addDescriptor(new BLE2902);

    BLECharacteristic *rx = svc->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    rx->setCallbacks(new MyRxCB());

    svc->start();

    cmdQ = xQueueCreate(8, sizeof(Cmd));
    xTaskCreatePinnedToCore(bleWorker, "BLE_WRK", 6 * 1024, nullptr, 1, nullptr, 0);
}

void startAdvertising() {
    if (!bleAdvertising && !bleConnected) {
        pServer->getAdvertising()->start();
        bleAdvertising = true;
    }
}

void stopAdvertising() {
    if (bleAdvertising) {
        pServer->getAdvertising()->stop();
        bleAdvertising = false;
    }
}

bool isConnected() {
    return bleConnected;
}

bool isAdvertising() {
    return bleAdvertising;
}

int getFileCount() {
    return totalFiles;
}

int getCurrentFileNumber() {
    return currentFileNumber;
}

bool isSending() {
    return currentlySending;
}
