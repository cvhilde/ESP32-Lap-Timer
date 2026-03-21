#include <ble.h>
#include <display.h>

BLEServer *pServer = nullptr;
BLECharacteristic *pTxChar = nullptr;

static const char b64tbl[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
static String rxBuf;

bool bleConnected = false;
bool bleAdvertising = false;

int totalFiles = 0;
int currentFileNumber = 0;
bool currentlySending = false;

QueueHandle_t cmdQ;
enum class TxState {
    IDLE,
    SENDING,
    PUT_RX
};

struct Cmd {
    enum Type {
        LIST, 
        GET, 
        ACK, 
        RESEND, 
        PURGE,
        PUTWPT,
        PUTDATA,
        PUTDONE
    } type;
    String arg;
    uint32_t n = 0;
};

static std::vector<uint8_t> putBuf;
static uint32_t putSeq = 0;

static struct {
    File file;
    String fname;
    uint32_t size = 0;
    uint32_t crc = 0;
    uint32_t seq = 0;
    TxState st = TxState::IDLE;
} tx;


class MyServerCB : public BLEServerCallbacks {
    void onConnect(BLEServer*) override {
        bleConnected = true;
        bleAdvertising = false;
        Serial.println("BLE connected");
    }

    void onDisconnect(BLEServer*) override {
        bleConnected = false;
        tx.st = TxState::IDLE;
        Serial.println("BLE disconnected");
    }
};

class MyRxCB : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *c) override {
        std::string v = c->getValue();
        rxBuf += v.c_str();

        while(true) {
            int nl = rxBuf.indexOf('\n');
            if (nl < 0) {
                break;
            }

            String line = rxBuf.substring(0, nl);
            rxBuf.remove(0, nl + 1);
            line.trim();

            Cmd cmd;
            if (line == "LIST") {
                cmd.type = Cmd::LIST;
            } else if (line.startsWith("GET,")) {
                cmd.type = Cmd::GET;
                cmd.arg = line.substring(4);
            } else if (line.startsWith("ACK,")) {
                cmd.type = Cmd::ACK;
                cmd.n = line.substring(4).toInt();
            } else if (line.startsWith("RESEND,")) {
                cmd.type = Cmd::RESEND;
                cmd.n = line.substring(7).toInt();
            } else if (line == "PURGE") {
                cmd.type = Cmd::PURGE;
            } else if (line == "PUTWPT") {
                cmd.type = Cmd::PUTWPT;
            } else if (line.startsWith("PUTDATA,")) {
                cmd.type = Cmd::PUTDATA;
                cmd.arg = line.substring(8);
            } else if (line.startsWith("PUTDONE,")) {
                cmd.type = Cmd::PUTDONE;
                cmd.arg = line.substring(8);
            }
            
            else {
                continue;
            }

            xQueueSend(cmdQ, &cmd, 0);
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

void txLine(const String &s) {
    pTxChar->setValue((uint8_t*)s.c_str(), s.length());
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
        }

        else if (SPIFFS.exists("/route_" + ts + ".csv")) {
            txLine("/route_" + ts + ".csv\n");
            vTaskDelay(1);
            totalFiles += 1;
        }
    }

    manifest.close();
    txLine("END\n");
}

static void handleGet(const String &raw) {
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
    tx.file.seek(0);
    tx.seq = 0;
    tx.st = TxState::SENDING;

    txLine("SIZE," + String(tx.size) + "," + String(tx.crc, HEX) + "\n");
    sendChunk();
}

static void handleAck(uint32_t seq) {
    if (tx.st != TxState::SENDING || seq != tx.seq) {
        return;
    }

    if (tx.file.available()) {
        for (int i = 0; i < 4 && tx.file.available(); i++) {
            tx.seq++;
            sendChunk();
        }
    } else {
        txLine("DONE," + tx.fname + "\n");
        tx.file.close();
        tx.st = TxState::IDLE;
        currentlySending = false;
    }
}

static void handleResend(uint32_t seq) {
    if (tx.st != TxState::SENDING) {
        return;
    }
    size_t pos = seq * 180;
    tx.file.seek(pos);
    tx.seq = seq;
    sendChunk();
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
        currentlySending = false;
        txLine("PURGED\n");
    } else {
        Serial.println("SPIFFS erase failed");
    }

    clearPurgingMessage();
}

uint32_t sendChunk() {
    if (tx.st != TxState::SENDING) {
        return 0;
    }

    const size_t RAW = 180;
    uint8_t buf[RAW];
    int read = tx.file.read(buf, RAW);

    String b64 = base64_encode(buf, read);
    txLine("DATA," + String (tx.seq) + "," + b64 + "\n");
    return read;
}

static void handlePutWpt() {
    putBuf.clear();
    putSeq = 0;
    txLine("READY\n");
}

static void handlePutData(const String& b64) {
    appendBase64Chunk(putBuf, b64);
    txLine("ACK," + String(++putSeq) + "\n");
}

static void handlePutDone(const String& crcHex) {
    uint32_t crcRef = strtoul(crcHex.c_str(), nullptr, 16);
    uint32_t crcCalc = esp_rom_crc32_le(0xFFFFFFFF, putBuf.data(), putBuf.size()) ^ 0xFFFFFFFF;

    if (crcCalc == crcRef) {
        writeWaypointsFile(putBuf.data(), putBuf.size());
        putBuf.clear();
        putSeq = 0;
        txLine("OK\n");
    } else {
        putBuf.clear();
        txLine("BADCRC\n");
    }
}

static void bleWorker(void*) {
    for (Cmd cmd;;) {
        if (xQueueReceive(cmdQ, &cmd, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        switch(cmd.type) {
            case Cmd::LIST:handleList();
                break;
            case Cmd::GET:handleGet(cmd.arg);
                break;
            case Cmd::ACK:handleAck(cmd.n);
                break;
            case Cmd::RESEND:handleResend(cmd.n);
                break;
            case Cmd::PURGE:handlePurge();
                break;
            case Cmd::PUTWPT:handlePutWpt();
                break;
            case Cmd::PUTDATA:handlePutData(cmd.arg);
                break;
            case Cmd::PUTDONE:handlePutDone(cmd.arg);
                break;
        }
    }
}

void initBLE() {
    BLEDevice::init("ESP32_LapTimer");
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

void BLE_loop() {
    
}