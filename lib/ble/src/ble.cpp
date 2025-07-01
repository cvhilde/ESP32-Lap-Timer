#include <ble.h>

BLEServer *pServer = nullptr;
BLECharacteristic *pTxChar = nullptr;

static const char b64tbl[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
static String rxBuf;

bool bleConnected = false;
bool bleAdvertising = false;

QueueHandle_t cmdQ;
enum class TxState {
    IDLE,
    SENDING
};

struct Cmd {
    enum Type {
        LIST, 
        GET, 
        ACK, 
        RESEND, 
        PURGE
    } type;
    String arg;
    uint32_t n = 0;
};

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
            } else {
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

void txLine(const String &s) {
    pTxChar->setValue((uint8_t*)s.c_str(), s.length());
    pTxChar->notify();
}

static void handleList() {
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

        txLine("/log_" + ts + ".csv\n");
        vTaskDelay(1);
        txLine("/timestamps_" + ts + ".csv\n");
        vTaskDelay(1);
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
        tx.seq++;
        sendChunk();
    } else {
        txLine("DONE," + tx.fname + "\n");
        tx.file.close();
        tx.st = TxState::IDLE;
    }
}

static void handleResend(uint32_t seq) {
    if (tx.st != TxState::SENDING) {
        return;
    }
    size_t pos = seq * 128;
    tx.file.seek(pos);
    tx.seq = seq;
    sendChunk();
}

static void handlePurge() {
    File root = SPIFFS.open("/");
    File file = root.openNextFile();

    while (file) {
        String name = file.name();
        file.close();

        if (name.startsWith("/log_") || name.startsWith("/timestamps_")) {
            SPIFFS.remove(name);
        }
        file = root.openNextFile();
    }
    root.close();

    File manifest = SPIFFS.open("/sessions.txt", "w");
    if (manifest) {
        manifest.close();
    }
    txLine("PURGED\n");
}

uint32_t sendChunk() {
    if (tx.st != TxState::SENDING) {
        return 0;
    }

    const size_t RAW = 128;
    uint8_t buf[RAW];
    int read = tx.file.read(buf, RAW);

    String b64 = base64_encode(buf, read);
    txLine("DATA," + String (tx.seq) + "," + b64 + "\n");
    return read;
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

void BLE_loop() {
    
}