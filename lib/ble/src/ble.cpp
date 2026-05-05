///===========================================================================
///
/// ble.cpp
///
/// Implements BLE UART setup, advertising/status handling, and command-based
/// file transfer for log downloads, storage purge, and waypoint uploads.
///
///===========================================================================

#include <ble.h>
#include <display.h>
#include <cstring>
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_rom_crc.h>
#include <freertos/queue.h>
#include <vector>
#include <storage.h>
#include <led.h>
#include <prefs.h>

//----------------------------------------------------------------------------
// Private namespace
//----------------------------------------------------------------------------
namespace
{
    // BLE UUID constance
    constexpr char SERVICE_UUID[]           = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
    constexpr char CHARACTERISTIC_UUID_RX[] = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
    constexpr char CHARACTERISTIC_UUID_TX[] = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

    // Base64 code string
    constexpr char     BASE64_KEY[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    // Constants for cmd size and default MTU's
    constexpr size_t   CMD_SIZE         = 256;
    constexpr uint16_t DESIRED_MTU_SIZE = 247;
    constexpr uint16_t DEFAULT_MTU_SIZE = 23;

    // Constants for legacy BLE transfers
    constexpr size_t   LEGACY_RAW_CHUNK_SIZE = 180;
    constexpr uint32_t LEGACY_TX_WINDOW      = 4;

    // Constants for the fast/binary BLE transfers
    constexpr uint8_t  BINARY_FRAME_MARKER        = 0xA5;
    constexpr uint8_t  BINARY_FRAME_TYPE_DATA     = 0x01;
    constexpr uint8_t  BINARY_FRAME_TYPE_PUT_DATA = 0x02;
    constexpr size_t   BINARY_FRAME_HEADER_SIZE   = 8;
    constexpr uint32_t BINARY_TX_WINDOW           = 8;
    constexpr uint32_t BINARY_PUT_ACK_INTERVAL    = 4;

    // Constants for BLE advertising button/led logic
    constexpr unsigned ADVERTISING_LED_INTERVAL     = 1000U;
    constexpr unsigned ADVERTISING_TIMEOUT_INTERVAL = 200U;
    constexpr unsigned ADVERTISING_TIMEOUT_DURATION = 1000U;
    constexpr unsigned ADVERTISING_TIMEOUT          = 60000U;

    // BLE receiver/transceiver status
    enum class TxState
    {
        IDLE,
        SENDING,
        PUT_RX
    };

    // Transfer type
    enum class TransferMode
    {
        LEGACY_BASE64,
        FAST_BINARY
    };

    // Command type
    enum class CommandType
    {
        LIST,
        GET,
        GET_FAST,
        ACK,
        RESEND,
        PURGE,
        PUT_BEGIN,
        PUT_BEGIN_FAST,
        PUT_DATA,
        PUT_END,
        SET_LAP_HZ,
        SET_ROUTE_HZ,
        BAD_HZ,
        INVALID
    };

    // All information associated with a command
    struct Cmd
    {
        CommandType type;
        char arg[CMD_SIZE];
        int n;
        int length;

        Cmd():
            type(CommandType::INVALID),
            arg{},
            n(0),
            length(0)
        {}
    };

    // All information associated with a file transfer
    struct Tx
    {
        File file;
        String name;
        int size;
        int crc;
        int nextSeq;
        int windowBase;
        int totalChunks;
        int windowSize;
        int chunkSize;
        TransferMode mode;
        TxState state;

        Tx():
            size(0),
            crc(0),
            nextSeq(0),
            windowBase(0),
            totalChunks(0),
            windowSize(1),
            chunkSize(LEGACY_RAW_CHUNK_SIZE),
            mode(TransferMode::LEGACY_BASE64),
            state(TxState::IDLE)
        {}
    };

    // Persistent data store for BLE
    struct Data
    {
        std::vector<uint8_t> putBuffer;
        int putSequence;
        int putExpectedSize;
        int putExpectedCRC;
        TransferMode putMode;
        uint16_t putChunkSize;

        QueueHandle_t cmdQ;

        String rxBuffer;
        uint16_t bleConnectionId;
        uint16_t blePeerMTU;

        bool bleConnected;
        bool bleAdvertising;
        bool currentlySending;
        int totalFiles;
        int currentFileNumber;

        unsigned long advertisingStartTime;

        Data():
            putSequence(0),
            putExpectedSize(0),
            putExpectedCRC(0),
            putMode(TransferMode::LEGACY_BASE64),
            putChunkSize(LEGACY_RAW_CHUNK_SIZE),
            bleConnectionId(0),
            blePeerMTU(DEFAULT_MTU_SIZE),
            bleConnected(false),
            bleAdvertising(false),
            currentlySending(false),
            totalFiles(0),
            currentFileNumber(0),
            advertisingStartTime(0)
        {}
    };

    // Persistent data store for current file transfer
    Tx _tx;

    // Persistent data store for BLE logic
    Data _data;

    // BLEServer object
    BLEServer *_BLEServer = nullptr;

    // BLE broadcast characteristics
    BLECharacteristic *_TxChar = nullptr;

    //------------------------------------------------------------------------
    void StartAdvertising()
    {
        if (!_data.bleAdvertising && !_data.bleConnected)
        {
            _BLEServer->getAdvertising()->start();
            _data.bleAdvertising = true;
        }
    }

    //------------------------------------------------------------------------
    void StopAdvertising()
    {
        if (_data.bleAdvertising)
        {
            _BLEServer->getAdvertising()->stop();
            _data.bleAdvertising = false;
        }
    }

    //------------------------------------------------------------------------
    uint32_t CRC32File(File &f)
    {
        uint32_t crc = 0xFFFFFFFF;
        uint8_t buf[256];
        while (f.available()) {
            int read = f.read(buf, sizeof(buf));
            crc = esp_rom_crc32_le(crc, buf, read);
        }
        return crc ^ 0xFFFFFFFF;
    }

    //------------------------------------------------------------------------
    String Base64Encode(const uint8_t *in, size_t len)
    {
        String out; out.reserve((len + 2) / 3 * 4);
        for (size_t i = 0; i < len; i += 3) {
            uint32_t v = in[i] << 16 | (i + 1 < len ? in[i + 1] << 8 : 0) | (i + 2 < len ? in[i + 2] : 0);
            out += BASE64_KEY[(v >> 18) & 0x3F];
            out += BASE64_KEY[(v >> 12) & 0x3F];
            out += (i + 1 < len) ? BASE64_KEY[(v >> 6) & 0x3F] : '=';
            out += (i + 2 < len) ? BASE64_KEY[ v & 0x3F] : '=';
        }
        return out;
    }

    //------------------------------------------------------------------------
    int Base64Decode(uint8_t* out, const char* in, size_t len)
    {
        static uint8_t lut[256];
        static bool init = false;

        if (!init)
        {
            const char* p = BASE64_KEY;
            for (int j = 0; j < 64; j++)
            {
                lut[(uint8_t)p[j]] = j;
            }
            init = true;
        }

        size_t i = 0;
        int o = 0;

        while (i + 3 < len)
        {
            char c0 = in[i++];
            char c1 = in[i++];
            char c2 = in[i++];
            char c3 = in[i++];

            uint32_t v =
                ((uint32_t)lut[(uint8_t)c0] << 18) |
                ((uint32_t)lut[(uint8_t)c1] << 12) |
                ((uint32_t)lut[(uint8_t)c2] <<  6) |
                ((uint32_t)lut[(uint8_t)c3]);

            out[o++] = (v >> 16) & 0xFF;

            if (c2 != '=')
            {
                out[o++] = (v >> 8) & 0xFF;
            }

            if (c3 != '=')
            {
                out[o++] = v & 0xFF;
            }
        }

        return o;
    }

    //------------------------------------------------------------------------
    void AppendBase64Chunk(std::vector<uint8_t>& buf, const String& b64)
    {
        int rawLen = (b64.length() * 3) / 4;
        size_t idx = buf.size();
        buf.resize(idx + rawLen);
        int actual = Base64Decode(&buf[idx], b64.c_str(), b64.length());
        buf.resize(idx + actual);
    }

    //------------------------------------------------------------------------
    void TxLine(const String &s)
    {
        _TxChar->setValue((uint8_t*)s.c_str(), s.length());
        _TxChar->notify();
    }

    //------------------------------------------------------------------------
    void AppendRawChunk(std::vector<uint8_t>& buf, const uint8_t* data, size_t len)
    {
        size_t idx = buf.size();
        buf.resize(idx + len);
        memcpy(&buf[idx], data, len);
    }

    //------------------------------------------------------------------------
    void WriteLE16(uint8_t* dst, uint16_t value)
    {
        dst[0] = value & 0xFF;
        dst[1] = (value >> 8) & 0xFF;
    }

    //------------------------------------------------------------------------
    void WriteLE32(uint8_t* dst, uint32_t value)
    {
        dst[0] = value & 0xFF;
        dst[1] = (value >> 8) & 0xFF;
        dst[2] = (value >> 16) & 0xFF;
        dst[3] = (value >> 24) & 0xFF;
    }

    //------------------------------------------------------------------------
    void TxBinaryFrame(uint8_t type,
                       uint32_t seq,
                       const uint8_t* payload,
                       uint16_t payloadLen)
    {
        std::vector<uint8_t> frame(BINARY_FRAME_HEADER_SIZE + payloadLen);
        frame[0] = BINARY_FRAME_MARKER;
        frame[1] = type;
        WriteLE32(frame.data() + 2, seq);
        WriteLE16(frame.data() + 6, payloadLen);

        if (payloadLen > 0) {
            memcpy(frame.data() + BINARY_FRAME_HEADER_SIZE, payload, payloadLen);
        }

        _TxChar->setValue(frame.data(), frame.size());
        _TxChar->notify();
    }

    //------------------------------------------------------------------------
    void ResetPutState()
    {
        _data.putBuffer.clear();
        _data.putSequence = 0;
        _data.putExpectedSize = 0;
        _data.putExpectedCRC = 0;
        _data.putMode = TransferMode::LEGACY_BASE64;
        _data.putChunkSize = LEGACY_RAW_CHUNK_SIZE;
    }

    //------------------------------------------------------------------------
    void SetCmdArg(Cmd& cmd, const String& value)
    {
        value.toCharArray(cmd.arg, sizeof(cmd.arg));
        cmd.length = strlen(cmd.arg);
    }

    //------------------------------------------------------------------------
    void SetCmdPayload(Cmd& cmd, const uint8_t* data, size_t len)
    {
        if (len > sizeof(cmd.arg))
        {
            len = sizeof(cmd.arg);
        }

        memcpy(cmd.arg, data, len);
        cmd.length = len;
    }

    //------------------------------------------------------------------------
    bool ParsePositiveUInt(const String& value, unsigned& parsed)
    {
        String trimmed = value;
        trimmed.trim();

        if (trimmed.isEmpty())
        {
            return false;
        }

        for (size_t i = 0; i < trimmed.length(); i++)
        {
            if (!isDigit(trimmed[i]))
            {
                return false;
            }
        }

        unsigned long raw = strtoul(trimmed.c_str(), nullptr, 10);
        if (raw == 0)
        {
            return false;
        }

        if (raw > Prefs::MAX_FREQUENCY)
        {
            raw = Prefs::MAX_FREQUENCY;
        }

        parsed = static_cast<unsigned>(raw);
        return true;
    }

    //------------------------------------------------------------------------
    uint16_t ReadLE16(const uint8_t* src)
    {
        return static_cast<uint16_t>(src[0]) |
            (static_cast<uint16_t>(src[1]) << 8);
    }

    //------------------------------------------------------------------------
    uint32_t ReadLE32(const uint8_t* src)
    {
        return static_cast<uint32_t>(src[0]) |
            (static_cast<uint32_t>(src[1]) << 8) |
            (static_cast<uint32_t>(src[2]) << 16) |
            (static_cast<uint32_t>(src[3]) << 24);
    }

    //------------------------------------------------------------------------
    uint16_t GetBinaryPayloadCapacity(uint16_t mtu = _data.blePeerMTU)
    {
        size_t attPayload = mtu > 3 ? mtu - 3 : 20;
        if (attPayload <= BINARY_FRAME_HEADER_SIZE)
        {
            return 1;
        }

        size_t payload = attPayload - BINARY_FRAME_HEADER_SIZE;
        if (payload > CMD_SIZE)
        {
            payload = CMD_SIZE;
        }

        return static_cast<uint16_t>(payload);
    }

    //------------------------------------------------------------------------
    uint32_t GetChunkCount(uint32_t size, uint16_t chunkSize)
    {
        if (chunkSize == 0)
        {
            return 0;
        }

        return (size + chunkSize - 1) / chunkSize;
    }

    //------------------------------------------------------------------------
    bool DecodeBinaryFrame(
        const uint8_t* data,
        size_t len,
        uint8_t& type,
        uint32_t& seq,
        const uint8_t*& payload,
        uint16_t& payloadLen)
    {
        if (len < BINARY_FRAME_HEADER_SIZE || data[0] != BINARY_FRAME_MARKER)
        {
            return false;
        }

        type = data[1];
        seq = ReadLE32(data + 2);
        payloadLen = ReadLE16(data + 6);
        if (payloadLen > CMD_SIZE || (BINARY_FRAME_HEADER_SIZE + payloadLen) != len)
        {
            return false;
        }

        payload = data + BINARY_FRAME_HEADER_SIZE;
        return true;
    }

    //------------------------------------------------------------------------
    void ResetTxState()
    {
        if (_tx.file)
        {
            _tx.file.close();
        }

        _tx.name = "";
        _tx.size = 0;
        _tx.crc = 0;
        _tx.nextSeq = 0;
        _tx.windowBase = 0;
        _tx.totalChunks = 0;
        _tx.windowSize = 1;
        _tx.chunkSize = LEGACY_RAW_CHUNK_SIZE;
        _tx.mode = TransferMode::LEGACY_BASE64;
        _tx.state = TxState::IDLE;
        _data.currentlySending = false;
    }

    //------------------------------------------------------------------------
    void QueueCommand(const Cmd& cmd)
    {
        xQueueSend(_data.cmdQ, &cmd, 0);
    }

    //------------------------------------------------------------------------
    void QueueTextCommand(const String& line)
    {
        Cmd cmd = {};
        if (line == "LIST")
        {
            cmd.type = CommandType::LIST;
        }
        else if (line.startsWith("GET_FAST,"))
        {
            cmd.type = CommandType::GET_FAST;
            SetCmdArg(cmd, line.substring(9));
        }
        else if (line.startsWith("GET,"))
        {
            cmd.type = CommandType::GET;
            SetCmdArg(cmd, line.substring(4));
        }
        else if (line.startsWith("ACK,"))
        {
            cmd.type = CommandType::ACK;
            cmd.n = line.substring(4).toInt();
        }
        else if (line.startsWith("RESEND,"))
        {
            cmd.type = CommandType::RESEND;
            cmd.n = line.substring(7).toInt();
        }
        else if (line == "PURGE")
        {
            cmd.type = CommandType::PURGE;
        }
        else if (line.startsWith("PUT_BEGIN_FAST,"))
        {
            cmd.type = CommandType::PUT_BEGIN_FAST;
            SetCmdArg(cmd, line.substring(15));
        }
        else if (line.startsWith("PUT_BEGIN,"))
        {
            cmd.type = CommandType::PUT_BEGIN;
            SetCmdArg(cmd, line.substring(10));
        }
        else if (line.startsWith("PUT_DATA,"))
        {
            int comma = line.indexOf(',', 9);
            if (comma < 0)
            {
                return;
            }
            cmd.type = CommandType::PUT_DATA;
            cmd.n = line.substring(9, comma).toInt();
            SetCmdArg(cmd, line.substring(comma + 1));
        }
        else if (line == "PUT_END")
        {
            cmd.type = CommandType::PUT_END;
        }
        else if (line.startsWith("SET_LAP_HZ,"))
        {
            unsigned hz = 0;
            if (!ParsePositiveUInt(line.substring(11), hz))
            {
                cmd.type = CommandType::BAD_HZ;
            }
            else
            {
                cmd.type = CommandType::SET_LAP_HZ;
                cmd.n = hz;
            }
        }
        else if (line.startsWith("SET_ROUTE_HZ,"))
        {
            unsigned hz = 0;
            if (!ParsePositiveUInt(line.substring(13), hz))
            {
                cmd.type = CommandType::BAD_HZ;
            }
            else
            {
                cmd.type = CommandType::SET_ROUTE_HZ;
                cmd.n = hz;
            }
        }
        else
        {
            return;
        }

        QueueCommand(cmd);
    }

    //------------------------------------------------------------------------
    void HandleList()
    {
        _data.totalFiles = 0;

        File manifest = Storage::GetFile(Storage::MANIFEST_FILE, "r");
        if (!manifest)
        {
            TxLine("LIST,0\nEND\n");
            return;
        }

        TxLine("LIST, 0\n");
        while (manifest.available())
        {
            String ts = manifest.readStringUntil('\n');
            ts.trim();

            if (ts.isEmpty())
            {
                continue;
            }

            // Read for lap timing logs. Lap log filenames may include the
            // track name, so resolve the actual file from the filesystem.
            String lapLogFile = "";
            {
                const String exactLapLog = 
                    Storage::LAP_LOG_PREFIX + ts + Storage::FILE_TYPE;

                const String lapSuffix = "_" + ts + Storage::FILE_TYPE;

                File root = Storage::GetFile("/", "r");
                if (root && root.isDirectory())
                {
                    File entry = root.openNextFile();

                    // Continue reading until no more files.
                    while (entry)
                    {
                        if (!entry.isDirectory())
                        {
                            const char* rawPath = entry.path();
                            if (rawPath != nullptr)
                            {
                                const String entryPath(rawPath);

                                // If the file meets either condition, the file was
                                // found. Exit the loop and transmit the data.
                                if (entryPath == exactLapLog
                                    || (entryPath.startsWith(Storage::LAP_LOG_PREFIX)
                                        && entryPath.endsWith(lapSuffix)))
                                {
                                    lapLogFile = entryPath;
                                    entry.close();
                                    break;
                                }
                            }
                        }

                        // File wasn't found yet. Search the next one.
                        entry.close();
                        entry = root.openNextFile();
                    }

                    root.close();
                }
            }

            if (!lapLogFile.isEmpty())
            {
                TxLine(lapLogFile + "\n");
                vTaskDelay(1);
                TxLine(Storage::LAP_TIMESTAMPS_PREFIX + ts + Storage::FILE_TYPE + "\n");
                vTaskDelay(1);
                _data.totalFiles += 2;

                if (Storage::FileExists(Storage::SUMMARY_PREFIX + ts + Storage::FILE_TYPE))
                {
                    TxLine(Storage::SUMMARY_PREFIX + ts + Storage::FILE_TYPE + "\n");
                    vTaskDelay(1);
                    _data.totalFiles += 1;
                }
            }

            // Read for route tracking logs
            if (Storage::FileExists(Storage::ROUTE_LOG_PREFIX + ts + Storage::FILE_TYPE))
            {
                TxLine(Storage::ROUTE_LOG_PREFIX + ts + Storage::FILE_TYPE + "\n");
                vTaskDelay(1);
                _data.totalFiles += 1;

                if (Storage::FileExists(Storage::SUMMARY_PREFIX + ts + Storage::FILE_TYPE))
                {
                    TxLine(Storage::SUMMARY_PREFIX + ts + Storage::FILE_TYPE + "\n");
                    vTaskDelay(1);
                    _data.totalFiles += 1;
                }
            }
        }

        manifest.close();
        TxLine("END\n");
    }

    //------------------------------------------------------------------------
    void FinishTransfer()
    {
        TxLine("DONE," + _tx.name + "\n");
        ResetTxState();
    }

    //------------------------------------------------------------------------
    uint32_t SendChunk(uint32_t seq)
    {
        if (_tx.state != TxState::SENDING || seq >= _tx.totalChunks)
        {
            return 0;
        }

        const uint32_t offset = seq * _tx.chunkSize;
        if (!_tx.file.seek(offset))
        {
            return 0;
        }

        uint8_t buf[CMD_SIZE];
        int read = _tx.file.read(buf, _tx.chunkSize);
        if (read <= 0)
        {
            return 0;
        }

        if (_tx.mode == TransferMode::FAST_BINARY)
        {
            TxBinaryFrame(BINARY_FRAME_TYPE_DATA, seq, buf, read);
        }
        else
        {
            String b64 = Base64Encode(buf, read);
            TxLine("DATA," +
                String(seq) + "," +
                b64 +
                "\n");
        }

        return read;
    }

    //------------------------------------------------------------------------
    void SendAvailableChunks()
    {
        while (_tx.nextSeq < _tx.totalChunks &&
              (_tx.nextSeq - _tx.windowBase) < _tx.windowSize)
        {
            if (SendChunk(_tx.nextSeq) == 0)
            {
                break;
            }
            _tx.nextSeq++;
        }
    }

    //------------------------------------------------------------------------
    void HandleGet(const String &raw, TransferMode mode)
    {
        String fname = raw;
        fname.trim();
        fname.replace("\r", "");
        fname.replace("\n", "");
        if (!fname.startsWith("/"))
        {
            fname = "/" + fname;
        }

        if (_tx.state != TxState::IDLE)
        {
            return;
        }

        _tx.file = Storage::GetFile(fname, "r");
        if (!_tx.file)
        {
            TxLine("ERR,NOFILE\n");
            return;
        }

        _data.currentFileNumber++;
        _data.currentlySending = true;

        _tx.name = fname;
        _tx.size = _tx.file.size();
        _tx.crc = CRC32File(_tx.file);
        _tx.mode = mode;
        _tx.chunkSize = mode == TransferMode::FAST_BINARY ?
            GetBinaryPayloadCapacity() : LEGACY_RAW_CHUNK_SIZE;
        _tx.windowSize = mode == TransferMode::FAST_BINARY ?
            BINARY_TX_WINDOW : LEGACY_TX_WINDOW;
        _tx.totalChunks = GetChunkCount(_tx.size, _tx.chunkSize);
        _tx.windowBase = 0;
        _tx.nextSeq = 0;
        _tx.state = TxState::SENDING;

        if (mode == TransferMode::FAST_BINARY)
        {
            TxLine(
                "SIZE_FAST," +
                String(_tx.size) + "," +
                String(_tx.crc, HEX) + "," +
                String(_tx.chunkSize) + "," +
                String(_tx.windowSize) +
                "\n"
            );
        }
        else
        {
            TxLine("SIZE," +
                String(_tx.size) + "," +
                String(_tx.crc, HEX) +
                "\n");
        }

        if (_tx.totalChunks == 0)
        {
            FinishTransfer();
            return;
        }

        if (mode == TransferMode::FAST_BINARY)
        {
            SendAvailableChunks();
        }
        else
        {
            if (SendChunk(_tx.nextSeq) > 0)
            {
                _tx.nextSeq++;
            }
        }
    }

    //------------------------------------------------------------------------
    void HandleAck(uint32_t seq)
    {
        if (_tx.state != TxState::SENDING || seq >= _tx.totalChunks)
        {
            return;
        }

        uint32_t ackBase = seq + 1;
        if (ackBase <= _tx.windowBase)
        {
            return;
        }

        _tx.windowBase = ackBase;
        if (_tx.windowBase >= _tx.totalChunks)
        {
            FinishTransfer();
            return;
        }

        SendAvailableChunks();
    }

    //------------------------------------------------------------------------
    void HandleResend(uint32_t seq)
    {
        if (_tx.state != TxState::SENDING || seq >= _tx.totalChunks)
        {
            return;
        }

        SendChunk(seq);
    }

    //------------------------------------------------------------------------
    void HandlePurge()
    {
        bool backedUp(Storage::BackupWaypoints());

        Display::DisplayPurgingMessage();
        Led::StartBlink(250);

        if (Storage::PurgeFlash())
        {
            if (backedUp)
            {
                Storage::LoadBackedupWaypoints();
            }

            _data.totalFiles = 0;
            _data.currentFileNumber = 0;
            ResetTxState();
            TxLine("PURGED\n");
        }

        Display::ClearPurgingMessage();
        Led::StopBlink();
    }

    //------------------------------------------------------------------------
    void HandleSetLapHz(unsigned hz)
    {
        Prefs::SetLapLogFrequency(hz);
        TxLine("LAP_HZ," + String(Prefs::PersistConfig().lapLogHz) + "\n");
    }

    //------------------------------------------------------------------------
    void HandleSetRouteHz(unsigned hz)
    {
        Prefs::SetRouteLogFrequency(hz);
        TxLine("ROUTE_HZ," + String(Prefs::PersistConfig().routeLogHz) + "\n");
    }

    //------------------------------------------------------------------------
    void HandlePutBegin(const String& meta, TransferMode mode)
    {
        int comma = meta.indexOf(',');
        if (comma < 0)
        {
            TxLine("ERR,BAD_BEGIN\n");
            return;
        }

        uint32_t size = strtoul(meta.substring(0, comma).c_str(), nullptr, 10);
        uint32_t crc = strtoul(meta.substring(comma + 1).c_str(), nullptr, 16);

        ResetPutState();
        _data.putExpectedSize = size;
        _data.putExpectedCRC = crc;
        _data.putMode = mode;
        _data.putChunkSize = mode == TransferMode::FAST_BINARY ?
            GetBinaryPayloadCapacity() : LEGACY_RAW_CHUNK_SIZE;
        _tx.state = TxState::PUT_RX;
        if (mode == TransferMode::FAST_BINARY)
        {
            TxLine("READY_FAST," +
                String(_data.putChunkSize) +
                "," + String(BINARY_PUT_ACK_INTERVAL) +
                "\n");
        }
        else
        {
            TxLine("READY\n");
        }
        Display::DisplayGettingMessage();
    }

    //------------------------------------------------------------------------
    void HandlePutData(const Cmd& cmd)
    {
        if (_tx.state != TxState::PUT_RX)
        {
            TxLine("ERR,NO_PUT\n");
            return;
        }

        if (cmd.n != _data.putSequence)
        {
            TxLine("RESEND," + String(_data.putSequence) + "\n");
            return;
        }

        if (_data.putMode == TransferMode::FAST_BINARY)
        {
            AppendRawChunk(_data.putBuffer,
                reinterpret_cast<const uint8_t*>(cmd.arg), cmd.length);
        }
        else
        {
            AppendBase64Chunk(_data.putBuffer, String(cmd.arg));
        }

        if (_data.putBuffer.size() > _data.putExpectedSize)
        {
            ResetPutState();
            _tx.state = TxState::IDLE;

            TxLine("ERR,SIZE\n");
            return;
        }

        _data.putSequence++;
        if (_data.putMode == TransferMode::LEGACY_BASE64)
        {
            TxLine("ACK," + String(cmd.n) + "\n");
        }
        else if ((_data.putSequence% BINARY_PUT_ACK_INTERVAL) == 0 ||
            _data.putBuffer.size() == _data.putExpectedSize)
        {
            TxLine("ACK," + String(_data.putSequence - 1) + "\n");
        }
    }

    //------------------------------------------------------------------------
    void HandlePutEnd()
    {
        if (_tx.state != TxState::PUT_RX)
        {
            TxLine("ERR,NO_PUT\n");
            return;
        }

        if (_data.putBuffer.size() != _data.putExpectedSize)
        {
            ResetPutState();
            _tx.state = TxState::IDLE;

            TxLine("ERR,SIZE\n");
            return;
        }

        uint32_t crcCalc = esp_rom_crc32_le(0xFFFFFFFF,
            _data.putBuffer.data(),
            _data.putBuffer.size()) ^ 0xFFFFFFFF;

        if (crcCalc == _data.putExpectedCRC)
        {
            Storage::WriteWaypointsFile(_data.putBuffer.data(),
                _data.putBuffer.size());

            ResetPutState();
            _tx.state = TxState::IDLE;

            TxLine("STORED,/waypoints.json\n");
        }
        else
        {
            ResetPutState();
            _tx.state = TxState::IDLE;

            TxLine("BADCRC\n");
        }

        Display::ClearGettingMessage();
    }

    //------------------------------------------------------------------------
    void BleWorker(void*)
    {
        for (Cmd cmd;;)
        {
            if (xQueueReceive(_data.cmdQ, &cmd, portMAX_DELAY) != pdTRUE)
            {
                continue;
            }

            switch(cmd.type)
            {
                case CommandType::LIST:
                    HandleList();
                    break;
                case CommandType::GET:
                    HandleGet(cmd.arg, TransferMode::LEGACY_BASE64);
                    break;
                case CommandType::GET_FAST:
                    HandleGet(cmd.arg, TransferMode::FAST_BINARY);
                    break;
                case CommandType::ACK:
                    HandleAck(cmd.n);
                    break;
                case CommandType::RESEND:
                    HandleResend(cmd.n);
                    break;
                case CommandType::PURGE:
                    HandlePurge();
                    break;
                case CommandType::PUT_BEGIN:
                    HandlePutBegin(cmd.arg, TransferMode::LEGACY_BASE64);
                    break;
                case CommandType::PUT_BEGIN_FAST:
                    HandlePutBegin(cmd.arg, TransferMode::FAST_BINARY);
                    break;
                case CommandType::PUT_DATA:
                    HandlePutData(cmd);
                    break;
                case CommandType::PUT_END:
                    HandlePutEnd();
                    break;
                case CommandType::SET_LAP_HZ:
                    HandleSetLapHz(cmd.n);
                    break;
                case CommandType::SET_ROUTE_HZ:
                    HandleSetRouteHz(cmd.n);
                    break;
                case CommandType::BAD_HZ:
                    TxLine("ERR,BAD_HZ\n");
                    break;
                case CommandType::INVALID: // <=== PASSTHROUGH
                default:
                    // Do nothing
                    break;
            }
        }
    }

    //------------------------------------------------------------------------
    class ServerCB : public BLEServerCallbacks
    {
        //--------------------------------------------------------------------
        void onConnect(BLEServer* server) override
        {
            _data.bleConnected = true;
            _data.bleAdvertising = false;
            _data.blePeerMTU = BLEDevice::getMTU();
            Led::StopBlink();
        }

        //--------------------------------------------------------------------
        void onConnect(BLEServer* server, esp_ble_gatts_cb_param_t* param) override
        {
            onConnect(server);
            if (param != nullptr)
            {
                _data.bleConnectionId = param->connect.conn_id;
                _data.blePeerMTU = server->getPeerMTU(_data.bleConnectionId);
            }
        }

        //--------------------------------------------------------------------
        void onDisconnect(BLEServer*) override
        {
            _data.bleConnected = false;
            _data.bleConnectionId = 0;
            _data.blePeerMTU = DEFAULT_MTU_SIZE;
            ResetTxState();
            ResetPutState();
        }

        //--------------------------------------------------------------------
        void onMtuChanged(BLEServer*, esp_ble_gatts_cb_param_t* param) override
        {
            if (param != nullptr)
            {
                _data.blePeerMTU = param->mtu.mtu;
            }
        }
    };

    //------------------------------------------------------------------------
    class RxCB : public BLECharacteristicCallbacks
    {
        //--------------------------------------------------------------------
        void onWrite(BLECharacteristic *c, esp_ble_gatts_cb_param_t* param) override
        {
            const uint8_t* data = param != nullptr
                ? param->write.value
                : reinterpret_cast<const uint8_t*>(c->getData());
            size_t len = param != nullptr ? param->write.len : c->getLength();
            if (len == 0 || data == nullptr)
            {
                return;
            }

            if (data[0] == BINARY_FRAME_MARKER)
            {
                uint8_t type = 0;
                uint32_t seq = 0;
                const uint8_t* payload = nullptr;
                uint16_t payloadLen = 0;
                if (!DecodeBinaryFrame(data, len, type, seq, payload, payloadLen))
                {
                    return;
                }

                if (type == BINARY_FRAME_TYPE_PUT_DATA)
                {
                    Cmd cmd = {};
                    cmd.type = CommandType::PUT_DATA;
                    cmd.n = seq;
                    SetCmdPayload(cmd, payload, payloadLen);
                    QueueCommand(cmd);
                }

                return;
            }

            for (size_t i = 0; i < len; i++)
            {
                _data.rxBuffer += static_cast<char>(data[i]);
            }

            while (true)
            {
                int nl = _data.rxBuffer.indexOf('\n');
                if (nl < 0)
                {
                    break;
                }

                String line = _data.rxBuffer.substring(0, nl);
                _data.rxBuffer.remove(0, nl + 1);
                line.trim();
                if (!line.isEmpty())
                {
                    QueueTextCommand(line);
                }
            }
        }
    };
}

//----------------------------------------------------------------------------
// BLE Public namespace
//----------------------------------------------------------------------------
namespace BLE
{
    //------------------------------------------------------------------------
    void InitializeBLE()
    {
        BLEDevice::init("ESP32_LapTimer");
        BLEDevice::setMTU(DESIRED_MTU_SIZE);
        _BLEServer = BLEDevice::createServer();
        _BLEServer->setCallbacks(new ServerCB());
        BLEService *svc = _BLEServer->createService(SERVICE_UUID);

        _TxChar = svc->createCharacteristic(CHARACTERISTIC_UUID_TX,
            BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);

        _TxChar->addDescriptor(new BLE2902);

        BLECharacteristic *rx = svc->createCharacteristic(CHARACTERISTIC_UUID_RX,
            BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
        rx->setCallbacks(new RxCB());

        svc->start();

        _data.cmdQ = xQueueCreate(8, sizeof(Cmd));
        xTaskCreatePinnedToCore(BleWorker, "BLE_WRK", 6 * 1024, nullptr, 1, nullptr, 0);
    }

    //------------------------------------------------------------------------
    void UpdateBLE(const Button::Mode& mode)
    {
        if (mode == Button::Mode::VERY_LONG)
        {
            if (!_data.bleAdvertising)
            {
                StartAdvertising();
                Led::StartBlink(ADVERTISING_LED_INTERVAL);
                _data.advertisingStartTime = millis();
            }
        }

        if (    _data.bleAdvertising
            && !_data.bleConnected
            && (millis() - _data.advertisingStartTime >= ADVERTISING_TIMEOUT))
        {
            StopAdvertising();
            Led::StopBlink();
            Led::StartOneShotBlink(ADVERTISING_TIMEOUT_INTERVAL,
                ADVERTISING_TIMEOUT_DURATION);
        }
    }

    //------------------------------------------------------------------------
    BLE::Status GetStatus()
    {
        return {
            _data.bleConnected,
            _data.bleAdvertising,
            _data.currentlySending,
            static_cast<unsigned>(_data.totalFiles),
            static_cast<unsigned>(_data.currentFileNumber)
        };
    }
}
