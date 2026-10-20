"""Strict decoding and interpretation of MCU startup ID10 diagnostics.

Contains no serial, network, Bridge, or outgoing-command capability. These
reports describe past startup transactions, never current position freshness.
"""
import struct

WIRE_SIZE = 96
TARGET_ID = 10
KNOWN_IDS = tuple(range(10, 15)) + tuple(range(20, 25)) + tuple(range(30, 35))
COUNTER_NAMES = (
    "tx_count", "rx_count", "drained_count", "valid_frames", "crc_errors",
    "device_error_frames", "known_frames", "unknown_frames", "echoes",
    "unexpected_length_frames", "valid_read_frames", "known_id_mask",
)
REQUESTS = {
    0x01: bytes.fromhex("FF FF 0A 02 01 F2"),
    0x02: bytes.fromhex("FF FF 0A 04 02 38 06 B1"),
    0x82: bytes.fromhex("FF FF FE 05 82 38 06 0A 32"),
}


def decode_report(payload):
    if not isinstance(payload, (bytes, bytearray)) or len(payload) != WIRE_SIZE:
        raise ValueError("DQP1 report must be exactly 96 bytes")
    if payload[:4] != b"DQP1" or payload[4] != 1:
        raise ValueError("unsupported DQP1 magic/version")
    flags, opcode, target_id, attempt, address, length, requested = payload[5:12]
    if flags & ~0x0F or not flags & 1:
        raise ValueError("invalid/incomplete report flags")
    if opcode not in REQUESTS or target_id != TARGET_ID or not 1 <= attempt <= 3:
        raise ValueError("report is outside fixed ID10 probe plan")
    expected_address, expected_length = (255, 0) if opcode == 1 else (56, 6)
    if (address, length, requested) != (expected_address, expected_length, len(REQUESTS[opcode])):
        raise ValueError("query context does not match its fixed request")
    counts = dict(zip(COUNTER_NAMES, struct.unpack_from("<12H", payload, 12)))
    first, last, window, elapsed = struct.unpack_from("<4I", payload, 36)
    last_id, status, reply_length, prefix_length = payload[52:56]
    position, speed = struct.unpack_from("<2H", payload, 56)
    position_valid, unknown_id = payload[60:62]
    if payload[62:64] != b"\0\0" or position_valid not in (0, 1):
        raise ValueError("invalid position-valid/reserved fields")
    if window not in (20000, 50000):
        raise ValueError("unsupported capture window")
    if prefix_length != min(32, counts["rx_count"]):
        raise ValueError("RX prefix length inconsistent with RX count")
    if payload[64 + prefix_length:96] != bytes(32 - prefix_length):
        raise ValueError("non-zero RX-prefix padding")
    if bool(flags & 4) != (counts["rx_count"] > 32):
        raise ValueError("RX-prefix truncation flag inconsistent with RX count")
    if counts["tx_count"] > requested:
        raise ValueError("TX count exceeds fixed request")
    if flags & 8:
        if counts["tx_count"] != 0 or flags & 2:
            raise ValueError("quiet timeout cannot have transmitted a request")
    elif bool(flags & 2) != (counts["tx_count"] != requested):
        raise ValueError("short-TX flag inconsistent with TX count")
    if not flags & 8 and elapsed < window:
        raise ValueError("finished capture is shorter than its requested window")
    if counts["valid_frames"] != counts["known_frames"] + counts["unknown_frames"]:
        raise ValueError("frame counters inconsistent")
    if counts["device_error_frames"] > counts["valid_frames"]:
        raise ValueError("device-error counter exceeds valid frames")
    if counts["known_id_mask"] & ~0x7FFF:
        raise ValueError("known ID mask exceeds the 15 configured IDs")
    if counts["valid_read_frames"] > counts["known_frames"]:
        raise ValueError("read count exceeds known replies")
    if bool(position_valid) != bool(counts["valid_read_frames"]):
        raise ValueError("position-valid flag inconsistent with valid reads")
    if position_valid and (opcode == 1 or flags & (2 | 8)):
        raise ValueError("PING/aborted transaction cannot produce a valid position")
    if position_valid and not counts["known_id_mask"] & 1:
        raise ValueError("valid ID10 position requires an actual ID10 reply")
    if counts["rx_count"] == 0:
        if (first, last) != (0xFFFFFFFF, 0xFFFFFFFF):
            raise ValueError("empty RX must have unset RX timestamps")
    elif first > last or last > elapsed:
        raise ValueError("RX observation timestamps inconsistent")
    return {
        "magic": "DQP1", "version": 1, "flags": flags, "opcode": opcode,
        "target_id": target_id, "attempt": attempt, "address": address,
        "data_length": length, "tx_requested": requested, **counts,
        "first_rx_us_after_tx_observed": None if first == 0xFFFFFFFF else first,
        "last_rx_us_after_tx_observed": None if last == 0xFFFFFFFF else last,
        "window_us": window, "elapsed_us": elapsed, "last_frame_id": last_id,
        "last_status": status, "last_reply_length": reply_length,
        "raw_prefix_hex": payload[64:64 + prefix_length].hex(" ").upper(),
        "position_valid": bool(position_valid),
        "raw_position": position if position_valid else None,
        "raw_speed": speed if position_valid else None,
        "last_unknown_id": None if unknown_id == 255 else unknown_id,
        "known_reply_ids": [sid for i, sid in enumerate(KNOWN_IDS)
                            if counts["known_id_mask"] & (1 << i)],
        "motion_lock": "verify separately using fresh DQS1 flags 0x20 and enabled=false",
    }


def summarize(reports):
    """Interpret only validated reports. No diagnosis from absence alone."""
    def result(category, title, detail):
        return {"category": category, "title": title, "detail": detail}

    if not reports:
        return result("not_received", "尚无本次启动探测报告",
                      "可能尚未部署新版固件或主机未收到报告；不能据此认定舵机故障。")
    if any(r["flags"] & (2 | 8) for r in reports):
        return result("tx_incomplete", "探测含短发送或串口安静期超时",
                      "本次请求未完整发送或未能开始接收窗口，先检查串口状态；不能认定线束接反。")
    if len(reports) < 9:
        return result("collecting", "正在收集启动探测报告",
                      "只探测 ID10，等待 PING、普通读取和单 ID 同步读取各三次的报告。")
    if sum(r["device_error_frames"] for r in reports):
        return result("device_error", "收到设备错误状态应答",
                      "串口识别到了设备应答；查看错误状态和对应 ID，错误帧不表示位置有效。")
    read_ok = any(r["opcode"] == 2 and r["position_valid"] for r in reports)
    sync_ok = any(r["opcode"] == 0x82 and r["position_valid"] for r in reports)
    if read_ok and sync_ok:
        return result("read_ok", "ID10 普通与同步读取均收到有效位置",
                      "仅证明这些启动事务成功；实时 15 个舵机、供电和机械零位仍需分别核验。")
    if read_ok:
        return result("individual_read_only", "普通读取成功，同步读取未获有效位置",
                      "优先排查同步读取兼容性及回复时序；这不是线束接反的证据。")
    if sync_ok:
        return result("sync_read_only", "同步读取成功，普通读取未获有效位置",
                      "已经收到 ID10 的有效位置回复，比较各次事务和错误计数；仍非全机验收。")
    if any(r["opcode"] == 1 and r["valid_frames"] and
           10 in r["known_reply_ids"] and r["last_frame_id"] == 10 and
           r["last_status"] == 0 and r["last_reply_length"] == 2 for r in reports):
        return result("ping_only", "ID10 PING 有应答，位置读取未成功",
                      "PING 不能提供位置；继续查读取指令兼容性、状态及回复时序。")
    if sum(r["valid_frames"] for r in reports):
        return result("replying", "识别到有效帧，但没有 ID10 有效位置",
                      "查看应答 ID、长度和错误状态；其他 ID 的帧不能冒充 ID10 当前位置。")
    if not sum(r["rx_count"] for r in reports):
        return result("no_rx", "请求已发送，未收到串口字节",
                      "仅说明 ID10 在本次波特率和启动窗口未应答；供电、编号、线束及收发电路仍需区分。")
    if sum(r["echoes"] for r in reports):
        return result("echo_only", "仅识别到请求回显，没有设备有效应答",
                      "回显不等于舵机通信成功；同时检查噪声、校验错误和半双工接收路径。")
    return result("rx_unparsed", "收到串口字节，但未解析出有效应答",
                  "查看原始字节、校验和长度；优先区分波特率、回显及电气信号问题。")
