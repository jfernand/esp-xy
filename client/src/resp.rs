//! RESP frame decoding over BLE notification stream.

/// A parsed incoming RESP frame from the device.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RespFrame {
    /// Command accepted (`+OK\r\n`)
    Ok,
    /// Command error (`-ERR <message>\r\n`)
    Error(String),
    /// Bulk string reply to query (`$<len>\r\n<payload>\r\n`)
    Bulk(String),
    /// Unsolicited asynchronous push (`><len>\r\n<payload>\r\n`)
    Push(String),
}

/// Buffer and parser for incoming bytes over BLE notifications.
#[derive(Debug, Default)]
pub struct RespDecoder {
    buffer: Vec<u8>,
}

impl RespDecoder {
    pub fn new() -> Self {
        Self { buffer: Vec::new() }
    }

    /// Appends received bytes to internal buffer.
    pub fn feed(&mut self, bytes: &[u8]) {
        self.buffer
            .extend_from_slice(bytes);
    }

    /// Attempts to extract the next complete [`RespFrame`] from the buffer.
    pub fn next_frame(&mut self) -> Option<RespFrame> {
        loop {
            if self
                .buffer
                .is_empty()
            {
                return None;
            }

            match self.buffer[0] {
                b'+' => {
                    let end = self.find_crlf(0)?;
                    let _line = std::str::from_utf8(&self.buffer[1..end]).ok()?;
                    self.consume(end + 2);
                    return Some(RespFrame::Ok);
                }
                b'-' => {
                    let end = self.find_crlf(0)?;
                    let line = std::str::from_utf8(&self.buffer[1..end]).ok()?;
                    let err_msg = line
                        .strip_prefix("ERR ")
                        .unwrap_or(line)
                        .to_string();
                    self.consume(end + 2);
                    return Some(RespFrame::Error(err_msg));
                }
                b'$' | b'>' => {
                    let is_push = self.buffer[0] == b'>';
                    let header_end = self.find_crlf(0)?;
                    let len_str = std::str::from_utf8(&self.buffer[1..header_end]).ok();
                    let payload_len: usize = match len_str.and_then(|s| {
                        s.parse()
                            .ok()
                    }) {
                        Some(len) => len,
                        None => {
                            // Malformed length header, drop corrupt byte and retry
                            self.consume(1);
                            continue;
                        }
                    };

                    let payload_start = header_end + 2;
                    let payload_end = payload_start + payload_len;
                    let total_end = payload_end + 2; // includes trailing CRLF

                    if self
                        .buffer
                        .len()
                        < total_end
                    {
                        return None; // Need more bytes
                    }

                    if &self.buffer[payload_end..total_end] != b"\r\n" {
                        // Framing mismatch, drop the corrupt byte and try recovery
                        self.consume(1);
                        continue;
                    }

                    let payload = std::str::from_utf8(&self.buffer[payload_start..payload_end])
                        .ok()?
                        .to_string();
                    self.consume(total_end);

                    if is_push {
                        return Some(RespFrame::Push(payload));
                    } else {
                        return Some(RespFrame::Bulk(payload));
                    }
                }
                _ => {
                    // Skip invalid byte to re-synchronize
                    self.consume(1);
                }
            }
        }
    }

    fn find_crlf(&self, start: usize) -> Option<usize> {
        self.buffer[start..]
            .windows(2)
            .position(|w| w == b"\r\n")
            .map(|pos| start + pos)
    }

    fn consume(&mut self, count: usize) {
        if count
            >= self
                .buffer
                .len()
        {
            self.buffer
                .clear();
        } else {
            self.buffer
                .drain(..count);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decode_ok_and_err() {
        let mut decoder = RespDecoder::new();
        decoder.feed(b"+OK\r\n-ERR busy\r\n-other error\r\n");

        assert_eq!(decoder.next_frame(), Some(RespFrame::Ok));
        assert_eq!(
            decoder.next_frame(),
            Some(RespFrame::Error("busy".to_string()))
        );
        assert_eq!(
            decoder.next_frame(),
            Some(RespFrame::Error("other error".to_string()))
        );
        assert_eq!(decoder.next_frame(), None);
    }

    #[test]
    fn decode_bulk_and_push_across_chunks() {
        let mut decoder = RespDecoder::new();
        decoder.feed(b"$9\r\nenab");
        assert_eq!(decoder.next_frame(), None);

        decoder.feed(b"led 1\r\n>8\r\npos 1234\r\n");
        assert_eq!(
            decoder.next_frame(),
            Some(RespFrame::Bulk("enabled 1".to_string()))
        );
        assert_eq!(
            decoder.next_frame(),
            Some(RespFrame::Push("pos 1234".to_string()))
        );
        assert_eq!(decoder.next_frame(), None);
    }

    #[test]
    fn decode_empty_bulk_and_push() {
        let mut decoder = RespDecoder::new();
        decoder.feed(b"$0\r\n\r\n>0\r\n\r\n");

        assert_eq!(decoder.next_frame(), Some(RespFrame::Bulk(String::new())));
        assert_eq!(decoder.next_frame(), Some(RespFrame::Push(String::new())));
        assert_eq!(decoder.next_frame(), None);
    }

    #[test]
    fn decode_byte_by_byte_stream() {
        let stream = b"+OK\r\n$5\r\nhello\r\n>4\r\npush\r\n-ERR fail\r\n";
        let mut decoder = RespDecoder::new();
        let mut frames = Vec::new();

        for byte in stream {
            decoder.feed(&[*byte]);
            while let Some(frame) = decoder.next_frame() {
                frames.push(frame);
            }
        }

        assert_eq!(
            frames,
            vec![
                RespFrame::Ok,
                RespFrame::Bulk("hello".to_string()),
                RespFrame::Push("push".to_string()),
                RespFrame::Error("fail".to_string()),
            ]
        );
    }

    #[test]
    fn decode_noise_and_corruption_recovery() {
        let mut decoder = RespDecoder::new();
        // Leading garbage before valid OK frame
        decoder.feed(b"junk1234+OK\r\n");
        assert_eq!(decoder.next_frame(), Some(RespFrame::Ok));

        // Corrupt frame: declared length 4 but missing CRLF at expected position
        decoder.feed(b"$4\r\ntestNO_CRLF+OK\r\n");
        // The parser should skip corrupt bytes and recover to parse +OK
        let mut recovered_frames = Vec::new();
        while let Some(frame) = decoder.next_frame() {
            recovered_frames.push(frame);
        }
        assert_eq!(recovered_frames, vec![RespFrame::Ok]);
    }
}
