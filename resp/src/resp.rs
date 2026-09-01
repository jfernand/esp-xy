//! A small RESP-inspired (Redis-protocol-style) framing for a command/status link.
//!
//! Requests are inline commands: plain ASCII, space-separated, CRLF-terminated (the same
//! "inline command" fallback Redis itself accepts over telnet) -- so any dumb terminal that
//! can send text works as a client with no protocol library of its own. Replies and pushes are
//! RESP-typed so a real client can tell them apart unambiguously instead of scraping text:
//!
//! - `+OK\r\n` -- a command was accepted
//! - `-ERR <message>\r\n` -- a command was rejected
//! - `$<len>\r\n<payload>\r\n` -- a bulk-string reply to a query command
//! - `><len>\r\n<payload>\r\n` -- an unsolicited push (borrowed from RESP3's dedicated push
//!   type), so a client can tell "you asked for this" apart from "this just showed up"
//!
//! This module only knows about bytes -- no BLE, no UART -- so the same encoding/parsing is
//! reusable regardless of what carries the bytes.

/// Maximum tokens in one inline command line (command name + arguments).
pub const MAX_TOKENS: usize = 4;

/// One parsed inline command: up to [`MAX_TOKENS`] whitespace-separated fields from a line,
/// borrowed from the caller's line buffer.
pub struct Command<'a> {
    tokens: [&'a str; MAX_TOKENS],
    count: usize,
}

impl<'a> Command<'a> {
    /// Parses a line (without its terminating CR/LF) into whitespace-separated tokens.
    /// Extra tokens beyond [`MAX_TOKENS`] are silently dropped.
    pub fn parse(line: &'a str) -> Self {
        let mut tokens = [""; MAX_TOKENS];
        let mut count = 0;
        for token in line.split_ascii_whitespace() {
            if count == MAX_TOKENS {
                break;
            }
            tokens[count] = token;
            count += 1;
        }
        Self { tokens, count }
    }

    /// The command name (first token), uppercased comparisons are the caller's job since we
    /// don't allocate here.
    pub fn name(&self) -> &'a str {
        self.tokens
            .first()
            .copied()
            .unwrap_or("")
    }

    /// Positional argument after the command name (0-indexed).
    pub fn arg(&self, index: usize) -> Option<&'a str> {
        self.tokens
            .get(index + 1)
            .copied()
            .filter(|s| !s.is_empty())
    }

    /// Total token count, including the command name.
    pub fn len(&self) -> usize {
        self.count
    }

    pub fn is_empty(&self) -> bool {
        self.count == 0
    }
}

/// Writes `+OK\r\n` into `buf`, returning the number of bytes written, or `None` if `buf` is
/// too small.
pub fn write_ok(buf: &mut [u8]) -> Option<usize> {
    write_bytes(buf, b"+OK\r\n")
}

/// Writes `-ERR <message>\r\n` into `buf`, returning the number of bytes written, or `None` if
/// `buf` is too small.
pub fn write_err(buf: &mut [u8], message: &str) -> Option<usize> {
    let mut written = write_bytes(buf, b"-ERR ")?;
    written += write_bytes(&mut buf[written..], message.as_bytes())?;
    written += write_bytes(&mut buf[written..], b"\r\n")?;
    Some(written)
}

/// Writes a bulk-string reply (`$<len>\r\n<payload>\r\n`) into `buf`.
pub fn write_bulk(buf: &mut [u8], payload: &str) -> Option<usize> {
    write_framed(buf, b'$', payload)
}

/// Writes a push frame (`><len>\r\n<payload>\r\n`) into `buf`.
pub fn write_push(buf: &mut [u8], payload: &str) -> Option<usize> {
    write_framed(buf, b'>', payload)
}

fn write_framed(buf: &mut [u8], prefix: u8, payload: &str) -> Option<usize> {
    let mut written = write_bytes(buf, &[prefix])?;
    written += write_decimal(&mut buf[written..], payload.len() as u32)?;
    written += write_bytes(&mut buf[written..], b"\r\n")?;
    written += write_bytes(&mut buf[written..], payload.as_bytes())?;
    written += write_bytes(&mut buf[written..], b"\r\n")?;
    Some(written)
}

fn write_bytes(buf: &mut [u8], bytes: &[u8]) -> Option<usize> {
    if bytes.len() > buf.len() {
        return None;
    }
    buf[..bytes.len()].copy_from_slice(bytes);
    Some(bytes.len())
}

fn write_decimal(buf: &mut [u8], mut value: u32) -> Option<usize> {
    let mut digits = [0u8; 10];
    let mut count = 0;
    loop {
        digits[count] = b'0' + (value % 10) as u8;
        count += 1;
        value /= 10;
        if value == 0 {
            break;
        }
    }
    if count > buf.len() {
        return None;
    }
    for i in 0..count {
        buf[i] = digits[count - 1 - i];
    }
    Some(count)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_commands() {
        let cmd = Command::parse("MODE FEED");
        assert_eq!(cmd.name(), "MODE");
        assert_eq!(cmd.arg(0), Some("FEED"));
        assert_eq!(cmd.arg(1), None);
        assert_eq!(cmd.len(), 2);
        assert!(!cmd.is_empty());

        let cmd_empty = Command::parse("");
        assert_eq!(cmd_empty.name(), "");
        assert_eq!(cmd_empty.arg(0), None);
        assert_eq!(cmd_empty.len(), 0);
        assert!(cmd_empty.is_empty());
    }

    #[test]
    fn framing_outputs() {
        let mut buf = [0u8; 64];

        let n = write_ok(&mut buf).unwrap();
        assert_eq!(&buf[..n], b"+OK\r\n");

        let n = write_err(&mut buf, "busy").unwrap();
        assert_eq!(&buf[..n], b"-ERR busy\r\n");

        let n = write_bulk(&mut buf, "enabled 1").unwrap();
        assert_eq!(&buf[..n], b"$9\r\nenabled 1\r\n");

        let n = write_push(&mut buf, "pos 1234").unwrap();
        assert_eq!(&buf[..n], b">8\r\npos 1234\r\n");
    }
}
