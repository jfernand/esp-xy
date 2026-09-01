//! Bluetooth Low Energy (BLE) communication client using Nordic UART Service (NUS).

use std::sync::Arc;
use std::time::Duration;

use btleplug::api::{
    Central, Characteristic, Manager as _, Peripheral as _, ScanFilter, ValueNotification,
    WriteType,
};
use btleplug::platform::{Adapter, Manager, Peripheral};
use futures::StreamExt;
use tokio::sync::{Mutex, mpsc};
use uuid::{Uuid, uuid};

use resp::{RespDecoder, RespFrame};
use crate::types::{Direction, Mode, Status};

/// Nordic UART Service (NUS) primary service UUID.
pub const NUS_SERVICE_UUID: Uuid = uuid!("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
/// NUS RX Characteristic (Write / Write Without Response, Client -> ESP32-C6).
pub const NUS_RX_CHAR_UUID: Uuid = uuid!("6e400002-b5a3-f393-e0a9-e50e24dcca9e");
/// NUS TX Characteristic (Notify / Read, ESP32-C6 -> Client).
pub const NUS_TX_CHAR_UUID: Uuid = uuid!("6e400003-b5a3-f393-e0a9-e50e24dcca9e");

#[derive(thiserror::Error, Debug)]
pub enum BleError {
    #[error("Bluetooth error: {0}")]
    Btleplug(#[from] btleplug::Error),
    #[error("No Bluetooth adapter found")]
    NoAdapter,
    #[error("Target device not found")]
    DeviceNotFound,
    #[error("NUS characteristic '{0}' not found on device")]
    CharacteristicNotFound(&'static str),
    #[error("Communication timed out")]
    Timeout,
    #[error("Channel closed")]
    ChannelClosed,
    #[error("Protocol error: {0}")]
    Protocol(String),
}

/// Discovered BLE peripheral candidate.
#[derive(Clone, Debug)]
pub struct DiscoveredDevice {
    pub peripheral: Peripheral,
    pub name: String,
    pub address: String,
}

/// High-level Bluetooth client manager.
pub struct BleClient {
    adapter: Adapter,
}

impl BleClient {
    /// Initializes BLE client with the first available Bluetooth adapter.
    pub async fn new() -> Result<Self, BleError> {
        let manager = Manager::new().await?;
        let adapters = manager
            .adapters()
            .await?;
        let adapter = adapters
            .into_iter()
            .next()
            .ok_or(BleError::NoAdapter)?;
        Ok(Self { adapter })
    }

    /// Scans for nearby BLE devices for the given duration.
    pub async fn scan(&self, timeout: Duration) -> Result<Vec<DiscoveredDevice>, BleError> {
        self.adapter
            .start_scan(ScanFilter::default())
            .await?;
        tokio::time::sleep(timeout).await;
        self.adapter
            .stop_scan()
            .await?;

        let peripherals = self
            .adapter
            .peripherals()
            .await?;
        let mut discovered = Vec::new();

        for peripheral in peripherals {
            let properties = peripheral
                .properties()
                .await?
                .unwrap_or_default();
            let name = properties
                .local_name
                .unwrap_or_else(|| "Unknown".to_string());
            let address = peripheral
                .address()
                .to_string();

            // Check if device advertises NUS or matches typical name
            let has_nus = properties
                .services
                .contains(&NUS_SERVICE_UUID);
            if has_nus || name.contains("esp") || name.contains("leadscrew") || name != "Unknown" {
                discovered.push(DiscoveredDevice {
                    peripheral,
                    name,
                    address,
                });
            }
        }

        Ok(discovered)
    }

    /// Connects to a peripheral and negotiates NUS characteristics.
    pub async fn connect(&self, peripheral: &Peripheral) -> Result<BleConnection, BleError> {
        if !peripheral
            .is_connected()
            .await?
        {
            peripheral
                .connect()
                .await?;
        }
        peripheral
            .discover_services()
            .await?;

        let chars = peripheral.characteristics();
        let rx_char = chars
            .iter()
            .find(|c| c.uuid == NUS_RX_CHAR_UUID)
            .cloned()
            .ok_or(BleError::CharacteristicNotFound("NUS RX (Write)"))?;
        let tx_char = chars
            .iter()
            .find(|c| c.uuid == NUS_TX_CHAR_UUID)
            .cloned()
            .ok_or(BleError::CharacteristicNotFound("NUS TX (Notify)"))?;

        peripheral
            .subscribe(&tx_char)
            .await?;
        let notifications = peripheral
            .notifications()
            .await?;

        let (frame_tx, frame_rx) = mpsc::channel(64);

        // Background task to process stream of incoming notifications into RESP frames
        tokio::spawn(async move {
            let mut decoder = RespDecoder::new();
            let mut stream = notifications;
            while let Some(ValueNotification { value, .. }) = stream
                .next()
                .await
            {
                decoder.feed(&value);
                while let Some(frame) = decoder.next_frame() {
                    if frame_tx
                        .send(frame)
                        .await
                        .is_err()
                    {
                        return;
                    }
                }
            }
        });

        Ok(BleConnection {
            peripheral: peripheral.clone(),
            rx_char,
            frame_rx: Arc::new(Mutex::new(frame_rx)),
        })
    }
}

/// Active connected BLE session with the ESP-XY device.
#[derive(Clone)]
pub struct BleConnection {
    peripheral: Peripheral,
    rx_char: Characteristic,
    frame_rx: Arc<Mutex<mpsc::Receiver<RespFrame>>>,
}

impl BleConnection {
    /// Formats a command string to ensure CRLF termination for transmission over BLE.
    pub fn format_raw_command(command: &str) -> Vec<u8> {
        let mut data = command
            .as_bytes()
            .to_vec();
        if !data.ends_with(b"\r\n") {
            data.extend_from_slice(b"\r\n");
        }
        data
    }

    /// Sends a raw command line (auto-appending `\r\n`) to the device.
    pub async fn send_raw(&self, command: &str) -> Result<(), BleError> {
        let data = Self::format_raw_command(command);
        self.peripheral
            .write(&self.rx_char, &data, WriteType::WithoutResponse)
            .await?;
        Ok(())
    }

    /// Sends a command and waits for the next reply frame within timeout.
    pub async fn send_command(
        &self,
        command: &str,
        timeout: Duration,
    ) -> Result<RespFrame, BleError> {
        self.send_raw(command)
            .await?;
        tokio::time::timeout(timeout, self.next_frame())
            .await
            .map_err(|_| BleError::Timeout)?
    }

    /// Receives the next incoming RESP frame (reply or push).
    pub async fn next_frame(&self) -> Result<RespFrame, BleError> {
        let mut rx = self
            .frame_rx
            .lock()
            .await;
        rx.recv()
            .await
            .ok_or(BleError::ChannelClosed)
    }

    /// Queries device status via `STATUS` command.
    pub async fn get_status(&self, timeout: Duration) -> Result<Status, BleError> {
        match self
            .send_command("STATUS", timeout)
            .await?
        {
            RespFrame::Bulk(payload) => Status::parse(&payload).map_err(BleError::Protocol),
            RespFrame::Error(e) => Err(BleError::Protocol(e)),
            _ => Err(BleError::Protocol("unexpected response format".into())),
        }
    }

    /// Sets operating mode (`MODE FEED`, `MODE THREAD`, or `MODE JOG`).
    pub async fn set_mode(&self, mode: Mode, timeout: Duration) -> Result<(), BleError> {
        match self
            .send_command(&format!("MODE {mode}"), timeout)
            .await?
        {
            RespFrame::Ok => Ok(()),
            RespFrame::Error(e) => Err(BleError::Protocol(e)),
            _ => Err(BleError::Protocol("unexpected response format".into())),
        }
    }

    /// Sets electronic pitch ratio in micrometers (`RATIO <um>`).
    pub async fn set_ratio(&self, ratio_um: i64, timeout: Duration) -> Result<(), BleError> {
        match self
            .send_command(&format!("RATIO {ratio_um}"), timeout)
            .await?
        {
            RespFrame::Ok => Ok(()),
            RespFrame::Error(e) => Err(BleError::Protocol(e)),
            _ => Err(BleError::Protocol("unexpected response format".into())),
        }
    }

    /// Sets stepper direction (`DIR FWD` or `DIR REV`).
    pub async fn set_direction(
        &self,
        direction: Direction,
        timeout: Duration,
    ) -> Result<(), BleError> {
        match self
            .send_command(&format!("DIR {direction}"), timeout)
            .await?
        {
            RespFrame::Ok => Ok(()),
            RespFrame::Error(e) => Err(BleError::Protocol(e)),
            _ => Err(BleError::Protocol("unexpected response format".into())),
        }
    }

    /// Enables the stepper drive (`ENABLE`).
    pub async fn enable(&self, timeout: Duration) -> Result<(), BleError> {
        match self
            .send_command("ENABLE", timeout)
            .await?
        {
            RespFrame::Ok => Ok(()),
            RespFrame::Error(e) => Err(BleError::Protocol(e)),
            _ => Err(BleError::Protocol("unexpected response format".into())),
        }
    }

    /// Disables the stepper drive (`DISABLE`).
    pub async fn disable(&self, timeout: Duration) -> Result<(), BleError> {
        match self
            .send_command("DISABLE", timeout)
            .await?
        {
            RespFrame::Ok => Ok(()),
            RespFrame::Error(e) => Err(BleError::Protocol(e)),
            _ => Err(BleError::Protocol("unexpected response format".into())),
        }
    }

    /// Clears any trip/fault state (`CLEAR`).
    pub async fn clear_fault(&self, timeout: Duration) -> Result<(), BleError> {
        match self
            .send_command("CLEAR", timeout)
            .await?
        {
            RespFrame::Ok => Ok(()),
            RespFrame::Error(e) => Err(BleError::Protocol(e)),
            _ => Err(BleError::Protocol("unexpected response format".into())),
        }
    }

    /// Disconnects from the BLE peripheral.
    pub async fn disconnect(&self) -> Result<(), BleError> {
        self.peripheral
            .disconnect()
            .await?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nus_uuid_constants() {
        assert_eq!(
            NUS_SERVICE_UUID,
            Uuid::parse_str("6e400001-b5a3-f393-e0a9-e50e24dcca9e").unwrap()
        );
        assert_eq!(
            NUS_RX_CHAR_UUID,
            Uuid::parse_str("6e400002-b5a3-f393-e0a9-e50e24dcca9e").unwrap()
        );
        assert_eq!(
            NUS_TX_CHAR_UUID,
            Uuid::parse_str("6e400003-b5a3-f393-e0a9-e50e24dcca9e").unwrap()
        );
    }

    #[test]
    fn ble_error_display() {
        assert_eq!(
            BleError::NoAdapter.to_string(),
            "No Bluetooth adapter found"
        );
        assert_eq!(
            BleError::DeviceNotFound.to_string(),
            "Target device not found"
        );
        assert_eq!(
            BleError::CharacteristicNotFound("NUS RX (Write)").to_string(),
            "NUS characteristic 'NUS RX (Write)' not found on device"
        );
        assert_eq!(BleError::Timeout.to_string(), "Communication timed out");
        assert_eq!(BleError::ChannelClosed.to_string(), "Channel closed");
        assert_eq!(
            BleError::Protocol("corrupt frame".to_string()).to_string(),
            "Protocol error: corrupt frame"
        );
    }

    #[test]
    fn format_raw_command() {
        assert_eq!(BleConnection::format_raw_command("STATUS"), b"STATUS\r\n");
        assert_eq!(
            BleConnection::format_raw_command("MODE FEED\r\n"),
            b"MODE FEED\r\n"
        );
        assert_eq!(
            BleConnection::format_raw_command("RATIO 1500"),
            b"RATIO 1500\r\n"
        );
    }
}
