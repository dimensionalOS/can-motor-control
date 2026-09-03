//! Native macOS transport for adapters implementing the gs_usb protocol.

mod protocol;
mod selection;
mod timing;
mod worker_core;

#[cfg(any(target_os = "macos", test))]
mod macos;

#[cfg(target_os = "macos")]
pub use macos::{list_gs_usb_devices, GsUsbBus, GsUsbConfig, GsUsbDeviceInfo};
#[cfg(target_os = "macos")]
pub use worker_core::GsUsbStatistics;

#[cfg(target_os = "macos")]
mod nusb_api {
    use std::time::Duration;

    use nusb::transfer::{Bulk, In, Out};
    use nusb::{Endpoint, Interface, MaybeFuture};

    /// Compile-time proof of the pinned nusb lifecycle operations used by the
    /// worker. Interface release is deliberately RAII: every endpoint and
    /// interface clone must be dropped after cancelled completions are
    /// collected.
    #[allow(dead_code)]
    fn verify_endpoint_lifecycle_api(
        interface: Interface,
        mut input: Endpoint<Bulk, In>,
        mut output: Endpoint<Bulk, Out>,
    ) -> Result<(), nusb::Error> {
        let request_size = input.max_packet_size();
        let mut input_buffer = input.allocate(request_size);
        input_buffer.set_requested_len(request_size);
        input.submit(input_buffer);

        let mut output_buffer = output.allocate(20);
        output_buffer.extend_from_slice(&[0_u8; 20]);
        output.submit(output_buffer);

        let _ = input.wait_next_complete(Duration::ZERO);
        let _ = output.wait_next_complete(Duration::ZERO);

        input.cancel_all();
        output.cancel_all();
        while input.pending() != 0 {
            let _ = input.wait_next_complete(Duration::from_secs(1));
        }
        while output.pending() != 0 {
            let _ = output.wait_next_complete(Duration::from_secs(1));
        }
        input.clear_halt().wait()?;
        output.clear_halt().wait()?;

        drop(input);
        drop(output);
        drop(interface);
        Ok(())
    }
}
