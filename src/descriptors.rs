#![allow(dead_code)]

use modular_bitfield::prelude::*;

#[bitfield]
pub struct AudioClass {
    pub subtype: B8, // Fixed at "HEADER" (1)
    pub spec_number_major: B8,
    pub spec_number_minor: B8,
    pub category: B8,
    pub total_length: B16,
    pub latency_control: B2,
    reserved_controls: B6,
}

#[bitfield]
#[derive(BitfieldSpecifier)]
pub struct ChannelConfig {
    pub channels: B32,
}

#[bitfield]
#[derive(BitfieldSpecifier)]
pub struct InputTerminalControls {
    pub foo: B16,
}

#[bitfield]
pub struct InputTerminal {
    pub subtype: B8, // Fixed at "INPUT_TERMINAL" (2)
    pub terminal_id: B8,
    pub terminal_type: B16,
    pub associated_terminal_id: B8,
    pub clock_source_id: B8,
    pub num_channels: B8,
    pub channel_config: ChannelConfig,
    pub channel_names: B8, // Index of a string descriptor
    pub controls: InputTerminalControls,
    pub terminal: B8,
}

/*
3, // Output terminal
3, // Terminal ID (unique)
0x01, 0x03, // Terminal type
0, // bAssocTerminal
2, // Source ID
2, // Clock source
0, 0, // bmControls
0, // iTerminal
*/
#[bitfield]
pub struct OutputTerminal {
    pub foo: B8,
}

#[derive(BitfieldSpecifier)]
#[bits = 2]
pub enum ClockType {
    External,
    InternalFixed,
    InternalVariable,
    InternalProgrammable,
}

/*
                0xA, // CLOCK_SOURCE
                2, // clock ID
                0, // bmAttributes (external, not sync'd to SOF)
                0, // bmControls (no controls present)
                1, // bAssocTerminal
                0, // iClockSource
*/
#[bitfield]
pub struct ClockSource {
    pub subtype: B8, // Fixed at "CLOCK_SOURCE" (0xA)
    pub clock_id: B8,
    pub clock_type: ClockType,
    pub sync_to_sof: bool,
    reserved_attributes: B5,
    pub associated_terminal_id: B8,
    pub name: B8,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn audio_descriptor_correct() {
        let desc = AudioClass::new()
            .with_subtype(1)
            .with_spec_number_major(0)
            .with_spec_number_minor(0)
            .with_category(0xff)
            .with_total_length(8 + 17*2 + 12*2)
            .with_latency_control(0)
            .into_bytes();
        assert_eq!(desc, [
            1, // Header
            0x00, 0x00, // BCD ADC
            0xff, // Category
            8 + 17*2 + 12*2, 0x00, // Total length
            0,    // Latency controls
        ]);
    }

    #[test]
    fn input_term_descriptor_correct() {
        let desc = InputTerminal::new()
            .with_subtype(2)
            .with_terminal_id(1)
            .with_terminal_type(0x0101)
            .with_associated_terminal_id(0)
            .with_clock_source_id(2)
            .with_num_channels(2)
            .with_channel_config(ChannelConfig::new().with_channels(3))
            .with_channel_names(0)
            .with_controls(InputTerminalControls::new())
            .with_terminal(0)
            .into_bytes();
        assert_eq!(desc, [
            2, // Input terminal
            1, // Terminal ID (unique)
            0x01, 0x01, // Terminal type
            0,    // bAssocTerminal
            2,    // bCSourceID
            2,    // bNrChannels
            0x3, 0, 0, 0, // bmChannelConfig (front left, front right)
            0, // iChannelNames
            0, 0, // bmControls
            0, // iTerminal
        ]);
    }
}
