#![allow(dead_code)]

use modular_bitfield::prelude::*;

#[derive(BitfieldSpecifier)]
#[bits = 2]
pub enum Control {
    NotPresent = 0,
    ReadOnly = 1,
    Invalid = 2,
    ReadWrite = 3,
}

// TODO: Figure out if we can make this a default on the structs,
// so we don't have to manually specify it.
#[derive(BitfieldSpecifier)]
#[bits = 8]
pub enum Subtype {
    Header = 1,
    InputTerminal = 2,
    OutputTerminal = 3,
    ClockSource = 0xA,
}

#[derive(BitfieldSpecifier)]
#[bits = 8]
pub enum StreamingSubtype {
    General = 1,
    FormatType = 2,
}

#[derive(BitfieldSpecifier)]
#[bits = 8]
pub enum EndpointDescriptorSubtype {
    General = 1,
}

#[bitfield]
pub struct AudioClass {
    pub subtype: Subtype, // Fixed at "HEADER" (1)
    pub spec_number_major: B8,
    pub spec_number_minor: B8,
    pub category: B8,
    pub total_length: B16,
    pub latency_control: B2,
    reserved_controls: B6,
}

#[bitfield]
#[derive(BitfieldSpecifier, Clone, Copy, Debug, PartialEq)]
pub struct ChannelConfig {
    pub channels: B31,
    pub raw: bool,
}

#[bitfield]
#[derive(BitfieldSpecifier)]
pub struct InputTerminalControls {
    pub foo: B16,
}

#[bitfield]
pub struct InputTerminal {
    pub subtype: Subtype, // Fixed at "INPUT_TERMINAL" (2)
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

#[bitfield]
pub struct OutputTerminal {
    pub subtype: Subtype, // Fixed at "OUTPUT_TERMINAL"
    pub terminal_id: B8,
    pub terminal_type: B16,
    pub associated_terminal_id: B8,
    pub source_id: B8,
    pub clock_source_id: B8,
    pub copy_protect: Control,
    pub connector: Control,
    pub overload: Control,
    pub underflow: Control,
    pub overflow: Control,
    reserved_controls: B6,
    pub name: B8,
}

#[derive(BitfieldSpecifier)]
#[bits = 2]
pub enum ClockType {
    External,
    InternalFixed,
    InternalVariable,
    InternalProgrammable,
}

#[bitfield]
pub struct ClockSource {
    pub subtype: Subtype, // Fixed at "CLOCK_SOURCE" (0xA)
    pub clock_id: B8,
    pub clock_type: ClockType,
    pub sync_to_sof: bool,
    reserved_attributes: B5,
    pub frequency: Control,
    pub validity: Control,
    reserved_controls: B4,
    pub associated_terminal_id: B8,
    pub name: B8,
}

#[derive(BitfieldSpecifier)]
#[bits = 8]
pub enum FormatType {
    Undefinted = 0,
    Type1 = 1,
    Type2 = 2,
    Type3 = 3,
    Type4 = 4,
    ExtType1 = 0x81,
    ExtType2 = 0x82,
    ExtType3 = 0x83,
}

#[bitfield]
#[derive(BitfieldSpecifier)]
pub struct Formats {
    pub pcm: bool,
    pub pcm8: bool,
    pub ieee_float: bool,
    pub alaw: bool,
    pub mulaw: bool,
    reserved: B26,
    pub type_1_raw_data: bool,
}

#[bitfield]
pub struct AudioStreamingInterface {
    pub subtype: StreamingSubtype,
    pub terminal_id: B8,
    pub active_alternate_setting: Control,
    pub valid_alternate_setting: Control,
    reserved_controls: B4,
    pub format_type: FormatType,
    pub formats: Formats,
    pub num_channels: B8,
    pub channel_config: ChannelConfig,
    pub channel_names: B8,
}

#[bitfield]
pub struct Type1Format {
    pub subtype: StreamingSubtype,
    pub format_type: FormatType,
    pub subslot_size: B8,
    pub bit_resolution: B8,
}

#[derive(BitfieldSpecifier)]
#[bits = 8]
pub enum LockDelayUnits {
    Undefined = 0,
    Milliseconds = 1,
    DecodedPcmSamples = 2,
}

#[bitfield]
pub struct AudioStreamingEndpoint {
    pub subtype: EndpointDescriptorSubtype,
    reserved_attributes: B7,
    pub max_packets_only: bool,
    pub pitch: Control,
    pub data_overrun: Control,
    pub data_underrun: Control,
    reserved_controls: B2,
    pub lock_delay_units: LockDelayUnits,
    pub lock_delay: B16,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn audio_descriptor_correct() {
        let desc = AudioClass::new()
            .with_subtype(Subtype::Header)
            .with_spec_number_major(0)
            .with_spec_number_minor(0)
            .with_category(0xff)
            .with_total_length(8 + 17 * 2 + 12 * 2)
            .with_latency_control(0)
            .into_bytes();
        assert_eq!(
            desc,
            [
                1, // Header
                0x00,
                0x00, // BCD ADC
                0xff, // Category
                8 + 17 * 2 + 12 * 2,
                0x00, // Total length
                0,    // Latency controls
            ]
        );
    }

    #[test]
    fn input_term_descriptor_correct() {
        let desc = InputTerminal::new()
            .with_subtype(Subtype::InputTerminal)
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
        assert_eq!(
            desc,
            [
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
            ]
        );
    }

    #[test]
    fn output_term_descriptor_correct() {
        let desc = OutputTerminal::new()
            .with_subtype(Subtype::OutputTerminal)
            .with_terminal_id(3)
            .with_terminal_type(0x0301)
            .with_associated_terminal_id(0)
            .with_source_id(2)
            .with_clock_source_id(2)
            .with_name(0)
            .into_bytes();
        assert_eq!(
            desc,
            [
                3, // Output terminal
                3, // Terminal ID (unique)
                0x01, 0x03, // Terminal type
                0,    // bAssocTerminal
                2,    // Source ID
                2,    // Clock source
                0, 0, // bmControls
                0, // iTerminal
            ]
        );
    }

    #[test]
    fn clock_source_descriptor_correct() {
        let desc = ClockSource::new()
            .with_subtype(Subtype::ClockSource)
            .with_clock_id(2)
            .with_clock_type(ClockType::External)
            .with_sync_to_sof(false)
            .with_associated_terminal_id(1)
            .with_name(0)
            .into_bytes();
        assert_eq!(
            desc,
            [
                0xA, // CLOCK_SOURCE
                2,   // clock ID
                0,   // bmAttributes (external, not sync'd to SOF)
                0,   // bmControls (no controls present)
                1,   // bAssocTerminal
                0,   // iClockSource
            ]
        );
    }

    #[test]
    fn as_iface_descriptor_correct() {
        let desc = AudioStreamingInterface::new()
            .with_subtype(StreamingSubtype::General)
            .with_terminal_id(1)
            .with_format_type(FormatType::Type1)
            .with_formats(Formats::new().with_pcm(true))
            .with_num_channels(2)
            .with_channel_config(ChannelConfig::new().with_channels(3))
            .with_channel_names(0)
            .into_bytes();
        assert_eq!(
            desc,
            [
                1, // AS_GENERAL
                1, // bTerminalLink (terminal ID of output term)
                0, // bmControls
                1, // bFormatType (PCM)
                0x1, 0x0, 0x0, 0x0, // bmFormats (PCM)
                2,   // bNrChannels
                0x3, 0, 0, 0, // bmChannelConfig
                0, // iChannelNames
            ]
        );
    }

    #[test]
    fn format_type_1_descriptor_correct() {
        let desc = Type1Format::new()
            .with_subtype(StreamingSubtype::FormatType)
            .with_format_type(FormatType::Type1)
            .with_subslot_size(2)
            .with_bit_resolution(16)
            .into_bytes();
        assert_eq!(
            desc,
            [
                0x02, // FORMAT_TYPE
                0x01, // FORMAT_TYPE_I
                2,    // bSubslotSize (1 byte samples)
                16,   // bBitResolution (8 bits in byte used)
            ]
        );
    }
}
