use crate::descriptors::*;

pub enum SpatialChannel {
    FrontLeft,
    FrontRight,
    FrontCenter,
    LowFrequencyEffects,
    BackLeft,
    BackRight,
    FrontLeftOfCenter,
    FrontRightOfCenter,
    BackCenter,
    SideLeft,
    SideRight,
    TopCenter,
    TopFrontLeft,
    TopFrontCenter,
    TopFrontRight,
    TopBackLeft,
    TopBackCenter,
    TopBackRight,
    TopFrontLeftOfCenter,
    TopFrontRightOfCenter,
    LeftLowFrequencyEffects,
    RightLowFrequencyEffects,
    TopSideLeft,
    TopSideRight,
    BottomCenter,
    BackLeftOfCenter,
    BackRightOfCenter,
    Reserved27,
    Reserved28,
    Reserved29,
    Reserved30,
}

pub struct ChannelSet {
    channels: ChannelConfig,
    non_spatial_channels: u8,
}

impl ChannelSet {
    pub fn new_raw(channel_count: u8) -> Self {
        ChannelSet {
            channels: ChannelConfig::new().with_raw(true),
            non_spatial_channels: channel_count,
        }
    }

    pub fn new() -> ChannelSet {
        ChannelSet {
            channels: ChannelConfig::new(),
            non_spatial_channels: 0,
        }
    }

    pub fn with_spatial(mut self, channel: SpatialChannel) -> Self {
        if self.is_raw() {
            panic!("Cannot call with_spatial on raw set");
        }
        let shift = match channel {
            SpatialChannel::FrontLeft => 0,
            SpatialChannel::FrontRight => 1,
            SpatialChannel::FrontCenter => 2,
            SpatialChannel::LowFrequencyEffects => 3,
            SpatialChannel::BackLeft => 4,
            SpatialChannel::BackRight => 5,
            SpatialChannel::FrontLeftOfCenter => 6,
            SpatialChannel::FrontRightOfCenter => 7,
            SpatialChannel::BackCenter => 8,
            SpatialChannel::SideLeft => 9,
            SpatialChannel::SideRight => 10,
            SpatialChannel::TopCenter => 11,
            SpatialChannel::TopFrontLeft => 12,
            SpatialChannel::TopFrontCenter => 13,
            SpatialChannel::TopFrontRight => 14,
            SpatialChannel::TopBackLeft => 15,
            SpatialChannel::TopBackCenter => 16,
            SpatialChannel::TopBackRight => 17,
            SpatialChannel::TopFrontLeftOfCenter => 18,
            SpatialChannel::TopFrontRightOfCenter => 19,
            SpatialChannel::LeftLowFrequencyEffects => 20,
            SpatialChannel::RightLowFrequencyEffects => 21,
            SpatialChannel::TopSideLeft => 22,
            SpatialChannel::TopSideRight => 23,
            SpatialChannel::BottomCenter => 24,
            SpatialChannel::BackLeftOfCenter => 25,
            SpatialChannel::BackRightOfCenter => 26,
            SpatialChannel::Reserved27 => 27,
            SpatialChannel::Reserved28 => 28,
            SpatialChannel::Reserved29 => 29,
            SpatialChannel::Reserved30 => 30,
        };
        self.channels
            .set_channels(self.channels.channels() | (1 << shift));
        self
    }

    pub fn with_nonspatial(mut self, count: u8) -> Self {
        self.non_spatial_channels = count;
        self
    }

    pub fn config(&self) -> ChannelConfig {
        self.channels.clone()
    }

    pub fn config_raw(&self) -> u32 {
        u32::from_le_bytes(self.channels.into_bytes())
    }

    pub fn count(&self) -> u8 {
        self.channels.channels().count_ones() as u8 + self.non_spatial_channels
    }

    fn is_raw(&self) -> bool {
        self.channels.raw()
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn raw_channelset() {
        let cs = ChannelSet::new_raw(5);
        assert_eq!(cs.count(), 5);
        assert_eq!(cs.config_raw(), 0x8000_0000);
    }

    #[test]
    fn set_channels() {
        let cs = ChannelSet::new()
            .with_spatial(SpatialChannel::FrontLeft)
            .with_spatial(SpatialChannel::FrontRight)
            .with_spatial(SpatialChannel::TopSideLeft);
        assert_eq!(cs.count(), 3);
        assert_eq!(cs.config_raw(), 0x0040_0003);
    }

    #[test]
    fn nonspatial() {
        let cs = ChannelSet::new()
            .with_spatial(SpatialChannel::FrontLeft)
            .with_spatial(SpatialChannel::FrontRight)
            .with_spatial(SpatialChannel::TopSideLeft)
            .with_nonspatial(5);
        assert_eq!(cs.count(), 8);
        assert_eq!(cs.config_raw(), 0x0040_0003);
    }
}
