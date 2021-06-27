#![allow(non_snake_case)]

/// The ADC samples at 93696Hz, we want to output at 44100Hz
/// So we need to resample at 1/2.1246, which we'll approximate with
/// 1000/2125 or 8/17 (dividing out 125), giving us a true rate of ~44092Hz
///
/// For interpolation, we insert 7 zeroes after every sample, then we
/// filter everything above Fs/16. The resulting sample rate Fsi = 749,568Hz
///
/// For decimation, we filter out everything above 20kHz (Fsi/38), and take
/// every 17th sample.
///
/// For each output sample, we skip ahead 2 1/8th samples. Thus, with
/// 8 filters, we consume 17 samples (2*8+8/8) and output 8 samples.
///
/// In general terms, with an N-tap filter M[0..N], output O[i] is:
/// O[i] = sum(V[i-N+x] * M[x] for x in 0..N)
/// V[i] = I[i/17] if i%17 == 0
///      = 0 otherwise
///
/// Therefore, with N=50:
/// O[0] = V[-50] * M[0] + V[-33] * M[17] + V[-16] * M[34]
/// O[1] = ?
pub struct PolyphaseFilterBank {
    sample_offset: usize,
    nsamps: usize,
    raw_samples: [f32; 50],
}

const FILTER: [f32; 50] = [
    -1.5571565755041362e-20,
    5.299655672641262e-06,
    3.729570824646748e-05,
    0.00012457763600283632,
    0.00030323871904505245,
    0.0006182758531686576,
    0.0011238702437696656,
    0.0018821463770603017,
    0.0029601834785402945,
    0.004425282087467552,
    0.006338740597132057,
    0.008748640145754124,
    0.011682338016292702,
    0.015139500358883382,
    0.019086542908262744,
    0.02345328251739534,
    0.028132434417186106,
    0.03298233459649817,
    0.03783294946319836,
    0.04249489045854946,
    0.046770818593343866,
    0.050468341909227984,
    0.0534133117654359,
    0.055462336662509264,
    0.05651336783135692,
    0.05651336783135692,
    0.05546233666250927,
    0.05341331176543591,
    0.050468341909227984,
    0.04677081859334388,
    0.04249489045854947,
    0.03783294946319837,
    0.032982334596498186,
    0.028132434417186106,
    0.023453282517395355,
    0.019086542908262744,
    0.015139500358883395,
    0.011682338016292707,
    0.008748640145754133,
    0.006338740597132062,
    0.00442528208746756,
    0.002960183478540299,
    0.0018821463770603017,
    0.0011238702437696673,
    0.0006182758531686576,
    0.0003032387190450536,
    0.0001245776360028364,
    3.7295708246467734e-05,
    5.299655672641164e-06,
    -1.5571565755041362e-20,
];

impl PolyphaseFilterBank {
    pub fn new() -> Self {
        Self {
            sample_offset: 0,
            nsamps: 0,
            raw_samples: [0.0; 50],
        }
    }

    pub fn consume(&mut self, data: &[f32]) -> alloc::vec::Vec<f32> {
        let mut result = vec![];
        data.iter().for_each(|sample| {
            self.raw_samples[(self.sample_offset + self.nsamps) % 50] = *sample;
            self.nsamps = (self.nsamps + 1) % 50;
            while self.nsamps >= 21 {
                let mut i = [0.0; 21];
                for idx in 0..21 {
                    i[idx] = self.raw_samples[(self.sample_offset + idx) % 50];
                }
                self.nsamps -= 17;
                self.sample_offset = (self.sample_offset + 17) % 50;
                let chunk = self.compute(&i);
                for elem in chunk.iter() {
                    result.push(*elem);
                }
            }
        });
        result
    }

    fn compute(&self, I: &[f32; 21]) -> [f32; 8] {
        let F = &FILTER;
        let mut O = [0.0; 8];
        O[0] =
            I[0] * F[7] + I[1] * F[15] + I[2] * F[23] + I[3] * F[31] + I[4] * F[39] + I[5] * F[47];
        O[1] =
            I[2] * F[6] + I[3] * F[14] + I[4] * F[22] + I[5] * F[30] + I[6] * F[38] + I[7] * F[46];
        O[2] =
            I[4] * F[5] + I[5] * F[13] + I[6] * F[21] + I[7] * F[29] + I[8] * F[37] + I[9] * F[45];
        O[3] = I[6] * F[4]
            + I[7] * F[12]
            + I[8] * F[20]
            + I[9] * F[28]
            + I[10] * F[36]
            + I[11] * F[44];
        O[4] = I[8] * F[3]
            + I[9] * F[11]
            + I[10] * F[19]
            + I[11] * F[27]
            + I[12] * F[35]
            + I[13] * F[43];
        O[5] = I[10] * F[2]
            + I[11] * F[10]
            + I[12] * F[18]
            + I[13] * F[26]
            + I[14] * F[34]
            + I[15] * F[42];
        O[6] = I[12] * F[1]
            + I[13] * F[9]
            + I[14] * F[17]
            + I[15] * F[25]
            + I[16] * F[33]
            + I[17] * F[41]
            + I[18] * F[49];
        O[7] = I[14] * F[0]
            + I[15] * F[8]
            + I[16] * F[16]
            + I[17] * F[24]
            + I[18] * F[32]
            + I[19] * F[40]
            + I[20] * F[48];
        O
    }
}
