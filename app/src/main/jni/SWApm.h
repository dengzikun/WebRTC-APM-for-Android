#ifndef SWAPM_H__
#define SWAPM_H__

class ApmWrapper;

namespace SWAudioProcessingModule{


	
	class Apm{
		
		public:
			enum class AEC_SuppressionLevel{
				LowSuppression,
				ModerateSuppression,
				HighSuppression
			};
			
			// Recommended settings for particular audio routes. In general, the louder
			// the echo is expected to be, the higher this value should be set. The
			// preferred setting may vary from device to device.

			enum class AECM_RoutingMode {
				QuietEarpieceOrHeadset,
				Earpiece,
				LoudEarpiece,
				Speakerphone,
				LoudSpeakerphone
			};


 		    enum class AGC_Mode {
				// Adaptive mode intended for use if an analog volume control is available
				// on the capture device. It will require the user to provide coupling
				// between the OS mixer controls and AGC through the |stream_analog_level()|
				// functions.
				//
				// It consists of an analog gain prescription for the audio device and a
				// digital compression stage.
				AdaptiveAnalog,

				// Adaptive mode intended for situations in which an analog volume control
				// is unavailable. It operates in a similar fashion to the adaptive analog
				// mode, but with scaling instead applied in the digital domain. As with
				// the analog mode, it additionally uses a digital compression stage.
				AdaptiveDigital,

				// Fixed mode which enables only the digital compression stage also used by
				// the two adaptive modes.
				//
				// It is distinguished from the adaptive modes by considering only a
				// short time-window of the input signal. It applies a fixed gain through
				// most of the input level range, and compresses (gradually reduces gain
				// with increasing level) the input signal at higher levels. This mode is
				// preferred on embedded devices where the capture signal level is
				// predictable, so that a known gain can be applied.
				FixedDigital
		    };


			// Determines the aggressiveness of the suppression. Increasing the level
			// will reduce the noise level at the expense of a higher speech distortion.
			enum class NS_Level {
				Low,
				Moderate,
				High,
				VeryHigh
			};

			// Specifies the likelihood that a frame will be declared to contain voice.
			// A higher value makes it more likely that speech will not be clipped, at
			// the expense of more noise being detected as voice.
			enum class VAD_Likelihood {
				VeryLowLikelihood,
				LowLikelihood,
				ModerateLikelihood,
				HighLikelihood
			};
		
		public:
			Apm(bool aecExtendFilter, bool speechIntelligibilityEnhance, bool delayAgnostic, bool beamforming, bool nextGenerationAec, bool experimentalNs, bool experimentalAgc);
			~Apm();
			
			int ProcessStream(int16_t* data); //本地数据 // 16K, 16Bits, 单声道， 10ms
			int ProcessReverseStream(int16_t* data); // 远端数据 // 16K, 16Bits, 单声道， 10ms
			int SetStreamDelay(int delay_ms);
			
			int HighPassFilter(bool enable);
			
			//AEC PC
			int AECClockDriftCompensation(bool enable);
			int AECSetSuppressionLevel(AEC_SuppressionLevel level); //[0, 1, 2]
			int AEC(bool enable);
			
			// AEC Mobile
			int AECMSetSuppressionLevel(AECM_RoutingMode level);//[0, 1, 2, 3, 4]
			int AECM(bool enable);
			
			// NS
			int NS(bool enable);
			int NSSetLevel(NS_Level level);// [0, 1, 2, 3]
			
			// AGC
			int AGC(bool enable);
			
			// Sets the |minimum| and |maximum| analog levels of the audio capture device.
			// Must be set if and only if an analog mode is used. Limited to [0, 65535].
			int AGCSetAnalogLevelLimits(int minimum, int maximum); // limit to [0, 65535]
			int AGCSetMode(AGC_Mode mode); // [0, 1, 2]
			
			// Sets the target peak |level| (or envelope) of the AGC in dBFs (decibels
			// from digital full-scale). The convention is to use positive values. For
			// instance, passing in a value of 3 corresponds to -3 dBFs, or a target
			// level 3 dB below full-scale. Limited to [0, 31].
			int AGCSetTargetLevelDbfs(int level);
			
			// Sets the maximum |gain| the digital compression stage may apply, in dB. A
			// higher number corresponds to greater compression, while a value of 0 will
			// leave the signal uncompressed. Limited to [0, 90].
			int AGCSetcompressionGainDb(int gain);
			
			// When enabled, the compression stage will hard limit the signal to the
			// target level. Otherwise, the signal will be compressed but not limited
			// above the target level.
			int AGCEnableLimiter(bool enable);
			
			// When an analog mode is set, this must be called prior to |ProcessStream()|
			// to pass the current analog level from the audio HAL. Must be within the
			// range provided to |set_analog_level_limits()|.
			int AGCSetStreamAnalogLevel(int level);
			int AGCStreamAnalogLevel();
			
			
			// VAD
			int VAD(bool enable);
			
			int VADSetLikeHood(VAD_Likelihood likelihood);
			bool VADHasVoice();
		
		private:
			ApmWrapper *impl;
	};
	
}


#endif