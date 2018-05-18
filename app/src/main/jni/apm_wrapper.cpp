/**
 * These are wrappers of native webrtc echo cancellation mobile edition functions.
 *
 *@author billhoo E-mail: billhoo@126.com
 *@version 0.1 2013-3-8
 */




#include <jni.h>


#include <stdlib.h> // for NULL
#include <assert.h>
#include <stddef.h>
#include <unistd.h>
#include <memory>

#include <android/log.h>
#define TAG "APM"
#define LOGV(...) __android_log_print(ANDROID_LOG_VERBOSE, TAG, __VA_ARGS__)

#include "webrtc/modules/audio_processing/include/audio_processing.h"
#include "webrtc/modules/include/module_common_types.h"
#include "webrtc/common_audio/channel_buffer.h"
#include "webrtc/common_audio/include/audio_util.h"
#include "webrtc/common.h"
//#include "webrtc/modules/audio_processing/beamformer/mock_nonlinear_beamformer.h"

#include "com_sinowave_ddp_Apm.h"



#include "SWApm.h"

using namespace std;
using namespace webrtc;


static void set_ctx(JNIEnv *env, jobject thiz, void *ctx) {
    jclass cls = env->GetObjectClass(thiz);
    jfieldID fid = env->GetFieldID(cls, "objData", "J");
    env->SetLongField(thiz, fid, (jlong)ctx);
}

static void *get_ctx(JNIEnv *env, jobject thiz) {
    jclass cls = env->GetObjectClass(thiz);
    jfieldID fid = env->GetFieldID(cls, "objData", "J");
    return (void*)env->GetLongField(thiz, fid);
}


class ApmWrapper{
    const int sample_rate_hz = AudioProcessing::kSampleRate16kHz;
    const int num_input_channels = 1;

    const int reverse_sample_rate_hz = AudioProcessing::kSampleRate16kHz;
    const int num_reverse_channels = 1;

public:
    ApmWrapper(bool aecExtendFilter, bool speechIntelligibilityEnhance, bool delayAgnostic, bool beamforming, bool nextGenerationAec, bool experimentalNs, bool experimentalAgc){

        _beamForming = beamforming;

        Config config;
        config.Set<ExtendedFilter>(new ExtendedFilter(aecExtendFilter));
        config.Set<Intelligibility>(new Intelligibility(speechIntelligibilityEnhance));
        config.Set<DelayAgnostic>(new DelayAgnostic(delayAgnostic));
/*
        MockNonlinearBeamformer* beamFormer = nullptr;
        if(beamforming) {
            std::vector<webrtc::Point> geometry;
            geometry.push_back(webrtc::Point(0.f, 0.f, 0.f));
            geometry.push_back(webrtc::Point(0.05f, 0.f, 0.f));
            config.Set<Beamforming>(new Beamforming(beamforming, geometry));
            beamFormer = new MockNonlinearBeamformer(geometry);
        }*/

        config.Set<NextGenerationAec>(new NextGenerationAec(nextGenerationAec));
        config.Set<ExperimentalNs>(new ExperimentalNs(experimentalNs));
        config.Set<ExperimentalAgc>(new ExperimentalAgc(experimentalAgc));

        _apm.reset(AudioProcessing::Create(config));

        /*
        if(beamforming) {
            _apm.reset(AudioProcessing::Create(config, beamFormer));
        }else {
            _apm.reset(AudioProcessing::Create(config));
        }*/

        _frame = new AudioFrame();
        _reverseFrame = new AudioFrame();

        SetContainerFormat(sample_rate_hz, num_input_channels, _frame, &_float_cb);

        SetContainerFormat(reverse_sample_rate_hz, num_reverse_channels, _reverseFrame,
                           &_revfloat_cb);

        _apm->Initialize({{{_frame->sample_rate_hz_, _frame->num_channels_},
                         {_frame->sample_rate_hz_, _frame->num_channels_},
                         {_reverseFrame->sample_rate_hz_, _reverseFrame->num_channels_},
                         {_reverseFrame->sample_rate_hz_, _reverseFrame->num_channels_}}});
    }

    ~ApmWrapper(){
        delete _frame;
        delete _reverseFrame;
    }

    int ProcessStream(int16_t* data){
        std::copy(data, data + _frame->samples_per_channel_, _frame->data_);
//        ConvertToFloat(*_frame, _float_cb.get());
        int ret = _apm->ProcessStream(_frame);
        std::copy(_frame->data_, _frame->data_ + _frame->samples_per_channel_, data);
        return ret;
    }

    int ProcessReverseStream(int16_t* data){
        std::copy(data, data + _reverseFrame->samples_per_channel_, _reverseFrame->data_);
//        ConvertToFloat(*_reverseFrame, _revfloat_cb.get());
        int ret = _apm->ProcessReverseStream(_reverseFrame);
        if(_beamForming){
            std::copy(_reverseFrame->data_, _reverseFrame->data_ + _reverseFrame->samples_per_channel_, data);
        }
        return ret;
//        return _apm->AnalyzeReverseStream(_reverseFrame);
    }


    void SetDirectBuffer_C(void *buffer, size_t size){
        mDirectBufferAddress_C = buffer;
        mDirectBufferCapacityBytes_C = size;
    }

    void SetDirectBuffer_R(void *buffer, size_t size){
        mDirectBufferAddress_R = buffer;
        mDirectBufferCapacityBytes_R = size;
    }

    int ProcessStream(){
        auto src = (short*)mDirectBufferAddress_C;
        std::copy(src, src + _frame->samples_per_channel_, _frame->data_);
//        ConvertToFloat(*_frame, _float_cb.get());
        int ret = _apm->ProcessStream(_frame);
        std::copy(_frame->data_, _frame->data_ + _frame->samples_per_channel_, src);
        return ret;
    }

    int ProcessReverseStream(){
        auto src = (short*)mDirectBufferAddress_R;
        std::copy(src, src + _reverseFrame->samples_per_channel_, _reverseFrame->data_);
//        ConvertToFloat(*_reverseFrame, _revfloat_cb.get());
        int ret = _apm->ProcessReverseStream(_reverseFrame);
        if(_beamForming){
            std::copy(_reverseFrame->data_, _reverseFrame->data_ + _reverseFrame->samples_per_channel_, src);
        }
        return ret;
//        return _apm->AnalyzeReverseStream(_reverseFrame);
    }

// jni interface
private:

    static jboolean JNICALL Create(JNIEnv *env, jobject thiz, jboolean aecExtendFilter, jboolean speechIntelligibilityEnhance, jboolean delayAgnostic, jboolean beamforming, jboolean nextGenerationAec, jboolean experimentalNs, jboolean experimentalAgc) {
        ApmWrapper* apm = new ApmWrapper(aecExtendFilter, speechIntelligibilityEnhance, delayAgnostic, beamforming, nextGenerationAec, experimentalNs, experimentalAgc);

        if (apm == nullptr)
            return JNI_FALSE;
        else {
            set_ctx(env, thiz, apm);
            LOGV("created");
            return JNI_TRUE;
        }
    }

    static void JNICALL Destroy(JNIEnv *env, jobject thiz) {
        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);
        delete apm;
        LOGV("destroyed");
    }

    static jint JNICALL high_pass_filter_enable(JNIEnv *env, jobject thiz, jboolean enable){

        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);
        return apm->_apm->high_pass_filter()->Enable(enable);
    }

    static jint JNICALL aec_enable(JNIEnv *env, jobject thiz, jboolean enable){

        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);
        return apm->_apm->echo_cancellation()->Enable(enable);
    }


    static jint JNICALL aec_clock_drift_compensation_enable
            (JNIEnv *env, jobject thiz, jboolean enable){

        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);
        return apm->_apm->echo_cancellation()->enable_drift_compensation(enable);

    }

    static jint JNICALL aec_set_suppression_level(JNIEnv *env, jobject thiz, jint level){

        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);
        if ( level < EchoCancellation::kLowSuppression ){
            level = EchoCancellation::kLowSuppression;
        }else if(level > EchoCancellation::kHighSuppression){
            level = EchoCancellation::kHighSuppression;
        }
        return apm->_apm->echo_cancellation()->set_suppression_level((EchoCancellation::SuppressionLevel)level);
    }

    static jint JNICALL aecm_enable(JNIEnv *env, jobject thiz, jboolean enable){

        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);
        return apm->_apm->echo_control_mobile()->Enable(enable);
    }

    static jint JNICALL aecm_set_suppression_level(JNIEnv *env, jobject thiz, jint level){

        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);
        if(level < EchoControlMobile::kQuietEarpieceOrHeadset){
            level = EchoControlMobile::kQuietEarpieceOrHeadset;
        }else if(level > EchoControlMobile::kLoudSpeakerphone){
            level = EchoControlMobile::kLoudSpeakerphone;
        }
        return apm->_apm->echo_control_mobile()->set_routing_mode((EchoControlMobile::RoutingMode)level);

    }

    static jint JNICALL ns_enable(JNIEnv *env, jobject thiz, jboolean enable){

        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);
        return apm->_apm->noise_suppression()->Enable(enable);
    }

    static jint JNICALL ns_set_level(JNIEnv *env, jobject thiz, jint level){

        if(level < NoiseSuppression::kLow){
            level = NoiseSuppression::kLow;
        }else if(level > NoiseSuppression::kVeryHigh){
            level = NoiseSuppression::kVeryHigh;
        }

        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);
        return apm->_apm->noise_suppression()->set_level((NoiseSuppression::Level)level);
    }


    static int JNICALL agc_enable(JNIEnv *env, jobject thiz, jboolean enable){

        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);
        return apm->_apm->gain_control()->Enable(enable);

    }

    static jint JNICALL agc_set_target_level_dbfs(JNIEnv *env, jobject thiz, jint level){

        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);
        return apm->_apm->gain_control()->set_target_level_dbfs(level);
    }

    static jint JNICALL agc_set_compression_gain_db(JNIEnv *env, jobject thiz, jint gain){
        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);
        return apm->_apm->gain_control()->set_compression_gain_db(gain);
    }


    static jint JNICALL agc_enable_limiter(JNIEnv *env, jobject thiz, jboolean enable){
        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);
        return apm->_apm->gain_control()->enable_limiter(enable);
    }

    static jint JNICALL agc_set_analog_level_limits(JNIEnv *env, jobject thiz, jint minimum, jint maximum){

        if(minimum < 0){
            minimum = 0;
        }else if(minimum > 65535){
            minimum = 65535;
        }

        if(maximum < 0){
            maximum = 0;
        }else if(maximum > 65535){
            maximum = 65535;
        }

        if(minimum > maximum){
            int temp = minimum;
            minimum = maximum;
            maximum = temp;
        }

        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);
        return apm->_apm->gain_control()->set_analog_level_limits(minimum, maximum);
    }

    static jint JNICALL agc_set_mode(JNIEnv *env, jobject thiz, jint mode){

        if(mode < GainControl::Mode::kAdaptiveAnalog){
            mode = GainControl::Mode::kAdaptiveAnalog;
        }else if(mode > GainControl::Mode::kFixedDigital){
            mode = GainControl::Mode::kFixedDigital;
        }

        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);
        return apm->_apm->gain_control()->set_mode((GainControl::Mode)mode);
    }

    static jint JNICALL agc_set_stream_analog_level
            (JNIEnv *env, jobject thiz, jint level){

        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);
        return apm->_apm->gain_control()->set_stream_analog_level(level);
    }

    static jint JNICALL agc_stream_analog_level(JNIEnv *env, jobject thiz){

        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);
        return apm->_apm->gain_control()->stream_analog_level();
    }


    static jint JNICALL vad_enable(JNIEnv *env, jobject thiz, jboolean enable){

        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);
        return apm->_apm->voice_detection()->Enable(enable);
    }

    static jint JNICALL vad_set_likelihood(JNIEnv *env, jobject thiz, jint likelihood){

        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);
        if(likelihood < VoiceDetection::kVeryLowLikelihood){
            likelihood = VoiceDetection::kVeryLowLikelihood;
        }else if(likelihood > VoiceDetection::kHighLikelihood){
            likelihood = VoiceDetection::kHighLikelihood;
        }

        return apm->_apm->voice_detection()->set_likelihood((VoiceDetection::Likelihood)likelihood);
    }

    static jboolean JNICALL vad_stream_has_voice(JNIEnv *env, jobject thiz){

        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);
        return apm->_apm->voice_detection()->stream_has_voice();
    }


    static jint JNICALL nativeProcessStream(JNIEnv *env, jobject thiz, jshortArray nearEnd, jint offset){

        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);
        short *buffer = (short*)env->GetShortArrayElements(nearEnd, nullptr);
        int ret = apm->ProcessStream(buffer + offset);
        env->ReleaseShortArrayElements(nearEnd, buffer, 0);
        return ret;
    }

    static jint JNICALL nativeProcessReverseStream(JNIEnv *env, jobject thiz, jshortArray farEnd, jint offset){

        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);
        short *buffer = (short*)env->GetShortArrayElements(farEnd, nullptr);
        int ret = apm->ProcessReverseStream(buffer + offset);

        env->ReleaseShortArrayElements(farEnd, buffer, 0);
        return ret;
    }

    static jint JNICALL set_stream_delay_ms(JNIEnv *env, jobject thiz, jint delay){

        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);
        return apm->_apm->set_stream_delay_ms(delay);
    }


    static jint JNICALL nativeCaptureStreamCacheDirectBufferAddress(JNIEnv *env, jobject thiz, jobject byte_buffer){
        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);

        void *p = env->GetDirectBufferAddress(byte_buffer);
        if(!p)return -1;

        jlong capacity = env->GetDirectBufferCapacity(byte_buffer);

        apm->SetDirectBuffer_C(p, capacity);
        return 0;

    }
    static jint JNICALL nativeRenderStreamCacheDirectBufferAddress(JNIEnv *env, jobject thiz, jobject byte_buffer){
        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);

        void *p = env->GetDirectBufferAddress(byte_buffer);
        if(!p)return -1;

        jlong capacity = env->GetDirectBufferCapacity(byte_buffer);

        apm->SetDirectBuffer_R(p, static_cast<size_t>(capacity));
        return 0;
    }

    static jint JNICALL ProcessStreamEx(JNIEnv *env, jobject thiz) //本地数据 // 16K, 16Bits, 单声道， 10ms
    {
        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);
        return apm->ProcessStream();
    }

    static jint JNICALL ProcessReverseStreamEx(JNIEnv *env, jobject thiz) // 远端数据 // 16K, 16Bits, 单声道， 10ms
    {
        ApmWrapper *apm = (ApmWrapper*) get_ctx(env, thiz);
        return apm->ProcessReverseStream();
    }


    static inline JNINativeMethod gMethods[] = {
            { "Create", "(ZZZZZZZ)Z", reinterpret_cast<void*>(&ApmWrapper::Create)},
            {"Destroy", "()V", reinterpret_cast<void*>(&ApmWrapper::Destroy)},
            {"high_pass_filter_enable", "(Z)I", reinterpret_cast<void*>(&ApmWrapper::high_pass_filter_enable)},
            {"aec_enable", "(Z)I", reinterpret_cast<void*>(&ApmWrapper::aec_enable)},
            {"aec_clock_drift_compensation_enable", "(Z)I", reinterpret_cast<void*>(&ApmWrapper::aec_clock_drift_compensation_enable)},
            {"aec_set_suppression_level", "(I)I", reinterpret_cast<void*>(&ApmWrapper::aec_set_suppression_level)},
            {"aecm_enable", "(Z)I", reinterpret_cast<void*>(&ApmWrapper::aecm_enable)},
            {"aecm_set_suppression_level", "(I)I", reinterpret_cast<void*>(&ApmWrapper::aecm_set_suppression_level)},
            {"ns_enable", "(Z)I", reinterpret_cast<void*>(&ApmWrapper::ns_enable)},
            {"ns_set_level", "(I)I", reinterpret_cast<void*>(&ApmWrapper::ns_set_level)},
            {"agc_enable", "(Z)I", reinterpret_cast<void*>(&ApmWrapper::agc_enable)},
            {"agc_set_target_level_dbfs", "(I)I", reinterpret_cast<void*>(&ApmWrapper::agc_set_target_level_dbfs)},
            {"agc_set_compression_gain_db", "(I)I", reinterpret_cast<void*>(&ApmWrapper::agc_set_compression_gain_db)},
            {"agc_enable_limiter", "(Z)I", reinterpret_cast<void*>(&ApmWrapper::agc_enable_limiter)},
            {"agc_set_analog_level_limits", "(II)I", reinterpret_cast<void*>(&ApmWrapper::agc_set_analog_level_limits)},
            {"agc_set_mode", "(I)I", reinterpret_cast<void*>(&ApmWrapper::agc_set_mode)},
            {"agc_set_stream_analog_level", "(I)I", reinterpret_cast<void*>(&ApmWrapper::agc_set_stream_analog_level)},
            {"agc_stream_analog_level", "()I", reinterpret_cast<void*>(&ApmWrapper::agc_stream_analog_level)},
            {"vad_enable", "(Z)I", reinterpret_cast<void*>(&ApmWrapper::vad_enable)},
            {"vad_set_likelihood", "(I)I", reinterpret_cast<void*>(&ApmWrapper::vad_set_likelihood)},
            {"vad_stream_has_voice", "()Z", reinterpret_cast<void*>(&ApmWrapper::vad_stream_has_voice)},
            {"ProcessStream", "([SI)I", reinterpret_cast<void*>(&ApmWrapper::nativeProcessStream)},
            {"ProcessReverseStream", "([SI)I", reinterpret_cast<void*>(&ApmWrapper::nativeProcessReverseStream)},
            {"set_stream_delay_ms", "(I)I", reinterpret_cast<void*>(&ApmWrapper::set_stream_delay_ms)},
            {"nativeCaptureStreamCacheDirectBufferAddress", "(Ljava/nio/ByteBuffer;)I", reinterpret_cast<void*>(&ApmWrapper::nativeCaptureStreamCacheDirectBufferAddress)},
            {"nativeRenderStreamCacheDirectBufferAddress", "(Ljava/nio/ByteBuffer;)I", reinterpret_cast<void*>(&ApmWrapper::nativeRenderStreamCacheDirectBufferAddress)},
            {"ProcessStreamEx", "()I", reinterpret_cast<void*>(&ApmWrapper::ProcessStreamEx)},
            {"ProcessReverseStreamEx", "()I", reinterpret_cast<void*>(&ApmWrapper::ProcessReverseStreamEx)}



    };

public:
    static jboolean registerNativeMethods(JNIEnv* env)
    {
        jclass clazz;
        clazz = env->FindClass("com/sinowave/ddp/audio/aec/Apm");
        if (clazz == NULL) {
            return JNI_FALSE;
        }
        if (env->RegisterNatives(clazz, gMethods, sizeof(gMethods)/sizeof(gMethods[0])) < 0){
            return JNI_FALSE;
        }

        return JNI_TRUE;
    }

public:
    unique_ptr<AudioProcessing> _apm;

private:
    template <typename T>
    void SetContainerFormat(int sample_rate_hz,
                            size_t num_channels,
                            AudioFrame* frame,
                            unique_ptr<ChannelBuffer<T> >* cb) {
        SetFrameSampleRate(frame, sample_rate_hz);
        frame->num_channels_ = num_channels;
        cb->reset(new ChannelBuffer<T>(frame->samples_per_channel_, num_channels));
    }

    void SetFrameSampleRate(AudioFrame* frame,
                            int sample_rate_hz) {
        frame->sample_rate_hz_ = sample_rate_hz;
        frame->samples_per_channel_ = AudioProcessing::kChunkSizeMs *
                                      sample_rate_hz / 1000;
    }

    void ConvertToFloat(const int16_t* int_data, ChannelBuffer<float>* cb) {
        ChannelBuffer<int16_t> cb_int(cb->num_frames(),
                                      cb->num_channels());
        Deinterleave(int_data,
                     cb->num_frames(),
                     cb->num_channels(),
                     cb_int.channels());
        for (size_t i = 0; i < cb->num_channels(); ++i) {
            S16ToFloat(cb_int.channels()[i],
                       cb->num_frames(),
                       cb->channels()[i]);
        }
    }

    void ConvertToFloat(const AudioFrame& frame, ChannelBuffer<float>* cb) {
        ConvertToFloat(frame.data_, cb);
    }

private:
    AudioFrame *_frame;
    AudioFrame *_reverseFrame;

    unique_ptr<ChannelBuffer<float>> _float_cb;
    unique_ptr<ChannelBuffer<float>> _revfloat_cb;

    bool _beamForming = false;


    // Cached copy of address to direct audio buffer owned by |j_audio_record_|.
    void* mDirectBufferAddress_C = nullptr;

    // Number of bytes in the direct audio buffer owned by |j_audio_record_|.
    size_t mDirectBufferCapacityBytes_C = 0;


    // Cached copy of address to direct audio buffer owned by |j_audio_record_|.
    void* mDirectBufferAddress_R = nullptr;

    // Number of bytes in the direct audio buffer owned by |j_audio_record_|.
    size_t mDirectBufferCapacityBytes_R = 0;

};


namespace SWAudioProcessingModule{
	
	Apm::Apm(bool aecExtendFilter, bool speechIntelligibilityEnhance, bool delayAgnostic, bool beamforming, bool nextGenerationAec, bool experimentalNs, bool experimentalAgc){
		impl = new ApmWrapper(aecExtendFilter, speechIntelligibilityEnhance, delayAgnostic, beamforming, nextGenerationAec, experimentalNs, experimentalAgc);
	}
	
	Apm::~Apm(){
		delete impl;
	}
	
	int Apm::ProcessStream(int16_t* data){
		return impl->ProcessStream(data);
	}

	int Apm::ProcessReverseStream(int16_t* data){
		return impl->ProcessReverseStream(data);
	}

	int Apm::SetStreamDelay(int delay_ms)
	{
	    return impl->_apm->set_stream_delay_ms(delay_ms);
	}
	
	int Apm::HighPassFilter(bool enable){
		return impl->_apm->high_pass_filter()->Enable(enable);
	}
			
	// AEC PC		
	int Apm::AECClockDriftCompensation(bool enable){
		return impl->_apm->echo_cancellation()->enable_drift_compensation(enable);
	}
	
	int Apm::AECSetSuppressionLevel(AEC_SuppressionLevel level){ //[0, 1, 2]
		return impl->_apm->echo_cancellation()->set_suppression_level((EchoCancellation::SuppressionLevel)level);
	}

	int Apm::AEC(bool enable){
	    return impl->_apm->echo_cancellation()->Enable(enable);
	}
	
	// AEC Mobile
	int Apm::AECMSetSuppressionLevel(AECM_RoutingMode level){//[0, 1, 2, 3, 4]
		return impl->_apm->echo_control_mobile()->set_routing_mode((EchoControlMobile::RoutingMode)level);
	}
	
	int Apm::AECM(bool enable){
		return impl->_apm->echo_control_mobile()->Enable(enable);
	}
			
	// NS
	int Apm::NS(bool enable){
		return impl->_apm->noise_suppression()->Enable(enable);
	}
	
	int Apm::NSSetLevel(NS_Level level){// [0, 1, 2, 3]
		return impl->_apm->noise_suppression()->set_level((NoiseSuppression::Level)level);
	}
			
	// AGC
	int Apm::AGC(bool enable){
		return impl->_apm->gain_control()->Enable(enable);
	}
			
	int Apm::AGCSetAnalogLevelLimits(int minimum, int maximum){ // limit to [0, 65535]
		return impl->_apm->gain_control()->set_analog_level_limits(minimum, maximum);
	}
	
	int Apm::AGCSetMode(AGC_Mode mode){ // [0, 1, 2]
		return impl->_apm->gain_control()->set_mode(static_cast<GainControl::Mode>(mode));
	}
	
	int Apm::AGCSetTargetLevelDbfs(int level){
		return impl->_apm->gain_control()->set_target_level_dbfs(level);
	}
	int Apm::AGCSetcompressionGainDb(int gain){
		return impl->_apm->gain_control()->set_compression_gain_db(gain);
	}
	
	int Apm::AGCEnableLimiter(bool enable){
		return impl->_apm->gain_control()->enable_limiter(enable);
	}
	int Apm::AGCSetStreamAnalogLevel(int level){
		 return impl->_apm->gain_control()->set_stream_analog_level(level);
	}
	int Apm::AGCStreamAnalogLevel(){
		 return impl->_apm->gain_control()->stream_analog_level();
	}
					
	// VAD
	int Apm::VAD(bool enable){
		return impl->_apm->voice_detection()->Enable(enable);
	}
			
	int Apm::VADSetLikeHood(VAD_Likelihood likelihood){
		return impl->_apm->voice_detection()->set_likelihood(static_cast<VoiceDetection::Likelihood>(likelihood));
	}
	bool Apm::VADHasVoice(){
		return impl->_apm->voice_detection()->stream_has_voice();
	}
	
}


extern "C" JNIEXPORT jint JNI_OnLoad(JavaVM *vm, void *reserved) {

    JNIEnv *env;
    if (vm->GetEnv((void **) &env, JNI_VERSION_1_6) != JNI_OK) {
        return -1;
    }

    if(!ApmWrapper::registerNativeMethods(env))return -1;

    return JNI_VERSION_1_6;
}
