/***********************************************************************************************************************
*                                                                                                                      *
* libscopehal v0.1                                                                                                     *
*                                                                                                                      *
* Copyright (c) 2012-2023 Andrew D. Zonenberg and contributors                                                         *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

#ifndef LiteXOscilloscope_h
#define LiteXOscilloscope_h

//class EdgeTrigger;

#include "RemoteBridgeOscilloscope.h"

/**
	@brief LiteXOscilloscope - driver for talking to the LitePCIe driver
 */
class LiteXOscilloscope 	: public virtual RemoteBridgeOscilloscope
{
public:
	LiteXOscilloscope(SCPITransport* transport);
	virtual ~LiteXOscilloscope();

	//not copyable or assignable
	LiteXOscilloscope(const LiteXOscilloscope& rhs) =delete;
	LiteXOscilloscope& operator=(const LiteXOscilloscope& rhs) =delete;

public:

	//Device information
	virtual unsigned int GetInstrumentTypes() const override;
	virtual uint32_t GetInstrumentTypesForChannel(size_t i) const override;

	virtual void FlushConfigCache();

	//Channel configuration
	virtual bool IsChannelEnabled(size_t i);
	virtual void EnableChannel(size_t i);
	virtual void DisableChannel(size_t i);
	virtual std::vector<OscilloscopeChannel::CouplingType> GetAvailableCouplings(size_t i);
	virtual OscilloscopeChannel::CouplingType GetChannelCoupling(size_t i);

	virtual double GetChannelAttenuation(size_t i);
	void SetChannelAttenuation(size_t i, double atten);
	virtual unsigned int GetChannelBandwidthLimit(size_t i);
	virtual void SetChannelBandwidthLimit(size_t i, unsigned int limit_mhz);
	virtual OscilloscopeChannel* GetExternalTrigger();
	virtual bool CanEnableChannel(size_t i);

	//Triggering
	virtual Oscilloscope::TriggerMode PollTrigger();
	virtual bool AcquireData();
	virtual bool IsTriggerArmed();
	virtual void PushTrigger();

	//Timebase
	virtual bool CanInterleave();
	virtual std::vector<uint64_t> GetSampleRatesNonInterleaved();
	virtual std::vector<uint64_t> GetSampleRatesInterleaved();
	virtual std::set<InterleaveConflict> GetInterleaveConflicts();
	virtual std::vector<uint64_t> GetSampleDepthsNonInterleaved();
	virtual std::vector<uint64_t> GetSampleDepthsInterleaved();
	virtual uint64_t GetSampleRate();
	virtual uint64_t GetSampleDepth();
	virtual void SetSampleDepth(uint64_t depth);
	virtual void SetSampleRate(uint64_t rate);
	virtual void SetTriggerOffset(int64_t offset);
	virtual int64_t GetTriggerOffset();
	virtual bool IsInterleaving();
	virtual bool SetInterleaving(bool combine);

	//ADC configuration
	virtual std::vector<AnalogBank> GetAnalogBanks();
	virtual AnalogBank GetAnalogBank(size_t channel);
	virtual bool IsADCModeConfigurable();
	virtual std::vector<std::string> GetADCModeNames(size_t channel);
	virtual size_t GetADCMode(size_t channel);
	virtual void SetADCMode(size_t channel, size_t mode);

	enum ADCMode
	{
		ADC_MODE_8BIT	= 0,
		ADC_MODE_10BIT	= 1,
		ADC_MODE_12BIT	= 2
	};


protected:
	void IdentifyHardware();


	//Helpers for determining legal configurations
	bool Is10BitModeAvailable();
	bool Is12BitModeAvailable();
	size_t GetEnabledAnalogChannelCount();
	size_t GetEnabledDigitalPodCount();

	size_t GetEnabledAnalogChannelCountRange(size_t start, size_t end);

	size_t GetEnabledAnalogChannelCountAToD()
	{ return GetEnabledAnalogChannelCountRange(0, 3); }
	size_t GetEnabledAnalogChannelCountEToH()
	{ return GetEnabledAnalogChannelCountRange(4, 7); }
	size_t GetEnabledAnalogChannelCountAToB()
	{ return GetEnabledAnalogChannelCountRange(0, 1); }
	size_t GetEnabledAnalogChannelCountCToD()
	{ return GetEnabledAnalogChannelCountRange(2, 3); }
	size_t GetEnabledAnalogChannelCountEToF()
	{ return GetEnabledAnalogChannelCountRange(4, 5); }
	size_t GetEnabledAnalogChannelCountGToH()
	{ return GetEnabledAnalogChannelCountRange(6, 7); }

	// bool CanEnableChannel6000Series8Bit(size_t i);
	// bool CanEnableChannel6000Series10Bit(size_t i);
	// bool CanEnableChannel6000Series12Bit(size_t i);

	std::string GetChannelColor(size_t i);

	//hardware analog channel count, independent of LA option etc
	size_t m_analogChannelCount;

	OscilloscopeChannel* m_extTrigChannel;

	//Most Pico API calls are write only, so we have to maintain all state clientside.
	//This isn't strictly a cache anymore since it's never flushed!
	std::map<size_t, double> m_channelAttenuations;
	ADCMode m_adcMode;

	//Series m_series;

	///@brief Buffers for storing raw ADC samples before converting to fp32
	std::vector<std::unique_ptr<AcceleratorBuffer<int16_t> > > m_analogRawWaveformBuffers;

	//Vulkan waveform conversion
	std::shared_ptr<QueueHandle> m_queue;
	std::unique_ptr<vk::raii::CommandPool> m_pool;
	std::unique_ptr<vk::raii::CommandBuffer> m_cmdBuf;
	std::unique_ptr<ComputePipeline> m_conversionPipeline;

public:

	static std::string GetDriverNameInternal();
	OSCILLOSCOPE_INITPROC(LiteXOscilloscope)
};

#endif
