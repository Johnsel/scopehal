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

#ifndef TSLitexOscilloscope_h
#define TSLitexOscilloscope_h

class EdgeTrigger;

#include "RemoteBridgeOscilloscope.h"
#include "../xptools/HzClock.h"

/**
	@brief TSLitexOscilloscope - driver for talking to the scopehal-pico-bridge daemons
 */
class TSLitexOscilloscope   : public virtual RemoteBridgeOscilloscope
{
public:
	TSLitexOscilloscope(SCPITransport* transport);
	virtual ~TSLitexOscilloscope();

	//not copyable or assignable
	TSLitexOscilloscope(const TSLitexOscilloscope& rhs) =delete;
	TSLitexOscilloscope& operator=(const TSLitexOscilloscope& rhs) =delete;

public:

	//Device information
	virtual unsigned int GetInstrumentTypes() const override;
	virtual void FlushConfigCache() override;

	//Channel configuration
	virtual std::vector<OscilloscopeChannel::CouplingType> GetAvailableCouplings(size_t i) override;
	virtual double GetChannelAttenuation(size_t i) override;
	virtual void SetChannelAttenuation(size_t i, double atten) override;
	virtual unsigned int GetChannelBandwidthLimit(size_t i) override;
	virtual void SetChannelBandwidthLimit(size_t i, unsigned int limit_mhz) override;
	virtual OscilloscopeChannel* GetExternalTrigger() override;
	virtual bool CanEnableChannel(size_t i) override;
	virtual uint32_t GetInstrumentTypesForChannel(size_t i) const override;

	//Triggering
	virtual Oscilloscope::TriggerMode PollTrigger() override;
	virtual bool AcquireData() override;

	// Captures
	virtual void Start() override;
	virtual void StartSingleTrigger() override;
	virtual void ForceTrigger() override;

	//Timebase
	virtual std::vector<uint64_t> GetSampleRatesNonInterleaved() override;
	virtual std::vector<uint64_t> GetSampleRatesInterleaved() override;
	virtual std::set<InterleaveConflict> GetInterleaveConflicts() override;
	virtual std::vector<uint64_t> GetSampleDepthsNonInterleaved() override;
	virtual std::vector<uint64_t> GetSampleDepthsInterleaved() override;
	virtual bool IsInterleaving() override;
	virtual bool SetInterleaving(bool combine) override;

protected:
	void IdentifyHardware();

	void ResetPerCaptureDiagnostics();

	std::string GetChannelColor(size_t i);

	size_t m_analogChannelCount;

	// Cache
	std::map<size_t, double> m_channelAttenuations;

	FilterParameter m_diag_hardwareWFMHz;
	FilterParameter m_diag_receivedWFMHz;
	FilterParameter m_diag_totalWFMs;
	FilterParameter m_diag_droppedWFMs;
	FilterParameter m_diag_droppedPercent;
	HzClock m_receiveClock;

	// //Most Pico API calls are write only, so we have to maintain all state clientside.
	// //This isn't strictly a cache anymore since it's never flushed!
	// std::map<size_t, double> m_channelAttenuations;
	// ADCMode m_adcMode;
	// std::map<int, bool> m_digitalBankPresent;
	// std::map<int, float> m_digitalThresholds;
	// std::map<int, float> m_digitalHysteresis;


	///@brief Buffers for storing raw ADC samples before converting to fp32
	std::vector<std::unique_ptr<AcceleratorBuffer<int16_t> > > m_analogRawWaveformBuffers;

	//Vulkan waveform conversion
	std::shared_ptr<QueueHandle> m_queue;
	std::unique_ptr<vk::raii::CommandPool> m_pool;
	std::unique_ptr<vk::raii::CommandBuffer> m_cmdBuf;
	std::unique_ptr<ComputePipeline> m_conversionPipeline;

public:

	static std::string GetDriverNameInternal();
	OSCILLOSCOPE_INITPROC(TSLitexOscilloscope)
};

#endif
