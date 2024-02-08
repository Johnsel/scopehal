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

#ifdef _WIN32
#include <chrono>
#include <thread>
#endif

#include "LiteXOscilloscope.h"

#include "scopehal.h"
//#include "EdgeTrigger.h"


// #include "liblitepcie.h"
// #include "litepcie_thunderscope.h"
// //#include "litepcie_public.h"
// #include <csr.h>
// #include <soc.h>


using namespace std;

#define RATE_1GSPS	(INT64_C(1000) * INT64_C(1000) * INT64_C(1000))
#define RATE_625MSPS	(INT64_C(625)  * INT64_C(1000) * INT64_C(1000))

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Construction / destruction

LiteXOscilloscope::LiteXOscilloscope(SCPITransport* transport)
	: SCPIDevice(transport)
	, SCPIInstrument(transport)
	, RemoteBridgeOscilloscope(transport)
{
	//Set up initial cache configuration as "not valid" and let it populate as we go

	// IdentifyHardware();

	// //Set resolution
	// SetADCMode(0, ADC_MODE_8BIT);

	// //Add analog channel objects
	// for(size_t i = 0; i < m_analogChannelCount; i++)
	// {
	// 	//Hardware name of the channel
	// 	string chname = "A";
	// 	chname[0] += i;

	// 	//Create the channel
	// 	auto chan = new OscilloscopeChannel(
	// 		this,
	// 		chname,
	// 		GetChannelColor(i),
	// 		Unit(Unit::UNIT_FS),
	// 		Unit(Unit::UNIT_VOLTS),
	// 		Stream::STREAM_TYPE_ANALOG,
	// 		i);
	// 	m_channels.push_back(chan);
	// 	chan->SetDefaultDisplayName();

	// 	//Set initial configuration so we have a well-defined instrument state
	// 	m_channelAttenuations[i] = 1;
	// 	SetChannelCoupling(i, OscilloscopeChannel::COUPLE_DC_1M);
	// 	SetChannelOffset(i, 0,  0);
	// 	SetChannelVoltageRange(i, 0, 5);
	// }

	// //Add digital channels (named 1D0...7 and 2D0...7)
	// m_digitalChannelBase = m_analogChannelCount;
	// for(size_t i=0; i<m_digitalChannelCount; i++)
	// {
	// 	//Hardware name of the channel
	// 	size_t ibank = i / 8;
	// 	size_t ichan = i % 8;
	// 	string chname = "1D0";
	// 	chname[0] += ibank;
	// 	chname[2] += ichan;

	// 	//Create the channel
	// 	size_t chnum = i + m_digitalChannelBase;
	// 	auto chan = new OscilloscopeChannel(
	// 		this,
	// 		chname,
	// 		GetChannelColor(ichan),
	// 		Unit(Unit::UNIT_FS),
	// 		Unit(Unit::UNIT_COUNTS),
	// 		Stream::STREAM_TYPE_DIGITAL,
	// 		chnum);
	// 	m_channels.push_back(chan);
	// 	chan->SetDefaultDisplayName();

	// 	SetDigitalHysteresis(chnum, 0.1);
	// 	SetDigitalThreshold(chnum, 0);
	// }

	// //Set initial memory configuration.
	// switch(m_series)
	// {
	// 	case SERIES_3x0xD:
	// 	case SERIES_3x0xDMSO:
	// 	{
	// 		//62.5 Msps is the highest rate the 3000 series supports with all channels, including MSO, active.
	// 		SetSampleRate(62500000L);
	// 		SetSampleDepth(100000);
	// 	}
	// 	break;

	// 	case SERIES_6403E:
	// 	case SERIES_6x0xE:
	// 	case SERIES_6x2xE:
	// 	{
	// 		//625 Msps is the highest rate the 6000 series supports with all channels, including MSO, active.
	// 		SetSampleRate(625000000L);
	// 		SetSampleDepth(1000000);
	// 	}
	// 	break;

	// 	default:
	// 		LogWarning("Unknown/unsupported Pico model\n");
	// 		break;
	// }

	// //Set initial AWG configuration
	// switch(m_series)
	// {
	// 	//has function generator
	// 	case SERIES_3x0xD:
	// 	case SERIES_3x0xDMSO:
	// 	case SERIES_6403E:
	// 	case SERIES_6x0xE:
	// 	case SERIES_6x2xE:
	// 		SetFunctionChannelAmplitude(0, 0.1);
	// 		SetFunctionChannelShape(0, SHAPE_SQUARE);
	// 		SetFunctionChannelDutyCycle(0, 0.5);
	// 		SetFunctionChannelFrequency(0, 1e6);
	// 		SetFunctionChannelOffset(0, 0);
	// 		SetFunctionChannelOutputImpedance(0, IMPEDANCE_HIGH_Z);
	// 		SetFunctionChannelActive(0, false);
	// 		m_awgChannel = new FunctionGeneratorChannel(
	// 			this,
	// 			"AWG",
	// 			"#808080",
	// 			m_channels.size());
	// 		m_channels.push_back(m_awgChannel);
	// 		break;

	// 	//no AWG
	// 	default:
	// 		m_awgChannel = nullptr;
	// }

	// //Add the external trigger input
	// m_extTrigChannel =
	// 	new OscilloscopeChannel(
	// 	this,
	// 	"EX",
	// 	"#808080",
	// 	Unit(Unit::UNIT_FS),
	// 	Unit(Unit::UNIT_COUNTS),
	// 	Stream::STREAM_TYPE_TRIGGER,
	// 	m_channels.size());
	// m_channels.push_back(m_extTrigChannel);
	// m_extTrigChannel->SetDefaultDisplayName();

	// //Configure the trigger
	// auto trig = new EdgeTrigger(this);
	// trig->SetType(EdgeTrigger::EDGE_RISING);
	// trig->SetLevel(0);
	// trig->SetInput(0, StreamDescriptor(GetOscilloscopeChannel(0)));
	// SetTrigger(trig);
	// PushTrigger();
	// SetTriggerOffset(10 * 1000L * 1000L);

	// //Initialize waveform buffers
	// for(size_t i=0; i<m_analogChannelCount; i++)
	// {
	// 	m_analogRawWaveformBuffers.push_back(std::make_unique<AcceleratorBuffer<int16_t> >());
	// 	m_analogRawWaveformBuffers[i]->SetCpuAccessHint(AcceleratorBuffer<int16_t>::HINT_LIKELY);
	// 	m_analogRawWaveformBuffers[i]->SetGpuAccessHint(AcceleratorBuffer<int16_t>::HINT_LIKELY);
	// }

	// //Create Vulkan objects for the waveform conversion
	// m_queue = g_vkQueueManager->GetComputeQueue("LiteXOscilloscope.queue");
	// vk::CommandPoolCreateInfo poolInfo(
	// 	vk::CommandPoolCreateFlagBits::eTransient | vk::CommandPoolCreateFlagBits::eResetCommandBuffer,
	// 	m_queue->m_family );
	// m_pool = make_unique<vk::raii::CommandPool>(*g_vkComputeDevice, poolInfo);

	// vk::CommandBufferAllocateInfo bufinfo(**m_pool, vk::CommandBufferLevel::ePrimary, 1);
	// m_cmdBuf = make_unique<vk::raii::CommandBuffer>(
	// 	move(vk::raii::CommandBuffers(*g_vkComputeDevice, bufinfo).front()));

	// if(g_hasDebugUtils)
	// {
	// 	string poolname = "LiteXOscilloscope.pool";
	// 	string bufname = "LiteXOscilloscope.cmdbuf";

	// 	g_vkComputeDevice->setDebugUtilsObjectNameEXT(
	// 		vk::DebugUtilsObjectNameInfoEXT(
	// 			vk::ObjectType::eCommandPool,
	// 			reinterpret_cast<int64_t>(static_cast<VkCommandPool>(**m_pool)),
	// 			poolname.c_str()));

	// 	g_vkComputeDevice->setDebugUtilsObjectNameEXT(
	// 		vk::DebugUtilsObjectNameInfoEXT(
	// 			vk::ObjectType::eCommandBuffer,
	// 			reinterpret_cast<int64_t>(static_cast<VkCommandBuffer>(**m_cmdBuf)),
	// 			bufname.c_str()));
	// }

	// m_conversionPipeline = make_unique<ComputePipeline>(
	// 		"shaders/Convert16BitSamples.spv", 2, sizeof(ConvertRawSamplesShaderArgs) );
}

/**
	@brief Color the channels based on Pico's standard color sequence (blue-red-green-yellow-purple-gray-cyan-magenta)
 */
string LiteXOscilloscope::GetChannelColor(size_t i)
{
	switch(i % 8)
	{
		case 0:
			return "#4040ff";

		case 1:
			return "#ff4040";

		case 2:
			return "#208020";

		case 3:
			return "#ffff00";

		case 4:
			return "#600080";

		case 5:
			return "#808080";

		case 6:
			return "#40a0a0";

		case 7:
		default:
			return "#e040e0";
	}
}

LiteXOscilloscope::~LiteXOscilloscope()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Accessors

std::string GetDriverName()
{
	return "LiteXOscilloscope";
}

unsigned int LiteXOscilloscope::GetInstrumentTypes() const
{
	return Instrument::INST_OSCILLOSCOPE;
}

uint32_t LiteXOscilloscope::GetInstrumentTypesForChannel(size_t i) const
{
	return Instrument::INST_OSCILLOSCOPE;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Device interface functions

string LiteXOscilloscope::GetDriverNameInternal()
{
	return "LiteX";
}

void LiteXOscilloscope::FlushConfigCache()
{
	lock_guard<recursive_mutex> lock(m_cacheMutex);

	//clear probe presence flags as those can change without our knowledge
	m_digitalBankPresent.clear();
}

bool LiteXOscilloscope::IsChannelEnabled(size_t i)
{
	// //ext trigger should never be displayed
	// if(i == m_extTrigChannel->GetIndex())
	// 	return false;

	// lock_guard<recursive_mutex> lock(m_cacheMutex);
	// return m_channelsEnabled[i];
}

void LiteXOscilloscope::EnableChannel(size_t i)
{
	//RemoteBridgeOscilloscope::EnableChannel(i);
}

void LiteXOscilloscope::DisableChannel(size_t i)
{
	{
		lock_guard<recursive_mutex> lock(m_cacheMutex);
		m_channelsEnabled[i] = false;
	}

	lock_guard<recursive_mutex> lock(m_mutex);
	m_transport->SendCommand(":" + m_channels[i]->GetHwname() + ":OFF");
}

vector<OscilloscopeChannel::CouplingType> LiteXOscilloscope::GetAvailableCouplings(size_t /*i*/)
{
	vector<OscilloscopeChannel::CouplingType> ret;
	ret.push_back(OscilloscopeChannel::COUPLE_DC_1M);
	ret.push_back(OscilloscopeChannel::COUPLE_AC_1M);
	ret.push_back(OscilloscopeChannel::COUPLE_DC_50);
	return ret;
}

double LiteXOscilloscope::GetChannelAttenuation(size_t i)
{
	if(GetOscilloscopeChannel(i) == m_extTrigChannel)
		return 1;

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	return m_channelAttenuations[i];
}

void LiteXOscilloscope::SetChannelAttenuation(size_t i, double atten)
{
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	double oldAtten = m_channelAttenuations[i];
	m_channelAttenuations[i] = atten;

	//Rescale channel voltage range and offset
	double delta = atten / oldAtten;
	m_channelVoltageRanges[i] *= delta;
	m_channelOffsets[i] *= delta;
}

unsigned int LiteXOscilloscope::GetChannelBandwidthLimit(size_t /*i*/)
{
	return 0;
}

void LiteXOscilloscope::SetChannelBandwidthLimit(size_t /*i*/, unsigned int /*limit_mhz*/)
{
}

OscilloscopeChannel* LiteXOscilloscope::GetExternalTrigger()
{
	//FIXME
	return NULL;
}

Oscilloscope::TriggerMode LiteXOscilloscope::PollTrigger()
{
	//Always report "triggered" so we can block on AcquireData() in ScopeThread
	//TODO: peek function of some sort?
	return TRIGGER_MODE_TRIGGERED;
}

bool LiteXOscilloscope::AcquireData()
{
	// #pragma pack(push, 1)
	// struct
	// {
	// 	//Number of channels in the current waveform
	// 	uint16_t numChannels;

	// 	//Sample interval.
	// 	//May be different from m_srate if we changed the rate after the trigger was armed
	// 	int64_t fs_per_sample;
	// } wfmhdrs;
	// #pragma pack(pop)

	// //Read global waveform settings (independent of each channel)
	// if(!m_transport->ReadRawData(sizeof(wfmhdrs), (uint8_t*)&wfmhdrs))
	// 	return false;
	// uint16_t numChannels = wfmhdrs.numChannels;
	// int64_t fs_per_sample = wfmhdrs.fs_per_sample;

	// //Acquire data for each channel
	// size_t chnum;
	// size_t memdepth;
	// float config[3];
	// SequenceSet s;
	// double t = GetTime();
	// int64_t fs = (t - floor(t)) * FS_PER_SECOND;

	// //Analog channels get processed separately
	// vector<UniformAnalogWaveform*> awfms;
	// vector<size_t> achans;
	// vector<float> scales;
	// vector<float> offsets;

	// for(size_t i=0; i<numChannels; i++)
	// {
	// 	size_t tmp[2];

	// 	//Get channel ID and memory depth (samples, not bytes)
	// 	if(!m_transport->ReadRawData(sizeof(tmp), (uint8_t*)&tmp))
	// 		return false;
	// 	chnum = tmp[0];
	// 	memdepth = tmp[1];

	// 	//Analog channels
	// 	if(chnum < m_analogChannelCount)
	// 	{
	// 		auto& abuf = m_analogRawWaveformBuffers[chnum];
	// 		abuf->resize(memdepth);
	// 		abuf->PrepareForCpuAccess();
	// 		achans.push_back(chnum);

	// 		//Scale and offset are sent in the header since they might have changed since the capture began
	// 		if(!m_transport->ReadRawData(sizeof(config), (uint8_t*)&config))
	// 			return false;
	// 		float scale = config[0];
	// 		float offset = config[1];
	// 		float trigphase = -config[2] * fs_per_sample;
	// 		scale *= GetChannelAttenuation(chnum);
	// 		offset *= GetChannelAttenuation(chnum);

	// 		//TODO: stream timestamp from the server
	// 		if(!m_transport->ReadRawData(memdepth * sizeof(int16_t), reinterpret_cast<uint8_t*>(abuf->GetCpuPointer())))
	// 			return false;

	// 		abuf->MarkModifiedFromCpu();

	// 		//Create our waveform
	// 		auto cap = AllocateAnalogWaveform(m_nickname + "." + GetOscilloscopeChannel(i)->GetHwname());
	// 		cap->m_timescale = fs_per_sample;
	// 		cap->m_triggerPhase = trigphase;
	// 		cap->m_startTimestamp = time(NULL);
	// 		cap->m_startFemtoseconds = fs;
	// 		cap->Resize(memdepth);
	// 		awfms.push_back(cap);
	// 		scales.push_back(scale);
	// 		offsets.push_back(offset);

	// 		s[GetOscilloscopeChannel(chnum)] = cap;
	// 	}


	// //If we have GPU support for int16, we can do the conversion on the card
	// //But only do this if we also have push-descriptor support, because doing N separate dispatches is likely
	// //to be slower than a parallel CPU-side conversion
	// //Note also that a strict benchmarking here may be slower than the CPU version due to transfer latency,
	// //but having the waveform on the GPU now means we don't have to do *that* later.
	// if(g_hasShaderInt16 && g_hasPushDescriptor)
	// {
	// 	m_cmdBuf->begin({});

	// 	m_conversionPipeline->Bind(*m_cmdBuf);

	// 	for(size_t i=0; i<awfms.size(); i++)
	// 	{
	// 		auto cap = awfms[i];

	// 		m_conversionPipeline->BindBufferNonblocking(0, cap->m_samples, *m_cmdBuf, true);
	// 		m_conversionPipeline->BindBufferNonblocking(1, *m_analogRawWaveformBuffers[achans[i]], *m_cmdBuf);

	// 		ConvertRawSamplesShaderArgs args;
	// 		args.size = cap->size();
	// 		args.gain = scales[i];
	// 		args.offset = -offsets[i];

	// 		m_conversionPipeline->DispatchNoRebind(*m_cmdBuf, args, GetComputeBlockCount(cap->size(), 64));

	// 		cap->MarkModifiedFromGpu();
	// 	}

	// 	m_cmdBuf->end();
	// 	m_queue->SubmitAndBlock(*m_cmdBuf);
	// }
	// else
	// {
	// 	//Fallback path
	// 	//Process analog captures in parallel
	// 	#pragma omp parallel for
	// 	for(size_t i=0; i<awfms.size(); i++)
	// 	{
	// 		auto cap = awfms[i];
	// 		cap->PrepareForCpuAccess();
	// 		Convert16BitSamples(
	// 			cap->m_samples.GetCpuPointer(),
	// 			m_analogRawWaveformBuffers[achans[i]]->GetCpuPointer(),
	// 			scales[i],
	// 			-offsets[i],
	// 			cap->size());

	// 		cap->MarkSamplesModifiedFromCpu();
	// 	}
	// }

	// //Save the waveforms to our queue
	// m_pendingWaveformsMutex.lock();
	// m_pendingWaveforms.push_back(s);
	// m_pendingWaveformsMutex.unlock();

	// //If this was a one-shot trigger we're no longer armed
	// if(m_triggerOneShot)
	// 	m_triggerArmed = false;

	return true;
}

bool LiteXOscilloscope::IsTriggerArmed()
{
	return m_triggerArmed;
}

bool LiteXOscilloscope::CanInterleave()
{
	return false;
}

vector<uint64_t> LiteXOscilloscope::GetSampleRatesNonInterleaved()
{
	vector<uint64_t> ret;

	// string rates;
	// {
	// 	lock_guard<recursive_mutex> lock(m_mutex);
	// 	m_transport->SendCommand("RATES?");
	// 	rates = m_transport->ReadReply();
	// }

	// size_t i=0;
	// while(true)
	// {
	// 	size_t istart = i;
	// 	i = rates.find(',', i+1);
	// 	if(i == string::npos)
	// 		break;

	// 	auto block = rates.substr(istart, i-istart);
	// 	uint64_t fs = stoull(block);
	// 	auto hz = FS_PER_SECOND / fs;
	// 	ret.push_back(hz);

	// 	//skip the comma
	// 	i++;
	// }

	ret.push_back(1000000);

	return ret;
}

vector<uint64_t> LiteXOscilloscope::GetSampleRatesInterleaved()
{
	//interleaving not supported
	vector<uint64_t> ret = {};
	return ret;
}

set<Oscilloscope::InterleaveConflict> LiteXOscilloscope::GetInterleaveConflicts()
{
	//interleaving not supported
	set<Oscilloscope::InterleaveConflict> ret;
	return ret;
}

vector<uint64_t> LiteXOscilloscope::GetSampleDepthsNonInterleaved()
{
	vector<uint64_t> ret;

	// string depths;
	// {
	// 	lock_guard<recursive_mutex> lock(m_mutex);
	// 	m_transport->SendCommand("DEPTHS?");
	// 	depths = m_transport->ReadReply();
	// }

	// size_t i=0;
	// while(true)
	// {
	// 	size_t istart = i;
	// 	i = depths.find(',', i+1);
	// 	if(i == string::npos)
	// 		break;

	// 	uint64_t sampleDepth = stoull(depths.substr(istart, i-istart));
	// 	ret.push_back(sampleDepth);

	// 	//skip the comma
	// 	i++;
	// }

	ret.push_back(10000);

	return ret;
}

vector<uint64_t> LiteXOscilloscope::GetSampleDepthsInterleaved()
{
	//interleaving not supported
	vector<uint64_t> ret;
	return ret;
}

uint64_t LiteXOscilloscope::GetSampleRate()
{
	return m_srate;
}

uint64_t LiteXOscilloscope::GetSampleDepth()
{
	return m_mdepth;
}

void LiteXOscilloscope::SetSampleDepth(uint64_t depth)
{
	// lock_guard<recursive_mutex> lock(m_mutex);
	// m_transport->SendCommand(string("DEPTH ") + to_string(depth));
	// m_mdepth = depth;
}

void LiteXOscilloscope::SetSampleRate(uint64_t rate)
{
	// m_srate = rate;

	// lock_guard<recursive_mutex> lock(m_mutex);
	// m_transport->SendCommand( string("RATE ") + to_string(rate));
}

void LiteXOscilloscope::SetTriggerOffset(int64_t offset)
{
	lock_guard<recursive_mutex> lock(m_mutex);

	//Don't allow setting trigger offset beyond the end of the capture
	//int64_t captureDuration = GetSampleDepth() * FS_PER_SECOND / GetSampleRate();
	//m_triggerOffset = min(offset, captureDuration);

	PushTrigger();
}

int64_t LiteXOscilloscope::GetTriggerOffset()
{
	return m_triggerOffset;
}

bool LiteXOscilloscope::IsInterleaving()
{
	//interleaving is done automatically in hardware based on sample rate, no user facing switch for it
	return false;
}

bool LiteXOscilloscope::SetInterleaving(bool /*combine*/)
{
	//interleaving is done automatically in hardware based on sample rate, no user facing switch for it
	return false;
}

void LiteXOscilloscope::PushTrigger()
{
	auto et = dynamic_cast<EdgeTrigger*>(m_trigger);
	if(et)
		PushEdgeTrigger(et);

	else
		LogWarning("Unknown trigger type (not an edge)\n");

	ClearPendingWaveforms();
}

vector<Oscilloscope::AnalogBank> LiteXOscilloscope::GetAnalogBanks()
{
	vector<AnalogBank> banks;
	banks.push_back(GetAnalogBank(0));
	return banks;
}

Oscilloscope::AnalogBank LiteXOscilloscope::GetAnalogBank(size_t /*channel*/)
{
	AnalogBank bank;
	return bank;
}

bool LiteXOscilloscope::IsADCModeConfigurable()
{
	return true;
}

vector<string> LiteXOscilloscope::GetADCModeNames(size_t /*channel*/)
{
	//All scopes with variable resolution start at 8 bit and go up from there
	vector<string> ret;
	ret.push_back("8 Bit");
	ret.push_back("10 Bit");
	ret.push_back("12 Bit");
	return ret;
}

size_t LiteXOscilloscope::GetADCMode(size_t /*channel*/)
{
	return m_adcMode;
}

void LiteXOscilloscope::SetADCMode(size_t /*channel*/, size_t mode)
{
	m_adcMode = (ADCMode)mode;

	// lock_guard<recursive_mutex> lock(m_mutex);
	// switch(mode)
	// {
	// 	case ADC_MODE_8BIT:
	// 		m_transport->SendCommand("BITS 8");
	// 		break;

	// 	case ADC_MODE_10BIT:
	// 		m_transport->SendCommand("BITS 10");
	// 		break;

	// 	case ADC_MODE_12BIT:
	// 		m_transport->SendCommand("BITS 12");
	// 		break;

	// 	default:
	// 		LogWarning("LiteXOscilloscope::SetADCMode requested invalid mode %zu, interpreting as 8 bit\n", mode);
	// 		m_adcMode = ADC_MODE_8BIT;
	// 		break;
	// }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Checking for validity of configurations

/**
	@brief Returns the total number of analog channels which are currently enabled
 */
size_t LiteXOscilloscope::GetEnabledAnalogChannelCount()
{
	size_t ret = 0;
	for(size_t i=0; i<m_analogChannelCount; i++)
	{
		if(IsChannelEnabled(i))
			ret ++;
	}
	return ret;
}

/**
	@brief Returns the total number of analog channels in the requested range which are currently enabled
 */
size_t LiteXOscilloscope::GetEnabledAnalogChannelCountRange(size_t start, size_t end)
{
	if(end >= m_analogChannelCount)
		end = m_analogChannelCount - 1;

	size_t n = 0;
	for(size_t i = start; i <= end; i++)
	{
		if(IsChannelEnabled(i))
			n ++;
	}
	return n;
}

bool LiteXOscilloscope::CanEnableChannel(size_t i)
{
	//If channel is already on, of course it can stay on
	if(IsChannelEnabled(i))
		return true;


	// switch(GetADCMode(0))
	// {
	// 	case ADC_MODE_8BIT:
	// 		return CanEnableChannel6000Series8Bit(i);

	// 	case ADC_MODE_10BIT:
	// 		return CanEnableChannel6000Series10Bit(i);

	// 	case ADC_MODE_12BIT:
	// 		return CanEnableChannel6000Series12Bit(i);

	// 	default:
	// 		break;
	// }

	//When in doubt, assume all channels are available
	LogWarning("LiteXOscilloscope::CanEnableChannel: Unknown ADC mode\n");
	return true;
}

// /**
// 	@brief Checks if we can enable a channel on a 6000 series scope configured for 8-bit ADC resolution
//  */
// bool LiteXOscilloscope::CanEnableChannel6000Series8Bit(size_t i)
// {
// 	int64_t rate = GetSampleRate();
// 	size_t EnabledChannelCount = GetEnabledAnalogChannelCount() + GetEnabledDigitalPodCount();

// 	//5 Gsps is the most restrictive configuration.
// 	if(rate >= RATE_5GSPS)
// 	{
// 		//If we already have too many channels/MSO pods active, we're out of RAM bandwidth.
// 		if(EnabledChannelCount >= 2)
// 			return false;

// 		//6403E only allows *one* 5 Gsps channel
// 		else if(m_series == SERIES_6403E)
// 			return (EnabledChannelCount == 0);

// 		//No banking restrictions for MSO pods if we have enough memory bandwidth
// 		else if(IsChannelIndexDigital(i))
// 			return true;

// 		//On 8 channel scopes, we can use one channel from the left bank (ABCD) and one from the right (EFGH).
// 		else if(m_analogChannelCount == 8)
// 		{
// 			//Can enable a left bank channel if there's none in use
// 			if(i < 4)
// 				return (GetEnabledAnalogChannelCountAToD() == 0);

// 			//Can enable a right bank channel if there's none in use
// 			else
// 				return (GetEnabledAnalogChannelCountEToH() == 0);
// 		}

// 		//On 4 channel scopes, we can use one channel from the left bank (AB) and one from the right (CD)
// 		else
// 		{
// 			//Can enable a left bank channel if there's none in use
// 			if(i < 2)
// 				return (GetEnabledAnalogChannelCountAToB() == 0);

// 			//Can enable a right bank channel if there's none in use
// 			else
// 				return (GetEnabledAnalogChannelCountCToD() == 0);
// 		}
// 	}

// 	//2.5 Gsps allows more stuff
// 	else if(rate >= RATE_2P5GSPS)
// 	{
// 		//If we already have too many channels/MSO pods active, we're out of RAM bandwidth.
// 		if(EnabledChannelCount >= 4)
// 			return false;

// 		//No banking restrictions for MSO pods if we have enough memory bandwidth
// 		else if(IsChannelIndexDigital(i))
// 			return true;

// 		//6403E allows up to 2 channels, one AB and one CD
// 		else if(m_series == SERIES_6403E)
// 		{
// 			//Can enable a left bank channel if there's none in use
// 			if(i < 2)
// 				return (GetEnabledAnalogChannelCountAToB() == 0);

// 			//Can enable a right bank channel if there's none in use
// 			else
// 				return (GetEnabledAnalogChannelCountCToD() == 0);
// 		}

// 		//8 channel scopes allow up to 4 channels but only one from A/B, C/D, E/F, G/H
// 		else if(m_analogChannelCount == 8)
// 		{
// 			if(i < 2)
// 				return (GetEnabledAnalogChannelCountAToB() == 0);
// 			else if(i < 4)
// 				return (GetEnabledAnalogChannelCountCToD() == 0);
// 			else if(i < 6)
// 				return (GetEnabledAnalogChannelCountEToF() == 0);
// 			else
// 				return (GetEnabledAnalogChannelCountGToH() == 0);
// 		}

// 		//On 4 channel scopes, we can run everything at 2.5 Gsps
// 		else
// 			return true;
// 	}

// 	//1.25 Gsps - just RAM bandwidth check
// 	else if( (rate >= RATE_1P25GSPS) && (EnabledChannelCount <= 7) )
// 		return true;

// 	//Slow enough that there's no capacity limits
// 	else
// 		return true;
// }

// /**
// 	@brief Checks if we can enable a channel on a 6000 series scope configured for 10-bit ADC resolution
//  */
// bool LiteXOscilloscope::CanEnableChannel6000Series10Bit(size_t i)
// {
// 	int64_t rate = GetSampleRate();
// 	size_t EnabledChannelCount = GetEnabledAnalogChannelCount() + GetEnabledDigitalPodCount();

// 	//5 Gsps is only allowed on a single channel/pod
// 	if(rate >= RATE_5GSPS)
// 		return (EnabledChannelCount == 0);

// 	//2.5 Gsps is allowed up to two channels/pods
// 	else if(rate >= RATE_2P5GSPS)
// 	{
// 		//Out of bandwidth
// 		if(EnabledChannelCount >= 2)
// 			return false;

// 		//No banking restrictions on MSO pods
// 		else if(IsChannelIndexDigital(i))
// 			return true;

// 		//8 channel scopes require the two channels to be in separate banks
// 		else if(m_analogChannelCount == 8)
// 		{
// 			//Can enable a left bank channel if there's none in use
// 			if(i < 4)
// 				return (GetEnabledAnalogChannelCountAToD() == 0);

// 			//Can enable a right bank channel if there's none in use
// 			else
// 				return (GetEnabledAnalogChannelCountEToH() == 0);
// 		}

// 		//No banking restrictions on 4 channel scopes
// 		else
// 			return true;
// 	}

// 	//1.25 Gsps is allowed up to 4 total channels/pods with no banking restrictions
// 	else if(rate >= RATE_1P25GSPS)
// 		return (EnabledChannelCount <= 3);

// 	//625 Msps allowed up to 8 total channels/pods with no banking restrictions
// 	else if(rate >= RATE_625MSPS)
// 		return (EnabledChannelCount <= 7);

// 	//Slow enough that there's no capacity limits
// 	else
// 		return true;
// }

// /**
// 	@brief Checks if we can enable a channel on a 6000 series scope configured for 12-bit ADC resolution
//  */
// bool LiteXOscilloscope::CanEnableChannel6000Series12Bit(size_t i)
// {
// 	int64_t rate = GetSampleRate();

// 	//Too many channels enabled?
// 	if(GetEnabledAnalogChannelCount() >= 2)
// 		return false;

// 	else if(rate > RATE_1P25GSPS)
// 		return false;

// 	//No banking restrictions on MSO pods
// 	else if(IsChannelIndexDigital(i))
// 		return true;

// 	else if(m_analogChannelCount == 8)
// 	{
// 		//Can enable a left bank channel if there's none in use
// 		if(i < 4)
// 			return (GetEnabledAnalogChannelCountAToD() == 0);

// 		//Can enable a right bank channel if there's none in use
// 		else
// 			return (GetEnabledAnalogChannelCountEToH() == 0);
// 	}

// 	else
// 	{
// 		//Can enable a left bank channel if there's none in use
// 		if(i < 2)
// 			return (GetEnabledAnalogChannelCountAToB() == 0);

// 		//Can enable a right bank channel if there's none in use
// 		else
// 			return (GetEnabledAnalogChannelCountCToD() == 0);
// 	}
// }

// bool LiteXOscilloscope::Is10BitModeAvailable()
// {
// 	//FlexRes only available on one series at the moment
// 	if(m_series != SERIES_6x2xE)
// 		return false;

// 	int64_t rate = GetSampleRate();
// 	size_t EnabledChannelCount = GetEnabledAnalogChannelCount() + GetEnabledDigitalPodCount();

// 	//5 Gsps is easy, just a bandwidth cap
// 	if(rate >= RATE_5GSPS)
// 		return (EnabledChannelCount <= 1);

// 	//2.5 Gsps has banking restrictions on 8 channel scopes
// 	else if(rate >= RATE_2P5GSPS)
// 	{
// 		if(EnabledChannelCount > 2)
// 			return false;

// 		else if(m_analogChannelCount == 8)
// 		{
// 			if(GetEnabledAnalogChannelCountAToB() > 1)
// 				return false;
// 			else if(GetEnabledAnalogChannelCountCToD() > 1)
// 				return false;
// 			else if(GetEnabledAnalogChannelCountEToF() > 1)
// 				return false;
// 			else if(GetEnabledAnalogChannelCountGToH() > 1)
// 				return false;
// 			else
// 				return true;
// 		}

// 		else
// 			return true;
// 	}

// 	//1.25 Gsps and 625 Msps are just bandwidth caps
// 	else if(rate >= RATE_1P25GSPS)
// 		return (EnabledChannelCount <= 4);
// 	else if(rate >= RATE_625MSPS)
// 		return (EnabledChannelCount <= 8);

// 	//No capacity limits
// 	else
// 		return true;
// }

// bool LiteXOscilloscope::Is12BitModeAvailable()
// {
// 	//FlexRes only available on one series at the moment
// 	if(m_series != SERIES_6x2xE)
// 		return false;

// 	int64_t rate = GetSampleRate();

// 	//12 bit mode only available at 1.25 Gsps and below
// 	if(rate > RATE_1P25GSPS)
// 		return false;

// 	//1.25 Gsps and below have the same banking restrictions: at most one channel from the left and right half
// 	else
// 	{
// 		if(m_analogChannelCount == 8)
// 			return (GetEnabledAnalogChannelCountAToD() <= 1) && (GetEnabledAnalogChannelCountEToH() <= 1);
// 		else
// 			return (GetEnabledAnalogChannelCountAToB() <= 1) && (GetEnabledAnalogChannelCountCToD() <= 1);
// 	}
// }


