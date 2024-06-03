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

#include "scopehal.h"
#include "TSLitexOscilloscope.h"
#include "EdgeTrigger.h"

#include "thunderscope.h"

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <inttypes.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>

#include <chrono>
#include <iostream>
#include <thread>


#ifdef _WIN32
#include <Windows.h>
#endif

//#define OPTPARSE_IMPLEMENTATION
//#include "optparse.h"

#include "liblitepcie.h"

// Test low-level library functions
#include "../src/spi.h"
#include "../src/i2c.h"
#include "../src/gpio.h"

#include "../src/ts_channel.h"
#include "../src/samples.h"

#if !defined(_WIN32)
#define INVALID_HANDLE_VALUE (-1)
#endif


#ifdef _WIN32
#define FILE_FLAGS  (FILE_ATTRIBUTE_NORMAL)
#else
#define FILE_FLAGS  (O_RDWR)
#endif


/* Parameters */
/*------------*/
#define TS_TEST_SAMPLE_FILE     "test_data.bin"


/* Variables */
/*-----------*/

static int litepcie_device_num;

uint32_t AFE_CONTROL_LDO_EN = (1 << 0);
uint32_t AFE_CONTROL_COUPLING = (1 << 8);
uint32_t AFE_CONTROL_ATTENUATION = (1 << 16);

uint32_t AFE_STATUS_LDO_PWR_GOOD = (1 << 0);

uint32_t AFE_AC_COUPLING = 0;
uint32_t AFE_DC_COUPLING = 1;

uint32_t AFE_1X_ATTENUATION = 0;
uint32_t AFE_10X_ATTENUATION = 1;

// ADC Constants------------------------------------------------------------------------------------

uint32_t ADC_CONTROL_LDO_EN = (1 << 0);
uint32_t ADC_CONTROL_PLL_EN = (1 << 1);
uint32_t ADC_CONTROL_RST = (1 << 2);
uint32_t ADC_CONTROL_PWR_DOWN = (1 << 3);

uint32_t ADC_STATUS_LDO_PWR_GOOD = (1 << 0);

uint32_t _SPI_CONTROL_START = (1 << 0);
uint32_t _SPI_CONTROL_LENGTH = (1 << 8);
uint32_t _SPI_STATUS_DONE = (1 << 0);


/* Main */
/*------*/

void configure_frontend_ldo(file_t fd, uint32_t enable) {
    uint32_t control_value = litepcie_readl(fd, CSR_FRONTEND_CONTROL_ADDR);
    control_value &= ~(1 * AFE_CONTROL_LDO_EN);
    control_value |= (enable * AFE_CONTROL_LDO_EN);
    litepcie_writel(fd, CSR_FRONTEND_CONTROL_ADDR, control_value);
}

void configure_adc_ldo(file_t fd, uint32_t enable) {
    uint32_t control_value = litepcie_readl(fd, CSR_ADC_CONTROL_ADDR);
    control_value &= ~(1 * ADC_CONTROL_LDO_EN);
    control_value |= (enable * ADC_CONTROL_LDO_EN);
    litepcie_writel(fd, CSR_ADC_CONTROL_ADDR, control_value);
}

void configure_pll_en(file_t fd, uint32_t enable) {
    uint32_t control_value = litepcie_readl(fd, CSR_ADC_CONTROL_ADDR);
    control_value &= ~(1 * ADC_CONTROL_PLL_EN);
    control_value |= (enable * ADC_CONTROL_PLL_EN);
    litepcie_writel(fd, CSR_ADC_CONTROL_ADDR, control_value);
}

void control_led(file_t fd, uint32_t enable) {
    uint32_t control_value = litepcie_readl(fd, CSR_LEDS_OUT_ADDR);
    control_value &= ~(1 * AFE_STATUS_LDO_PWR_GOOD);
    control_value |= (enable * AFE_STATUS_LDO_PWR_GOOD);
    litepcie_writel(fd, CSR_LEDS_OUT_ADDR, control_value);
}


// Functioning returning 
// current time
auto now()
{
    return std::chrono::steady_clock::now();
}

// Function calculating sleep time 
// with 500ms delay
auto awake_time()
{
    using std::chrono::operator"" ms;
    return now() + 500ms;
}

using namespace std;

#define RATE_1GSPS	(INT64_C(1000) * INT64_C(1000) * INT64_C(1000))
//#define RATE_625MSPS	(INT64_C(625)  * INT64_C(1000) * INT64_C(1000))

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Construction / destruction

TSLitexOscilloscope::TSLitexOscilloscope(SCPITransport* transport)
	: SCPIDevice(transport, false)
	, SCPIInstrument(transport)
	, RemoteBridgeOscilloscope(transport)
	, m_diag_hardwareWFMHz(FilterParameter::TYPE_FLOAT, Unit(Unit::UNIT_HZ))
	, m_diag_receivedWFMHz(FilterParameter::TYPE_FLOAT, Unit(Unit::UNIT_HZ))
	, m_diag_totalWFMs(FilterParameter::TYPE_INT, Unit(Unit::UNIT_COUNTS))
	, m_diag_droppedWFMs(FilterParameter::TYPE_INT, Unit(Unit::UNIT_COUNTS))
	, m_diag_droppedPercent(FilterParameter::TYPE_FLOAT, Unit(Unit::UNIT_PERCENT))
{
 	file_t fd;
	printf(LITEPCIE_CTRL_NAME(0));
	fd = litepcie_open(LITEPCIE_CTRL_NAME(0), FILE_FLAGS);
    if(fd == INVALID_HANDLE_VALUE) {
        fprintf(stderr, "Could not init driver\n");
        exit(1);
    }

	printf("\x1b[1m[> Scratch register test:\x1b[0m\n");
    printf("-------------------------\n");


    /* Write to scratch register. */
    printf("Write 0x12345678 to Scratch register:\n");
    litepcie_writel(fd, CSR_CTRL_SCRATCH_ADDR, 0x12345678);
    printf("Read: 0x%08x\n", litepcie_readl(fd, CSR_CTRL_SCRATCH_ADDR));

    /* Read from scratch register. */
    printf("Write 0xdeadbeef to Scratch register:\n");
    litepcie_writel(fd, CSR_CTRL_SCRATCH_ADDR, 0xdeadbeef);
    printf("Read: 0x%08x\n", litepcie_readl(fd, CSR_CTRL_SCRATCH_ADDR));

    printf("Enabling frontend LDO:\n");
    configure_frontend_ldo(fd, 1);

    printf("Enabling ADC LDO:\n");
    configure_adc_ldo(fd, 1);

    printf("Disabling PLL EN & waiting 500ms:\n");
    configure_pll_en(fd, 0);

    std::this_thread::sleep_until(awake_time());

    printf("Enabling PLL EN:\n");
    configure_pll_en(fd, 1);
	
    std::this_thread::sleep_until(awake_time());

    i2c_t i2cDev;
    i2cDev.fd = fd;

    for (unsigned char addr = 0; addr < 0x8f; addr++) {
        i2cDev.devAddr = addr;
        bool result = i2c_poll(i2cDev);
        if (addr % 0x10 == 0) {
            printf("\n0x%02X", addr);
        }
        if (result) {
            printf(" %02X", addr);
        }
        else {
            printf(" --");
        }
    }

    // spi_bus_t spimaster;
    // spi_bus_init(&spimaster, fd, CSR_MAIN_SPI_BASE, CSR_MAIN_SPI_CS_SEL_SIZE);

    // uint8_t data[2] = {0x01, 0x02};

    // for (int i = 0; i < CSR_MAIN_SPI_CS_SEL_SIZE; i++) {
    //     spi_dev_t spiDev;
    //     spi_dev_init(&spiDev, &spimaster, i);
    //     for (int reg = 0; reg < 10; reg++) {
    //         spi_write(spiDev, reg, data, 2);
    //         spi_busy_wait(spiDev);
    //     }
    // }

	/* Close LitePCIe device. */
    litepcie_close(fd);

	m_analogChannelCount = 4;

	//Add analog channel objects
	for(size_t i = 0; i < m_analogChannelCount; i++)
	{
		//Hardware name of the channel
		string chname = to_string(i);

		//Create the channel
		auto chan = new OscilloscopeChannel(
			this,
			chname,
			GetChannelColor(i),
			Unit(Unit::UNIT_FS),
			Unit(Unit::UNIT_VOLTS),
			Stream::STREAM_TYPE_ANALOG,
			i);
		m_channels.push_back(chan);

		string nicename = "ch" + chname;
		chan->SetDisplayName(nicename);

		//Set initial configuration so we have a well-defined instrument state
		m_channelAttenuations[i] = 10;
		SetChannelCoupling(i, OscilloscopeChannel::COUPLE_DC_1M);
		SetChannelOffset(i, 0,  0);
		SetChannelVoltageRange(i, 0, 5);
	}

	//Set initial memory configuration.
	SetSampleRate(1000000L);
	SetSampleDepth(10000);

	//Set up the data plane socket
	//auto csock = dynamic_cast<SCPITwinLanTransport*>(m_transport);
	// if(!csock)
	// 	LogFatal("TSLitexOscilloscope expects a SCPITwinLanTransport\n");

	//Configure the trigger
	auto trig = new EdgeTrigger(this);
	trig->SetType(EdgeTrigger::EDGE_RISING);
	trig->SetLevel(0);
	trig->SetInput(0, StreamDescriptor(GetOscilloscopeChannel(0)));
	SetTrigger(trig);
	PushTrigger();
	SetTriggerOffset(1000000000000); //1ms to allow trigphase interpolation

	m_diagnosticValues["Hardware WFM/s"] = &m_diag_hardwareWFMHz;
	m_diagnosticValues["Received WFM/s"] = &m_diag_receivedWFMHz;
	m_diagnosticValues["Total Waveforms Received"] = &m_diag_totalWFMs;
	m_diagnosticValues["Received Waveforms Dropped"] = &m_diag_droppedWFMs;
	m_diagnosticValues["% Received Waveforms Dropped"] = &m_diag_droppedPercent;

	ResetPerCaptureDiagnostics();

	//Initialize waveform buffers
	// for(size_t i=0; i<m_analogChannelCount; i++)
	// {
	// 	m_analogRawWaveformBuffers.push_back(std::make_unique<AcceleratorBuffer<int8_t> >());
	// 	m_analogRawWaveformBuffers[i]->SetCpuAccessHint(AcceleratorBuffer<int8_t>::HINT_LIKELY);
	// 	m_analogRawWaveformBuffers[i]->SetGpuAccessHint(AcceleratorBuffer<int8_t>::HINT_LIKELY);
	// }

	// //Create Vulkan objects for the waveform conversion
	// m_queue = g_vkQueueManager->GetComputeQueue("TSLitexOscilloscope.queue");
	// vk::CommandPoolCreateInfo poolInfo(
	// 	vk::CommandPoolCreateFlagBits::eTransient | vk::CommandPoolCreateFlagBits::eResetCommandBuffer,
	// 	m_queue->m_family );
	// m_pool = make_unique<vk::raii::CommandPool>(*g_vkComputeDevice, poolInfo);

	// vk::CommandBufferAllocateInfo bufinfo(**m_pool, vk::CommandBufferLevel::ePrimary, 1);
	// m_cmdBuf = make_unique<vk::raii::CommandBuffer>(
	// 	std::move(vk::raii::CommandBuffers(*g_vkComputeDevice, bufinfo).front()));

	// if(g_hasDebugUtils)
	// {
	// 	string poolname = "TSLitexOscilloscope.pool";
	// 	string bufname = "TSLitexOscilloscope.cmdbuf";

	// 	g_vkComputeDevice->setDebugUtilsObjectNameEXT(
	// 		vk::DebugUtilsObjectNameInfoEXT(
	// 			vk::ObjectType::eCommandPool,
	// 			reinterpret_cast<uint64_t>(static_cast<VkCommandPool>(**m_pool)),
	// 			poolname.c_str()));

	// 	g_vkComputeDevice->setDebugUtilsObjectNameEXT(
	// 		vk::DebugUtilsObjectNameInfoEXT(
	// 			vk::ObjectType::eCommandBuffer,
	// 			reinterpret_cast<uint64_t>(static_cast<VkCommandBuffer>(**m_cmdBuf)),
	// 			bufname.c_str()));
	// }

	// m_conversionPipeline = make_unique<ComputePipeline>(
	// 		"shaders/Convert8BitSamples.spv", 2, sizeof(ConvertRawSamplesShaderArgs) );

	
}


void TSLitexOscilloscope::ResetPerCaptureDiagnostics()
{
	m_diag_hardwareWFMHz.SetFloatVal(0);
	m_diag_receivedWFMHz.SetFloatVal(0);
	m_diag_totalWFMs.SetIntVal(0);
	m_diag_droppedWFMs.SetIntVal(0);
	m_diag_droppedPercent.SetFloatVal(1);
	m_receiveClock.Reset();
}

/**
	@brief Color the channels based on TSLitex's standard color sequence (blue-red-green-yellow-purple-gray-cyan-magenta)
 */
string TSLitexOscilloscope::GetChannelColor(size_t i)
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

void TSLitexOscilloscope::IdentifyHardware()
{
	//Ask the scope how many channels it has
	//m_transport->SendCommand("CHANS?");
	m_analogChannelCount = 4; //stoi(m_transport->ReadReply());
}

TSLitexOscilloscope::~TSLitexOscilloscope()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Accessors

unsigned int TSLitexOscilloscope::GetInstrumentTypes() const
{
	return Instrument::INST_OSCILLOSCOPE;
}

uint32_t TSLitexOscilloscope::GetInstrumentTypesForChannel(size_t i) const
{
	return Instrument::INST_OSCILLOSCOPE;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Device interface functions

string TSLitexOscilloscope::GetDriverNameInternal()
{
	return "tsLitex";
}

void TSLitexOscilloscope::FlushConfigCache()
{
	lock_guard<recursive_mutex> lock(m_cacheMutex);
}

// bool TSLitexOscilloscope::IsChannelEnabled(size_t i)
// {
// 	//ext trigger should never be displayed
// 	if(i == m_extTrigChannel->GetIndex())
// 		return false;

// 	lock_guard<recursive_mutex> lock(m_cacheMutex);
// 	return m_channelsEnabled[i];
// }

// void TSLitexOscilloscope::EnableChannel(size_t i)
// {
// 	// TODO: libtslitex call here
// 	//RemoteBridgeOscilloscope::EnableChannel(i);
// }

// void TSLitexOscilloscope::DisableChannel(size_t i)
// {
// 	{
// 		lock_guard<recursive_mutex> lock(m_cacheMutex);
// 		m_channelsEnabled[i] = false;
// 	}

// 	lock_guard<recursive_mutex> lock(m_mutex);
// 	// TODO: libtslitex call here
// 	//RemoteBridgeOscilloscope::DisableChannel(i);
// }

vector<OscilloscopeChannel::CouplingType> TSLitexOscilloscope::GetAvailableCouplings(size_t /*i*/)
{
	vector<OscilloscopeChannel::CouplingType> ret;
	
	ret.push_back(OscilloscopeChannel::COUPLE_DC_1M);
	
	// ret.push_back(OscilloscopeChannel::COUPLE_AC_1M);
	// ret.push_back(OscilloscopeChannel::COUPLE_DC_50);
	// ret.push_back(OscilloscopeChannel::COUPLE_GND);

	return ret;
}

double TSLitexOscilloscope::GetChannelAttenuation(size_t i)
{
	// if(GetOscilloscopeChannel(i) == m_extTrigChannel)
	// 	return 1;

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	return m_channelAttenuations[i];
}

void TSLitexOscilloscope::SetChannelAttenuation(size_t i, double atten)
{
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	double oldAtten = m_channelAttenuations[i];
	m_channelAttenuations[i] = atten;

	//Rescale channel voltage range and offset
	double delta = atten / oldAtten;
	m_channelVoltageRanges[i] *= delta;
	m_channelOffsets[i] *= delta;
}

unsigned int TSLitexOscilloscope::GetChannelBandwidthLimit(size_t /*i*/)
{
	return 0;
}

void TSLitexOscilloscope::SetChannelBandwidthLimit(size_t /*i*/, unsigned int /*limit_mhz*/)
{
}

OscilloscopeChannel* TSLitexOscilloscope::GetExternalTrigger()
{
	//FIXME
	return NULL;
}

Oscilloscope::TriggerMode TSLitexOscilloscope::PollTrigger()
{
	//Always report "triggered" so we can block on AcquireData() in ScopeThread
	//TODO: peek function of some sort?
	return TRIGGER_MODE_TRIGGERED;
}




bool TSLitexOscilloscope::AcquireData()
{
	const uint8_t r = 'K';
	m_transport->SendRawData(1, &r);

	//Read the sequence number of the current waveform
	uint32_t seqnum;
	if(!m_transport->ReadRawData(sizeof(seqnum), (uint8_t*)&seqnum))
		return false;

	//Read the number of channels in the current waveform
	uint16_t numChannels;
	if(!m_transport->ReadRawData(sizeof(numChannels), (uint8_t*)&numChannels))
		return false;

	//Get the sample interval.
	//May be different from m_srate if we changed the rate after the trigger was armed
	uint64_t fs_per_sample;
	if(!m_transport->ReadRawData(sizeof(fs_per_sample), (uint8_t*)&fs_per_sample))
		return false;

	//Get the de-facto trigger position.
	int64_t trigger_fs;
	if(!m_transport->ReadRawData(sizeof(trigger_fs), (uint8_t*)&trigger_fs))
		return false;

	{
		lock_guard<recursive_mutex> lock(m_mutex);
		if (m_triggerOffset != trigger_fs)
		{
			AddDiagnosticLog("Correcting trigger offset by " + to_string(m_triggerOffset - trigger_fs));
			m_triggerOffset = trigger_fs;
		}
	}

	//Get the de-facto hardware capture rate.
	double wfms_s;
	if(!m_transport->ReadRawData(sizeof(wfms_s), (uint8_t*)&wfms_s))
		return false;

	m_diag_hardwareWFMHz.SetFloatVal(wfms_s);

	//Acquire data for each channel
	uint8_t chnum;
	uint64_t memdepth;
	float config[3];
	SequenceSet s;
	double t = GetTime();
	int64_t fs = (t - floor(t)) * FS_PER_SECOND;

	//Analog channels get processed separately
	vector<uint8_t*> abufs;
	vector<UniformAnalogWaveform*> awfms;
	vector<float> scales;
	vector<float> offsets;

	for(size_t i=0; i<numChannels; i++)
	{
		//Get channel ID and memory depth (samples, not bytes)
		if(!m_transport->ReadRawData(sizeof(chnum), (uint8_t*)&chnum))
			return false;
		if(!m_transport->ReadRawData(sizeof(memdepth), (uint8_t*)&memdepth))
			return false;

		uint8_t* buf = new uint8_t[memdepth];

		//Analog channels
		if(chnum < m_analogChannelCount)
		{
			abufs.push_back(buf);

			//Scale and offset are sent in the header since they might have changed since the capture began
			if(!m_transport->ReadRawData(sizeof(config), (uint8_t*)&config))
				return false;
			float scale = config[0];
			float offset = config[1];
			float trigphase = -config[2] * fs_per_sample;
			scale *= GetChannelAttenuation(chnum);
			offset *= GetChannelAttenuation(chnum);

			bool clipping;
			if(!m_transport->ReadRawData(sizeof(clipping), (uint8_t*)&clipping))
				return false;

			//TODO: stream timestamp from the server

			if(!m_transport->ReadRawData(memdepth * sizeof(int8_t), (uint8_t*)buf))
				return false;

			for (uint64_t ii = 0; ii < memdepth; ii++)
			{
				int8_t* p = (int8_t*)&buf[ii];
				if ((*p == -128) || (*p == 127))
				{
					clipping = 1;
				}
			}

			//Create our waveform
			UniformAnalogWaveform* cap = AllocateAnalogWaveform(m_nickname + "." + GetChannel(i)->GetHwname());
			cap->m_timescale = fs_per_sample;
			cap->m_triggerPhase = trigphase;
			cap->m_startTimestamp = time(NULL);
			cap->m_startFemtoseconds = fs;
			if (clipping)
				cap->m_flags |= WaveformBase::WAVEFORM_CLIPPING;

			cap->Resize(memdepth);
			awfms.push_back(cap);
			scales.push_back(scale);
			offsets.push_back(offset);

			s[GetOscilloscopeChannel(chnum)] = cap;
		} else {
			LogFatal("???\n");
		}
	}

	//Process analog captures in parallel
	#pragma omp parallel for
	for(size_t i=0; i<awfms.size(); i++)
	{
		auto cap = awfms[i];
		cap->PrepareForCpuAccess();
		Convert8BitSamples(
			(float*)&cap->m_samples[0],
			(int8_t*)abufs[i],
			scales[i],
			offsets[i],
			cap->m_samples.size());
		delete[] abufs[i];
		cap->MarkModifiedFromCpu();
	}

	FilterParameter* param = &m_diag_totalWFMs;
	int total = param->GetIntVal() + 1;
	param->SetIntVal(total);

	param = &m_diag_droppedWFMs;
	int dropped = param->GetIntVal();

	//Save the waveforms to our queue
	m_pendingWaveformsMutex.lock();
	m_pendingWaveforms.push_back(s);

	while (m_pendingWaveforms.size() > 2)
	{
		SequenceSet set = *m_pendingWaveforms.begin();
		for(auto it : set)
			delete it.second;
		m_pendingWaveforms.pop_front();

		dropped++;
	}

	m_pendingWaveformsMutex.unlock();

	param->SetIntVal(dropped);

	param = &m_diag_droppedPercent;
	param->SetFloatVal((float)dropped / (float)total);

	m_receiveClock.Tick();
	m_diag_receivedWFMHz.SetFloatVal(m_receiveClock.GetAverageHz());

	//If this was a one-shot trigger we're no longer armed
	if(m_triggerOneShot)
		m_triggerArmed = false;

	return true;
}


// TODO: w/ gpu

// bool TSLitexOscilloscope::AcquireData()
// {
// 	#pragma pack(push, 1)
// 	struct
// 	{
// 		//Number of channels in the current waveform
// 		uint16_t numChannels;

// 		//Sample interval.
// 		//May be different from m_srate if we changed the rate after the trigger was armed
// 		int64_t fs_per_sample;
// 	} wfmhdrs;
// 	#pragma pack(pop)

// 	//Read global waveform settings (independent of each channel)
// 	if(!m_transport->ReadRawData(sizeof(wfmhdrs), (uint8_t*)&wfmhdrs))
// 		return false;
// 	uint16_t numChannels = wfmhdrs.numChannels;
// 	int64_t fs_per_sample = wfmhdrs.fs_per_sample;

// 	//Acquire data for each channel
// 	size_t chnum;
// 	size_t memdepth;
// 	float config[3];
// 	SequenceSet s;
// 	double t = GetTime();
// 	int64_t fs = (t - floor(t)) * FS_PER_SECOND;

// 	//Analog channels get processed separately
// 	vector<UniformAnalogWaveform*> awfms;
// 	vector<size_t> achans;
// 	vector<float> scales;
// 	vector<float> offsets;

// 	for(size_t i=0; i<numChannels; i++)
// 	{
// 		size_t tmp[2];

// 		//Get channel ID and memory depth (samples, not bytes)
// 		if(!m_transport->ReadRawData(sizeof(tmp), (uint8_t*)&tmp))
// 			return false;
// 		chnum = tmp[0];
// 		memdepth = tmp[1];

// 		//Analog channels
// 		if(chnum < m_analogChannelCount)
// 		{
// 			auto& abuf = m_analogRawWaveformBuffers[chnum];
// 			abuf->resize(memdepth);
// 			abuf->PrepareForCpuAccess();
// 			achans.push_back(chnum);

// 			//Scale and offset are sent in the header since they might have changed since the capture began
// 			if(!m_transport->ReadRawData(sizeof(config), (uint8_t*)&config))
// 				return false;
// 			float scale = config[0];
// 			float offset = config[1];
// 			float trigphase = -config[2] * fs_per_sample;
// 			scale *= GetChannelAttenuation(chnum);
// 			offset *= GetChannelAttenuation(chnum);

// 			//TODO: stream timestamp from the server
// 			if(!m_transport->ReadRawData(memdepth * sizeof(int16_t), reinterpret_cast<uint8_t*>(abuf->GetCpuPointer())))
// 				return false;

// 			abuf->MarkModifiedFromCpu();

// 			//Create our waveform
// 			auto cap = AllocateAnalogWaveform(m_nickname + "." + GetOscilloscopeChannel(i)->GetHwname());
// 			cap->m_timescale = fs_per_sample;
// 			cap->m_triggerPhase = trigphase;
// 			cap->m_startTimestamp = time(NULL);
// 			cap->m_startFemtoseconds = fs;
// 			cap->Resize(memdepth);
// 			awfms.push_back(cap);
// 			scales.push_back(scale);
// 			offsets.push_back(offset);

// 			s[GetOscilloscopeChannel(chnum)] = cap;
// 		}
// 	}

// 	//If we have GPU support for int8, we can do the conversion on the card
// 	//But only do this if we also have push-descriptor support, because doing N separate dispatches is likely
// 	//to be slower than a parallel CPU-side conversion
// 	//Note also that a strict benchmarking here may be slower than the CPU version due to transfer latency,
// 	//but having the waveform on the GPU now means we don't have to do *that* later.
// 	if(g_hasShaderInt8 && g_hasPushDescriptor)
// 	{
// 		m_cmdBuf->begin({});

// 		m_conversionPipeline->Bind(*m_cmdBuf);

// 		for(size_t i=0; i<awfms.size(); i++)
// 		{
// 			auto cap = awfms[i];

// 			m_conversionPipeline->BindBufferNonblocking(0, cap->m_samples, *m_cmdBuf, true);
// 			m_conversionPipeline->BindBufferNonblocking(1, *m_analogRawWaveformBuffers[achans[i]], *m_cmdBuf);

// 			ConvertRawSamplesShaderArgs args;
// 			args.size = cap->size();
// 			args.gain = scales[i];
// 			args.offset = -offsets[i];

// 			m_conversionPipeline->DispatchNoRebind(*m_cmdBuf, args, GetComputeBlockCount(cap->size(), 64));

// 			cap->MarkModifiedFromGpu();
// 		}

// 		m_cmdBuf->end();
// 		m_queue->SubmitAndBlock(*m_cmdBuf);
// 	}
// 	else
// 	{
// 		//Fallback path
// 		//Process analog captures in parallel
// 		#pragma omp parallel for
// 		for(size_t i=0; i<awfms.size(); i++)
// 		{
// 			auto cap = awfms[i];
// 			cap->PrepareForCpuAccess();
// 			Convert8BitSamples(
// 				cap->m_samples.GetCpuPointer(),
// 				m_analogRawWaveformBuffers[achans[i]]->GetCpuPointer(),
// 				scales[i],
// 				-offsets[i],
// 				cap->size());

// 			cap->MarkSamplesModifiedFromCpu();
// 		}
// 	}

// 	//Save the waveforms to our queue
// 	m_pendingWaveformsMutex.lock();
// 	m_pendingWaveforms.push_back(s);
// 	m_pendingWaveformsMutex.unlock();

// 	//If this was a one-shot trigger we're no longer armed
// 	if(m_triggerOneShot)
// 		m_triggerArmed = false;

// 	return true;
// }


void TSLitexOscilloscope::Start()
{
	m_triggerArmed = true; //FIXME

	RemoteBridgeOscilloscope::Start();
	ResetPerCaptureDiagnostics();
}

void TSLitexOscilloscope::StartSingleTrigger()
{
	RemoteBridgeOscilloscope::StartSingleTrigger();
	ResetPerCaptureDiagnostics();
}

void TSLitexOscilloscope::ForceTrigger()
{
	RemoteBridgeOscilloscope::ForceTrigger();
	ResetPerCaptureDiagnostics();
}


// bool TSLitexOscilloscope::IsTriggerArmed()
// {
// 	return m_triggerArmed;
// }

// bool TSLitexOscilloscope::CanInterleave()
// {
// 	return false;
// }

// vector<uint64_t> TSLitexOscilloscope::GetSampleDepthsInterleaved()
// {
// 	//interleaving not supported
// 	vector<uint64_t> ret;
// 	return ret;
// }

// uint64_t TSLitexOscilloscope::GetSampleRate()
// {
// 	return m_srate;
// }

// uint64_t TSLitexOscilloscope::GetSampleDepth()
// {
// 	return m_mdepth;
// }

// void TSLitexOscilloscope::SetSampleDepth(uint64_t depth)
// {
// 	// lock_guard<recursive_mutex> lock(m_mutex);
// 	// m_transport->SendCommand(string("DEPTH ") + to_string(depth));
// 	// m_mdepth = depth;
// }

// void TSLitexOscilloscope::SetSampleRate(uint64_t rate)
// {
// 	// m_srate = rate;

// 	// lock_guard<recursive_mutex> lock(m_mutex);
// 	// m_transport->SendCommand( string("RATE ") + to_string(rate));
// }

// void TSLitexOscilloscope::SetTriggerOffset(int64_t offset)
// {
// 	lock_guard<recursive_mutex> lock(m_mutex);

// 	//Don't allow setting trigger offset beyond the end of the capture
// 	int64_t captureDuration = GetSampleDepth() * FS_PER_SECOND / GetSampleRate();
// 	m_triggerOffset = min(offset, captureDuration);

// 	PushTrigger();
// }

// int64_t TSLitexOscilloscope::GetTriggerOffset()
// {
// 	return m_triggerOffset;
// }

// bool TSLitexOscilloscope::IsInterleaving()
// {
// 	//interleaving is done automatically in hardware based on sample rate, no user facing switch for it
// 	return false;
// }

// bool TSLitexOscilloscope::SetInterleaving(bool /*combine*/)
// {
// 	//interleaving is done automatically in hardware based on sample rate, no user facing switch for it
// 	return false;
// }

// void TSLitexOscilloscope::PushTrigger()
// {
// 	auto et = dynamic_cast<EdgeTrigger*>(m_trigger);
// 	if(et)
// 		PushEdgeTrigger(et);

// 	else
// 		LogWarning("Unknown trigger type (not an edge)\n");

// 	ClearPendingWaveforms();
// }

// vector<Oscilloscope::AnalogBank> TSLitexOscilloscope::GetAnalogBanks()
// {
// 	vector<AnalogBank> banks;
// 	banks.push_back(GetAnalogBank(0));
// 	return banks;
// }

// Oscilloscope::AnalogBank TSLitexOscilloscope::GetAnalogBank(size_t /*channel*/)
// {
// 	AnalogBank bank;
// 	return bank;
// }

// bool TSLitexOscilloscope::IsADCModeConfigurable()
// {
// 	return false;
// }

// vector<string> TSLitexOscilloscope::GetADCModeNames(size_t /*channel*/)
// {
// 	//All scopes with variable resolution start at 8 bit and go up from there
// 	vector<string> ret;
// 	ret.push_back("8 Bit");
// 	if(Is10BitModeAvailable())
// 	{
// 		ret.push_back("10 Bit");
// 		if(Is12BitModeAvailable())
// 			ret.push_back("12 Bit");
// 	}
// 	return ret;
// }

// size_t TSLitexOscilloscope::GetADCMode(size_t /*channel*/)
// {
// 	return m_adcMode;
// }

// void TSLitexOscilloscope::SetADCMode(size_t /*channel*/, size_t mode)
// {
// 	m_adcMode = (ADCMode)mode;

// 	lock_guard<recursive_mutex> lock(m_mutex);
// 	switch(mode)
// 	{
// 		case ADC_MODE_8BIT:
// 			m_transport->SendCommand("BITS 8");
// 			break;

// 		case ADC_MODE_10BIT:
// 			m_transport->SendCommand("BITS 10");
// 			break;

// 		case ADC_MODE_12BIT:
// 			m_transport->SendCommand("BITS 12");
// 			break;

// 		default:
// 			LogWarning("TSLitexOscilloscope::SetADCMode requested invalid mode %zu, interpreting as 8 bit\n", mode);
// 			m_adcMode = ADC_MODE_8BIT;
// 			break;
// 	}
// }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Checking for validity of configurations

// /**
// 	@brief Returns the total number of analog channels which are currently enabled
//  */
// size_t TSLitexOscilloscope::GetEnabledAnalogChannelCount()
// {
// 	size_t ret = 0;
// 	for(size_t i=0; i<m_analogChannelCount; i++)
// 	{
// 		if(IsChannelEnabled(i))
// 			ret ++;
// 	}
// 	return ret;
// }

// /**
// 	@brief Returns the total number of analog channels in the requested range which are currently enabled
//  */
// size_t TSLitexOscilloscope::GetEnabledAnalogChannelCountRange(size_t start, size_t end)
// {
// 	if(end >= m_analogChannelCount)
// 		end = m_analogChannelCount - 1;

// 	size_t n = 0;
// 	for(size_t i = start; i <= end; i++)
// 	{
// 		if(IsChannelEnabled(i))
// 			n ++;
// 	}
// 	return n;
// }

bool TSLitexOscilloscope::CanEnableChannel(size_t i)
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
	LogWarning("TSLitexOscilloscope::CanEnableChannel: Unknown ADC mode\n");
	return true;
}

// bool TSLitexOscilloscope::Is10BitModeAvailable()
// {
// 	return false;
// }

// bool TSLitexOscilloscope::Is12BitModeAvailable()
// {
// 	return false;
// }


vector<uint64_t> TSLitexOscilloscope::GetSampleRatesInterleaved()
{
	//interleaving not supported
	vector<uint64_t> ret = {};
	return ret;
}

set<Oscilloscope::InterleaveConflict> TSLitexOscilloscope::GetInterleaveConflicts()
{
	//interleaving not supported
	set<Oscilloscope::InterleaveConflict> ret;
	return ret;
}

vector<uint64_t> TSLitexOscilloscope::GetSampleDepthsNonInterleaved()
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

	// 	ret.push_back(stol(depths.substr(istart, i-istart)));

	// 	//skip the comma
	// 	i++;
	// }

	ret.push_back(10000);

	return ret;
}

vector<uint64_t> TSLitexOscilloscope::GetSampleDepthsInterleaved()
{
	//interleaving not supported
	vector<uint64_t> ret;
	return ret;
}

bool TSLitexOscilloscope::IsInterleaving()
{
	//interleaving not supported
	return false;
}

bool TSLitexOscilloscope::SetInterleaving(bool /*combine*/)
{
	//interleaving not supported
	return false;
}

vector<uint64_t> TSLitexOscilloscope::GetSampleRatesNonInterleaved()
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
	// 	auto fs = stol(block);
	// 	auto hz = FS_PER_SECOND / fs;
	// 	ret.push_back(hz);

	// 	//skip the comma
	// 	i++;
	// }

	ret.push_back(1000000L);

	return ret;
}
