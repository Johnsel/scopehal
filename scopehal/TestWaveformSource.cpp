/***********************************************************************************************************************
*                                                                                                                      *
* libscopehal v0.1                                                                                                     *
*                                                                                                                      *
* Copyright (c) 2012-2021 Andrew D. Zonenberg and contributors                                                         *
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
/**
	@file
	@author Andrew D. Zonenberg
	@brief Implementation of TestWaveformSource
 */
#include "scopehal.h"
#include "TestWaveformSource.h"
#include <complex>

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

TestWaveformSource::TestWaveformSource(minstd_rand& rng)
	: m_rng(rng)	
	, m_blackmanHarrisComputePipeline("shaders/BlackmanHarrisWindow.spv", 2, sizeof(WindowFunctionArgs))
	, m_rectangularComputePipeline("shaders/RectangularWindow.spv", 2, sizeof(WindowFunctionArgs))
	, m_deEmbedComputePipeline("shaders/DeEmbedFilter.spv", 3, sizeof(uint32_t))
	, m_normalizeComputePipeline("shaders/DeEmbedNormalization.spv", 2, sizeof(FFTDeEmbedNormalizationArgs)
	, m_complexToMagnitudeComputePipeline("shaders/ComplexToMagnitude.spv", 2, sizeof(ComplexToMagnitudeArgs))
	)
	

{
#ifndef _APPLE_SILICON
	// m_forwardPlan = NULL;
	// m_reversePlan = NULL;

	m_cachedNumPoints = 0;
	m_cachedRawSize = 0;

	// m_forwardInBuf = NULL;
	// m_forwardOutBuf = NULL;
	// m_reverseOutBuf = NULL;
#endif

//Create Vulkan objects for the waveform conversion
	m_queue = g_vkQueueManager->GetComputeQueue("TestWaveformSource.queue");
	vk::CommandPoolCreateInfo poolInfo(
		vk::CommandPoolCreateFlagBits::eTransient | vk::CommandPoolCreateFlagBits::eResetCommandBuffer,
		m_queue->m_family );
	m_pool = make_unique<vk::raii::CommandPool>(*g_vkComputeDevice, poolInfo);

	vk::CommandBufferAllocateInfo bufinfo(**m_pool, vk::CommandBufferLevel::ePrimary, 1);
	m_cmdBuf = make_unique<vk::raii::CommandBuffer>(
		std::move(vk::raii::CommandBuffers(*g_vkComputeDevice, bufinfo).front()));

	m_cachedNumPoints = 0;
	// m_cachedMaxGain = 0;



	m_scalarTempBuf1.SetCpuAccessHint(AcceleratorBuffer<float>::HINT_NEVER);
	m_scalarTempBuf1.SetGpuAccessHint(AcceleratorBuffer<float>::HINT_LIKELY);

	m_vectorTempBuf1.SetCpuAccessHint(AcceleratorBuffer<float>::HINT_NEVER);
	m_vectorTempBuf1.SetGpuAccessHint(AcceleratorBuffer<float>::HINT_LIKELY);

	m_vectorTempBuf2.SetCpuAccessHint(AcceleratorBuffer<float>::HINT_NEVER);
	m_vectorTempBuf2.SetGpuAccessHint(AcceleratorBuffer<float>::HINT_LIKELY);

	m_vectorTempBuf3.SetCpuAccessHint(AcceleratorBuffer<float>::HINT_NEVER);
	m_vectorTempBuf3.SetGpuAccessHint(AcceleratorBuffer<float>::HINT_LIKELY);

	m_vectorTempBuf4.SetCpuAccessHint(AcceleratorBuffer<float>::HINT_NEVER);
	m_vectorTempBuf4.SetGpuAccessHint(AcceleratorBuffer<float>::HINT_LIKELY);
	

}

TestWaveformSource::~TestWaveformSource()
{
#ifndef _APPLE_SILICON
	// if(m_forwardPlan)
	// 	ffts_free(m_forwardPlan);
	// if(m_reversePlan)
	// 	ffts_free(m_reversePlan);

	// m_allocator.deallocate(m_forwardInBuf);
	// m_allocator.deallocate(m_forwardOutBuf);
	// m_allocator.deallocate(m_reverseOutBuf);

	// m_forwardPlan = NULL;
	// m_reversePlan = NULL;
	// m_forwardInBuf = NULL;
	// m_forwardOutBuf = NULL;
	// m_reverseOutBuf = NULL;
#endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Signal generation

/**
	@brief Generates a unit step
 */
WaveformBase* TestWaveformSource::GenerateStep(
	float vlo,
	float vhi,
	int64_t sampleperiod,
	size_t depth)
{
	auto ret = new UniformAnalogWaveform("Step");
	ret->m_timescale = sampleperiod;
	ret->Resize(depth);

	size_t mid = depth/2;
	for(size_t i=0; i<depth; i++)
	{
		if(i < mid)
			ret->m_samples[i] = vlo;
		else
			ret->m_samples[i] = vhi;
	}

	return ret;
}

/**
	@brief Generates a sinewave with a bit of extra noise added
 */
WaveformBase* TestWaveformSource::GenerateNoisySinewave(
	float amplitude,
	float startphase,
	float period,
	int64_t sampleperiod,
	size_t depth,
	float noise_amplitude)
{
	auto ret = new UniformAnalogWaveform("NoisySine");
	ret->m_timescale = sampleperiod;
	ret->Resize(depth);

	normal_distribution<> noise(0, noise_amplitude);

	float samples_per_cycle = period * 1.0 / sampleperiod;
	float radians_per_sample = 2 * M_PI / samples_per_cycle;

	//sin is +/- 1, so need to divide amplitude by 2 to get scaling factor
	float scale = amplitude / 2;

	for(size_t i=0; i<depth; i++)
		ret->m_samples[i] = scale * sinf(i*radians_per_sample + startphase) + noise(m_rng);

	return ret;
}

/**
	@brief Generates a mix of two sinewaves plus some noise
 */
WaveformBase* TestWaveformSource::GenerateNoisySinewaveMix(
	float amplitude,
	float startphase1,
	float startphase2,
	float period1,
	float period2,
	int64_t sampleperiod,
	size_t depth,
	float noise_amplitude)
{
	auto ret = new UniformAnalogWaveform("NoisySineMix");
	ret->m_timescale = sampleperiod;
	ret->Resize(depth);

	normal_distribution<> noise(0, noise_amplitude);

	float radians_per_sample1 = 2 * M_PI * sampleperiod / period1;
	float radians_per_sample2 = 2 * M_PI * sampleperiod / period2;

	//sin is +/- 1, so need to divide amplitude by 2 to get scaling factor.
	//Divide by 2 again to avoid clipping the sum of them
	float scale = amplitude / 4;

	for(size_t i=0; i<depth; i++)
	{
		ret->m_samples[i] = scale *
			(sinf(i*radians_per_sample1 + startphase1) + sinf(i*radians_per_sample2 + startphase2))
			+ noise(m_rng);
	}

	return ret;
}

WaveformBase* TestWaveformSource::GeneratePRBS31(
	float amplitude,
	float period,
	int64_t sampleperiod,
	size_t depth,
	bool lpf,
	float noise_amplitude
	)
{
	auto ret = new UniformAnalogWaveform("PRBS31");
	ret->m_timescale = sampleperiod;
	ret->Resize(depth);

	//Generate the PRBS as a square wave. Interpolate zero crossings as needed.
	uint32_t prbs = rand();
	float scale = amplitude / 2;
	float phase_to_next_edge = period;
	bool value = false;
	for(size_t i=0; i<depth; i++)
	{
		//Increment phase accumulator
		float last_phase = phase_to_next_edge;
		phase_to_next_edge -= sampleperiod;

		bool last = value;
		if(phase_to_next_edge < 0)
		{
			uint32_t next = ( (prbs >> 30) ^ (prbs >> 27) ) & 1;
			prbs = (prbs << 1) | next;
			value = next;

			phase_to_next_edge += period;
		}

		//Not an edge, just repeat the value
		if(last == value)
			ret->m_samples[i] = value ? scale : -scale;

		//Edge - interpolate
		else
		{
			float last_voltage = last ? scale : -scale;
			float cur_voltage = value ? scale : -scale;

			float frac = 1 - (last_phase / sampleperiod);
			float delta = cur_voltage - last_voltage;

			ret->m_samples[i] = last_voltage + delta*frac;
		}
	}

	DegradeSerialData(ret, sampleperiod, depth, lpf, noise_amplitude);

	return ret;
}

WaveformBase* TestWaveformSource::Generate8b10b(
	float amplitude,
	float period,
	int64_t sampleperiod,
	size_t depth,
	bool lpf,
	float noise_amplitude)
{
	auto ret = new UniformAnalogWaveform("8B10B");
	ret->m_timescale = sampleperiod;
	ret->Resize(depth);

	const int patternlen = 20;
	const bool pattern[patternlen] =
	{
		0, 0, 1, 1, 1, 1, 1, 0, 1, 0,		//K28.5
		1, 0, 0, 1, 0, 0, 0, 1, 0, 1		//D16.2
	};

	//Generate the data stream as a square wave. Interpolate zero crossings as needed.
	float scale = amplitude / 2;
	float phase_to_next_edge = period;
	bool value = false;
	int nbit = 0;
	for(size_t i=0; i<depth; i++)
	{
		//Increment phase accumulator
		float last_phase = phase_to_next_edge;
		phase_to_next_edge -= sampleperiod;

		bool last = value;
		if(phase_to_next_edge < 0)
		{
			value = pattern[nbit ++];
			if(nbit >= patternlen)
				nbit = 0;

			phase_to_next_edge += period;
		}

		//Not an edge, just repeat the value
		if(last == value)
			ret->m_samples[i] = value ? scale : -scale;

		//Edge - interpolate
		else
		{
			float last_voltage = last ? scale : -scale;
			float cur_voltage = value ? scale : -scale;

			float frac = 1 - (last_phase / sampleperiod);
			float delta = cur_voltage - last_voltage;

			ret->m_samples[i] = last_voltage + delta*frac;
		}
	}

	DegradeSerialData(ret, sampleperiod, depth, lpf, noise_amplitude);

	return ret;
}

// /**
// 	@brief Zero-pad a scalar input to the proper length and FFT it

// 	Overwrites m_scalarTempBuf1
//  */
// void TestWaveformSource::ProcessScalarInput(
// 	vk::raii::CommandBuffer& cmdBuf,
// 	unique_ptr<VulkanFFTPlan>& plan,
// 	AcceleratorBuffer<float>& samplesIn,
// 	AcceleratorBuffer<float>& samplesOut,
// 	size_t npointsPadded,
// 	size_t npointsUnpadded
// 	)
// {
// 	//Copy and zero-pad the input as needed
// 	WindowFunctionArgs args;
// 	args.numActualSamples = npointsUnpadded;
// 	args.npoints = npointsPadded;
// 	args.scale = 0;
// 	args.alpha0 = 0;
// 	args.alpha1 = 0;
// 	args.offsetIn = 0;
// 	args.offsetOut = 0;
// 	m_rectangularComputePipeline.Bind(cmdBuf);
// 	m_rectangularComputePipeline.BindBufferNonblocking(0, samplesIn, cmdBuf);
// 	printf("bind 2");
// 	m_rectangularComputePipeline.BindBufferNonblocking(1, m_scalarTempBuf1, cmdBuf, true);
// 	m_rectangularComputePipeline.DispatchNoRebind(cmdBuf, args, GetComputeBlockCount(npointsPadded, 64));
// 	m_rectangularComputePipeline.AddComputeMemoryBarrier(cmdBuf);
// 	m_scalarTempBuf1.MarkModifiedFromGpu();

// 	//Do the actual FFT operation
// 	plan->AppendForward(m_scalarTempBuf1, samplesOut, cmdBuf);
// 	samplesOut.MarkModifiedFromGpu();
// 	m_rectangularComputePipeline.AddComputeMemoryBarrier(cmdBuf);
// }


// /**
// 	@brief Generates a scalar output from a complex input

// 	Overwrites m_scalarTempBuf1
//  */
// void TestWaveformSource::GenerateScalarOutput(
// 	vk::raii::CommandBuffer& cmdBuf,
// 	unique_ptr<VulkanFFTPlan>& plan,
// 	size_t istart,
// 	size_t iend,
// 	WaveformBase* refin,
// 	size_t stream,
// 	size_t npoints,
// 	int64_t phaseshift,
// 	AcceleratorBuffer<float>& samplesIn)
// {
// 	//Prepare the output waveform
// 	float scale = 1.0f / npoints;
// 	size_t outlen = iend - istart;
// 	auto cap = SetupEmptyUniformAnalogOutputWaveform(refin, stream);
// 	cap->Resize(outlen);

// 	//Apply phase shift for the group delay so we draw the waveform in the right place
// 	cap->m_triggerPhase = phaseshift;

// 	//Do the actual FFT operation
// 	plan->AppendReverse(samplesIn, m_scalarTempBuf1, cmdBuf);

// 	//Copy and normalize output
// 	//TODO: is there any way to fold this into vkFFT? They can normalize, but offset might be tricky...
// 	DeEmbedNormalizationArgs nargs;
// 	nargs.outlen = outlen;
// 	nargs.istart = istart;
// 	nargs.scale = scale;
// 	m_normalizeComputePipeline.Bind(cmdBuf);
// 	m_normalizeComputePipeline.BindBufferNonblocking(0, m_scalarTempBuf1, cmdBuf);
// 	m_normalizeComputePipeline.BindBufferNonblocking(1, cap->m_samples, cmdBuf, true);
// 	m_normalizeComputePipeline.DispatchNoRebind(cmdBuf, nargs, GetComputeBlockCount(npoints, 64));
// 	m_normalizeComputePipeline.AddComputeMemoryBarrier(cmdBuf);

// 	cap->MarkModifiedFromGpu();
// }


/**
	@brief Takes an idealized serial data stream and turns it into something less pretty

	by adding noise and a band-limiting filter

	TODO: apply a more realistic channel model, maybe a hard coded table of S-parameters or something?
 */
void TestWaveformSource::DegradeSerialData(
	UniformAnalogWaveform* cap,
	int64_t sampleperiod,
	size_t depth,
	bool lpf,
	float noise_amplitude)
{
	//RNGs
	normal_distribution<> noise(0, noise_amplitude);

	// ffts is not available on apple silicon, so for now we only apply noise there
#ifndef _APPLE_SILICON
	//Prepare for second pass: reallocate FFT buffer if sample depth changed
	auto dinFwd = dynamic_cast<UniformAnalogWaveform*>(cap);
	auto dinRev = dynamic_cast<UniformAnalogWaveform*>(cap);
	if(!dinFwd || !dinRev)
	{
		//SetData(nullptr, 0);
		printf("!dinFwd || !dinRev");

		return;
	}
	const size_t npoints_raw = min(dinFwd->size(), dinRev->size());



	const size_t npoints = next_pow2(depth);
	size_t nouts = npoints/2 + 1;

	printf("samples raw: %lld \t %lld \t %lld", npoints_raw, npoints, nouts);

	//Invalidate old vkFFT plans if size has changed
	if(m_vkForwardPlan)
	{
		if(m_vkForwardPlan->size() != npoints)
			m_vkForwardPlan = nullptr;
	}
	if(m_vkReversePlan)
	{
		if(m_vkReversePlan->size() != npoints)
			m_vkReversePlan = nullptr;
	}

	//Set up the FFT and allocate buffers if we change point count
	bool sizechange = false;
	if(m_cachedNumPoints != npoints)
	{
		// m_scalarTempBuf1.resize(npoints);
		// m_vectorTempBuf1.resize(2 * nouts);
		// m_vectorTempBuf2.resize(2 * nouts);
		m_vectorTempBuf3.resize(2 * nouts);
		// m_vectorTempBuf4.resize(2 * nouts);

		m_forwardInBuf.resize(npoints);
		m_forwardOutBuf.resize(2 * nouts);
		m_reverseOutBuf.resize(npoints);

		m_cachedNumPoints = npoints;
		sizechange = true;
	}

		//Set up new FFT plans
	if(!m_vkForwardPlan)
		m_vkForwardPlan = make_unique<VulkanFFTPlan>(npoints, nouts, VulkanFFTPlan::DIRECTION_FORWARD);
	if(!m_vkReversePlan)
		m_vkReversePlan = make_unique<VulkanFFTPlan>(npoints, nouts, VulkanFFTPlan::DIRECTION_REVERSE);

	// if(m_cachedNumPoints != npoints)
	// {
	// 	if(m_forwardPlan)
	// 		ffts_free(m_forwardPlan);
	// 	m_forwardPlan = ffts_init_1d_real(npoints, FFTS_FORWARD);

	// 	if(m_reversePlan)
	// 		ffts_free(m_reversePlan);
	// 	m_reversePlan = ffts_init_1d_real(npoints, FFTS_BACKWARD);

	// 	m_forwardInBuf = m_allocator.allocate(npoints);
	// 	m_forwardOutBuf = m_allocator.allocate(2*nouts);
	// 	m_reverseOutBuf = m_allocator.allocate(npoints);

	// 	m_cachedNumPoints = npoints;
	// }

	

	if(lpf)
	{

		//Copy the input, then fill any extra space with zeroes
		// memcpy(m_forwardInBuf, &cap->m_samples[0], depth*sizeof(float));
		// for(size_t i=depth; i<npoints; i++)
		// 	m_forwardInBuf[i] = 0;

		if(sizechange ){
			printf("sizechanged");
		}
		
		m_cmdBuf->begin({});


	//Look up some parameters
	double sample_ghz = 1e6 / sampleperiod;
	double bin_hz = round((0.5f * sample_ghz * 1e9f) / nouts);
	auto window = static_cast<WindowFunction>(m_parameters[m_windowName].GetIntVal());
	LogTrace("bin_hz: %f\n", bin_hz);

		//Set up output and copy time scales / configuration
	auto cap = SetupEmptyUniformAnalogOutputWaveform(din, 0);
	cap->m_triggerPhase = 1*bin_hz;
	cap->m_timescale = bin_hz;
	cap->Resize(nouts);

	//Output scale is based on the number of points we FFT that contain actual sample data.
	//(If we're zero padding, the zeroes don't contribute any power)
	size_t numActualSamples = min(dinFwd->size(), npoints);
	float scale = sqrt(2.0) / numActualSamples;

	//We also need to adjust the scale by the coherent power gain of the window function
	scale *= 2.805;

	//Configure the window
	WindowFunctionArgs args;
	args.numActualSamples = numActualSamples;
	args.npoints = npoints;
	args.scale = 2 * M_PI / numActualSamples;
	args.offsetIn = 0;
	args.offsetOut = 0;
	args.alpha0 = 25.0f / 46;
	args.alpha1 = 1 - args.alpha0;

	ComputePipeline* wpipe = &m_blackmanHarrisComputePipeline;

	wpipe->BindBufferNonblocking(0, data, cmdBuf);
	wpipe->BindBufferNonblocking(1, m_rdinbuf, cmdBuf, true);
	wpipe->Dispatch(cmdBuf, args, GetComputeBlockCount(npoints, 64));
	wpipe->AddComputeMemoryBarrier(cmdBuf);
	m_rdinbuf.MarkModifiedFromGpu();

	//Do the actual FFT operation
	m_vkPlan->AppendForward(m_rdinbuf, m_rdoutbuf, cmdBuf);

	//Convert complex to real
	ComputePipeline& pipe = m_complexToMagnitudeComputePipeline;
	ComplexToMagnitudeArgs cargs;
	cargs.npoints = nouts;
	if(log_output)
	{
		const float impedance = 50;
		cargs.scale = scale * scale / impedance;
	}
	else
		cargs.scale = scale;
	pipe.BindBuffer(0, m_rdoutbuf);
	pipe.BindBuffer(1, cap->m_samples);
	pipe.AddComputeMemoryBarrier(cmdBuf);
	pipe.Dispatch(cmdBuf, cargs, GetComputeBlockCount(nouts, 64));

	//Done, block until the compute operations finish
	cmdBuf.end();
	queue->SubmitAndBlock(cmdBuf);

	cap->MarkModifiedFromGpu();

		//Do the actual FFT operation
		m_vkReversePlan->AppendReverse(m_forwardOutBuf, m_reverseOutBuf, *m_cmdBuf);



		// //Do the forward FFT
		
		// ProcessScalarInput(*m_cmdBuf, m_vkForwardPlan, dinFwd->m_samples, m_vectorTempBuf3, npoints, npoints_raw);
		// //ffts_execute(m_forwardPlan, &m_forwardInBuf[0], &m_forwardOutBuf[0]);

		// // //Simple channel response model
		// double sample_ghz = 1e6 / sampleperiod;
		// // double bin_hz = round((0.5f * sample_ghz * 1e9f) / nouts);
		// // complex<float> pole(0, -FreqToPhase(5e9));
		// // float prescale = abs(pole);
		// // for(size_t i = 0; i<nouts; i++)
		// // {
		// // 	complex<float> s(0, FreqToPhase(bin_hz * i));
		// // 	complex<float> h = prescale * complex<float>(1, 0) / (s - pole);

		// // 	float binscale = abs(h);
		// // 	m_forwardOutBuf[i*2] *= binscale;		//real
		// // 	m_forwardOutBuf[i*2 + 1] *= binscale;	//imag
		// // }

		// size_t istart = 0;
		// size_t iend = npoints_raw;
		// int64_t phaseshift = 0;

		// //Calculate the inverse FFT
		// // ffts_execute(m_reversePlan, &m_forwardOutBuf[0], &m_reverseOutBuf[0]);
		// GenerateScalarOutput(m_cmdBuf, m_vkReversePlan, istart, iend, dinRev, 1, npoints, phaseshift, m_vectorTempBuf3);

		//Done, block until the compute operations finish
		m_cmdBuf->end();
		m_queue->SubmitAndBlock(*m_cmdBuf);

		//Rescale the FFT output and copy to the output, then add noise
		float fftscale = 1.0f / npoints;
		for(size_t i=0; i<depth; i++)
			cap->m_samples[i] = m_reverseOutBuf[i] * fftscale + noise(m_rng);
	}

	else
#endif
	{
		for(size_t i=0; i<depth; i++)
			cap->m_samples[i] += noise(m_rng);
	}
}



// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// // Helpers for various common boilerplate operations

// /**
// 	@brief Sets up an analog output waveform and copies basic metadata from the input.

// 	A new output waveform is created if necessary, but when possible the existing one is reused.

// 	@param din			Input waveform
// 	@param stream		Stream index
// 	@param clear		True to clear an existing waveform, false to leave it as-is

// 	@return	The ready-to-use output waveform
//  */
// UniformAnalogWaveform* TestWaveformSource::SetupEmptyUniformAnalogOutputWaveform(WaveformBase* din, size_t stream, bool clear)
// {
// 	//Create the waveform, but only if necessary
// 	auto cap = new UniformAnalogWaveform
	
// 	//Copy configuration
// 	cap->m_startTimestamp 		= din->m_startTimestamp;
// 	cap->m_startFemtoseconds	= din->m_startFemtoseconds;
// 	cap->m_triggerPhase			= din->m_triggerPhase;
// 	cap->m_timescale			= din->m_timescale;

// 	//Bump rev number
// 	cap->m_revision ++;

// 	//Clear output
// 	if(clear)
// 		cap->clear();

// 	return cap;
// }