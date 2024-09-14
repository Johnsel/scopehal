/***********************************************************************************************************************
*                                                                                                                      *
* libscopeprotocols                                                                                                    *
*                                                                                                                      *
* Copyright (c) 2012-2024 Andrew D. Zonenberg and contributors                                                         *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided canvas the     *
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
	@brief Implementation of EyeMask
 */

#include "scopehal.h"
#include "EyeWaveform.h"
#include "EyeMask.h"

#ifndef CANVAS_ITY_IMPLEMENTATION
#define CANVAS_ITY_IMPLEMENTATION
#include "../canvas_ity/src/canvas_ity.hpp"
#endif

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

EyeMask::EyeMask()
	: m_hitrate(0)
	, m_timebaseIsRelative(false)
{
}

EyeMask::~EyeMask()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Mask file parsing

bool EyeMask::Load(string path)
{
	//Clear out any previous state
	m_polygons.clear();
	m_hitrate = 0;
	m_timebaseIsRelative = false;
	m_maskname = "";

	try
	{
		m_fname = path;
		auto docs = YAML::LoadAllFromFile(path);
		if(!Load(docs[0]))
			return false;
	}
	catch(const YAML::BadFile& ex)
	{
		return false;
	}

	return true;
}

/**
	@brief Loads the YAML file
 */
bool EyeMask::Load(const YAML::Node& node)
{
	//Clear out any previous state
	m_polygons.clear();
	m_hitrate = 0;
	m_timebaseIsRelative = false;
	m_maskname = "";

	//Load protocol section
	auto proto = node["protocol"];
	for(auto it : proto)
	{
		auto name = it.first.as<string>();
		if(name == "name")
			m_maskname = it.second.as<string>();
	}

	//For now, ignore display limits

	//Load units
	auto units = node["units"];
	float yscale = 1;
	float timebaseScale = 1;
	for(auto it : units)
	{
		auto name = it.first.as<string>();
		if(name == "xscale")
		{
			auto scale = it.second.as<string>();
			if(scale == "ui")
				m_timebaseIsRelative = true;
			else if(scale == "ps")
			{
				m_timebaseIsRelative = false;
				timebaseScale = 1000;
			}
			else if(scale == "fs")
				m_timebaseIsRelative = false;
			else
				LogError("Unrecognized xscale \"%s\"\n", scale.c_str());
		}
		if(name == "yscale")
		{
			auto scale = it.second.as<string>();
			if(scale == "mv")
				yscale = 0.001f;
			else if(scale == "v")
				yscale = 0.001f;
			else
				LogError("Unrecognized yscale \"%s\"\n", scale.c_str());
		}
	}

	//Load pass conditions
	auto conditions = node["conditions"];
	for(auto it : conditions)
	{
		auto name = it.first.as<string>();
		if(name == "hitrate")
			m_hitrate = it.second.as<float>();
	}

	//Load actual mask polygons
	auto mask = node["mask"];
	for(auto p : mask)
	{
		EyeMaskPolygon poly;

		auto points = p["points"];
		for(auto v : points)
			poly.m_points.push_back(EyeMaskPoint(v["x"].as<float>() * timebaseScale, v["y"].as<float>() * yscale));

		m_polygons.push_back(poly);
	}

	return true;
}
/* 
void EyeMask::RenderForAnalysis(
		// canvas_ity::canvas canvas, // John: with the cairo context, gotta see if gets replaced with a canvas_ity ref
		EyeWaveform* waveform,
		float xscale,
		float xoff,
		float yscale,
		float yoff,
		float height) const
{
	// RenderInternal(waveform, xscale, xoff, yscale, yoff, height); 
} 
*/

/* 
void EyeMask::RenderInternal(
		// canvas_ity::canvas canvas, 	// Again with the cairo ref
		EyeWaveform* waveform,				// With the waveform object
		float xscale,						// Some scale and positioning params
		float xoff,
		float yscale,
		float yoff,
		float height) const
{

} */

/**
	@brief Checks a raw eye pattern dataset against the mask
 */
float EyeMask::CalculateHitRate(
	EyeWaveform* cap,
	size_t width,
	size_t height,
	float fullscalerange,
	float xscale,
	float xoff
	) const
{
	//TODO: performance optimization, don't re-render mask every waveform, only when we resize

	// John: Create canvas_ity surface
	canvas_ity::canvas canvas( width, height ); // width and height could be reversed

	// John: Fill everything with color black
	canvas.set_color( canvas_ity::fill_style, 0.0f, 0.0f, 0.0f, 1.0f);
    canvas.fill();
	
	//Software rendering
	float yscale = height / fullscalerange;
	float yoff = 0.0f;


	//John: clear background
	//cr->set_source_rgba(0, 0, 0, 1);
	canvas.set_color( canvas_ity::fill_style, 0.0f, 0.0f, 0.0f, 1.0f );

	//cr->move_to(-1e5, 0);
	//canvas.move_to( -1e5f, 0.0f ); 
	canvas.move_to( -1e5, 0 ); 
	
	

	//cr->line_to( 1e5, 0);
	//canvas.line_to( 1e5f, 0.0f );
	canvas.line_to( 1e5, 0 );

	//cr->line_to( 1e5, height);
	//canvas.line_to( 1e5f, height * 1.0f );
	canvas.line_to( 1e5, height );

	//cr->line_to(-1e5, height);
	//canvas.line_to(-1e5f, height * 1.0f );
	canvas.line_to(-1e5, height);
	
	
	//cr->fill();
	canvas.fill();

	// John: Set color to white and call 'real real' draw function' (simplify to one method for everything?)
	//cr->set_source_rgba(1, 1, 1, 1);
	canvas.set_color( canvas_ity::fill_style, 1.0f, 1.0f, 1.0f, 1.0f );


	//Draw each polygon
	for(auto poly : m_polygons)
	{
		for(size_t i=0; i<poly.m_points.size(); i++)
		{
			auto point = poly.m_points[i];

			//Convert from ps to UI if needed
			float time = point.m_time;
			if(m_timebaseIsRelative)
				time *= cap->GetUIWidth();

			float x = (time - xoff) * xscale;

			float y = height/2 - ( (point.m_voltage + yoff) * yscale );

			if(i == 0)
				canvas.move_to(x, y); // Set to starting point for line if first run
			else
				canvas.line_to(x, y); // Draw line to next coord
		}
		canvas.fill(); // fill the resultant line defined polygon with the current color (white)
	}


	unsigned char image_data[1280][720] = {{0}};

    
	int stride = 4; // John: every pixel four chars, right? (?)

	canvas.get_image_data(*image_data, width, height, stride, 0,0);

		/* 		int stride = surface->get_stride() / sizeof(uint32_t); 	// divide stride by uint32_t
		for(size_t y=0; y<height; y++)							// for each row (y)
		{
			auto row = data + (y*stride);						// row data = data pointer + stride_size (get next pixel in y direction's data)
			auto eyerow = accum + (y*width);					// accumulate same data as above ??
			for(size_t x=0; x<width; x++)						// for each column (x)
			{
																// If mask pixel isn't black, count violations
				 ;												// For x'th pixel in row (actual current pixel val)
				if( (pix & 0xff) != 0)							// If not white
				{
 */			


	//Test each pixel of the eye pattern against the mask
	float nmax = 0;

	// if(cap->GetType() == EyeWaveform::EYE_NORMAL)
	// {
	// 	auto accum = cap->GetAccumData();

	// 	uint32_t* data = reinterpret_cast<uint32_t*>(image_data); // pointer to char*
	// 	int stride = sizeof(uint32_t);
	// 	for(size_t y=0; y<height; y++)
	// 	{
	// 		auto row = data + (y*stride);
	// 		auto eyerow = accum + (y*width);
	// 		for(size_t x=0; x<width; x++)
	// 		{
	// 			//If mask pixel isn't black, count violations
	// 			uint32_t pix = row[x];
	// 			if( (pix & 0xff) != 0)
	// 			{
	// 				float rate = (eyerow[x] * 1.0f / cap->GetTotalUIs());
	// 				if(rate > nmax)
	// 					nmax = rate;
	// 			}
	// 		}
	// 	}
	// }
	// else //if(cap->GetType() == EyeWaveform::EYE_BER)
	// {
	// 	auto accum = cap->GetData();
	// 	uint32_t* data = reinterpret_cast<uint32_t*>(surface->get_data());
	// 	int stride = surface->get_stride() / sizeof(uint32_t);
	// 	for(size_t y=0; y<height; y++)
	// 	{
	// 		auto row = data + (y*stride);
	// 		auto eyerow = accum + (y*width);
	// 		for(size_t x=0; x<width; x++)
	// 		{
	// 			//If mask pixel isn't black, count violations
	// 			uint32_t pix = row[x];
	// 			if( (pix & 0xff) != 0)
	// 			{
	// 				//BER eyes don't need any preprocessing since the pixel values are already raw BER
	// 				float rate = eyerow[x];
	// 				if(rate > nmax)
	// 					nmax = rate;
	// 			}
	// 		}
	// 	}
	// }

	return nmax;
}
