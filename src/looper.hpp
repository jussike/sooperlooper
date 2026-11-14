/*
** Copyright (C) 2004 Jesse Chappell <jesse@essej.net>
**  
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
**  
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**  
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
**  
*/

#ifndef __sooperlooper_looper__
#define __sooperlooper_looper__

#include <string>
#include <stdint.h>
#include <cstddef>

#if HAVE_CONFIG_H
#include <config.h>
#endif

#include <samplerate.h>



#include "audio_driver.hpp"
#include "lockmonitor.hpp"
#include "ladspa.h"

#include "plugin.hpp"
#include "event.hpp"
#include "event_nonrt.hpp"
#include "utils.hpp"

#include <pbd/xml++.h>

namespace RubberBand {
	class RubberBandStretcher;
}


namespace SooperLooper {

class OnePoleFilter;	
class Panner;

	
class Looper 
{
  public:
	Looper (AudioDriver * driver, unsigned int index, unsigned int channel_count=1, float loopsecs=40.0, bool discrete=true);
	Looper (AudioDriver * driver, XMLNode & node);
	~Looper ();

	bool initialize (unsigned int index, unsigned int channel_count=1, float loopsecs=40.0, bool discrete=true);
	void destroy();
	
	bool operator() () const { return _ok; }
	void run (nframes_t offset, nframes_t nframes);

	void do_event (Event *ev);

	float get_control_value (Event::control_t ctrl);
	
	void set_port (ControlPort n, LADSPA_Data val);

	bool load_loop (std::string fname);
	bool save_loop (std::string fname = "", LoopFileEvent::FileFormat format = LoopFileEvent::FormatFloat);

	void set_buffer_size (nframes_t bufsize);

	sample_t * get_sync_in_buf() { return _our_syncin_buf; }
	sample_t * get_sync_out_buf() { return _our_syncout_buf; }

	void use_sync_buf(sample_t * buf);

	unsigned int get_index() const { return _index; }
	unsigned int get_channel_count() const { return _chan_count; }

	// number of loop-time samples that are consumed per real-time sample
	// (1.0 means normal speed).  This reflects the rate (resampling) or the
	// time stretch ratio, whichever is currently in effect in run_loops().
	double get_playback_speed () const {
		if (ports[Rate] != 1.0f) {
			return _src_in_ratio;
		}
		if (_stretch_ratio != 1.0 && _stretch_ratio > 0.0) {
			return 1.0 / _stretch_ratio;
		}
		return 1.0;
	}
	
	void set_use_common_ins (bool val);
	bool get_use_common_ins () const { return _use_common_ins; }
	void set_use_common_outs (bool val);
	bool get_use_common_outs () const { return _use_common_outs; }

    void set_discrete_outs_prefader (bool val);
    bool get_discrete_outs_prefader () const { return _discrete_prefader; }

	bool get_have_discrete_io () const { return _have_discrete_io; }

	void set_auto_latency (bool val) { _auto_latency = val; }
	bool get_auto_latency () const { return _auto_latency; }

	// disables any compensation but maintains the current values
	void set_disable_latency_compensation (bool val);
	bool get_disable_latency_compensation () const { return _disable_latency; }

	void set_samples_since_sync(nframes_t ssync);
	void set_replace_quantized(bool flag);

	// called when some loop instance is being soloed, index says which instance (may not be us)
	void set_soloed (int index, bool value, bool retrigger=false);
	bool is_soloed() const { return _is_soloed; }

	bool is_muted() const { return ports[State] == LooperStateMuted || ports[State] == LooperStateOffMuted; }
	bool has_loop() const ;

	// finishes any active state that may be going (rec, overdub, etc)
	bool finish_state();
	
	bool is_longpress (int command);

	void set_name(const std::string & name) { _name = name; }
	const std::string & get_name() const { return _name; }

	XMLNode& get_state () const;
	int set_state (const XMLNode&);

	void recompute_latencies();
	
  protected:

	void run_loops (nframes_t offset, nframes_t nframes);
	void run_loops_resampled (nframes_t offset, nframes_t nframes);

	// moves the loop position of all channels (loop time samples, positive = forward)
	void shift_loop_positions (double loopframes);

	// takes back the lead the loop has over the audible stretcher output
	void settle_stretch_position ();

	static void compute_peak (sample_t *buf, nframes_t nsamples, float& peak) {
		float p = peak;
		
		for (nframes_t n = 0; n < nsamples; ++n) {
			p = f_max (p, fabsf(buf[n]));
		}
		
		peak = p;
	}	
	
		
	int requested_cmd;
	int last_requested_cmd;
	
	AudioDriver *      _driver;

	port_id_t*       _input_ports;
	port_id_t*       _output_ports;

	unsigned int _index;
	unsigned int _chan_count;
	LADSPA_Handle *      _instances;
	float _loopsecs;
	
	LADSPA_Descriptor* descriptor;

	LADSPA_Data        ports[LASTPORT];

	float              _curr_dry;
	float              _target_dry;

	float              _curr_input_gain;
	float              _targ_input_gain;

    float              _curr_wet;
    float              _target_wet;

	bool               _relative_sync;

    bool               _discrete_prefader;

	// keeps track of down/up commands for SUS purposes
	nframes_t          _down_stamps[Event::LAST_COMMAND+1];
	nframes_t          _longpress_frames;
	nframes_t          _doubletap_frames;
	nframes_t          _running_frames;
	
	nframes_t          _buffersize;
	LADSPA_Data        * _our_syncin_buf;
	LADSPA_Data        * _our_syncout_buf;
	LADSPA_Data        * _use_sync_buf;
	LADSPA_Data        * _dummy_buf;

	LADSPA_Data        ** _tmp_io_bufs;

	Panner             * _panner; 

	float              _input_peak;
	float              _output_peak;
	float              _falloff_per_sample;
	
	LADSPA_Data         _slave_sync_port;
	LADSPA_Data         _slave_dummy_port;

	bool                _use_common_ins;
	bool                _use_common_outs;
	bool                _have_discrete_io;
	bool                _auto_latency;
	bool                _disable_latency;
	LADSPA_Data         _last_trigger_latency;
	LADSPA_Data         _last_input_latency;
	LADSPA_Data         _last_output_latency;

	bool                _pre_solo_muted;
	bool                _is_soloed;

	std::string         _name;

	// SRC stuff
	SRC_STATE**            _in_src_states;
	SRC_STATE**            _out_src_states;
	SRC_STATE*            _insync_src_state;
	SRC_STATE*            _outsync_src_state;
	float  *              _src_in_buffer;
	float  *              _src_sync_buffer;
	nframes_t             _src_buffer_len;

	double                _src_in_ratio;
	double                _src_out_ratio;
	SRC_DATA              _src_data;


	OnePoleFilter  **     _lp_filter;

	// rubberband stuff
	RubberBand::RubberBandStretcher * _in_stretcher;
	RubberBand::RubberBandStretcher * _out_stretcher;
	double                             _stretch_ratio;
	double                             _pitch_shift; // in semitones
	float *                            _stretch_buffer;

	bool                               _tempo_stretch;
	volatile bool                      _pending_stretch;
	volatile double                    _pending_stretch_ratio;

	// Delay line for the sync output of the stretch/pitch path.  The loop
	// produces its sync output in loop time, ahead of the (latent) stretched
	// audio.  Writing it through this delay line makes the pulses leave at the
	// same time as the audio they belong to, independent of how the stretcher
	// chunks its input requests.
	float *               _sync_fifo;
	uint64_t              _sync_fifo_size;     // power of two
	uint64_t              _sync_fifo_mask;
	uint64_t              _sync_fifo_wpos;     // loop time samples written so far
	uint64_t              _sync_fifo_ke;       // next loop time sample to be emitted
	double                _sync_fifo_rpos;     // loop time read position (may be negative)
	size_t                _sync_last_lat;      // stretcher latency used for the read position
	float                 _sync_pending_pulse; // pulse from a command pre-pass, not yet written
	bool                  _stretch_active;     // stretch path was used by the previous call
	bool                  _sync_fifo_reset;    // stretchers were reset, restart the delay line

	// Loop time accounting for the stretch path, used to find out how far the
	// loop position is ahead of the audible (delayed) output.  When the
	// stretcher is bypassed the loop position is moved back by that amount so
	// the audible loop stays continuous and in sync with the slaves.
	double                _stretch_consumed;   // loop time samples fed to the stretcher
	double                _stretch_emitted;    // loop time equivalent of the output taken from it
	double                _stretch_lat_loop;   // stretcher latency in loop time samples

	bool _ok;
	volatile bool request_pending;

	PBD::NonBlockingLock _loop_lock;
};

};

#endif
