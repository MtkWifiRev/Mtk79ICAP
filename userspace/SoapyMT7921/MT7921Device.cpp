// SPDX-License-Identifier: MIT
/*
 * SoapySDR module for the MT7921AU (MT7961) I/Q stream of the mt7921 fork
 * driver: /dev/mt7921_iq_<phy>, decoded by libmt7921iq.
 *
 * - One RX channel, CS16 (full scale 8192) or CF32, at 20.0008 MS/s (20 MHz
 *   channel, ~18 MHz usable) or 40.0016 MS/s (40 MHz channel, ~36 MHz
 *   usable). The sample rate selects the channel width.
 * - 80.0032 MS/s selects an 80 MHz channel (5 GHz; on 2.4 GHz, centers
 *   2422-2462 MHz, only with iq_dense). With the driver's
 *   module parameter iq_dense (DENSE80, firmware v64+ recommended) the
 *   stream is continuous at 80.0032 MS/s (320 MB/s on USB). Without it the
 *   driver sends snapshots: separate 8192-sample (102.4 us; 2048 with a
 *   16 KiB ICAP ring) captures, back to back in time order but NOT
 *   continuous; good for spectrum and waterfall views, not for
 *   demodulation. With iq_dense, 40.0016 MS/s is DENSE40 (160 MB/s).
 * - Tuning: the receiver tunes in 5 MHz steps (driver ioctl, ~15-20 ms gap).
 *   The remainder is shifted in software (NCO), so every requested
 *   frequency is exact. Retunes run on a tuner thread and coalesce, so
 *   setFrequency() never blocks the caller; until the tune is done,
 *   samples are dropped. Generations the driver starts on its own after a
 *   tune (a restart at a slip-prone DMA alignment, or after an MCU command)
 *   are at the same center and pass; those from before the tune do not.
 * - Chain: setting/device arg "chain" (0 or 1). With the driver's iq_dense,
 *   1 streams RX chain 1 (DFE tap 13) at the full rate; changing it retunes.
 *   Non-dense streams always carry chain 0.
 * - Gain: AGC by default (setGainMode(true)). Manual mode (setGainMode(false))
 *   holds the streamed RX chain at a fixed analog gain, live and across
 *   retunes. On a busy channel this removes the AGC's gain dips on other
 *   stations' frames (about -4 dB for 16-19 us each) at the same SNR. Two
 *   elements in approximate dB, both measured on 5 GHz:
 *    - "LNA": 0-18 dB in 6 dB steps (codes 2-5); it sets the noise figure;
 *    - "LPF": 0-16 dB in 2 dB steps (codes 0-8).
 *   The overall gain fills LNA first. 32 dB (LNA 5, LPF 7) is the AGC's
 *   idle gain and is the default. Manual output is about 13 dB below the
 *   AGC's level (no digital gain), with the same SNR. 2.4 GHz codes are not
 *   mapped (the steps differ there). Setting "LNA"/"LPF" in AGC mode only
 *   stores the values for manual mode.
 * - Discontinuities (driver-dropped blocks, ring overruns) are reported as
 *   SOAPY_SDR_OVERFLOW, dropping the first piece after the gap; retunes are
 *   not reported (the stream just continues at the new frequency).
 *
 * The device must be on a monitor interface (userspace/mt7921iq-setup.sh).
 */
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Registry.hpp>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <complex>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <glob.h>
#include <map>
#include <mutex>
#include <poll.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "iqstream_ioctl.h"
#include "mt7921iq.h"

namespace {

const double kRate20 = MT7921IQ_SAMPLE_RATE;
const double kRate40 = MT7921IQ_SAMPLE_RATE_40;
const double kRate80 = MT7921IQ_SAMPLE_RATE_80;
const double kRaster = 5e6;
const size_t kMTU = 1 << 16;
const size_t kRawRead = 4 << 20;
/* Generations from before the latest tunes; random 31-bit ids */
const size_t kStaleMax = 16;
/* Gain elements in approximate dB, 0 at the lowest usable code (5 GHz). */
const double kLnaMaxDb = (MT7921IQ_LNA_MAX - MT7921IQ_LNA_MIN) * MT7921IQ_LNA_STEP_DB;
const double kLpfMaxDb = (MT7921IQ_LPF_MAX - MT7921IQ_LPF_MIN) * MT7921IQ_LPF_STEP_DB;

unsigned lnaCode(double db)
{
	long k = std::lround(db / MT7921IQ_LNA_STEP_DB) + MT7921IQ_LNA_MIN;

	return (unsigned)std::min<long>(std::max<long>(k, MT7921IQ_LNA_MIN), MT7921IQ_LNA_MAX);
}

unsigned lpfCode(double db)
{
	long k = std::lround(db / MT7921IQ_LPF_STEP_DB) + MT7921IQ_LPF_MIN;

	return (unsigned)std::min<long>(std::max<long>(k, MT7921IQ_LPF_MIN), MT7921IQ_LPF_MAX);
}

double lnaDb(unsigned code) { return (code - MT7921IQ_LNA_MIN) * MT7921IQ_LNA_STEP_DB; }
double lpfDb(unsigned code) { return (code - MT7921IQ_LPF_MIN) * MT7921IQ_LPF_STEP_DB; }

struct Band {
	double lo, hi;	/* hardware centers: lo + k * 5 MHz up to hi */
};
const std::vector<Band> kBands20 = {
	{2412e6, 2472e6},	/* 2407 + 5 * ch */
	{2484e6, 2484e6},
	{5005e6, 5995e6},	/* 5000 + 5 * ch */
};
/* 40 MHz: control channel 10 MHz below the center must exist */
const std::vector<Band> kBands40 = {
	{2422e6, 2462e6},
	{5015e6, 5985e6},
};

/* 80 MHz: 2.4 GHz needs the driver's iq_dense (a 40 MHz channel widened by
 * the driver; the camera band 2402-2482 MHz fits at center 2442); 5 GHz with
 * the control channel 30 MHz below the center.
 */
const std::vector<Band> kBands80 = {
	{2422e6, 2462e6},
	{5035e6, 5965e6},
};

const std::vector<Band> &bands(unsigned bw)
{
	return bw == 80 ? kBands80 : bw == 40 ? kBands40 : kBands20;
}

double rateOf(unsigned bw)
{
	return bw == 80 ? kRate80 : bw == 40 ? kRate40 : kRate20;
}

/* Nearest hardware center for f, or 0 if f is outside every band. */
double nearestCenter(double f, double maxOffset, unsigned bw)
{
	double best = 0, bestDist = 1e30;

	for (const auto &b : bands(bw)) {
		double c = b.lo + std::round((f - b.lo) / kRaster) * kRaster;

		c = std::min(std::max(c, b.lo), b.hi);
		if (std::abs(f - c) <= maxOffset && std::abs(f - c) < bestDist) {
			best = c;
			bestDist = std::abs(f - c);
		}
	}
	return best;
}

std::vector<std::string> listDevices()
{
	std::vector<std::string> out;
	glob_t g;

	if (glob("/dev/mt7921_iq_*", 0, nullptr, &g) == 0) {
		for (size_t i = 0; i < g.gl_pathc; i++)
			out.push_back(g.gl_pathv[i]);
		globfree(&g);
	}
	return out;
}

class MT7921Device : public SoapySDR::Device {
public:
	explicit MT7921Device(const SoapySDR::Kwargs &args)
	{
		auto it = args.find("device");
		if (it != args.end()) {
			_path = it->second;
		} else {
			auto devs = listDevices();
			if (devs.empty())
				throw std::runtime_error("no /dev/mt7921_iq_* device");
			_path = devs[0];
		}
		it = args.find("max_offset");
		if (it != args.end())
			_maxOffset = std::stod(it->second);
		it = args.find("chain");
		if (it != args.end())
			_chain = std::stoi(it->second) ? 1 : 0;
		_tuner = std::thread(&MT7921Device::tunerLoop, this);
	}

	~MT7921Device() override
	{
		{
			std::lock_guard<std::mutex> l(_mtx);
			_quit = true;
		}
		_cv.notify_all();
		_tuner.join();
		closeDevice();
		if (_dec)
			mt7921iq_dec_free(_dec);
	}

	/* ---- identification -------------------------------------------- */
	std::string getDriverKey() const override { return "mt7921"; }
	std::string getHardwareKey() const override { return "MT7921AU"; }

	SoapySDR::Kwargs getHardwareInfo() const override
	{
		return {{"device", _path},
			{"sample_rates", std::to_string(kRate20) + "," + std::to_string(kRate40) +
					 "," + std::to_string(kRate80) +
					 " (80 MHz: continuous with iq_dense, else snapshots)"},
			{"tune_raster_hz", std::to_string(kRaster)}};
	}

	size_t getNumChannels(const int dir) const override
	{
		return dir == SOAPY_SDR_RX ? 1 : 0;
	}

	/* ---- stream ---------------------------------------------------- */
	std::vector<std::string> getStreamFormats(const int, const size_t) const override
	{
		return {SOAPY_SDR_CS16, SOAPY_SDR_CF32};
	}

	std::string getNativeStreamFormat(const int, const size_t,
					  double &fullScale) const override
	{
		fullScale = MT7921IQ_FULL_SCALE;
		return SOAPY_SDR_CS16;
	}

	SoapySDR::Stream *setupStream(const int dir, const std::string &format,
				      const std::vector<size_t> &channels,
				      const SoapySDR::Kwargs &) override
	{
		if (dir != SOAPY_SDR_RX)
			throw std::runtime_error("mt7921: RX only");
		if (channels.size() > 1 || (channels.size() == 1 && channels[0] != 0))
			throw std::runtime_error("mt7921: one channel (0)");
		if (format == SOAPY_SDR_CS16)
			_cs16 = true;
		else if (format == SOAPY_SDR_CF32)
			_cs16 = false;
		else
			throw std::runtime_error("mt7921: format must be CS16 or CF32");
		return reinterpret_cast<SoapySDR::Stream *>(this);
	}

	void closeStream(SoapySDR::Stream *) override { closeDevice(); }

	size_t getStreamMTU(SoapySDR::Stream *) const override { return kMTU; }

	int activateStream(SoapySDR::Stream *, const int, const long long,
			   const size_t) override
	{
		std::unique_lock<std::mutex> l(_mtx);

		if (_fd >= 0)
			return 0;
		int fd = ::open(_path.c_str(), O_RDONLY | O_NONBLOCK);
		if (fd < 0) {
			SoapySDR::logf(SOAPY_SDR_ERROR, "mt7921: open %s: %s%s",
				       _path.c_str(), strerror(errno),
				       errno == ENETDOWN ?
				       " (interface not in monitor mode?)" : "");
			return SOAPY_SDR_STREAM_ERROR;
		}
		if (!_dec)
			_dec = mt7921iq_dec_new();
		else
			mt7921iq_dec_reset(_dec);
		_raw.resize(kRawRead);
		_fd = fd;
		_hwFreq = 0;
		_waitGen = 0;
		_stale.clear();
		_dropUntilTuned = true;	/* first tune sets a known center */
		_tunePending = true;
		_gainPending = !_agc;	/* every open starts with the AGC */
		l.unlock();
		_cv.notify_all();
		return 0;
	}

	int deactivateStream(SoapySDR::Stream *, const int, const long long) override
	{
		closeDevice();
		return 0;
	}

	int readStream(SoapySDR::Stream *, void *const *buffs, const size_t numElems,
		       int &flags, long long &timeNs, const long timeoutUs) override
	{
		flags = 0;
		if (_fd < 0)
			return SOAPY_SDR_STREAM_ERROR;

		size_t want = std::min(numElems, kMTU);
		_cf.resize(want);
		const auto deadline = std::chrono::steady_clock::now() +
				      std::chrono::microseconds(timeoutUs);

		for (;;) {
			struct mt7921iq_chunk c;
			size_t n = mt7921iq_dec_read(_dec, MT7921IQ_CF32,
						     _cf.data(), want, &c);
			if (n) {
				int r = deliver(c, n, buffs[0], flags, timeNs);
				if (r != 0)
					return r;
				continue;	/* dropped (stale generation) */
			}
			if (_ended)
				return SOAPY_SDR_STREAM_ERROR;

			long left = std::chrono::duration_cast<std::chrono::milliseconds>(
				deadline - std::chrono::steady_clock::now()).count();
			if (left < 0)
				return SOAPY_SDR_TIMEOUT;
			struct pollfd p = {_fd, POLLIN, 0};
			if (::poll(&p, 1, (int)left) <= 0)
				return SOAPY_SDR_TIMEOUT;
			ssize_t got = ::read(_fd, _raw.data(), _raw.size());
			if (got < 0) {
				if (errno == EAGAIN || errno == EINTR)
					continue;
				return SOAPY_SDR_STREAM_ERROR;
			}
			if (got == 0) {
				/* driver ended the stream (reset, interface down) */
				mt7921iq_dec_flush(_dec);
				_ended = true;
				SoapySDR::log(SOAPY_SDR_ERROR, "mt7921: stream ended by the driver");
				continue;
			}
			if (mt7921iq_dec_push(_dec, _raw.data(), got))
				return SOAPY_SDR_CORRUPTION;
		}
	}

	/* ---- frequency ------------------------------------------------- */
	std::vector<std::string> listFrequencies(const int, const size_t) const override
	{
		return {"RF"};
	}

	SoapySDR::RangeList getFrequencyRange(const int, const size_t) const override
	{
		std::lock_guard<std::mutex> l(_mtx);
		SoapySDR::RangeList r;
		for (const auto &b : bands(_bw))
			r.push_back(SoapySDR::Range(b.lo - _maxOffset, b.hi + _maxOffset));
		return r;
	}

	SoapySDR::RangeList getFrequencyRange(const int d, const size_t c,
					      const std::string &) const override
	{
		return getFrequencyRange(d, c);
	}

	void setFrequency(const int, const size_t, const double frequency,
			  const SoapySDR::Kwargs &) override
	{
		setFreq(frequency);
	}

	void setFrequency(const int, const size_t, const std::string &,
			  const double frequency, const SoapySDR::Kwargs &) override
	{
		setFreq(frequency);
	}

	double getFrequency(const int, const size_t) const override
	{
		std::lock_guard<std::mutex> l(_mtx);
		return _freq;
	}

	double getFrequency(const int d, const size_t c, const std::string &) const override
	{
		return getFrequency(d, c);
	}

	/* ---- sample rate / bandwidth ----------------------------------- */
	/* The rate selects the channel width: 20.0008 MS/s -> 20 MHz,
	 * 40.0016 MS/s -> 40 MHz. A change while streaming retunes.
	 */
	void setSampleRate(const int, const size_t, const double rate) override
	{
		unsigned bw = rate > 60e6 ? 80 : rate > 30e6 ? 40 : 20;
		double want = rateOf(bw);

		if (std::abs(rate - want) > 0.01 * want)
			SoapySDR::logf(SOAPY_SDR_WARNING,
				       "mt7921: %.1f S/s is not supported, using %.1f",
				       rate, want);
		std::unique_lock<std::mutex> l(_mtx);
		if (bw == _bw)
			return;
		_bw = bw;
		if (_fd < 0)
			return;
		_tunePending = true;
		_dropUntilTuned = true;
		_waitGen = 0;
		l.unlock();
		_cv.notify_all();
	}

	double getSampleRate(const int, const size_t) const override
	{
		std::lock_guard<std::mutex> l(_mtx);
		return rateOf(_bw);
	}

	std::vector<double> listSampleRates(const int, const size_t) const override
	{
		return {kRate20, kRate40, kRate80};
	}

	SoapySDR::RangeList getSampleRateRange(const int, const size_t) const override
	{
		return {SoapySDR::Range(kRate20, kRate20), SoapySDR::Range(kRate40, kRate40),
			SoapySDR::Range(kRate80, kRate80)};
	}

	/* Usable bandwidth follows the rate; setBandwidth() is advisory. */
	double getBandwidth(const int, const size_t) const override
	{
		std::lock_guard<std::mutex> l(_mtx);
		return _bw == 80 ? 76e6 : _bw == 40 ? 36e6 : 18e6;
	}

	std::vector<double> listBandwidths(const int, const size_t) const override
	{
		return {18e6, 36e6, 76e6};
	}

	void setBandwidth(const int, const size_t, const double) override {}

	/* ---- antenna / gain -------------------------------------------- */
	std::vector<std::string> listAntennas(const int, const size_t) const override
	{
		return {"RX"};
	}

	std::string getAntenna(const int, const size_t) const override { return "RX"; }

	void setAntenna(const int, const size_t, const std::string &) override {}

	std::vector<std::string> listGains(const int, const size_t) const override
	{
		return {"LNA", "LPF"};
	}

	bool hasGainMode(const int, const size_t) const override { return true; }

	void setGainMode(const int, const size_t, const bool automatic) override
	{
		std::unique_lock<std::mutex> l(_mtx);
		if (automatic == _agc)
			return;
		_agc = automatic;
		requestGain(l);
	}

	bool getGainMode(const int, const size_t) const override
	{
		std::lock_guard<std::mutex> l(_mtx);
		return _agc;
	}

	/* Overall gain: LNA first (noise figure), the rest on the LPF. */
	void setGain(const int, const size_t, const double value) override
	{
		double v = std::min(std::max(value, 0.0), kLnaMaxDb + kLpfMaxDb);
		unsigned lna = MT7921IQ_LNA_MIN +
			       std::min<unsigned>(MT7921IQ_LNA_MAX - MT7921IQ_LNA_MIN,
						  (unsigned)(v / MT7921IQ_LNA_STEP_DB));
		double rest = v - (lna - MT7921IQ_LNA_MIN) * MT7921IQ_LNA_STEP_DB;

		std::unique_lock<std::mutex> l(_mtx);
		_lna = lna;
		_lpf = lpfCode(rest);
		if (!_agc)
			requestGain(l);
	}

	void setGain(const int, const size_t, const std::string &name,
		     const double value) override
	{
		std::unique_lock<std::mutex> l(_mtx);
		if (name == "LNA")
			_lna = lnaCode(value);
		else if (name == "LPF")
			_lpf = lpfCode(value);
		else
			throw std::runtime_error("mt7921: gain elements are LNA and LPF");
		if (!_agc)
			requestGain(l);
	}

	double getGain(const int, const size_t) const override
	{
		std::lock_guard<std::mutex> l(_mtx);
		return lnaDb(_lna) + lpfDb(_lpf);
	}

	double getGain(const int, const size_t, const std::string &name) const override
	{
		std::lock_guard<std::mutex> l(_mtx);
		if (name == "LNA")
			return lnaDb(_lna);
		if (name == "LPF")
			return lpfDb(_lpf);
		throw std::runtime_error("mt7921: gain elements are LNA and LPF");
	}

	SoapySDR::Range getGainRange(const int, const size_t) const override
	{
		return SoapySDR::Range(0, kLnaMaxDb + kLpfMaxDb, MT7921IQ_LPF_STEP_DB);
	}

	SoapySDR::Range getGainRange(const int, const size_t,
				     const std::string &name) const override
	{
		if (name == "LNA")
			return SoapySDR::Range(0, kLnaMaxDb, MT7921IQ_LNA_STEP_DB);
		if (name == "LPF")
			return SoapySDR::Range(0, kLpfMaxDb, MT7921IQ_LPF_STEP_DB);
		throw std::runtime_error("mt7921: gain elements are LNA and LPF");
	}

	/* ---- settings -------------------------------------------------- */
	SoapySDR::ArgInfoList getSettingInfo() const override
	{
		SoapySDR::ArgInfo a;
		a.key = "max_offset";
		a.name = "Max NCO offset";
		a.description = "Largest software frequency shift before the receiver retunes (Hz); 2.5e6 = always the nearest 5 MHz step";
		a.type = SoapySDR::ArgInfo::FLOAT;
		a.value = std::to_string(kRaster / 2);
		a.units = "Hz";
		a.range = SoapySDR::Range(kRaster / 2, 9e6);
		SoapySDR::ArgInfo c;
		c.key = "chain";
		c.name = "RX chain";
		c.description = "Receive chain of dense streams (driver iq_dense): 0 or 1; changing it retunes";
		c.type = SoapySDR::ArgInfo::INT;
		c.value = "0";
		c.options = {"0", "1"};
		return {a, c};
	}

	void writeSetting(const std::string &key, const std::string &value) override
	{
		if (key == "max_offset") {
			std::lock_guard<std::mutex> l(_mtx);
			_maxOffset = std::min(std::max(std::stod(value), kRaster / 2), 9e6);
		}
		if (key == "chain") {
			std::unique_lock<std::mutex> l(_mtx);
			int chain = std::stoi(value) ? 1 : 0;

			if (chain == _chain)
				return;
			_chain = chain;
			if (_fd < 0)
				return;		/* applied by activateStream */
			_tunePending = true;
			_dropUntilTuned = true;
			_waitGen = 0;
			l.unlock();
			_cv.notify_all();
		}
	}

	std::string readSetting(const std::string &key) const override
	{
		if (key == "max_offset")
			return std::to_string(_maxOffset);
		if (key == "chain")
			return std::to_string(_chain);
		return "";
	}

private:
	/* Caller-visible part of a retune: keep the NCO exact, ask the tuner
	 * thread for a new hardware center only when needed.
	 */
	void setFreq(double f)
	{
		std::unique_lock<std::mutex> l(_mtx);
		_freq = f;
		if (_fd < 0)
			return;		/* applied by activateStream */
		if (_hwFreq && _hwBw == _bw && std::abs(f - _hwFreq) <= _maxOffset)
			return;		/* NCO only, no gap */
		_tunePending = true;
		_dropUntilTuned = true;
		_waitGen = 0;		/* nothing passes until the new tune */
		l.unlock();
		_cv.notify_all();
	}

	/* Have the tuner thread apply the gain setting (the ioctl can wait for
	 * a retune in progress, so readStream() never makes it). Caller holds
	 * _mtx through l.
	 */
	void requestGain(std::unique_lock<std::mutex> &l)
	{
		if (_fd < 0)
			return;		/* applied by activateStream */
		_gainPending = true;
		l.unlock();
		_cv.notify_all();
	}

	void tunerLoop()
	{
		std::unique_lock<std::mutex> l(_mtx);
		for (;;) {
			_cv.wait(l, [this] {
				return _quit || ((_tunePending || _gainPending) && _fd >= 0);
			});
			if (_quit)
				return;
			if (!_tunePending) {
				/* Gain only: live, no gap. The driver keeps a hold
				 * across retunes, so the order against tunes does not
				 * matter.
				 */
				_gainPending = false;
				bool agc = _agc;
				unsigned lna = _lna, lpf = _lpf;
				int fd = _fd;
				l.unlock();
				int err = mt7921iq_set_gain(fd, !agc, lna, lpf);
				l.lock();
				if (err) {
					SoapySDR::logf(SOAPY_SDR_ERROR,
						       "mt7921: gain %s: %s; AGC",
						       agc ? "AGC" : "hold", strerror(-err));
					_agc = true;
				} else if (!agc) {
					SoapySDR::logf(SOAPY_SDR_DEBUG,
						       "mt7921: gain held at LNA %u, LPF %u",
						       lna, lpf);
				}
				continue;
			}
			_tunePending = false;
			unsigned bw = _bw;
			int chain = _chain;
			double want = _freq, center = nearestCenter(want, kRaster / 2, bw);
			int fd = _fd;
			if (!center) {
				SoapySDR::logf(SOAPY_SDR_ERROR,
					       "mt7921: %.6f MHz is outside the %u MHz tuning range",
					       want / 1e6, bw);
				_dropUntilTuned = false;
				continue;
			}
			l.unlock();
			/* The generation running now may not have reached
			 * deliver() yet: mark it old too.
			 */
			struct mt7921_iq_tune cur = {};
			bool haveCur = ioctl(fd, MT7921_IQ_IOC_GET, &cur) == 0;
			double actual = 0;
			uint32_t gen = 0;
			unsigned abw = 0;
			int err = mt7921iq_tune_chain(fd, center, bw, chain, &actual,
						      &abw, &gen);
			l.lock();
			if (err) {
				SoapySDR::logf(SOAPY_SDR_ERROR, "mt7921: tune %.3f MHz: %s",
					       center / 1e6, strerror(-err));
				_dropUntilTuned = false;
				continue;
			}
			/* Older generations are dropped after a retune, so only
			 * the newest one needs its hardware center.
			 */
			if (haveCur && cur.generation)
				addStale(cur.generation);
			for (const auto &g : _genFreq)
				addStale(g.first);
			_genFreq.clear();
			_genFreq[gen] = actual;
			if (!_tunePending) {
				_hwFreq = actual;
				_hwBw = abw;
				_waitGen = gen;
			}
		}
	}

	/* Hand one decoded chunk to the caller; returns samples, an error
	 * code, or 0 to drop it.
	 */
	int deliver(const struct mt7921iq_chunk &c, size_t n, void *out,
		    int &flags, long long &timeNs)
	{
		double hw;
		{
			std::lock_guard<std::mutex> l(_mtx);
			if (_dropUntilTuned) {
				if (!_waitGen)
					return 0;	/* tune requested or running */
				_dropUntilTuned = false;
			}
			auto it = _genFreq.find(c.generation);
			if (it == _genFreq.end()) {
				/* Not the tune's generation: one from before the
				 * tune, or a restart by the driver since (realign
				 * at a slip-prone DMA alignment, MCU command),
				 * which is at the tuned center.
				 */
				if (!_hwFreq || isStale(c.generation))
					return 0;
				it = _genFreq.emplace(c.generation, _hwFreq).first;
			}
			hw = it->second;
			_ncoStep = -2 * M_PI * (_freq - hw) /
				   (c.sample_rate > 0 ? c.sample_rate : kRate20);
		}
		/* Samples were lost before this chunk: report it the usual
		 * Soapy way. The first piece of the chunk (at most one MTU,
		 * ~3 ms) is dropped with it.
		 */
		if (c.flags & (MT7921IQ_F_SEQ_GAP | MT7921IQ_F_OVERRUN |
			       MT7921IQ_F_CURSOR)) {
			_overflows++;
			return SOAPY_SDR_OVERFLOW;
		}

		/* NCO: rotate by the offset between hardware center and request */
		std::complex<float> *s = _cf.data();
		std::complex<double> rot(std::cos(_ncoStep), std::sin(_ncoStep));
		for (size_t k = 0; k < n; k++) {
			s[k] *= std::complex<float>(_nco);
			_nco *= rot;
		}
		_nco /= std::abs(_nco);

		if (_cs16) {
			int16_t *o = static_cast<int16_t *>(out);
			for (size_t k = 0; k < n; k++) {
				float i = s[k].real() * MT7921IQ_FULL_SCALE;
				float q = s[k].imag() * MT7921IQ_FULL_SCALE;
				o[2 * k] = (int16_t)std::lrint(std::min(std::max(i, -32768.f), 32767.f));
				o[2 * k + 1] = (int16_t)std::lrint(std::min(std::max(q, -32768.f), 32767.f));
			}
		} else {
			std::memcpy(out, s, n * sizeof(*s));
		}
		flags |= SOAPY_SDR_HAS_TIME;
		timeNs = (long long)c.time_ns;
		return (int)n;
	}

	/* Caller holds _mtx. */
	void addStale(uint32_t gen)
	{
		if (isStale(gen))
			return;
		if (_stale.size() >= kStaleMax)
			_stale.pop_front();
		_stale.push_back(gen);
	}

	bool isStale(uint32_t gen) const
	{
		return std::find(_stale.begin(), _stale.end(), gen) != _stale.end();
	}

	void closeDevice()
	{
		std::lock_guard<std::mutex> l(_mtx);
		if (_fd >= 0)
			::close(_fd);
		_fd = -1;
		_ended = false;
		_genFreq.clear();
	}

	std::string _path;
	mutable std::mutex _mtx;
	std::condition_variable _cv;
	std::thread _tuner;
	bool _quit = false;

	int _fd = -1;
	bool _cs16 = true;
	bool _ended = false;
	struct mt7921iq_dec *_dec = nullptr;
	std::vector<uint8_t> _raw;
	std::vector<std::complex<float>> _cf;

	/* tuning */
	double _freq = 2437e6;		/* requested */
	double _hwFreq = 0;		/* hardware center of the latest tune */
	double _maxOffset = kRaster / 2;
	unsigned _bw = 20;		/* requested channel width (sample rate) */
	int _chain = 0;			/* RX chain of dense streams */
	/* gain: AGC, or a hold at (_lna, _lpf); default = the AGC's idle code */
	bool _agc = true;
	unsigned _lna = MT7921IQ_LNA_MAX, _lpf = 7;
	bool _gainPending = false;
	unsigned _hwBw = 0;		/* width of the latest tune */
	bool _tunePending = false;
	bool _dropUntilTuned = false;
	uint32_t _waitGen = 0;
	std::map<uint32_t, double> _genFreq;
	std::deque<uint32_t> _stale;	/* generations at an older center */

	/* NCO */
	double _ncoStep = 0;
	std::complex<double> _nco{1, 0};

	uint64_t _overflows = 0;
};

SoapySDR::KwargsList findMT7921(const SoapySDR::Kwargs &args)
{
	SoapySDR::KwargsList out;
	auto want = args.find("device");

	for (const auto &d : listDevices()) {
		if (want != args.end() && want->second != d)
			continue;
		SoapySDR::Kwargs k;
		k["driver"] = "mt7921";
		k["device"] = d;
		k["label"] = "MT7921AU I/Q (" + d.substr(d.rfind('_') + 1) + ")";
		out.push_back(k);
	}
	return out;
}

SoapySDR::Device *makeMT7921(const SoapySDR::Kwargs &args)
{
	return new MT7921Device(args);
}

SoapySDR::Registry registerMT7921("mt7921", &findMT7921, &makeMT7921,
				  SOAPY_SDR_ABI_VERSION);

} // namespace
