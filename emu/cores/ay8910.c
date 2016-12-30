/***************************************************************************

  ay8910.c

  Emulation of the AY-3-8910 / YM2149 sound chip.

  Based on various code snippets by Ville Hallik, Michael Cuddy,
  Tatsuyuki Satoh, Fabrice Frances, Nicola Salmoria.

  Mostly rewritten by couriersud in 2008

  Public documentation:

  - http://privatfrickler.de/blick-auf-den-chip-soundchip-general-instruments-ay-3-8910/
    Die pictures of the AY8910

  - US Patent 4933980

  Games using ADSR: gyruss

  A list with more games using ADSR can be found here:
        http://mametesters.org/view.php?id=3043

  TODO:
  * The AY8930 has an extended mode which is currently
    not emulated.
  * The YMZ284 only has one 1 output channel (mixed chan A,B,C).
    This should be forced.
  * YM2610 & YM2608 will need a separate flag in their config structures
    to distinguish between legacy and discrete mode.

  The rewrite also introduces a generic model for the DAC. This model is
  not perfect, but allows channel mixing based on a parametrized approach.
  This model also allows to factor in different loads on individual channels.
  If a better model is developped in the future or better measurements are
  available, the driver should be easy to change. The model is described
  later.

  In order to not break hundreds of existing drivers by default the flag
  AY8910_LEGACY_OUTPUT is used by drivers not changed to take into account the
  new model. All outputs are normalized to the old output range (i.e. 0 .. 7ffff).
  In the case of channel mixing, output range is 0...3 * 7fff.

  The main difference between the AY-3-8910 and the YM2149 is, that the
  AY-3-8910 datasheet mentions, that fixed volume level 0, which is set by
  registers 8 to 10 is "channel off". The YM2149 mentions, that the generated
  signal has a 2V DC component. This is confirmed by measurements. The approach
  taken here is to assume the 2V DC offset for all outputs for the YM2149.
  For the AY-3-8910, an offset is used if envelope is active for a channel.
  This is backed by oscilloscope pictures from the datasheet. If a fixed volume
  is set, i.e. envelope is disabled, the output voltage is set to 0V. Recordings
  I found on the web for gyruss indicate, that the AY-3-8910 offset should
  be around 0.2V. This will also make sound levels more compatible with
  user observations for scramble.

  The Model:
                     5V     5V
                      |      |
                      /      |
  Volume Level x >---|       Z
                      >      Z Pullup Resistor RU
                       |     Z
                       Z     |
                    Rx Z     |
                       Z     |
                       |     |
                       '-----+-------->  >---+----> Output signal
                             |               |
                             Z               Z
               Pulldown RD   Z               Z Load RL
                             Z               Z
                             |               |
                            GND             GND

Each Volume level x will select a different resistor Rx. Measurements from fpgaarcade.com
where used to calibrate channel mixing for the YM2149. This was done using
a least square approach using a fixed RL of 1K Ohm.

For the AY measurements cited in e.g. openmsx as "Hacker Kay" for a single
channel were taken. These were normalized to 0 ... 65535 and consequently
adapted to an offset of 0.2V and a VPP of 1.3V. These measurements are in
line e.g. with the formula used by pcmenc for the volume: vol(i) = exp(i/2-7.5).

The following is documentation from the code moved here and amended to reflect
the changes done:

Careful studies of the chip output prove that the chip counts up from 0
until the counter becomes greater or equal to the period. This is an
important difference when the program is rapidly changing the period to
modulate the sound. This is worthwhile noting, since the datasheets
say, that the chip counts down.
Also, note that period = 0 is the same as period = 1. This is mentioned
in the YM2203 data sheets. However, this does NOT apply to the Envelope
period. In that case, period = 0 is half as period = 1.

Envelope shapes:
    C AtAlH
    0 0 x x  \___
    0 1 x x  /___
    1 0 0 0  \\\\
    1 0 0 1  \___
    1 0 1 0  \/\/
    1 0 1 1  \```
    1 1 0 0  ////
    1 1 0 1  /```
    1 1 1 0  /\/\
    1 1 1 1  /___

The envelope counter on the AY-3-8910 has 16 steps. On the YM2149 it
has twice the steps, happening twice as fast.

***************************************************************************/

#include <stdlib.h>
#include <string.h>	// for memset
#include <stdio.h>

#include <stdtype.h>
#include "../snddef.h"
#include "../EmuStructs.h"
#include "ay8910.h"

/*************************************
 *
 *  Defines
 *
 *************************************/

#define MAX_OUTPUT 0x4000
#define NUM_CHANNELS 3

/* register id's */
#define AY_AFINE	(0)
#define AY_ACOARSE	(1)
#define AY_BFINE	(2)
#define AY_BCOARSE	(3)
#define AY_CFINE	(4)
#define AY_CCOARSE	(5)
#define AY_NOISEPER	(6)
#define AY_ENABLE	(7)
#define AY_AVOL		(8)
#define AY_BVOL		(9)
#define AY_CVOL		(10)
#define AY_EFINE	(11)
#define AY_ECOARSE	(12)
#define AY_ESHAPE	(13)

#define AY_PORTA	(14)
#define AY_PORTB	(15)

#define NOISE_ENABLEQ(_psg, _chan)	(((_psg)->regs[AY_ENABLE] >> (3 + _chan)) & 1)
#define TONE_ENABLEQ(_psg, _chan)	(((_psg)->regs[AY_ENABLE] >> (_chan)) & 1)
#define TONE_PERIOD(_psg, _chan)	( (_psg)->regs[(_chan) << 1] | (((_psg)->regs[((_chan) << 1) | 1] & 0x0f) << 8) )
#define NOISE_PERIOD(_psg)			( (_psg)->regs[AY_NOISEPER] & 0x1f)
#define TONE_VOLUME(_psg, _chan)	( (_psg)->regs[AY_AVOL + (_chan)] & 0x0f)
#define TONE_ENVELOPE(_psg, _chan)	(((_psg)->regs[AY_AVOL + (_chan)] >> 4) & (((_psg)->chip_type == CHTYPE_AY8914) ? 3 : 1))
#define ENVELOPE_PERIOD(_psg)		(((_psg)->regs[AY_EFINE] | ((_psg)->regs[AY_ECOARSE]<<8)))
#define NOISE_OUTPUT(_psg)			((_psg)->rng & 1)

#define CHTYPE_AY8910	0x00
#define CHTYPE_AY8912	0x01
#define CHTYPE_AY8913	0x02
#define CHTYPE_AY8930	0x03
#define CHTYPE_AY8914	0x04
#define CHTYPE_YM2149	0x10
#define CHTYPE_YM3439	0x11
#define CHTYPE_YMZ284	0x12
#define CHTYPE_YMZ294	0x13
#define CHTYPE_YM2203	0x20
#define CHTYPE_YM2608	0x21
#define CHTYPE_YM2610	0x22
#define CHTYPE_YM2610B	0x23

/*************************************
 *
 *  Type definitions
 *
 *************************************/

typedef struct _ay_ym_param ay_ym_param;
struct _ay_ym_param
{
	double r_up;
	double r_down;
	int    res_count;
	double res[32];
};

//typedef struct _ay8910_context ay8910_context;
struct _ay8910_context
{
	void* chipInf;
	
	ay8910_interface intf;
	UINT8 register_latch;
	UINT8 last_enable;
	UINT8 regs[16];
	INT32 count[NUM_CHANNELS];
	UINT8 output[NUM_CHANNELS];
	UINT8 prescale_noise;
	INT32 count_noise;
	INT32 count_env;
	INT8 env_step;
	UINT32 env_volume;
	UINT8 hold,alternate,attack,holding;
	INT32 rng;
	UINT8 env_step_mask;
	/* init parameters ... */
	int step;
	UINT8 zero_is_off;
	UINT8 vol_enabled[NUM_CHANNELS];
	const ay_ym_param *par;
	const ay_ym_param *par_env;
	INT32 vol_table[NUM_CHANNELS][16];
	INT32 env_table[NUM_CHANNELS][32];
	INT32 vol3d_table[8*32*32*32];
	UINT8 StereoMask[NUM_CHANNELS];
	UINT32 MuteMsk[NUM_CHANNELS];
	
	UINT32 clock;
	UINT8 chip_type;
	UINT8 chip_flags;
	UINT8 IsDisabled;
	
	DEVCB_SRATE_CHG SmpRateFunc;
	void* SmpRateData;
};

#define MAX_UPDATE_LEN	0x1000	// in samples
static DEV_SMPL AYBuf[NUM_CHANNELS][MAX_UPDATE_LEN];


/*************************************
 *
 *  Static
 *
 *************************************/

static const ay_ym_param ym2149_param =
{
	630, 801,
	16,
	{ 73770, 37586, 27458, 21451, 15864, 12371, 8922,  6796,
	   4763,  3521,  2403,  1737,  1123,   762,  438,   251 },
};

static const ay_ym_param ym2149_param_env =
{
	630, 801,
	32,
	{ 103350, 73770, 52657, 37586, 32125, 27458, 24269, 21451,
	   18447, 15864, 14009, 12371, 10506,  8922,  7787,  6796,
	    5689,  4763,  4095,  3521,  2909,  2403,  2043,  1737,
	    1397,  1123,   925,   762,   578,   438,   332,   251 },
};

#if 0
/* RL = 1000, Hacker Kay normalized, 2.1V to 3.2V */
static const ay_ym_param ay8910_param =
{
	664, 913,
	16,
	{ 85785, 34227, 26986, 20398, 14886, 10588,  7810,  4856,
	   4120,  2512,  1737,  1335,  1005,   747,   586,    451 },
};

/*
 * RL = 3000, Hacker Kay normalized pattern, 1.5V to 2.8V
 * These values correspond with guesses based on Gyruss schematics
 * They work well with scramble as well.
 */
static const ay_ym_param ay8910_param =
{
	930, 454,
	16,
	{ 85066, 34179, 27027, 20603, 15046, 10724, 7922, 4935,
	   4189,  2557,  1772,  1363,  1028,  766,   602,  464 },
};

/*
 * RL = 1000, Hacker Kay normalized pattern, 0.75V to 2.05V
 * These values correspond with guesses based on Gyruss schematics
 * They work well with scramble as well.
 */
static const ay_ym_param ay8910_param =
{
	1371, 313,
	16,
	{ 93399, 33289, 25808, 19285, 13940, 9846,  7237,  4493,
	   3814,  2337,  1629,  1263,   962,  727,   580,   458 },
};

/*
 * RL = 1000, Hacker Kay normalized pattern, 0.2V to 1.5V
 */
static const ay_ym_param ay8910_param =
{
	5806, 300,
	16,
	{ 118996, 42698, 33105, 24770, 17925, 12678,  9331,  5807,
        4936,  3038,  2129,  1658,  1271,   969,   781,   623 }
};
#endif

/*
 * RL = 2000, Based on Matthew Westcott's measurements from Dec 2001.
 * -------------------------------------------------------------------
 *
 * http://groups.google.com/group/comp.sys.sinclair/browse_thread/thread/fb3091da4c4caf26/d5959a800cda0b5e?lnk=gst&q=Matthew+Westcott#d5959a800cda0b5e
 * After what Russell mentioned a couple of weeks back about the lack of
 * publicised measurements of AY chip volumes - I've finally got round to
 * making these readings, and I'm placing them in the public domain - so
 * anyone's welcome to use them in emulators or anything else.

 * To make the readings, I set up the chip to produce a constant voltage on
 * channel C (setting bits 2 and 5 of register 6), and varied the amplitude
 * (the low 4 bits of register 10). The voltages were measured between the
 * channel C output (pin 1) and ground (pin 6).
 *
 * Level  Voltage
 *  0     1.147
 *  1     1.162
 *  2     1.169
 *  3     1.178
 *  4     1.192
 *  5     1.213
 *  6     1.238
 *  7     1.299
 *  8     1.336
 *  9     1.457
 * 10     1.573
 * 11     1.707
 * 12     1.882
 * 13     2.06
 * 14     2.32
 * 15     2.58
 * -------------------------------------------------------------------
 *
 * The ZX spectrum output circuit was modelled in SwitcherCAD and
 * the resistor values below create the voltage levels above.
 * RD was measured on a real chip to be 8m Ohm, RU was 0.8m Ohm.
 */

static const ay_ym_param ay8910_param =
{
	800000, 8000000,
	16,
	{ 15950, 15350, 15090, 14760, 14275, 13620, 12890, 11370,
      10600,  8590,  7190,  5985,  4820,  3945,  3017,  2345 }
};

/*************************************
 *
 *  Inline
 *
 *************************************/

INLINE void build_single_table(double rl, const ay_ym_param *par, int normalize, INT32 *tab, int zero_is_off)
{
	int j;
	double rt, rw = 0;
	double temp[32], min=10.0, max=0.0;

	for (j=0; j < par->res_count; j++)
	{
		rt = 1.0 / par->r_down + 1.0 / rl;

		rw = 1.0 / par->res[j];
		rt += 1.0 / par->res[j];

		if (!(zero_is_off && j == 0))
		{
			rw += 1.0 / par->r_up;
			rt += 1.0 / par->r_up;
		}

		temp[j] = rw / rt;
		if (temp[j] < min)
			min = temp[j];
		if (temp[j] > max)
			max = temp[j];
	}
	if (normalize)
	{
		for (j=0; j < par->res_count; j++)
			/* The following line generates values that cause clicks when starting/pausing/stopping
				because there're off (the center is at zero, not the base).
				That's quite bad for a player.
			tab[j] = MAX_OUTPUT * (((temp[j] - min)/(max-min)) - 0.25) * 0.5;
			*/
			tab[j] = MAX_OUTPUT * ((temp[j] - min)/(max-min)) / NUM_CHANNELS;
	}
	else
	{
		for (j=0; j < par->res_count; j++)
			tab[j] = MAX_OUTPUT * temp[j] / NUM_CHANNELS;
	}

}

INLINE UINT16 mix_3D(ay8910_context *psg)
{
	int indx = 0, chan;

	for (chan = 0; chan < NUM_CHANNELS; chan++)
		if (TONE_ENVELOPE(psg, chan) != 0)
		{
			if (psg->chip_type == CHTYPE_AY8914) // AY8914 Has a two bit tone_envelope field
			{
				indx |= (1 << (chan+15)) | ( psg->vol_enabled[chan] ? ((psg->env_volume >> (3-TONE_ENVELOPE(psg, chan))) << (chan*5)) : 0);
			}
			else
			{
				indx |= (1 << (chan+15)) | ( psg->vol_enabled[chan] ? psg->env_volume << (chan*5) : 0);
			}
		}
		else
		{
			indx |= (psg->vol_enabled[chan] ? TONE_VOLUME(psg, chan) << (chan*5) : 0);
		}
	return psg->vol3d_table[indx];
}

/*************************************
 *
 * Static functions
 *
 *************************************/

void ay8910_write_reg(ay8910_context *psg, UINT8 r, UINT8 v)
{
	if (r > 15) return;

	psg->regs[r] = v;

	switch( r )
	{
		case AY_AFINE:
		case AY_ACOARSE:
		case AY_BFINE:
		case AY_BCOARSE:
		case AY_CFINE:
		case AY_CCOARSE:
		case AY_NOISEPER:
		case AY_AVOL:
		case AY_BVOL:
		case AY_CVOL:
		case AY_EFINE:
		case AY_ECOARSE:
			/* No action required */
			break;
		case AY_ENABLE:
			if (psg->last_enable == 0xFF)
				psg->last_enable = ~psg->regs[AY_ENABLE];

			if ((psg->last_enable & 0x40) != (psg->regs[AY_ENABLE] & 0x40))
			{
				// write out 0xFF if port set to input
				//UINT8 data = (psg->regs[AY_ENABLE] & 0x40) ? psg->regs[AY_PORTA] : 0xFF;
				//psg->portAwrite(0x00, data);
			}

			if ((psg->last_enable & 0x80) != (psg->regs[AY_ENABLE] & 0x80))
			{
				// write out 0xFF if port set to input
				//UINT8 data = (psg->regs[AY_ENABLE] & 0x80) ? psg->regs[AY_PORTB] : 0xFF;
				//psg->portBwrite(0x00, data);
			}
			if (~v & 0x3F)	// one of the channels gets enabled -> enable emulation
				psg->IsDisabled = 0x00;

			psg->last_enable = psg->regs[AY_ENABLE] & 0xC0;
			break;
		case AY_ESHAPE:
			psg->attack = (psg->regs[AY_ESHAPE] & 0x04) ? psg->env_step_mask : 0x00;
			if ((psg->regs[AY_ESHAPE] & 0x08) == 0)
			{
				/* if Continue = 0, map the shape to the equivalent one which has Continue = 1 */
				psg->hold = 1;
				psg->alternate = psg->attack;
			}
			else
			{
				psg->hold = psg->regs[AY_ESHAPE] & 0x01;
				psg->alternate = psg->regs[AY_ESHAPE] & 0x02;
			}
			psg->env_step = psg->env_step_mask;
			psg->holding = 0;
			psg->env_volume = (psg->env_step ^ psg->attack);
			break;
		case AY_PORTA:
			//if (psg->regs[AY_ENABLE] & 0x40)
			//	psg->portAwrite(0x00, psg->regs[AY_PORTA]);
			break;
		case AY_PORTB:
			//if (psg->regs[AY_ENABLE] & 0x80)
			//	psg->portBwrite(0x00, psg->regs[AY_PORTB]);
			break;
	}
}

void ay8910_update_one(void *param, UINT32 samples, DEV_SMPL **outputs)
{
	ay8910_context *psg = (ay8910_context *)param;
	DEV_SMPL *buf[NUM_CHANNELS];
	int chan;
	UINT32 cursmpl;
	UINT32 buf_smpls;
	DEV_SMPL *bufL = outputs[0];
	DEV_SMPL *bufR = outputs[1];
	
	memset(outputs[0], 0x00, samples * sizeof(DEV_SMPL));
	memset(outputs[1], 0x00, samples * sizeof(DEV_SMPL));
	
	// Speed hack for OPN chips (YM2203, YM26xx), that have an often unused AY8910
	if (psg->IsDisabled)
		return;
	
	buf_smpls = samples;
	//buf[0] = outputs[0];
	buf[0] = AYBuf[0];
	buf[1] = AYBuf[1];
	buf[2] = AYBuf[2];

	/* hack to prevent us from hanging when starting filtered outputs */
	for (chan = 0; chan < NUM_CHANNELS; chan++)
		if (buf[chan] != NULL)
			memset(buf[chan], 0, samples * sizeof(*buf[chan]));

	/* The 8910 has three outputs, each output is the mix of one of the three */
	/* tone generators and of the (single) noise generator. The two are mixed */
	/* BEFORE going into the DAC. The formula to mix each channel is: */
	/* (ToneOn | ToneDisable) & (NoiseOn | NoiseDisable). */
	/* Note that this means that if both tone and noise are disabled, the output */
	/* is 1, not 0, and can be modulated changing the volume. */

	if (buf_smpls > MAX_UPDATE_LEN)
		buf_smpls = MAX_UPDATE_LEN;
	/* buffering loop */
	while (buf_smpls)
	{
		for (chan = 0; chan < NUM_CHANNELS; chan++)
		{
			psg->count[chan]++;
			if (psg->count[chan] >= TONE_PERIOD(psg, chan))
			{
				psg->output[chan] ^= 1;
				psg->count[chan] = 0;
			}
		}

		psg->count_noise++;
		if (psg->count_noise >= NOISE_PERIOD(psg))
		{
			/* toggle the prescaler output. Noise is no different to
			 * channels.
			 */
			psg->count_noise = 0;
			psg->prescale_noise ^= 1;

			if ( psg->prescale_noise)
			{
				/* The Random Number Generator of the 8910 is a 17-bit shift */
				/* register. The input to the shift register is bit0 XOR bit3 */
				/* (bit0 is the output). This was verified on AY-3-8910 and YM2149 chips. */

				psg->rng ^= (((psg->rng & 1) ^ ((psg->rng >> 3) & 1)) << 17);
				psg->rng >>= 1;
			}
		}

		for (chan = 0; chan < NUM_CHANNELS; chan++)
		{
			psg->vol_enabled[chan] = (psg->output[chan] | TONE_ENABLEQ(psg, chan)) & (NOISE_OUTPUT(psg) | NOISE_ENABLEQ(psg, chan));
		}

		/* update envelope */
		if (psg->holding == 0)
		{
			psg->count_env++;
			if (psg->count_env >= ENVELOPE_PERIOD(psg) * psg->step )
			{
				psg->count_env = 0;
				psg->env_step--;

				/* check envelope current position */
				if (psg->env_step < 0)
				{
					if (psg->hold)
					{
						if (psg->alternate)
							psg->attack ^= psg->env_step_mask;
						psg->holding = 1;
						psg->env_step = 0;
					}
					else
					{
						/* if CountEnv has looped an odd number of times (usually 1), */
						/* invert the output. */
						if (psg->alternate && (psg->env_step & (psg->env_step_mask + 1)))
 							psg->attack ^= psg->env_step_mask;

						psg->env_step &= psg->env_step_mask;
					}
				}

			}
		}
		psg->env_volume = (psg->env_step ^ psg->attack);

		for (chan = 0; chan < NUM_CHANNELS; chan++)
		{
			if (TONE_ENVELOPE(psg, chan) != 0)
			{
				/* Envolope has no "off" state */
				if (psg->chip_type == CHTYPE_AY8914) // AY8914 Has a two bit tone_envelope field
				{
					*(buf[chan]++) = psg->env_table[chan][psg->vol_enabled[chan] ? psg->env_volume >> (3-TONE_ENVELOPE(psg,chan)) : 0];
				}
				else
				{
					*(buf[chan]++) = psg->env_table[chan][psg->vol_enabled[chan] ? psg->env_volume : 0];
				}
			}
			else
			{
				*(buf[chan]++) = psg->vol_table[chan][psg->vol_enabled[chan] ? TONE_VOLUME(psg, chan) : 0];
			}
		}
		buf_smpls--;
		
	}//end while()
	
	buf_smpls = samples;
	if (buf_smpls > MAX_UPDATE_LEN)
		buf_smpls = MAX_UPDATE_LEN;
	for (cursmpl = 0; cursmpl < buf_smpls; cursmpl ++)
	{
		for (chan = 0; chan < NUM_CHANNELS; chan ++)
		{
			if (psg->StereoMask[chan] & 0x01)
				bufL[cursmpl] += AYBuf[chan][cursmpl] & psg->MuteMsk[chan];
			if (psg->StereoMask[chan] & 0x02)
				bufR[cursmpl] += AYBuf[chan][cursmpl] & psg->MuteMsk[chan];
		}
	}
}

static void build_mixer_table(ay8910_context *psg)
{
	int	normalize = 0;
	int	chan;

	if ((psg->intf.flags & AY8910_LEGACY_OUTPUT) != 0)
	{
#ifdef _DEBUG
		//logerror("AY-3-8910/YM2149 using legacy output levels!\n");
#endif
		normalize = 1;
	}
	normalize = 1;

	for (chan=0; chan < NUM_CHANNELS; chan++)
	{
		build_single_table(psg->intf.res_load[chan], psg->par, normalize, psg->vol_table[chan], psg->zero_is_off);
		build_single_table(psg->intf.res_load[chan], psg->par_env, normalize, psg->env_table[chan], 0);
	}
}

/*************************************
 *
 * Public functions
 *
 *   used by e.g. YM2203, YM2210 ...
 *
 *************************************/

UINT32 ay8910_start_ym(void **chip, UINT32 clock, UINT8 ay_type, UINT8 ay_flags)
{
	static const ay8910_interface generic_ay8910 =
	{
		AY8910_LEGACY_OUTPUT,
		AY8910_DEFAULT_LOADS
	};
	ay8910_context *info;

	info = (ay8910_context*)calloc(1, sizeof(ay8910_context));
	if (info == NULL)
		return 0;
	*chip = info;

	info->intf = generic_ay8910;
	info->intf.flags = ay_flags & 0x0F;
	info->SmpRateFunc = NULL;

	info->clock = clock;
	info->chip_type = ay_type;
	info->chip_flags = ay_flags;
	if ((info->chip_type & 0xF0) == 0x00)	// CHTYPE_AY89xx variants
	{
		info->step = 2;
		info->par = &ay8910_param;
		info->par_env = &ay8910_param;
		info->zero_is_off = 0;		/* FIXME: Remove after verification that off=vol(0) */
		info->env_step_mask = 0x0F;
	}
	else //if ((info->chip_type & 0xF0) >= 0x10)	// CHTYPE_YMxxxx variants (also YM2203/2608/2610)
	{
		info->step = 1;
		info->par = &ym2149_param;
		info->par_env = &ym2149_param_env;
		info->zero_is_off = 0;
		info->env_step_mask = 0x1F;
	}
	if (info->chip_flags & AY8910_ZX_STEREO)
	{
		// ABC Stereo
		info->StereoMask[0] = 0x01;
		info->StereoMask[1] = 0x03;
		info->StereoMask[2] = 0x02;
	}
	else
	{
		info->StereoMask[0] = 0x03;
		info->StereoMask[1] = 0x03;
		info->StereoMask[2] = 0x03;
	}

	build_mixer_table(info);

	//ay8910_set_clock_ym(info, clock);
	ay8910_set_mute_mask_ym(info, 0x00);

	return ay8910_get_sample_rate(info);
}

void ay8910_stop_ym(void *chip)
{
	free(chip);
}

void ay8910_reset_ym(void *chip)
{
	ay8910_context *psg = (ay8910_context *)chip;
	UINT8 i;

	psg->register_latch = 0;
	psg->rng = 1;
	psg->output[0] = 0;
	psg->output[1] = 0;
	psg->output[2] = 0;
	psg->count[0] = 0;
	psg->count[1] = 0;
	psg->count[2] = 0;
	psg->count_noise = 0;
	psg->count_env = 0;
	psg->prescale_noise = 0;
	psg->last_enable = 0xFF;	/* force a write */
	for (i = 0;i < AY_PORTA;i++)
		ay8910_write_reg(psg,i,0);

	if (psg->chip_type & 0x20)
		psg->IsDisabled = 0x01;	// YM2203/2608/2610 SSG optimization
}

void ay8910_set_clock_ym(void *chip, UINT32 clock)
{
	ay8910_context *psg = (ay8910_context *)chip;
	
	psg->clock = clock;
	if (psg->SmpRateFunc != NULL)
		psg->SmpRateFunc(psg->SmpRateData, ay8910_get_sample_rate(psg));
	
	return;
}

UINT32 ay8910_get_sample_rate(void *chip)
{
	ay8910_context *psg = (ay8910_context *)chip;
	UINT32 master_clock = psg->clock;
	
	if ((psg->chip_type & 0xF0) > 0x00)
	{
		// YM2149 master clock divider
		if (psg->chip_flags & YM2149_PIN26_LOW)
			master_clock /= 2;
	}
	/* The envelope is pacing twice as fast for the YM2149 as for the AY-3-8910,    */
	/* This handled by the step parameter. Consequently we use a divider of 8 here. */
	return master_clock / 8;
}

void ay8910_write_ym(void *chip, UINT8 addr, UINT8 data)
{
	ay8910_context *psg = (ay8910_context *)chip;

	if (addr & 1)
	{	/* Data port */
		UINT8 r = psg->register_latch;
		ay8910_write_reg(psg,r,data);
	}
	else
	{	/* Register port */
		psg->register_latch = data & 0x0f;
	}
}

UINT8 ay8910_read_ym(void *chip, UINT8 addr)
{
	ay8910_context *psg = (ay8910_context *)chip;
	UINT8 r = psg->register_latch;

	if (r > 15) return 0;

	switch (r)
	{
	case AY_PORTA:
		//if ((psg->regs[AY_ENABLE] & 0x40) != 0)
		//	logerror("warning: read from 8910 '%s' Port A set as output\n",psg->device->tag);
		/*
		   even if the port is set as output, we still need to return the external
		   data. Some games, like kidniki, need this to work.
		 */
		//psg->regs[AY_PORTA] = devcb_call_read8(&psg->portAread, 0);
		break;
	case AY_PORTB:
		//if ((psg->regs[AY_ENABLE] & 0x80) != 0)
		//	logerror("warning: read from 8910 '%s' Port B set as output\n",psg->device->tag);
		//psg->regs[AY_PORTB] = devcb_call_read8(&psg->portBread, 0);
		break;
	}

	/* Depending on chip type, unused bits in registers may or may not be accessible.
	Untested chips are assumed to regard them as 'ram'
	Tested and confirmed on hardware:
	- AY-3-8910: inaccessible bits (see masks below) read back as 0
	- YM2149: no anomaly
	*/
	if ((psg->chip_type & 0xF0) > 0x00) {
		const UINT8 mask[0x10]={
			0xff,0x0f,0xff,0x0f,0xff,0x0f,0x1f,0xff,0x1f,0x1f,0x1f,0xff,0xff,0x0f,0xff,0xff
		};

		return psg->regs[r] & mask[r];
	}
	else return psg->regs[r];
}

void ay8910_set_mute_mask_ym(void *chip, UINT32 MuteMask)
{
	ay8910_context *psg = (ay8910_context *)chip;
	UINT8 CurChn;
	
	for (CurChn = 0; CurChn < NUM_CHANNELS; CurChn ++)
		psg->MuteMsk[CurChn] = (MuteMask & (1 << CurChn)) ? 0 : ~0;
	
	return;
}

void ay8910_set_srchg_cb_ym(void *chip, DEVCB_SRATE_CHG CallbackFunc, void* DataPtr)
{
	ay8910_context *info = (ay8910_context *)chip;
	
	// set Sample Rate Change Callback routine
	info->SmpRateFunc = CallbackFunc;
	info->SmpRateData = DataPtr;
	
	return;
}