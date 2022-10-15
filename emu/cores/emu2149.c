/**
 * emu2149 v1.42
 * https://github.com/digital-sound-antiques/emu2149
 * Copyright (C) 2001-2022 Mitsutaka Okazaki
 *
 * This source refers to the following documents. The author would like to thank all the authors who have
 * contributed to the writing of them.
 * - psg.vhd        -- 2000 written by Kazuhiro Tsujikawa.
 * - s_fme7.c       -- 1999,2000 written by Mamiya (NEZplug).
 * - ay8910.c       -- 1998-2001 Author unknown (MAME).
 * - MSX-Datapack   -- 1991 ASCII Corp.
 * - AY-3-8910 data sheet
 * 
 * Further modifications:
 * - voltbl fix by rainwarrior
 * - linear resampling (ported from old EMU2413) by Valley Bell
 * - per-channel panning, optional YM2149 clock divider by Valley Bell
 */
#include <stdlib.h>
#include <string.h>

#include "../../stdtype.h"
#include "../snddef.h"
#include "../EmuStructs.h"
#include "../EmuCores.h"
#include "../EmuHelper.h"
#include "ayintf.h"
#include "emu2149.h"
#include "emu2149_private.h"
#include "../panning.h"


static DEVDEF_RWFUNC devFunc[] =
{
	{RWF_REGISTER | RWF_WRITE, DEVRW_A8D8, 0, EPSG_writeIO},
	{RWF_REGISTER | RWF_READ, DEVRW_A8D8, 0, EPSG_readIO},
	{RWF_REGISTER | RWF_QUICKWRITE, DEVRW_A8D8, 0, EPSG_writeReg},
	{RWF_REGISTER | RWF_QUICKREAD, DEVRW_A8D8, 0, EPSG_readReg},
	{RWF_REGISTER | RWF_WRITE, DEVRW_ALL, 0x5354, EPSG_setStereoMask},	// 0x5354 = 'ST' (stereo)
	{RWF_CLOCK | RWF_WRITE, DEVRW_VALUE, 0, EPSG_setClock},
	{RWF_SRATE | RWF_WRITE, DEVRW_VALUE, 0, EPSG_setRate},
	{RWF_CHN_MUTE | RWF_WRITE, DEVRW_ALL, 0, EPSG_setMuteMask},
	{RWF_CHN_PAN | RWF_WRITE, DEVRW_ALL, 0, ay8910_emu_pan},
	{0x00, 0x00, 0, NULL}
};
DEV_DEF devDef_YM2149_Emu =
{
	"YM2149", "EMU2149", FCC_EMU_,
	
	(DEVFUNC_START)device_start_ay8910_emu,
	(DEVFUNC_CTRL)EPSG_delete,
	(DEVFUNC_CTRL)EPSG_reset,
	(DEVFUNC_UPDATE)EPSG_calc_stereo,
	
	ay8910_emu_set_options,
	(DEVFUNC_OPTMASK)EPSG_setMuteMask,
	ay8910_emu_pan,
	NULL,	// SetSampleRateChangeCallback
	NULL,	// SetLoggingCallback
	NULL,	// LinkDevice
	
	devFunc,	// rwFuncs
};


static const uint32_t voltbl[2][32] = {
  /* YM2149 - 32 steps */
  {0x00, 0x01, 0x01, 0x02, 0x02, 0x03, 0x03, 0x04, 0x05, 0x06, 0x07, 0x09,
   0x0B, 0x0D, 0x0F, 0x12,
   0x16, 0x1A, 0x1F, 0x25, 0x2D, 0x35, 0x3F, 0x4C, 0x5A, 0x6A, 0x7F, 0x97,
   0xB4, 0xD6, 0xFF, 0xFF},
  /* AY-3-8910 - 16 steps */
  {0x00, 0x00, 0x03, 0x03, 0x04, 0x04, 0x06, 0x06, 0x09, 0x09, 0x0D, 0x0D,
   0x12, 0x12, 0x1D, 0x1D,
   0x22, 0x22, 0x37, 0x37, 0x4D, 0x4D, 0x62, 0x62, 0x82, 0x82, 0xA6, 0xA6,
   0xD0, 0xD0, 0xFF, 0xFF}
};

static const uint8_t regmsk[16] = {
    0xff, 0x0f, 0xff, 0x0f, 0xff, 0x0f, 0x1f, 0x3f,
    0x1f, 0x1f, 0x1f, 0xff, 0xff, 0x0f, 0xff, 0xff
};

#define GETA_BITS 24

static void
internal_refresh (EPSG * psg)
{
  uint32_t f_master = psg->clk;

  if (psg->clk_div)
    f_master /= 2;

  if (psg->quality)
  {
    psg->base_incr = 1 << GETA_BITS;
    psg->realstep = f_master;
    psg->psgstep = psg->rate * 8;
    psg->psgtime = 0;
  }
  else
  {
    psg->base_incr = (uint32_t)((double)f_master * (1 << GETA_BITS) / 8 / psg->rate);
    psg->freq_limit = 0;
  }
}

void
EPSG_setClock(EPSG * psg, UINT32 clock)
{
  if (psg->clk != clock) {
    psg->clk = clock;
    internal_refresh(psg);
  }
}

void 
EPSG_setClockDivider(EPSG *psg, UINT8 enable)
{
  if (psg->clk_div != enable) {
    psg->clk_div = enable;  
    internal_refresh (psg);
  }
}

void
EPSG_setRate (EPSG * psg, UINT32 rate)
{
  uint32_t r = rate ? rate : 44100;
  if (psg->rate != r) {
    psg->rate = r;
    internal_refresh(psg);
  }
}

void
EPSG_setQuality (EPSG * psg, UINT8 q)
{
  if (psg->quality != q) {
    psg->quality = q;
    internal_refresh(psg);
  }
}

static UINT8 device_start_ay8910_emu(const AY8910_CFG* cfg, DEV_INFO* retDevInf)
{
	EPSG* chip;
	UINT8 isYM;
	UINT8 flags;
	UINT32 clock;
	UINT32 rate;
	
	clock = cfg->_genCfg.clock;
	isYM = ((cfg->chipType & 0xF0) > 0x00);
	flags = cfg->chipFlags;
	if (! isYM)
		flags &= ~YM2149_PIN26_LOW;
	
	if (flags & YM2149_PIN26_LOW)
		rate = clock / 2 / 8;
	else
		rate = clock / 8;
	SRATE_CUSTOM_HIGHEST(cfg->_genCfg.srMode, rate, cfg->_genCfg.smplRate);
	
	chip = EPSG_new(clock, rate);
	if (chip == NULL)
		return 0xFF;
	EPSG_setQuality(chip, 0);	// disable internal sample rate converter
	EPSG_setVolumeMode(chip, isYM ? 1 : 2);
	EPSG_setFlags(chip, flags);
	
	chip->_devData.chipInf = chip;
	INIT_DEVINF(retDevInf, &chip->_devData, rate, &devDef_YM2149_Emu);
	return 0x00;
}

EPSG *
EPSG_new (UINT32 clock, UINT32 rate)
{
  EPSG *psg;
  uint8_t i;

  psg = (EPSG *) calloc (1, sizeof (EPSG));
  if (psg == NULL)
    return NULL;

  EPSG_setVolumeMode (psg, 0);
  psg->clk = clock;
  psg->clk_div = 0;
  psg->rate = rate ? rate : 44100;
  psg->chp_flags = 0x00;
  psg->quality = 0;
  internal_refresh(psg);

  for (i = 0; i < 3; i++)
  {
    psg->stereo_mask[i] = 0x03;
    Panning_Centre(psg->pan[i]);
  }
  psg->pcm3ch = 0;
  psg->pcm3ch_detect = 0;

  EPSG_setMask(psg, 0x00);

  return psg;
}

void
EPSG_setFlags (EPSG * psg, UINT8 flags)
{
  psg->chp_flags = flags;

  // in case of changed clock divider pin
  EPSG_setClockDivider(psg, psg->chp_flags & YM2149_PIN26_LOW ? 1 : 0);

  if (psg->chp_flags & AY8910_ZX_STEREO)
  {
    // ABC Stereo
    psg->stereo_mask[0] = 0x01;
    psg->stereo_mask[1] = 0x03;
    psg->stereo_mask[2] = 0x02;
  }
  else
  {
    psg->stereo_mask[0] = 0x03;
    psg->stereo_mask[1] = 0x03;
    psg->stereo_mask[2] = 0x03;
  }

  return;
}

void
EPSG_setVolumeMode (EPSG * psg, int type)
{
  switch (type)
  {
  case 1:
    psg->voltbl = voltbl[0]; /* YM2149 */
    break;
  case 2:
    psg->voltbl = voltbl[1]; /* AY-3-8910 */
    break;
  default:
    psg->voltbl = voltbl[0]; /* fallback: YM2149 */
    break;
  }
}

uint32_t
EPSG_setMask (EPSG *psg, uint32_t mask)
{
  uint32_t ret = 0;
  if(psg)
  {
    ret = psg->mask;
    psg->mask = mask;
  }
  return ret;
}

uint32_t
EPSG_toggleMask (EPSG *psg, uint32_t mask)
{
  uint32_t ret = 0;
  if(psg)
  {
    ret = psg->mask;
    psg->mask ^= mask;
  }
  return ret;
}

void
EPSG_setMuteMask (EPSG *psg, UINT32 mask)
{
  psg->mask = mask;
  return;
}

void
EPSG_setStereoMask (EPSG *psg, UINT32 mask)
{
  if(psg)
  {
    uint8_t i;
    for (i = 0; i < 3; i++)
    {
      psg->stereo_mask[i] = mask & 0x03;
      mask >>= 2;
    }
  }  
}

void
EPSG_reset (EPSG * psg)
{
  int i;

  psg->base_count = 0;

  for (i = 0; i < 3; i++)
  {
    psg->count[i] = 0;
    psg->freq[i] = 0;
    psg->edge[i] = 0;
    psg->volume[i] = 0;
    psg->ch_out[i] = 0;
  }

  //psg->mask = 0;

  for (i = 0; i < 16; i++)
    psg->reg[i] = 0;
  psg->adr = 0;

  psg->noise_seed = 0xffff;
  psg->noise_scaler = 0;
  psg->noise_count = 0;
  psg->noise_freq = 0;

  psg->env_ptr = 0;
  psg->env_freq = 0;
  psg->env_count = 0;
  psg->env_pause = 1;

  //psg->out = 0;

}

void
EPSG_delete (EPSG * psg)
{
  free (psg);
}

UINT8
EPSG_readIO (EPSG * psg, UINT8 adr)
{
  // note: "adr" parameter is not used
  return (UINT8) (psg->reg[psg->adr]);
}

UINT8
EPSG_readReg (EPSG * psg, UINT8 reg)
{
  return (UINT8) (psg->reg[reg & 0x1f]);
}

void
EPSG_writeIO (EPSG * psg, UINT8 adr, UINT8 val)
{
  if (adr & 1)
    EPSG_writeReg (psg, psg->adr, val);
  else
    psg->adr = val & 0x1f;
}

INLINE void
update_output (EPSG * psg)
{

  int i, noise;
  uint8_t incr;

  psg->base_count += psg->base_incr;
  incr = (psg->base_count >> GETA_BITS);
  psg->base_count &= (1 << GETA_BITS) - 1;

  /* Envelope */
  psg->env_count += incr;
  
  if (psg->env_count >= psg->env_freq)
  {
    if (!psg->env_pause)
    {
      if(psg->env_face)
        psg->env_ptr = (psg->env_ptr + 1) & 0x3f ; 
      else
        psg->env_ptr = (psg->env_ptr + 0x3f) & 0x3f;
    }

    if (psg->env_ptr & 0x20) /* if carry or borrow */
    {
      if (psg->env_continue)
      {
        if (psg->env_alternate^psg->env_hold) psg->env_face ^= 1;
        if (psg->env_hold) psg->env_pause = 1;
        psg->env_ptr = psg->env_face ? 0 : 0x1f;       
      }
      else
      {
        psg->env_pause = 1;
        psg->env_ptr = 0;
      }
    }

    if (psg->env_freq >= incr) 
    psg->env_count -= psg->env_freq;
    else
      psg->env_count = 0;
  }

  /* Noise */
  psg->noise_count += incr;
  if (psg->noise_count >= psg->noise_freq)
  {
    psg->noise_scaler ^= 1;
    if (psg->noise_scaler) 
    { 
      if (psg->noise_seed & 1)
        psg->noise_seed ^= 0x24000;
      psg->noise_seed >>= 1;
    }
    
    if (psg->noise_freq >= incr)
      psg->noise_count -= psg->noise_freq;
    else
      psg->noise_count = 0;
  }
  noise = psg->noise_seed & 1;

  /* Tone */
  for (i = 0; i < 3; i++)
  {
    psg->count[i] += incr;
    if (psg->count[i] >= psg->freq[i])
    {
      psg->edge[i] = !psg->edge[i];

      if (psg->freq[i] >= incr) 
        psg->count[i] -= psg->freq[i];
      else
        psg->count[i] = 0;
    }

    if (0 < psg->freq_limit && psg->freq[i] <= psg->freq_limit && psg->nmask[i])
    {
      /* Mute the channel if the pitch is higher than the Nyquist frequency at the current sample rate, 
       * to prevent aliased or broken tones from being generated. Of course, this logic doesn't exist 
       * on the actual chip, but practically all tones higher than the Nyquist frequency are usually 
       * removed by a low-pass circuit somewhere, so we here halt the output. */
      continue;
    }

    if (psg->mask & EPSG_MASK_CH(i)) 
    {
      psg->ch_out[i] = 0;
      continue;
    }

    if ((psg->tmask[i]||psg->edge[i]) && (psg->nmask[i]||noise))
    {
      if (!(psg->volume[i] & 32))
        psg->ch_out[i] = (psg->voltbl[psg->volume[i] & 31] << 5);
      else
        psg->ch_out[i] = (psg->voltbl[psg->env_ptr] << 5);
    }
    else 
    {
      psg->ch_out[i] = 0;
    }
  }

}

#if 0
INLINE int16_t
mix_output(EPSG *psg) {
  return (int16_t)(psg->ch_out[0] + psg->ch_out[1] + psg->ch_out[2]);
}

int16_t
EPSG_calc (EPSG * psg)
{
  if (!psg->quality) {
    update_output(psg);
    return mix_output(psg);
  }

  /* Simple rate converter */
  while (psg->realstep > psg->psgtime)
  {
    psg->psgtime += psg->psgstep;
    update_output(psg);
  }
  psg->psgtime = psg->psgtime - psg->realstep;

  return mix_output(psg);
}
#endif

INLINE void
mix_output_stereo(EPSG *psg, int32_t out[2])
{
  int i;

  out[0] = out[1] = 0;
  for (i = 0; i < 3; i++)
  {
    if (! (~psg->stereo_mask[i] & 0x03) && ! psg->pcm3ch)
    {
      // mono channel
      out[0] += APPLY_PANNING_S(psg->ch_out[i], psg->pan[i][0]);
      out[1] += APPLY_PANNING_S(psg->ch_out[i], psg->pan[i][1]);
    }
    else
    {
      // hard-panned to L or R
      if (psg->stereo_mask[i] & 0x01)
        out[0] += psg->ch_out[i];
      if (psg->stereo_mask[i] & 0x02)
        out[1] += psg->ch_out[i];
    }
  }

  return;
}

void
EPSG_calc_stereo (EPSG * psg, UINT32 samples, DEV_SMPL **out)
{
  DEV_SMPL *bufMO = out[0];
  DEV_SMPL *bufRO = out[1];
  int32_t buffers[2];
  UINT32 i;

  if (!psg->quality)
  {
    for (i = 0; i < samples; i ++)
    {
      update_output(psg);
      mix_output_stereo(psg, buffers);
      bufMO[i] = buffers[0];
      bufRO[i] = buffers[1];
    }
  }
  else
  {
    for (i = 0; i < samples; i ++)
    {
      while (psg->realstep > psg->psgtime)
      { 
        psg->psgtime += psg->psgstep;
        psg->sprev[0] = psg->snext[0];
        psg->sprev[1] = psg->snext[1];
        update_output(psg);
        mix_output_stereo(psg, psg->snext);
      }

      psg->psgtime -= psg->realstep;
      bufMO[i] = (DEV_SMPL) (((double) psg->snext[0] * (psg->psgstep - psg->psgtime)
                            + (double) psg->sprev[0] * psg->psgtime) / psg->psgstep);
      bufRO[i] = (DEV_SMPL) (((double) psg->snext[1] * (psg->psgstep - psg->psgtime)
                            + (double) psg->sprev[1] * psg->psgtime) / psg->psgstep);
    }
  }
}

static void EPSG_Is3ChPcm(EPSG* psg)
{
  uint8_t tone_mask = psg->tmask[0] | psg->tmask[1] | psg->tmask[2];	// 1 = disabled, 0 = enabled
  uint8_t noise_mask = psg->nmask[0] | psg->nmask[1] | psg->nmask[2];	// 1 = disabled, 0 = enabled

  psg->pcm3ch = 0x00;
  if (!psg->pcm3ch_detect)
    return; // 3-channel PCM detection disabled
  if (~noise_mask & 0x38)
    return; // at least one noise channel is enabled - no 3-channel PCM possible

  // bit 0 - tone channels disabled
  psg->pcm3ch |= !(~tone_mask & 0x07) << 0;
  // bit 1 - all channels have frequency <= 1
  psg->pcm3ch |= (psg->freq[0] <= 1 && psg->freq[1] <= 1 && psg->freq[2] <= 1) << 1;
  return;
}

void
EPSG_writeReg (EPSG * psg, UINT8 reg, UINT8 val)
{
  int c;

  if (reg > 15) return;

  val &= regmsk[reg];

  psg->reg[reg] = (uint8_t) val;

  switch (reg)
  {
  case 0:
  case 2:
  case 4:
  case 1:
  case 3:
  case 5:
    c = reg >> 1;
    psg->freq[c] = ((psg->reg[c * 2 + 1] & 15) << 8) + psg->reg[c * 2];
    EPSG_Is3ChPcm(psg);
    break;

  case 6:
    psg->noise_freq = val & 31;
    break;

  case 7:
    psg->tmask[0] = (val & 1);
    psg->tmask[1] = (val & 2);
    psg->tmask[2] = (val & 4);
    psg->nmask[0] = (val & 8);
    psg->nmask[1] = (val & 16);
    psg->nmask[2] = (val & 32);
    EPSG_Is3ChPcm(psg);
    break;

  case 8:
  case 9:
  case 10:
    psg->volume[reg - 8] = val << 1;
    break;

  case 11:
  case 12:
    psg->env_freq = (psg->reg[12] << 8) + psg->reg[11];
    break;

  case 13:
    psg->env_continue = (val >> 3) & 1;
    psg->env_attack = (val >> 2) & 1;
    psg->env_alternate = (val >> 1) & 1;
    psg->env_hold = val & 1;
    psg->env_face = psg->env_attack;
    psg->env_pause = 0;
    psg->env_ptr = psg->env_face ? 0 : 0x1f;
    break;

  case 14:
  case 15:
  default:
    break;
  }

  return;
}

void EPSG_set_pan (EPSG * psg, uint8_t ch, int16_t pan)
{
  if (ch >= 3)
    return;
  
  Panning_Calculate( psg->pan[ch], pan );
}

static void ay8910_emu_set_options(void *chip, UINT32 Flags)
{
  EPSG* psg = (EPSG*)chip;

  psg->pcm3ch_detect = (Flags >> 0) & 0x01;

  return;
}

static void ay8910_emu_pan(void* chip, const INT16* PanVals)
{
  UINT8 curChn;
  
  for (curChn = 0; curChn < 3; curChn ++)
    EPSG_set_pan((EPSG*)chip, curChn, PanVals[curChn]);
  
  return;
}
