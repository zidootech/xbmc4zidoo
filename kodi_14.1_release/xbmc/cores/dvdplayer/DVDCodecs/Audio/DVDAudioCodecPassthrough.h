#pragma once

/*
 *      Copyright (C) 2010-2013 Team XBMC
 *      http://xbmc.org
 *
 *  This Program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  This Program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with XBMC; see the file COPYING.  If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 */

#include <list>

#include "system.h"
#include "DVDAudioCodec.h"
#include "cores/AudioEngine/Utils/AEAudioFormat.h"
#include "cores/AudioEngine/Utils/AEStreamInfo.h"
#include "cores/AudioEngine/Utils/AEBitstreamPacker.h"

// MSTAR PATCH BEGIN
#include <queue>
#include <vector>
#include <boost/shared_ptr.hpp>
#include "DVDStreamInfo.h"
#include "android/jni/ByteBuffer.h"
#include "android/jni/MediaCodec.h"
#include "android/jni/MediaCrypto.h"
#include "android/jni/MediaFormat.h"
#include "android/jni/MediaCodecList.h"
#include "android/jni/MediaCodecInfo.h"
#include "threads/Thread.h"

class CJNIMediaCodec;
class CJNIByteBuffer;

typedef struct audio_packet {
  uint8_t      *pData;
  unsigned int iSize;
  double       dts;
  double       duration;
} audio_packet;

// MSTAR PATCH END

class CDVDAudioCodecPassthrough : public CDVDAudioCodec
{
public:
  CDVDAudioCodecPassthrough();
  virtual ~CDVDAudioCodecPassthrough();

  virtual bool Open(CDVDStreamInfo &hints, CDVDCodecOptions &options);
  virtual void Dispose();
  virtual int  Decode(uint8_t* pData, int iSize) {return 0;}
// MSTAR PATCH BEGIN
  virtual int  Decode(uint8_t* pData, int iSize, double dts, double duration);
// MSTAR PATCH END
  virtual int  GetData(uint8_t** dst);
  virtual void Reset();
  virtual int  GetChannels               ();
  virtual int  GetEncodedChannels        ();
  virtual CAEChannelInfo GetChannelMap       ();
  virtual int  GetSampleRate             ();
  virtual int  GetEncodedSampleRate      ();
  virtual enum AEDataFormat GetDataFormat();
  virtual bool NeedPassthrough           () { return true;          }
  virtual const char* GetName            () { return "passthrough"; }
  virtual int  GetBufferSize();

// MSTAR PATCH BEGIN
  class DecodeThread : public CThread {
  public:
      DecodeThread(CDVDAudioCodecPassthrough *codec):CThread("CDVDAudioCodecPassthrough"),pCodec(codec){}
      ~DecodeThread() {StopThread();}
	  
  protected:
	  virtual void OnStartup();
      virtual void OnExit();
      virtual void Process();

  private:
      CDVDAudioCodecPassthrough *pCodec;
  };
// MSTAR PATCH END
private:
// MSTAR PATCH BEGIN
  bool            ConfigureMediaCodec(void);
  CDVDStreamInfo  m_hints;
  std::string     m_mime;
  boost::shared_ptr<CJNIMediaCodec> m_codec;
  bool            m_opened;
  std::vector<CJNIByteBuffer> m_input;
  const char     *m_formatname;
  DecodeThread   *m_decodeThread;
  std::queue<audio_packet> m_packets;
// MSTAR PATCH END
  
  CAEStreamInfo      m_info;
  CAEBitstreamPacker m_packer;
  uint8_t*           m_buffer;
  unsigned int       m_bufferSize;
};

