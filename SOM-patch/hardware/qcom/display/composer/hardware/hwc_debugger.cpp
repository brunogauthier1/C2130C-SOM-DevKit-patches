/*
* Copyright (c) 2014 - 2020, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following
*       disclaimer in the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of The Linux Foundation nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
* ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Changes from Qualcomm Innovation Center are provided under the following license:
*
* Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted (subject to the limitations in the
* disclaimer below) provided that the following conditions are met:
*
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*
*    * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
* GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
* HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
* IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <utils/constants.h>
#include <cutils/properties.h>
#include <display_properties.h>

#include "hwc_debugger.h"

#include <chrono>
#include <iomanip>
#include <fstream>
#include <sstream>

#define LOG_BUF_SIZE 1024

namespace sdm {

HWCDebugHandler HWCDebugHandler::debug_handler_;

HWCDebugHandler::HWCDebugHandler() {
  DebugHandler::Set(HWCDebugHandler::Get());
}

void HWCDebugHandler::DebugAll(bool enable, int verbose_level) {

  // ACHEUL
  enable = true;

  if (enable) {
    debug_handler_.log_mask_ = 0x7FFFFFFF;
    if (verbose_level) {
      // Enable verbose scalar logs only when explicitly enabled
      debug_handler_.log_mask_[kTagScalar] = 0;
    }
    debug_handler_.verbose_level_ = verbose_level;
  } else {
    debug_handler_.log_mask_ = 0x1;   // kTagNone should always be printed.
    debug_handler_.verbose_level_ = 0;
  }

  DebugHandler::SetLogMask(debug_handler_.log_mask_);
}

void HWCDebugHandler::DebugResources(bool enable, int verbose_level) {

  // ACHEUL
  enable = true;


  if (enable) {
    debug_handler_.log_mask_[kTagResources] = 1;
    debug_handler_.verbose_level_ = verbose_level;
  } else {
    debug_handler_.log_mask_[kTagResources] = 0;
    debug_handler_.verbose_level_ = 0;
  }

  DebugHandler::SetLogMask(debug_handler_.log_mask_);
}

void HWCDebugHandler::DebugStrategy(bool enable, int verbose_level) {


  // ACHEUL
  enable = true;



  if (enable) {
    debug_handler_.log_mask_[kTagStrategy] = 1;
    debug_handler_.verbose_level_ = verbose_level;
  } else {
    debug_handler_.log_mask_[kTagStrategy] = 0;
    debug_handler_.verbose_level_ = 0;
  }

  DebugHandler::SetLogMask(debug_handler_.log_mask_);
}

void HWCDebugHandler::DebugCompManager(bool enable, int verbose_level) {

  // ACHEUL
  enable = true;


  if (enable) {
    debug_handler_.log_mask_[kTagCompManager] = 1;
    debug_handler_.verbose_level_ = verbose_level;
  } else {
    debug_handler_.log_mask_[kTagCompManager] = 0;
    debug_handler_.verbose_level_ = 0;
  }

  DebugHandler::SetLogMask(debug_handler_.log_mask_);
}

void HWCDebugHandler::DebugDriverConfig(bool enable, int verbose_level) {

  // ACHEUL
  enable = true;

  if (enable) {
    debug_handler_.log_mask_[kTagDriverConfig] = 1;
    debug_handler_.verbose_level_ = verbose_level;
  } else {
    debug_handler_.log_mask_[kTagDriverConfig] = 0;
    debug_handler_.verbose_level_ = 0;
  }

  DebugHandler::SetLogMask(debug_handler_.log_mask_);
}

void HWCDebugHandler::DebugRotator(bool enable, int verbose_level) {
  // ACHEUL
  enable = true;

  if (enable) {
    debug_handler_.log_mask_[kTagRotator] = 1;
    debug_handler_.verbose_level_ = verbose_level;
  } else {
    debug_handler_.log_mask_[kTagRotator] = 0;
    debug_handler_.verbose_level_ = 0;
  }

  DebugHandler::SetLogMask(debug_handler_.log_mask_);
}

void HWCDebugHandler::DebugScalar(bool enable, int verbose_level) {


  // ACHEUL
  enable = true;

  if (enable) {
    debug_handler_.log_mask_[kTagScalar] = 1;
    debug_handler_.verbose_level_ = verbose_level;
  } else {
    debug_handler_.log_mask_[kTagScalar] = 0;
    debug_handler_.verbose_level_ = 0;
  }

  DebugHandler::SetLogMask(debug_handler_.log_mask_);
}

void HWCDebugHandler::DebugQdcm(bool enable, int verbose_level) {


  // ACHEUL
  enable = true;

  if (enable) {
    debug_handler_.log_mask_[kTagQDCM] = 1;
    debug_handler_.verbose_level_ = verbose_level;
  } else {
    debug_handler_.log_mask_[kTagQDCM] = 0;
    debug_handler_.verbose_level_ = 0;
  }

  DebugHandler::SetLogMask(debug_handler_.log_mask_);
}

void HWCDebugHandler::DebugClient(bool enable, int verbose_level) {


  // ACHEUL
  enable = true;

  if (enable) {
    debug_handler_.log_mask_[kTagClient] = 1;
    debug_handler_.verbose_level_ = verbose_level;
  } else {
    debug_handler_.log_mask_[kTagClient] = 0;
    debug_handler_.verbose_level_ = 0;
  }

  DebugHandler::SetLogMask(debug_handler_.log_mask_);
}

void HWCDebugHandler::DebugDisplay(bool enable, int verbose_level) {

  // ACHEUL
  enable = true;

  if (enable) {
    debug_handler_.log_mask_[kTagDisplay] = 1;
    debug_handler_.verbose_level_ = verbose_level;
  } else {
    debug_handler_.log_mask_[kTagDisplay] = 0;
    debug_handler_.verbose_level_ = 0;
  }

  DebugHandler::SetLogMask(debug_handler_.log_mask_);
}

void HWCDebugHandler::DebugQos(bool enable, int verbose_level) {
  if (enable) {
    debug_handler_.log_mask_[kTagQOSClient] = 1;
    debug_handler_.log_mask_[kTagQOSImpl] = 1;
    debug_handler_.verbose_level_ = verbose_level;
  } else {
    debug_handler_.log_mask_[kTagQOSClient] = 0;
    debug_handler_.log_mask_[kTagQOSImpl] = 0;
    debug_handler_.verbose_level_ = 0;
  }

  DebugHandler::SetLogMask(debug_handler_.log_mask_);
}

void HWCDebugHandler::LogPrintWrapper(int prio, const char* tag,
                                      const char *fmt, va_list ap) {
  debug_handler_.GetProperty(LOG_SINK_PROP, &(debug_handler_.log_sink_));

  if (debug_handler_.log_sink_ & kLogAndroidSink) {
    __android_log_vprint(prio, tag, fmt, ap);
  }

  if (debug_handler_.log_sink_ & kLogFileSink) {
    if (debug_handler_.boot_time.size() == 0) {
      auto now = std::chrono::system_clock::now();
      auto in_time_t = std::chrono::system_clock::to_time_t(now);
      std::stringstream datetime;
      datetime << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
      debug_handler_.boot_time = datetime.str();
    }
    std::ofstream dbg_file(std::string(DumpDir()) + "/sdm_log_" +
                           debug_handler_.boot_time + ".txt", std::ios_base::app);
    if (!dbg_file) {
      return;
    }
    char buf[LOG_BUF_SIZE];
    vsnprintf(buf, LOG_BUF_SIZE, fmt, ap);
    dbg_file << buf << std::endl;
    // TODO: Optimize file handling. Prevent multiple file open and close calls.
    dbg_file.close();
  }
}

void HWCDebugHandler::Error(const char *fmt, ...) {
  va_list list;
  va_start(list, fmt);
  debug_handler_.LogPrintWrapper(ANDROID_LOG_ERROR, LOG_TAG, fmt, list);
}

void HWCDebugHandler::Warning(const char *fmt, ...) {
  va_list list;
  va_start(list, fmt);
  debug_handler_.LogPrintWrapper(ANDROID_LOG_WARN, LOG_TAG, fmt, list);
}

void HWCDebugHandler::Info(const char *fmt, ...) {
  va_list list;
  va_start(list, fmt);
  debug_handler_.LogPrintWrapper(ANDROID_LOG_INFO, LOG_TAG, fmt, list);
}

void HWCDebugHandler::Debug(const char *fmt, ...) {
  va_list list;
  va_start(list, fmt);
  debug_handler_.LogPrintWrapper(ANDROID_LOG_DEBUG, LOG_TAG, fmt, list);
}

void HWCDebugHandler::Verbose(const char *fmt, ...) {
  // ACHEUL
  // if (debug_handler_.verbose_level_) {
  if (1) {
    va_list list;
    va_start(list, fmt);
    debug_handler_.LogPrintWrapper(ANDROID_LOG_VERBOSE, LOG_TAG, fmt, list);
  }
}

void HWCDebugHandler::BeginTrace(const char *class_name, const char *function_name,
                                 const char *custom_string) {

  // ACHEUL
  //if (atrace_is_tag_enabled(ATRACE_TAG)) {
  if (1) {
    char name[PATH_MAX] = {0};
    snprintf(name, sizeof(name), "%s::%s::%s", class_name, function_name, custom_string);
    atrace_begin(ATRACE_TAG, name);
  }
}

void HWCDebugHandler::EndTrace() {
  atrace_end(ATRACE_TAG);
}

int  HWCDebugHandler::GetIdleTimeoutMs() {
  int value = IDLE_TIMEOUT_DEFAULT_MS;
  debug_handler_.GetProperty(IDLE_TIME_PROP, &value);

  return value;
}

int HWCDebugHandler::GetProperty(const char *property_name, int *value) {
  char property[PROPERTY_VALUE_MAX];

  if (property_get(property_name, property, NULL) > 0) {
    *value = atoi(property);
    return kErrorNone;
  }

  return kErrorNotSupported;
}

int HWCDebugHandler::GetProperty(const char *property_name, char *value) {
  if (property_get(property_name, value, NULL) > 0) {
    return kErrorNone;
  }

  return kErrorNotSupported;
}

}  // namespace sdm

