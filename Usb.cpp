/*
 * Copyright (C) 2018 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#define LOG_TAG "android.hardware.usb@1.1-service.stm32mp1"

#include <android-base/logging.h>
#include <assert.h>
#include <chrono>
#include <dirent.h>
#include <pthread.h>
#include <regex>
#include <stdio.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>

#include <cutils/uevent.h>
#include <sys/epoll.h>
#include <utils/Errors.h>
#include <utils/StrongPointer.h>

#include "Usb.h"

namespace android {
namespace hardware {
namespace usb {
namespace V1_1 {
namespace implementation {

// Set by the signal handler to destroy the thread
volatile bool destroyThread;

Usb::Usb()
  : mLock(PTHREAD_MUTEX_INITIALIZER) {
  pthread_condattr_t attr;
  if (pthread_condattr_init(&attr)) {
    ALOGE("pthread_condattr_init failed: %s", strerror(errno));
    abort();
  }
  if (pthread_condattr_setclock(&attr, CLOCK_MONOTONIC)) {
    ALOGE("pthread_condattr_setclock failed: %s", strerror(errno));
    abort();
  }
  if (pthread_cond_init(&mPartnerCV, &attr))  {
    ALOGE("pthread_cond_init failed: %s", strerror(errno));
    abort();
  }
  if (pthread_condattr_destroy(&attr)) {
    ALOGE("pthread_condattr_destroy failed: %s", strerror(errno));
    abort();
  }
}

Return<void> Usb::switchRole(const hidl_string &/*portName*/,
                             const V1_0::PortRole &/*newRole*/) {
  ALOGE("Usb switch role not supported");
  return Void();
}

/*
 * Reuse the same method for both V1_0 and V1_1 callback objects.
 * The caller of this method would reconstruct the V1_0::PortStatus
 * object if required.
 */
Status getPortStatusHelper(hidl_vec<PortStatus_1_1> *currentPortStatus_1_1,
    bool /*V1_0*/) {
  std::vector<std::string> names;

  names.push_back("port0");
  currentPortStatus_1_1->resize(names.size());

  for (std::vector<std::string>::size_type i = 0; i < names.size(); ++i) {
    ALOGI("getPortStatus %s", names[i].c_str());

    (*currentPortStatus_1_1)[i].status.portName = names[i];
    (*currentPortStatus_1_1)[i].status.currentPowerRole = PortPowerRole::NONE;
    (*currentPortStatus_1_1)[i].status.currentDataRole = PortDataRole::DEVICE;
    (*currentPortStatus_1_1)[i].currentMode = PortMode_1_1::UFP;
    (*currentPortStatus_1_1)[i].status.currentMode = V1_0::PortMode::UFP;
    (*currentPortStatus_1_1)[i].status.canChangeMode = false;
    (*currentPortStatus_1_1)[i].status.canChangeDataRole = false;
    (*currentPortStatus_1_1)[i].status.canChangePowerRole = false;

    ALOGI("canChangeMode:%d canChagedata:%d canChangePower:%d",
          (*currentPortStatus_1_1)[i].status.canChangeMode,
          (*currentPortStatus_1_1)[i].status.canChangeDataRole,
          (*currentPortStatus_1_1)[i].status.canChangePowerRole);

    (*currentPortStatus_1_1)[i].status.supportedModes = V1_0::PortMode::UFP;
    (*currentPortStatus_1_1)[i].supportedModes = PortMode_1_1::NONE | PortMode_1_1::UFP;
  }
  return Status::SUCCESS;
}

Return<void> Usb::queryPortStatus() {
  hidl_vec<PortStatus_1_1> currentPortStatus_1_1;
  hidl_vec<V1_0::PortStatus> currentPortStatus;
  Status status;
  sp<IUsbCallback> callback_V1_1 = IUsbCallback::castFrom(mCallback_1_0);

  pthread_mutex_lock(&mLock);
  if (mCallback_1_0 != NULL) {
    if (callback_V1_1 != NULL) {
      status = getPortStatusHelper(&currentPortStatus_1_1, false);
    } else {
      status = getPortStatusHelper(&currentPortStatus_1_1, true);
      currentPortStatus.resize(currentPortStatus_1_1.size());
      for (unsigned long i = 0; i < currentPortStatus_1_1.size(); i++)
        currentPortStatus[i] = currentPortStatus_1_1[i].status;
    }

    Return<void> ret;

    if (callback_V1_1 != NULL)
      ret = callback_V1_1->notifyPortStatusChange_1_1(currentPortStatus_1_1, status);
    else
      ret = mCallback_1_0->notifyPortStatusChange(currentPortStatus, status);

    if (!ret.isOk())
      ALOGE("queryPortStatus_1_1 error %s", ret.description().c_str());
  } else {
    ALOGI("Notifying userspace skipped. Callback is NULL");
  }
  pthread_mutex_unlock(&mLock);

  return Void();
}

struct data {
  int uevent_fd;
  android::hardware::usb::V1_1::implementation::Usb *usb;
};

static void uevent_event(uint32_t /*epevents*/, struct data *payload) {
  /* Handle Usb Dual Role uevent */
  char msg[UEVENT_MSG_LEN + 2];
  char *cp;
  int n;

  n = uevent_kernel_multicast_recv(payload->uevent_fd, msg, UEVENT_MSG_LEN);
  if (n <= 0) return;
  if (n >= UEVENT_MSG_LEN) /* overflow -- discard */
    return;

  msg[n] = '\0';
  msg[n + 1] = '\0';
  cp = msg;

  /* TODO */
  return;
}

void *work(void *param) {
  int epoll_fd, uevent_fd;
  struct epoll_event ev;
  int nevents = 0;
  struct data payload;

  uevent_fd = uevent_open_socket(64 * 1024, true);

  if (uevent_fd < 0) {
    ALOGE("uevent_init: uevent_open_socket failed\n");
    return NULL;
  }

  payload.uevent_fd = uevent_fd;
  payload.usb = (android::hardware::usb::V1_1::implementation::Usb *)param;

  fcntl(uevent_fd, F_SETFL, O_NONBLOCK);

  ev.events = EPOLLIN;
  ev.data.ptr = (void *)uevent_event;

  epoll_fd = epoll_create(64);
  if (epoll_fd == -1) {
    ALOGE("epoll_create failed; errno=%d", errno);
    goto error;
  }

  if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, uevent_fd, &ev) == -1) {
    ALOGE("epoll_ctl failed; errno=%d", errno);
    goto error;
  }

  while (!destroyThread) {
    struct epoll_event events[64];

    nevents = epoll_wait(epoll_fd, events, 64, -1);
    if (nevents == -1) {
      if (errno == EINTR) continue;
      ALOGE("usb epoll_wait failed; errno=%d", errno);
      break;
    }

    for (int n = 0; n < nevents; ++n) {
      if (events[n].data.ptr)
        (*(void (*)(int, struct data *payload))events[n].data.ptr)(
            events[n].events, &payload);
    }
  }

  ALOGI("exiting worker thread");
error:
  close(uevent_fd);

  if (epoll_fd >= 0) close(epoll_fd);

  return NULL;
}

void sighandler(int sig) {
  if (sig == SIGUSR1) {
    destroyThread = true;
    ALOGI("destroy set");
    return;
  }
  signal(SIGUSR1, sighandler);
}

Return<void> Usb::setCallback(const sp<V1_0::IUsbCallback> &callback) {

  sp<IUsbCallback> callback_V1_1 = IUsbCallback::castFrom(callback);

  if (callback != NULL)
      if (callback_V1_1 == NULL)
          ALOGI("Registering 1.0 callback");

  pthread_mutex_lock(&mLock);
  /*
   * When both the old callback and new callback values are NULL,
   * there is no need to spin off the worker thread.
   * When both the values are not NULL, we would already have a
   * worker thread running, so updating the callback object would
   * be suffice.
   */
  if ((mCallback_1_0 == NULL && callback == NULL) ||
      (mCallback_1_0 != NULL && callback != NULL)) {
    /*
     * Always store as V1_0 callback object. Type cast to V1_1
     * when the callback is actually invoked.
     */
    mCallback_1_0 = callback;
    pthread_mutex_unlock(&mLock);
    return Void();
  }

  mCallback_1_0 = callback;
  ALOGI("registering callback");

  // Kill the worker thread if the new callback is NULL.
  if (mCallback_1_0 == NULL) {
    pthread_mutex_unlock(&mLock);
    if (!pthread_kill(mPoll, SIGUSR1)) {
      pthread_join(mPoll, NULL);
      ALOGI("pthread destroyed");
    }
    return Void();
  }

  destroyThread = false;
  signal(SIGUSR1, sighandler);

  /*
   * Create a background thread if the old callback value is NULL
   * and being updated with a new value.
   */
  if (pthread_create(&mPoll, NULL, work, this)) {
    ALOGE("pthread creation failed %d", errno);
    mCallback_1_0 = NULL;
  }

  pthread_mutex_unlock(&mLock);
  return Void();
}

}  // namespace implementation
}  // namespace V1_0
}  // namespace usb
}  // namespace hardware
}  // namespace android
