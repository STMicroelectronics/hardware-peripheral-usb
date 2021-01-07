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
#define LOG_TAG "android.hardware.usb@1.1-service.stm32mp1-c"

//#define LOG_NDEBUG 0

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
#include <iostream>
#include <fstream>

#include <cutils/uevent.h>
#include <sys/epoll.h>
#include <utils/Errors.h>
#include <utils/StrongPointer.h>

#include "UsbC.h"

namespace android {
namespace hardware {
namespace usb {
namespace V1_1 {
namespace implementation {

// Set by the signal handler to destroy the thread
volatile bool destroyThread;

static int32_t readFile(std::string filename, std::string& contents) {
    std::ifstream file(filename);
    std::string out;

    if (file.is_open()) {
        getline(file, out);
        unsigned start = out.find("[");
        unsigned end = out.find("]");
        contents = out.substr(start+1, end-start-1); //Remove first and last char => [ & ]
        file.close();
        return 0;
    }
    return -1;
}

std::string appendRoleNodeHelper(const std::string portName, PortRoleType type) {
    std::string node("/sys/class/typec/" + portName);

    switch(type) {
        case PortRoleType::DATA_ROLE:
            return node + "/data_role";
        case PortRoleType::POWER_ROLE:
            return node + "/power_role";
        case PortRoleType::MODE:
            return node + "/port_type";
        default:
            return "";
    }
}

std::string convertRoletoString(PortRole role) {
    if (role.type == PortRoleType::POWER_ROLE) {
        if (role.role == static_cast<uint32_t> (PortPowerRole::SOURCE))
            return "source";
        else if (role.role ==  static_cast<uint32_t> (PortPowerRole::SINK))
            return "sink";
    } else if (role.type == PortRoleType::DATA_ROLE) {
        if (role.role == static_cast<uint32_t> (PortDataRole::HOST))
            return "host";
        if (role.role == static_cast<uint32_t> (PortDataRole::DEVICE))
            return "device";
    } else if (role.type == PortRoleType::MODE) {
        if (role.role == static_cast<uint32_t> (V1_0::PortMode::UFP))
            return "sink";
        if (role.role == static_cast<uint32_t> (V1_0::PortMode::DFP))
            return "source";
    }
    return "none";
}

void extractRole(std::string *roleName) {
  std::size_t first, last;

  first = roleName->find("[");
  last = roleName->find("]");

  if (first != std::string::npos && last != std::string::npos) {
    *roleName = roleName->substr(first + 1, last - first - 1);
  }
}

Status getCurrentRoleHelper(std::string portName,
        PortRoleType type, uint32_t &currentRole)  {
    std::string filename;
    std::string roleName;

    filename = appendRoleNodeHelper(portName, type);
    currentRole = static_cast<uint32_t>(V1_0::PortPowerRole::NONE);

    if (readFile(filename, roleName)) {
        ALOGE("Can't read %s", filename.c_str());
        return Status::ERROR;
    }

    extractRole(&roleName);

    if (roleName == "dfp")
        currentRole = static_cast<uint32_t> (V1_0::PortMode::DFP);
    else if (roleName == "ufp")
        currentRole = static_cast<uint32_t> (V1_0::PortMode::UFP);
    else if (roleName == "source")
        currentRole = static_cast<uint32_t> (V1_0::PortPowerRole::SOURCE);
    else if (roleName == "sink")
        currentRole = static_cast<uint32_t> (V1_0::PortPowerRole::SINK);
    else if (roleName == "host")
        currentRole = static_cast<uint32_t> (V1_0::PortDataRole::HOST);
    else if (roleName == "device")
        currentRole = static_cast<uint32_t> (V1_0::PortDataRole::DEVICE);
    else if (roleName != "none") {
         /* case for none has already been addressed.
          * so we check if the role isnt none.
          */
        ALOGE("Unrecognized role = %s", roleName.c_str());
        return Status::UNRECOGNIZED_ROLE;
    }

    return Status::SUCCESS;
}

Status getTypeCPortNamesHelper(std::vector<std::string>& names) {
    DIR *dp;

    dp = opendir("/sys/class/typec");
    if (dp != NULL)
    {
rescan:
        int32_t ports = 0;
        int32_t current = 0;
        struct dirent *ep;

        /* Get number of ports available */
        while ((ep = readdir (dp))) {
            if (ep->d_type == DT_LNK) {
                ports++;
            }
        }

        if (ports == 0) {
            closedir(dp);
            return Status::ERROR;
        }

        names.resize(ports);
        rewinddir(dp);

        while ((ep = readdir (dp))) {
            if (ep->d_type == DT_LNK) {
                /* Check to see if new ports were added since the first pass. */
                if (current >= ports) {
                    rewinddir(dp);
                    goto rescan;
                }
                names[current++] = ep->d_name;
            }
        }

        closedir (dp);
        return Status::SUCCESS;
    }

    ALOGE("Failed to open /sys/class/typec");
    return Status::ERROR;
}

bool canSwitchRoleHelper(const std::string portName, PortRoleType type)  {
    std::string filename = appendRoleNodeHelper(portName, type);
    std::ofstream file(filename);

    if (file.is_open()) {
        file.close();
        ALOGD("canSwitchRoleHelper: true for, %s - %d", portName.c_str(), (int)type);
        return true;
    }

    ALOGD("canSwitchRoleHelper: false for, %s - %d", portName.c_str(), (int)type);
    return false;
}

Status getPortStatusHelper (hidl_vec<PortStatus_1_1> *currentPortStatus_1_1, bool V1_0) {
    std::vector<std::string> names;
    Status result = getTypeCPortNamesHelper(names);

    if (result == Status::SUCCESS) {
        currentPortStatus_1_1->resize(names.size());
        for(std::vector<std::string>::size_type i = 0; i < names.size(); i++) {
            ALOGI("%s", names[i].c_str());
            (*currentPortStatus_1_1)[i].status.portName = names[i];

            uint32_t currentRole;
            if (getCurrentRoleHelper(names[i], PortRoleType::POWER_ROLE,
                    currentRole) == Status::SUCCESS) {
                (*currentPortStatus_1_1)[i].status.currentPowerRole =
                static_cast<PortPowerRole> (currentRole);
            } else {
                ALOGE("Error while retreiving portNames");
                goto done;
            }

            if (getCurrentRoleHelper(names[i],
                    PortRoleType::DATA_ROLE, currentRole) == Status::SUCCESS) {
                (*currentPortStatus_1_1)[i].status.currentDataRole =
                        static_cast<PortDataRole> (currentRole);
            } else {
                ALOGE("Error while retreiving current port role");
                goto done;
            }

            if (getCurrentRoleHelper(names[i], PortRoleType::MODE,
                    currentRole) == Status::SUCCESS) {
                (*currentPortStatus_1_1)[i].currentMode =
                    static_cast<PortMode_1_1>(currentRole);
                (*currentPortStatus_1_1)[i].status.currentMode =
                    static_cast<V1_0::PortMode> (currentRole);
            } else {
                ALOGE("Error while retreiving current data role");
                goto done;
            }

            (*currentPortStatus_1_1)[i].status.canChangeMode =
                canSwitchRoleHelper(names[i], PortRoleType::MODE);
            (*currentPortStatus_1_1)[i].status.canChangeDataRole =
                canSwitchRoleHelper(names[i], PortRoleType::DATA_ROLE);
            (*currentPortStatus_1_1)[i].status.canChangePowerRole =
                canSwitchRoleHelper(names[i], PortRoleType::POWER_ROLE);

            ALOGI("ports : %s, canChangeMode: %d canChagedata: %d canChangePower:%d",
                names[i].c_str(),
                (*currentPortStatus_1_1)[i].status.canChangeMode,
                (*currentPortStatus_1_1)[i].status.canChangeDataRole,
                (*currentPortStatus_1_1)[i].status.canChangePowerRole);


            if (V1_0) {
                (*currentPortStatus_1_1)[i].status.supportedModes = V1_0::PortMode::DFP;
            } else {
                (*currentPortStatus_1_1)[i].supportedModes = PortMode_1_1::UFP | PortMode_1_1::DFP;
                (*currentPortStatus_1_1)[i].status.supportedModes = V1_0::PortMode::NONE;
                (*currentPortStatus_1_1)[i].status.currentMode = V1_0::PortMode::NONE;
            }
        }
        return Status::SUCCESS;
    }

    ALOGD("Error while retreiving portNames");
done:
    return Status::ERROR;
}

UsbC::UsbC()
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

Return<void> UsbC::switchRole(const hidl_string &portName,
                             const V1_0::PortRole &newRole) {
    std::string filename = appendRoleNodeHelper(std::string(portName.c_str()),
        newRole.type);
    std::ofstream file(filename);
    std::string written;

    if (file.is_open()) {
        std::string temp = std::string("[" + convertRoletoString(newRole) + "]"); //Adding [ & ] to write the role
        file << temp.c_str();
        file.close();
        if (!readFile(filename, written)) {
            extractRole(&written);
            ALOGI("written: %s", written.c_str());
            if (written == convertRoletoString(newRole)) {
                Return<void> ret =
                    mCallback_1_0->notifyRoleSwitchStatus(portName, newRole,
                    Status::SUCCESS);
                if (!ret.isOk())
                    ALOGE("RoleSwitchStatus error %s",
                        ret.description().c_str());
            }
        }
    }

    Return<void> ret = mCallback_1_0->notifyRoleSwitchStatus(portName, newRole, Status::ERROR);
    if (!ret.isOk())
        ALOGE("RoleSwitchStatus error %s", ret.description().c_str());

    return Void();
}

Return<void> UsbC::queryPortStatus() {
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
  android::hardware::usb::V1_1::implementation::UsbC *usb;
};

static void uevent_event(uint32_t /*epevents*/, struct data *payload) {
    char msg[UEVENT_MSG_LEN + 2];
    char *cp;
    int n;

    n = uevent_kernel_multicast_recv(payload->uevent_fd, msg, UEVENT_MSG_LEN);
    if (n <= 0)
        return;
    if (n >= UEVENT_MSG_LEN)   /* overflow -- discard */
        return;

    msg[n] = '\0';
    msg[n + 1] = '\0';
    cp = msg;

    while (*cp) {
        if (std::regex_match(cp, std::regex("(add)(.*)(-partner)"))) {
            ALOGI("partner added");
            pthread_mutex_lock(&payload->usb->mPartnerLock);
            payload->usb->mPartnerUp = true;
            pthread_cond_signal(&payload->usb->mPartnerCV);
            pthread_mutex_unlock(&payload->usb->mPartnerLock);
        } else if (!strncmp(cp, "DEVTYPE=typec", strlen("DEVTYPE=typec"))) {
            ALOGE("uevent catch %s", cp);
            hidl_vec<PortStatus_1_1> currentPortStatus_1_1;
            if (payload->usb->mCallback_1_0 != NULL) {
                sp<IUsbCallback> callback_V1_1 = IUsbCallback::castFrom(payload->usb->mCallback_1_0);
                Return<void> ret;

                // V1_1 callback
                if (callback_V1_1 != NULL) {
                    Status status = getPortStatusHelper(&currentPortStatus_1_1, false);
                    ret = callback_V1_1->notifyPortStatusChange_1_1(
                      currentPortStatus_1_1, status);
                } else { // V1_0 callback
                    Status status = getPortStatusHelper(&currentPortStatus_1_1, true);

                    /*
                    * Copying the result from getPortStatusHelper
                    * into V1_0::PortStatus to pass back through
                    * the V1_0 callback object.
                    */
                    hidl_vec<V1_0::PortStatus> currentPortStatus;
                    currentPortStatus.resize(currentPortStatus_1_1.size());
                    for (unsigned long i = 0; i < currentPortStatus_1_1.size(); i++)
                    currentPortStatus[i] = currentPortStatus_1_1[i].status;

                    ret = payload->usb->mCallback_1_0->notifyPortStatusChange(
                      currentPortStatus, status);
                }

                if (!ret.isOk())
                    ALOGE("error %s", ret.description().c_str());
                } else {
                    ALOGI("Notifying userspace skipped. Callback is NULL");
                }

            }
        /* advance to after the next \0 */
        while (*cp++);
    }
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
  payload.usb = (android::hardware::usb::V1_1::implementation::UsbC *)param;

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

Return<void> UsbC::setCallback(const sp<V1_0::IUsbCallback> &callback) {

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
