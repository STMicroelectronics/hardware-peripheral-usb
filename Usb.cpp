/*
 * Copyright (C) 2021 The Android Open Source Project
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

#define LOG_TAG "android.hardware.usb-service.stm32mpu"

#include <aidl/android/hardware/usb/PortRole.h>
#include <android-base/logging.h>
#include <android-base/properties.h>
#include <android-base/strings.h>
#include <assert.h>
#include <dirent.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <chrono>
#include <regex>
#include <thread>
#include <unordered_map>

#include <cutils/uevent.h>
#include <sys/epoll.h>
#include <utils/Errors.h>
#include <utils/StrongPointer.h>

#include "Usb.h"

using android::base::GetProperty;
using android::base::Trim;

namespace aidl {
namespace android {
namespace hardware {
namespace usb {

constexpr char kTypecPath[] = "/sys/class/typec/";
constexpr char kDataRoleNode[] = "/data_role";
constexpr char kPowerRoleNode[] = "/power_role";

// Set by the signal handler to destroy the thread
volatile bool destroyThread;

void queryVersionHelper(android::hardware::usb::Usb *usb,
                        std::vector<PortStatus> *currentPortStatus);

ScopedAStatus Usb::enableUsbData(const string& in_portName, bool in_enable, int64_t in_transactionId) {
    std::vector<PortStatus> currentPortStatus;

    pthread_mutex_lock(&mLock);
    if (mCallback != NULL) {
        ScopedAStatus ret = mCallback->notifyEnableUsbDataStatus(
            in_portName, true, in_enable ? Status::SUCCESS : Status::ERROR, in_transactionId);
        if (!ret.isOk())
            ALOGE("notifyEnableUsbDataStatus error %s", ret.getDescription().c_str());
    } else {
        ALOGE("Not notifying the userspace. Callback is not set");
    }
    pthread_mutex_unlock(&mLock);
    queryVersionHelper(this, &currentPortStatus);

    return ScopedAStatus::ok();
}

ScopedAStatus Usb::enableUsbDataWhileDocked(const string& in_portName, int64_t in_transactionId) {

    pthread_mutex_lock(&mLock);
    if (mCallback != NULL) {
        ScopedAStatus ret = mCallback->notifyEnableUsbDataWhileDockedStatus(
            in_portName, Status::NOT_SUPPORTED, in_transactionId);
        if (!ret.isOk())
            ALOGE("notifyEnableUsbDataWhileDockedStatus error %s", ret.getDescription().c_str());
    } else {
        ALOGE("Not notifying the userspace. Callback is not set");
    }
    pthread_mutex_unlock(&mLock);

    return ScopedAStatus::ok();
}

ScopedAStatus Usb::resetUsbPort(const string& in_portName, int64_t in_transactionId) {

    pthread_mutex_lock(&mLock);
    if (mCallback != NULL) {
        ScopedAStatus ret = mCallback->notifyResetUsbPortStatus(
            in_portName, Status::NOT_SUPPORTED, in_transactionId);
        if (!ret.isOk())
            ALOGE("notifyResetUsbPortStatus error %s", ret.getDescription().c_str());
    } else {
        ALOGE("Not notifying the userspace. Callback is not set");
    }
    pthread_mutex_unlock(&mLock);

    return ScopedAStatus::ok();
}

Status queryMoistureDetectionStatus(std::vector<PortStatus> *currentPortStatus) {
    string enabled, status, path, DetectedPath;

    for (int i = 0; i < currentPortStatus->size(); i++) {
        (*currentPortStatus)[i].supportedContaminantProtectionModes
                .push_back(ContaminantProtectionMode::NONE);
        (*currentPortStatus)[i].contaminantProtectionStatus
                = ContaminantProtectionStatus::NONE;
        (*currentPortStatus)[i].contaminantDetectionStatus
                = ContaminantDetectionStatus::NOT_SUPPORTED;
        (*currentPortStatus)[i].supportsEnableContaminantPresenceDetection = false;
        (*currentPortStatus)[i].supportsEnableContaminantPresenceProtection = false;
    }

    return Status::SUCCESS;
}

string appendRoleNodeHelper(const string &portName, PortRole::Tag tag) {
    string node(kTypecPath + portName);

    switch (tag) {
        case PortRole::dataRole:
            return node + kDataRoleNode;
        case PortRole::powerRole:
            return node + kPowerRoleNode;
        case PortRole::mode:
            return node + "/port_type";
        default:
            return "";
    }
}

string convertRoletoString(PortRole role) {
    if (role.getTag() == PortRole::powerRole) {
        if (role.get<PortRole::powerRole>() == PortPowerRole::SOURCE)
            return "source";
        else if (role.get<PortRole::powerRole>() == PortPowerRole::SINK)
            return "sink";
    } else if (role.getTag() == PortRole::dataRole) {
        if (role.get<PortRole::dataRole>() == PortDataRole::HOST)
            return "host";
        if (role.get<PortRole::dataRole>() == PortDataRole::DEVICE)
            return "device";
    } else if (role.getTag() == PortRole::mode) {
        if (role.get<PortRole::mode>() == PortMode::UFP)
            return "sink";
        if (role.get<PortRole::mode>() == PortMode::DFP)
            return "source";
    }
    return "none";
}

Usb::Usb()
    : mLock(PTHREAD_MUTEX_INITIALIZER),
      mRoleSwitchLock(PTHREAD_MUTEX_INITIALIZER),
      mPartnerLock(PTHREAD_MUTEX_INITIALIZER),
      mPartnerUp(false)
{
    pthread_condattr_t attr;
    if (pthread_condattr_init(&attr)) {
        ALOGE("pthread_condattr_init failed: %s", strerror(errno));
        abort();
    }
    if (pthread_condattr_setclock(&attr, CLOCK_MONOTONIC)) {
        ALOGE("pthread_condattr_setclock failed: %s", strerror(errno));
        abort();
    }
    if (pthread_cond_init(&mPartnerCV, &attr)) {
        ALOGE("pthread_cond_init failed: %s", strerror(errno));
        abort();
    }
    if (pthread_condattr_destroy(&attr)) {
        ALOGE("pthread_condattr_destroy failed: %s", strerror(errno));
        abort();
    }
}

ScopedAStatus Usb::switchRole(const string& in_portName,
    const PortRole& in_role, int64_t in_transactionId) {

    bool roleSwitch = false;

    ALOGE("Usb role switch is not supported");

    pthread_mutex_lock(&mLock);
    if (mCallback != NULL) {
         ScopedAStatus ret = mCallback->notifyRoleSwitchStatus(
            in_portName, in_role, roleSwitch ? Status::SUCCESS : Status::ERROR, in_transactionId);
        if (!ret.isOk())
            ALOGE("RoleSwitchStatus error %s", ret.getDescription().c_str());
    } else {
        ALOGE("Not notifying the userspace. Callback is not set");
    }
    pthread_mutex_unlock(&mLock);

    return ScopedAStatus::ok();
}

ScopedAStatus Usb::limitPowerTransfer(const string& in_portName, bool /*in_limit*/,
        int64_t in_transactionId) {
    std::vector<PortStatus> currentPortStatus;

    pthread_mutex_lock(&mLock);
    if (mCallback != NULL && in_transactionId >= 0) {
        ScopedAStatus ret = mCallback->notifyLimitPowerTransferStatus(
                in_portName, false, Status::NOT_SUPPORTED, in_transactionId);
        if (!ret.isOk())
            ALOGE("limitPowerTransfer error %s", ret.getDescription().c_str());
    } else {
        ALOGE("Not notifying the userspace. Callback is not set");
    }
    pthread_mutex_unlock(&mLock);

    return ScopedAStatus::ok();
}

Status getAccessoryConnected(const string &portName __unused, string *accessory) {
    *accessory = "none";
    return Status::SUCCESS;
}

Status getCurrentRoleHelper(const string &portName, bool connected, PortRole *currentRole) {
    string filename;
    string roleName;
    string accessory;

    // Mode
    if (currentRole->getTag() == PortRole::powerRole) {
        filename = kTypecPath + portName + kPowerRoleNode;
        currentRole->set<PortRole::powerRole>(PortPowerRole::NONE);
    } else if (currentRole->getTag() == PortRole::dataRole) {
        filename = kTypecPath + portName + kDataRoleNode;
        currentRole->set<PortRole::dataRole>(PortDataRole::DEVICE);
    } else if (currentRole->getTag() == PortRole::mode) {
        filename = kTypecPath + portName + kDataRoleNode;
        currentRole->set<PortRole::mode>(PortMode::UFP);
    } else {
        return Status::ERROR;
    }

    if (!connected)
        return Status::SUCCESS;

    if (currentRole->getTag() == PortRole::mode) {
        if (getAccessoryConnected(portName, &accessory) != Status::SUCCESS) {
            return Status::ERROR;
        }
        if (accessory == "analog_audio") {
            currentRole->set<PortRole::mode>(PortMode::AUDIO_ACCESSORY);
            return Status::SUCCESS;
        } else if (accessory == "debug") {
            currentRole->set<PortRole::mode>(PortMode::DEBUG_ACCESSORY);
            return Status::SUCCESS;
        }
    }
    return Status::SUCCESS;
}

Status getTypeCPortNamesHelper(std::unordered_map<string, bool> *names) {
    names->insert({"port0", false});
    return Status::SUCCESS;
}

bool canSwitchRoleHelper(const string &portName) {
    string filename = kTypecPath + portName + "-partner/supports_usb_power_delivery";
    return false;
}

Status getPortStatusHelper(std::vector<PortStatus> *currentPortStatus) {
    std::unordered_map<string, bool> names;
    Status result = getTypeCPortNamesHelper(&names);
    int i = -1;

    if (result == Status::SUCCESS) {
        currentPortStatus->resize(names.size());
        for (std::pair<string, bool> port : names) {
            i++;
            ALOGI("%s", port.first.c_str());
            (*currentPortStatus)[i].portName = port.first;

            PortRole currentRole;
            currentRole.set<PortRole::powerRole>(PortPowerRole::NONE);
            if (getCurrentRoleHelper(port.first, port.second, &currentRole) == Status::SUCCESS){
                (*currentPortStatus)[i].currentPowerRole = currentRole.get<PortRole::powerRole>();
            } else {
                ALOGE("Error while retrieving portNames");
                goto done;
            }

            currentRole.set<PortRole::dataRole>(PortDataRole::NONE);
            if (getCurrentRoleHelper(port.first, port.second, &currentRole) == Status::SUCCESS) {
                (*currentPortStatus)[i].currentDataRole = currentRole.get<PortRole::dataRole>();
            } else {
                ALOGE("Error while retrieving current port role");
                goto done;
            }

            currentRole.set<PortRole::mode>(PortMode::NONE);
            if (getCurrentRoleHelper(port.first, port.second, &currentRole) == Status::SUCCESS) {
                (*currentPortStatus)[i].currentMode = currentRole.get<PortRole::mode>();
            } else {
                ALOGE("Error while retrieving current data role");
                goto done;
            }

            (*currentPortStatus)[i].canChangeMode = false;
            (*currentPortStatus)[i].canChangeDataRole = false;
            (*currentPortStatus)[i].canChangePowerRole = false;

            (*currentPortStatus)[i].supportedModes.push_back(PortMode::UFP);
            (*currentPortStatus)[i].supportedModes.push_back(PortMode::NONE); //line added
            (*currentPortStatus)[i].usbDataStatus.push_back(UsbDataStatus::ENABLED);

            ALOGI("%d:%s connected:%d canChangeMode:%d canChagedata:%d canChangePower:%d "
                "usbDataEnabled:%d",
                i, port.first.c_str(), port.second,
                (*currentPortStatus)[i].canChangeMode,
                (*currentPortStatus)[i].canChangeDataRole,
                (*currentPortStatus)[i].canChangePowerRole, 0);
        }

        return Status::SUCCESS;
    }
done:
    return Status::ERROR;
}

void queryVersionHelper(android::hardware::usb::Usb *usb,
                        std::vector<PortStatus> *currentPortStatus) {
    Status status;
    pthread_mutex_lock(&usb->mLock);
    status = getPortStatusHelper(currentPortStatus);
    queryMoistureDetectionStatus(currentPortStatus);
    if (usb->mCallback != NULL) {
        ScopedAStatus ret = usb->mCallback->notifyPortStatusChange(*currentPortStatus,
            status);
        if (!ret.isOk())
            ALOGE("queryPortStatus error %s", ret.getDescription().c_str());
    } else {
        ALOGI("Notifying userspace skipped. Callback is NULL");
    }
    pthread_mutex_unlock(&usb->mLock);
}

ScopedAStatus Usb::queryPortStatus(int64_t in_transactionId) {
    std::vector<PortStatus> currentPortStatus;

    queryVersionHelper(this, &currentPortStatus);
    pthread_mutex_lock(&mLock);
    if (mCallback != NULL) {
        ScopedAStatus ret = mCallback->notifyQueryPortStatus(
            "all", Status::SUCCESS, in_transactionId);
        if (!ret.isOk())
            ALOGE("notifyQueryPortStatus error %s", ret.getDescription().c_str());
    } else {
        ALOGE("Not notifying the userspace. Callback is not set");
    }
    pthread_mutex_unlock(&mLock);

    return ScopedAStatus::ok();
}

ScopedAStatus Usb::enableContaminantPresenceDetection(const string& in_portName,
        bool /*in_enable*/, int64_t in_transactionId) {
    std::vector<PortStatus> currentPortStatus;

    pthread_mutex_lock(&mLock);
    if (mCallback != NULL) {
        ScopedAStatus ret = mCallback->notifyContaminantEnabledStatus(
            in_portName, false, Status::ERROR, in_transactionId);
        if (!ret.isOk())
            ALOGE("enableContaminantPresenceDetection  error %s", ret.getDescription().c_str());
    } else {
        ALOGE("Not notifying the userspace. Callback is not set");
    }
    pthread_mutex_unlock(&mLock);

    queryVersionHelper(this, &currentPortStatus);
    return ScopedAStatus::ok();
}


struct data {
    int uevent_fd;
    ::aidl::android::hardware::usb::Usb *usb;
};

static void uevent_event(uint32_t /*epevents*/, struct data *payload) {
    char msg[UEVENT_MSG_LEN + 2];
    char *cp;
    int n;

    n = uevent_kernel_multicast_recv(payload->uevent_fd, msg, UEVENT_MSG_LEN);
    if (n <= 0)
        return;
    if (n >= UEVENT_MSG_LEN) /* overflow -- discard */
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
        }
        while (*cp++) {
        }
    }
}

void *work(void *param) {
    int epoll_fd, uevent_fd;
    struct epoll_event ev;
    int nevents = 0;
    struct data payload;

    uevent_fd = uevent_open_socket(UEVENT_MAX_EVENTS * UEVENT_MSG_LEN, true);

    if (uevent_fd < 0) {
        ALOGE("uevent_init: uevent_open_socket failed\n");
        return NULL;
    }

    payload.uevent_fd = uevent_fd;
    payload.usb = (::aidl::android::hardware::usb::Usb *)param;

    fcntl(uevent_fd, F_SETFL, O_NONBLOCK);

    ev.events = EPOLLIN;
    ev.data.ptr = (void *)uevent_event;

    epoll_fd = epoll_create(UEVENT_MAX_EVENTS);
    if (epoll_fd == -1) {
        ALOGE("epoll_create failed; errno=%d", errno);
        goto error;
    }

    if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, uevent_fd, &ev) == -1) {
        ALOGE("epoll_ctl failed; errno=%d", errno);
        goto error;
    }

    while (!destroyThread) {
        struct epoll_event events[UEVENT_MAX_EVENTS];

        nevents = epoll_wait(epoll_fd, events, UEVENT_MAX_EVENTS, -1);
        if (nevents == -1) {
            if (errno == EINTR)
                continue;
            ALOGE("usb epoll_wait failed; errno=%d", errno);
            break;
        }

        for (int n = 0; n < nevents; ++n) {
            if (events[n].data.ptr)
                (*(void (*)(int, struct data *payload))events[n].data.ptr)(events[n].events,
                                                                           &payload);
        }
    }

    ALOGI("exiting worker thread");
error:
    close(uevent_fd);

    if (epoll_fd >= 0)
        close(epoll_fd);

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

ScopedAStatus Usb::setCallback(
        const shared_ptr<IUsbCallback>& in_callback) {

    pthread_mutex_lock(&mLock);
    if ((mCallback == NULL && in_callback == NULL) ||
            (mCallback != NULL && in_callback != NULL)) {
        mCallback = in_callback;
        pthread_mutex_unlock(&mLock);
        return ScopedAStatus::ok();
    }

    mCallback = in_callback;
    ALOGI("registering callback");

    if (mCallback == NULL) {
        if  (!pthread_kill(mPoll, SIGUSR1)) {
            pthread_join(mPoll, NULL);
            ALOGI("pthread destroyed");
        }
        pthread_mutex_unlock(&mLock);
        return ScopedAStatus::ok();
    }

    destroyThread = false;
    signal(SIGUSR1, sighandler);

    /*
     * Create a background thread if the old callback value is NULL
     * and being updated with a new value.
     */
    if (pthread_create(&mPoll, NULL, work, this)) {
        ALOGE("pthread creation failed %d", errno);
        mCallback = NULL;
    }

    pthread_mutex_unlock(&mLock);
    return ScopedAStatus::ok();
}

} // namespace usb
} // namespace hardware
} // namespace android
} // aidl
