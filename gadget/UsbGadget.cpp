/*
 * Copyright (C) 2020 The Android Open Source Project
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

#define LOG_TAG "android.hardware.usb.gadget-service.stm32mpu"

#include "UsbGadget.h"
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/inotify.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <aidl/android/frameworks/stats/IStats.h>

namespace aidl {
namespace android {
namespace hardware {
namespace usb {
namespace gadget {

using ::android::base::GetProperty;

UsbGadget::UsbGadget() {
    if (access(OS_DESC_PATH, R_OK) != 0) {
        ALOGE("configfs setup not done yet");
        abort();
    }
    gadgetName = GetProperty("sys.usb.controller", GADGET_NAME).c_str();
    monitorFfs.setGadgetName(gadgetName);
}

void currentFunctionsAppliedCallback(bool functionsApplied, void* payload) {
    UsbGadget* gadget = (UsbGadget*)payload;
    gadget->mCurrentUsbFunctionsApplied = functionsApplied;
}

ScopedAStatus UsbGadget::getCurrentUsbFunctions(const shared_ptr<IUsbGadgetCallback>& callback,
                                                int64_t in_transactionId) {
    if (callback == nullptr) {
        return ScopedAStatus::fromExceptionCode(EX_NULL_POINTER);
    }
    ScopedAStatus ret = callback->getCurrentUsbFunctionsCb(
        mCurrentUsbFunctions,
        mCurrentUsbFunctionsApplied ? Status::FUNCTIONS_APPLIED : Status::FUNCTIONS_NOT_APPLIED,
	in_transactionId);
    if (!ret.isOk())
        ALOGE("Call to getCurrentUsbFunctionsCb failed %s", ret.getDescription().c_str());

    return ScopedAStatus::ok();
}

ScopedAStatus UsbGadget::getUsbSpeed(const shared_ptr<IUsbGadgetCallback> &callback,
	int64_t in_transactionId) {
    std::string current_speed;
    if (ReadFileToString(UDC_PATH + gadgetName + SPEED_PATH, &current_speed)) {
        current_speed = Trim(current_speed);
        ALOGI("current USB speed is %s", current_speed.c_str());
        if (current_speed == "low-speed")
            mUsbSpeed = UsbSpeed::LOWSPEED;
        else if (current_speed == "full-speed")
            mUsbSpeed = UsbSpeed::FULLSPEED;
        else if (current_speed == "high-speed")
            mUsbSpeed = UsbSpeed::HIGHSPEED;
        else if (current_speed == "super-speed")
            mUsbSpeed = UsbSpeed::SUPERSPEED;
        else if (current_speed == "super-speed-plus")
            mUsbSpeed = UsbSpeed::SUPERSPEED_10Gb;
        else if (current_speed == "UNKNOWN")
            mUsbSpeed = UsbSpeed::UNKNOWN;
        else {
            mUsbSpeed = UsbSpeed::UNKNOWN;
        }
    } else {
        ALOGE("Fail to read current speed : %s%s%s", UDC_PATH, gadgetName.c_str(), SPEED_PATH);
        mUsbSpeed = UsbSpeed::UNKNOWN;
    }

    if (callback) {
        ScopedAStatus ret = callback->getUsbSpeedCb(mUsbSpeed, in_transactionId);

        if (!ret.isOk())
            ALOGE("Call to getUsbSpeedCb failed %s", ret.getDescription().c_str());
    }

    return ScopedAStatus::ok();
}

Status UsbGadget::tearDownGadget() {
    if (resetGadget() != Status::SUCCESS) return Status::ERROR;

    if (monitorFfs.isMonitorRunning()) {
        monitorFfs.reset();
    } else {
        ALOGI("mMonitor not running");
    }
    return Status::SUCCESS;
}

ScopedAStatus UsbGadget::reset(const shared_ptr<IUsbGadgetCallback> &callback,
        int64_t in_transactionId) {

    if (!WriteStringToFile("none", PULLUP_PATH)) {
        ALOGI("Gadget cannot be pulled down");
        return ScopedAStatus::fromServiceSpecificErrorWithMessage(
                -1, "Gadget cannot be pulled down");
    }

    usleep(kDisconnectWaitUs);

    if (!WriteStringToFile(gadgetName, PULLUP_PATH)) {
        ALOGI("Gadget cannot be pulled up");
        return ScopedAStatus::fromServiceSpecificErrorWithMessage(
                -1, "Gadget cannot be pulled up");
    }

    if (callback)
        callback->resetCb(Status::SUCCESS, in_transactionId);

    return ScopedAStatus::ok();
}

static Status validateAndSetVidPid(uint64_t functions) {
    Status ret = Status::SUCCESS;

    switch (functions) {
        case GadgetFunction::MTP:
            ret = setVidPid("0x0483", "0x0105");
            if (!WriteStringToFile("mtp", CONFIG_STRING_PATH)) {
                ALOGE("Can't write gadget configuration");
            }
            break;
        case GadgetFunction::ADB | GadgetFunction::MTP:
            ret = setVidPid("0x0483", "0x0104");
            if (!WriteStringToFile("mtp_adb", CONFIG_STRING_PATH)) {
                ALOGE("Can't write gadget configuration");
            }
            break;
        case GadgetFunction::NCM:
        case GadgetFunction::RNDIS:
            ret = setVidPid("0x0483", "0x0103");
            if (!WriteStringToFile("ncm", CONFIG_STRING_PATH)) {
                ALOGE("Can't write gadget configuration");
            }
            break;
        case GadgetFunction::ADB | GadgetFunction::NCM:
        case GadgetFunction::ADB | GadgetFunction::RNDIS:
            ret = setVidPid("0x0483", "0x0102");
            if (!WriteStringToFile("ncm_adb", CONFIG_STRING_PATH)) {
                ALOGE("Can't write gadget configuration");
            }
            break;
        case GadgetFunction::PTP:
            ret = setVidPid("0x0483", "0x0107");
            if (!WriteStringToFile("ptp", CONFIG_STRING_PATH)) {
                ALOGE("Can't write gadget configuration");
            }
            break;
        case GadgetFunction::ADB | GadgetFunction::PTP:
            ret = setVidPid("0x0483", "0x0106");
            if (!WriteStringToFile("ptp_adb", CONFIG_STRING_PATH)) {
                ALOGE("Can't write gadget configuration");
            }
            break;
        case GadgetFunction::ADB:
            ret = setVidPid("0x0483", "0x0adb");
            if (!WriteStringToFile("adb", CONFIG_STRING_PATH)) {
                ALOGE("Can't write gadget configuration");
            }
            break;
        default:
            ALOGE("Combination not supported");
            ret = Status::CONFIGURATION_NOT_SUPPORTED;
    }
    return ret;
}

Status UsbGadget::setupFunctions(long functions,
	const shared_ptr<IUsbGadgetCallback> &callback, uint64_t timeout,
	int64_t in_transactionId) {
    bool ffsEnabled = false;
    int i = 0;

    if (timeout == 0) {
	ALOGI("timeout not setup");
    }

    if (addGenericAndroidFunctions(&monitorFfs, functions, &ffsEnabled, &i) !=
        Status::SUCCESS)
        return Status::ERROR;

    if ((functions & GadgetFunction::ADB) != 0) {
        ffsEnabled = true;
        if (addAdb(&monitorFfs, &i) != Status::SUCCESS) return Status::ERROR;
    }

    // Pull up the gadget right away when there are no ffs functions.
    if (!ffsEnabled) {
        if (!WriteStringToFile(gadgetName, PULLUP_PATH)) return Status::ERROR;
        mCurrentUsbFunctionsApplied = true;
        if (callback)
            callback->setCurrentUsbFunctionsCb(functions, Status::SUCCESS, in_transactionId);
        return Status::SUCCESS;
    }

    monitorFfs.registerFunctionsAppliedCallback(&currentFunctionsAppliedCallback, this);
    // Monitors the ffs paths to pull up the gadget when descriptors are written.
    // Also takes of the pulling up the gadget again if the userspace process
    // dies and restarts.
    monitorFfs.startMonitor();

    if (kDebug) ALOGI("Mainthread in Cv");

    if (callback) {
        bool pullup = monitorFfs.waitForPullUp(timeout);
        callback->setCurrentUsbFunctionsCb(functions, pullup ? Status::SUCCESS : Status::ERROR,
            in_transactionId);
    }

    return Status::SUCCESS;
}

ScopedAStatus UsbGadget::setCurrentUsbFunctions(int64_t functions,
                                               const shared_ptr<IUsbGadgetCallback> &callback,
					       int64_t timeoutMs,
					       int64_t in_transactionId) {
    std::unique_lock<std::mutex> lk(mLockSetCurrentFunction);

    mCurrentUsbFunctions = functions;
    mCurrentUsbFunctionsApplied = false;

    // Unlink the gadget and stop the monitor if running.
    Status status = tearDownGadget();
    if (status != Status::SUCCESS) {
        goto error;
    }

    ALOGI("Returned from tearDown gadget");

    // Leave the gadget pulled down to give time for the host to sense disconnect.
    usleep(kDisconnectWaitUs);

    if (functions == GadgetFunction::NONE) {
        if (callback == NULL)
            return ScopedAStatus::fromServiceSpecificErrorWithMessage(
                -1, "callback == NULL");
        ScopedAStatus ret = callback->setCurrentUsbFunctionsCb(functions, Status::SUCCESS, in_transactionId);
        if (!ret.isOk())
            ALOGE("Error while calling setCurrentUsbFunctionsCb %s", ret.getDescription().c_str());
        return ScopedAStatus::fromServiceSpecificErrorWithMessage(
                -1, "Error while calling setCurrentUsbFunctionsCb");
    }

    status = validateAndSetVidPid(functions);

    if (status != Status::SUCCESS) {
        goto error;
    }

    status = setupFunctions(functions, callback, timeoutMs, in_transactionId);
    if (status != Status::SUCCESS) {
        goto error;
    }

    ALOGI("Usb Gadget setcurrent functions called successfully");
    return ScopedAStatus::fromServiceSpecificErrorWithMessage(
                -1, "Usb Gadget setcurrent functions called successfully");


error:
    ALOGI("Usb Gadget setcurrent functions failed");
    if (callback == NULL)
        return ScopedAStatus::fromServiceSpecificErrorWithMessage(
                -1, "Usb Gadget setcurrent functions failed");
    ScopedAStatus ret = callback->setCurrentUsbFunctionsCb(functions, status, in_transactionId);
    if (!ret.isOk())
        ALOGE("Error while calling setCurrentUsbFunctionsCb %s", ret.getDescription().c_str());
    return ScopedAStatus::fromServiceSpecificErrorWithMessage(
                -1, "Error while calling setCurrentUsbFunctionsCb");
}
}  // namespace gadget
}  // namespace usb
}  // namespace hardware
}  // namespace android
}  // aidl
