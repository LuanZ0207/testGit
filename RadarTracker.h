/////////////////////////////////////////////////////////////////////////////////////////
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA CORPORATION & AFFILIATES assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA CORPORATION & AFFILIATES. No third party distribution is allowed unless
// expressly authorized by NVIDIA. Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA CORPORATION & AFFILIATES products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA CORPORATION & AFFILIATES.
//
// SPDX-FileCopyrightText: Copyright (c) 2017-2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//
// NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
// property and proprietary rights in and to this material, related
// documentation and any modifications thereto. Any use, reproduction,
// disclosure or distribution of this material and related documentation
// without an express license agreement from NVIDIA CORPORATION or
// its affiliates is strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

/**
 * @file
 * <b>NVIDIA DriveWorks API: RadarTracker</b>
 *
 * @b Description: This file defines the ethernet radar tracker.
 */

/**
 * @defgroup radartracker_group RadarTracker Interface
 *
 * @brief Defines radar tracker module based on Continental ARS430 ethernet radar.
 *
 * @{
 */

#ifndef DW_PERCEPTION_OBJECT_RADAR_RADARTRACKER_H__
#define DW_PERCEPTION_OBJECT_RADAR_RADARTRACKER_H__

#include <dw/core/context/Context.h>
#include <dw/sensors/radar/Radar.h>
#include <dw/egomotion/Egomotion.h>
#include <dw/egomotion/EgomotionState.h>
#include <dw/world/Object.h>
#include <dw/world/ObjectArray.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * This is the maximum number of tracks that will be populated on any dwRadarTracker_process() call
 *
 * @note This number of point tracks created depends on the scene.
 * This is the buffer size that should be allocated, as the module
 * will assume a maximum buffer size of 2000 tracks.
 */
#define DW_RADARTRACKER_MAX_TRACKS 2000U

#define DW_RADARCLUSTERER_MAX_CLUSTERS 400U

#define DW_RADARSTATE_MAX_DETECTIONS 2000U

#define DW_RADARTRACKER_MAX_NUM_RADARS 16u // max number of radars

typedef struct dwRadarTrackerObject* dwRadarTrackerHandle_t;

typedef dwRadarScan dwRadarTrackList;

typedef enum dwRadarTrackerFusionMode {
    DW_RADAR_TRACKER_FUSIONMODE_EARLY = 0, // early fusion module, max 16 radar sensors
} dwRadarTrackerFusionMode;

/// Defines the parameters used by the radar tracker module
typedef struct dwRadarTrackerParams
{
    /// Specifies the properties of the radar sensor used
    dwRadarProperties radarProperties;

    /// Specifies the pose transformation from sensor to rig space
    dwTransformation3f sensorToRig;

    /// Number of Radars sent to the tracker
    uint32_t numberOfRadars;

    /// Fusion Implementation
    dwRadarTrackerFusionMode fusionMode;

    /// Whether or not to send to track type output to clusterer not.
    /// If so, velocity of dwRadarTrack will be WRT ground (not relative to rig),
    /// else the radar output is sent directly to fusion, it should be relative to rig
    /// This will also determine max output size
    bool radarTrackOutputSentToClusterer;

    /// Uses a fixed delta time between radar scans instead of using the host time
    float32_t fixedDeltaScanTime;

    /// Whether or not to use the host time or a fixed delta time between radar scans
    bool useFixedDeltaScanTime;

    /// Filter out unclassified tracks
    bool filterByClass;
} dwRadarTrackerParams;

/// Defines a single watchdog
typedef struct dwRadarTrackerWatchdogInfo
{
    dwStatus status;                   // DW_SUCCESS; DW_NOT_READY (never been kicked off or ego motion is not ready); DW_TIME_OUT
    dwTime_t lastTimeoutHostTimestamp; // the earliest host timestamp that the watchdog has been found timed-out before it will be kicked again next time
    dwTime_t lastSuccessHostTimestamp; // the latest host timestamp that the watchdog has been kicked off (i.e., has become successful)
} dwRadarTrackerWatchdogInfo;

/// Defines an array of watchdogs, that can be passed between dwNodes
typedef struct dwRadarTrackerWatchdogArray
{
    uint32_t watchdogsCount;                                              // counts how many watchdogs are available in total in this array
    dwRadarTrackerWatchdogInfo watchdogs[DW_RADARTRACKER_MAX_NUM_RADARS]; // the array of watchdogs
} dwRadarTrackerWatchdogArray;

typedef struct dwRadarTrackerRadarState
{
    /// Rig info
    uint32_t radarSensorIndex; // Sensor index
    uint32_t sensorRigIndex;   // Sensor index in the rig coordinates

    /// Time info
    dwTime_t hostTimeStamp;   // Host timestamp when the radar scan was received
    dwTime_t sensorTimeStamp; // Sensor timestamp when the radar scan was received
    dwTime_t nodeProcessTime; // The latest timestamp when dwRadarTrackerNodeImpl::process() is called

    /// Ego motion info
    dwVector3f egoLinearVelocity;  // 3 DOF linear velocity
    dwVector3f egoAngularVelocity; // 3 DOF angular velocity

    /// Doppler ambiguity [m/s] of the radar detection scan last used
    float32_t radarScanDopplerAmbiguity;

    /// Transformation matrix from the radar to vehicle coordinate
    dwTransformation3f radar2rig;

    /// Detection info
    dwRadarDetection radarDetectionList[DW_RADARSTATE_MAX_DETECTIONS]; // Latest detection list from the radar in vehicle coordinates
    uint32_t numDetections;                                            // Total number of the radar detections in the list
    dwRadarScanType scanType;
    dwRadarDynamicState radarDetectionDynamicState[DW_RADARSTATE_MAX_DETECTIONS]; // Latest list of dynamic motion state classifications for the radar detection list

    /// Watchdog
    dwRadarTrackerWatchdogInfo watchdog; // watchdog monitoring Short Range Scan
} dwRadarTrackerRadarState;

/**
 * Creates and initialize a radar tracker
 *
 * @param[out] obj A pointer to the radar tracker handle is returned here.
 * @param[in] params Specifies the parameters of the radar tracker.
 * @param[in] ctx Specifies the handle to the dw context.
 *
 * @return DW_INVALID_ARGUMENT if obj is NULL. <br>
 *         DW_SUCCESS otherwise.<br>
 */
DW_API_PUBLIC
dwStatus dwRadarTracker_initialize(dwRadarTrackerHandle_t* obj, const dwRadarTrackerParams* params, dwContextHandle_t ctx);

/**
 * Releases the radar tracker.
 * This method releases all resources associated with the radar tracker.
 *
 * @note This method renders the handle unusable.
 * @param[in] obj The object handle to be released.
 *
 * @return DW_INVALID_HANDLE if radar tracker handle is NULL.<br>
 *         DW_SUCCESS otherwise.<br>
 */
DW_API_PUBLIC
dwStatus dwRadarTracker_release(dwRadarTrackerHandle_t obj);

/**
 * Bind the input to a dwRadarTrackEgomotionState.
 *
 * @param[in] egomotionState egomotion state handle to use
 * @param[in] obj Handle to the Radar Tracker module
 * @return DW_SUCCESS
 *         DW_INVALID_ARGUMENT   When laneFusionHandle or egomotionState is nullptr.<br>
 */
DW_API_PUBLIC
dwStatus dwRadarTracker_bindInputEgomotionState(dwConstEgomotionStateHandle_t egomotionState,
                                                dwRadarTrackerHandle_t obj);

/**
 * Sets the sensor to rig transformation. To be used in case the
 * extrinsics of the sensor change post initialization.
 *
 * @param[in] sensorToRig Specifies the pose transformation from sensor to rig space.
 * @param[in] obj Specifies radar tracker handle.
 *
 * @return DW_INVALID_HANDLE if radar tracker handle is NULL.<br>
 *         DW_SUCCESS otherwise.<br>
 */
DW_API_PUBLIC
dwStatus dwRadarTracker_setSensorToRig(const dwTransformation3f* sensorToRig, dwRadarTrackerHandle_t obj);

/**
 * Sets the tracker to enable/disable tracking with elevation information.
 * @note Elevation information is only available for Long Range Scan in the Continental ARS430.
 * Hence, this method is only applicable to Long Range data.
 *
 * @param[in] enableElevation Specifies whether tracker output should contain elevation information or not
 * @param[in] obj Specifies radar tracker handle.
 *
 * @return DW_INVALID_HANDLE if radar tracker handle is NULL.<br>
 *         DW_SUCCESS otherwise.<br>
 */
DW_API_PUBLIC
dwStatus dwRadarTracker_setElevationTracking(bool enableElevation, dwRadarTrackerHandle_t obj);

/**
 * Sets the FOV, about the central ray, within which tracking should not be performed.
 * @note This is the absolute angle in degrees from the center where tracking should not be performed(eg: If the required FOV to avoid tracking is 20 degrees,
 * the angleToAvoid should be +- 10 degrees). Currently only enabled for Short range scan, to accurately process elevation information coming from the Long Range scan.
 *
 * @param[in] angleToAvoid Specifies the angle about the center, within which detections should not be considered [deg]
 * @param[in] obj Specifies radar tracker handle.
 *
 * @return DW_INVALID_HANDLE if radar tracker handle is NULL.<br>
 *         DW_SUCCESS otherwise.<br>
 */
DW_API_PUBLIC
dwStatus dwRadarTracker_setAngleToAvoidTracking(float32_t angleToAvoid, dwRadarTrackerHandle_t obj);

/**
 * Add radar scan to be consumed by the radar tracker
 *
 * @note This needs to be followed by dwRadarTracker_process(), or behavior will be undefined
 *
 * @param[in] radarScan pointer to single radar scan consisting of detections captured at a unique timestamp
 * @param[in] radar sensor index for which the scan came from (indexed by radar sensors)
 * @param[in] sensor rig index for which the scan came from (indexed by all sensors)
 * @param[in] obj Specifies radar tracker handle.
 *
 * @return DW_INVALID_HANDLE if radar tracker handle is NULL.<br>
 *         DW_SUCCESS otherwise.<br>
 */
DW_API_PUBLIC
dwStatus dwRadarTracker_addRadarScan(const dwRadarScan* radarScan, uint32_t radarSensorIndex, uint32_t sensorRigIndex, dwRadarTrackerHandle_t obj);

/**
 * Run tracker with last fed radar scan
 *
 * @note This needs to be invoked after every dwRadarTracker_addRadarScan() or behavior will be undefined
 *
 * @param[in/out] latestRadarState pointer to the radar state in the current scan
 * @param[in] obj Specifies radar tracker handle.
 *
 * @return DW_INVALID_HANDLE if radar tracker handle is NULL.<br>
 *         DW_INVALID_ARGUMENT if egomotion is invalid.<br>
 *         DW_SUCCESS otherwise.<br>
 */
DW_API_PUBLIC
dwStatus dwRadarTracker_process(dwRadarTrackerRadarState* latestRadarState, dwRadarTrackerHandle_t obj);

/**
 * Estimates track states at estimateTime
 *
 * @param[in] estimateTime Specifies the time at which we want the track estimates.
 * @param[in] obj Specifies radar tracker handle.
 *
 * @return DW_INVALID_HANDLE if radar tracker handle is NULL, or if no prior observations have been fed<br>
 *         DW_SUCCESS otherwise.<br>
 */
DW_API_PUBLIC
dwStatus dwRadarTracker_estimate(dwTime_t estimateTime, dwRadarTrackerHandle_t obj);

/**
 * Bind the output to a dwRadarScan.
 *
 * @note The memory is owned by the calling application and is valid
 *        till dwRadarTracker_estimate() is called again.
 *
 * @param[in/out] trackList Pointer to the dwRadarScan/dwRadarTrackList that will get overwritten every time estimate() is called.
 * @param[in] obj Handle to the Radar Tracker module
 *
 * @return DW_INVALID_HANDLE if radar tracker handle is NULL.<br>
 *         DW_INVALID_ARGUMENT if trackList is NULL.<br>
 *         DW_SUCCESS otherwise.<br>
 */
DW_API_PUBLIC
dwStatus dwRadarTracker_bindOutput(dwRadarTrackList* trackList, dwRadarTrackerHandle_t obj);

/**
 * Binds the output to a dwObjectRadar. It needs to be called only one time.
 * Output velocity and acceleration is relative to ground
 *
 * @note This list is valid only till the next dwRadarTracker_process() is called.
 *
 * @param[in/out] pointer to objectArray Array to bind to.
 * @param[in] obj Handle to the Radar Tracker module
 * @return DW_INVALID_HANDLE if radar tracker handle is NULL.<br>
 *         DW_INVALID_ARGUMENT if objectArray is NULL.<br>
 *         DW_INVALID_ARGUMENT if objectArray object is NULL.<br>
 *         DW_SUCCESS otherwise.<br>
 */
DW_API_PUBLIC
dwStatus dwRadarTracker_bindOutputObjectArray(dwObjectArray* objectArray, dwRadarTrackerHandle_t obj);

/**
 * Resets tracker
 *
 * @param[in] obj Specifies radar tracker handle.
 *
 * @return DW_INVALID_HANDLE if radar tracker handle is NULL, or if no prior observations have been fed<br>
 *         DW_SUCCESS otherwise.<br>
 */
DW_API_PUBLIC
dwStatus dwRadarTracker_reset(dwRadarTrackerHandle_t obj);

/**
 * Inquire watchdog by specifying a radar sensor index and scan range
 * @param[out] watchdogInfo Specifies pointer to the watchdog information.
 * @param[in] radarSensorIdx Specifies radar sensor index.
 * @param[in] scanRange Specifies radar scan range type.
 * @param[in] obj Specifies radar tracker handle.
 *
 * @return DW_INVALID_HANDLE if radar tracker handle is NULL, ...<br>
 *         DW_SUCCESS otherwise.<br>
 */
DW_API_PUBLIC
dwStatus dwRadarTracker_getWatchdogInfo(dwRadarTrackerWatchdogInfo* watchdogInfo, const uint32_t radarSensorIdx, const dwRadarRange scanRange, dwRadarTrackerHandle_t obj);

#ifdef __cplusplus
}
#endif
/** @} */
#endif
