/*
* \file process1_live.hpp
* \brief header file for Alg 1
* \author (1) Suman Ghosh
* \date 2024-09-29
* \author (2) Valentina Cavinato
* \date 2024-09-29
* \author (3) Guillermo Gallego
* \date 2024-09-29
* Copyright/Rights of Use:
* 2024, Technische Universität Berlin
* Prof. Guillermo Gallego
* Robotic Interactive Perception
* Marchstrasse 23, Sekr. MAR 5-5
* 10587 Berlin, Germany
*/

#pragma once

#include <mapper/mapper_emvs_stereo.hpp>
#include <vector>
// #include <tf/tf.h>

// Compute both DSIs, fuse by harmonic mean, then extract semi-dense depth map
void process_1(
    const std::string world_frame_id,
    const std::string left_cam_frame_id,
    const PinholeCameraModel& cam0,
    const PinholeCameraModel& cam1,
    const PinholeCameraModel& cam2,
    const std::shared_ptr<tf2::BufferCore> tf_,
    const std::vector<Event>& events0,
    const std::vector<Event>& events1,
    const std::vector<Event>& events2,
    const EMVS::OptionsDepthMap& opts_depth_map,
    const EMVS::ShapeDSI& dsi_shape,
    EMVS::MapperEMVS& mapper_fused,
    EMVS::MapperEMVS& mapper0,
    EMVS::MapperEMVS& mapper1,
    EMVS::MapperEMVS& mapper2,
    const std::string& out_path,
    tf2::TimePoint ts,
    int fusion_method,
    Transformation& T_rv_w
);
