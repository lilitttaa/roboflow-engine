#include "MotionMatching.hpp"
#include "MotionMatchingUtils.hpp"
#include <iostream>
#include <algorithm>
#include <limits>
#include <cstdlib>
#include <ctime>

namespace mf {

// ============================================================================
// MotionDatabase 实现
// ============================================================================

size_t MotionDatabase::addClip(std::unique_ptr<MotionData> motion, const std::string& tag) {
    size_t index = m_clips.size();
    m_clips.push_back(std::move(motion));
    m_clipTags.push_back(tag.empty() ? "clip_" + std::to_string(index) : tag);
    m_preprocessed = false;
    return index;
}

bool MotionDatabase::loadClip(const std::string& path, const std::string& tag) {
    auto motion = std::make_unique<MotionData>();
    if (!motion->loadFromFile(path)) {
        std::cerr << "[MotionDatabase] Failed to load: " << path << std::endl;
        return false;
    }
    
    std::string clipTag = tag;
    if (clipTag.empty()) {
        // 从路径提取文件名作为标签
        size_t lastSlash = path.find_last_of("/\\");
        size_t lastDot = path.find_last_of(".");
        if (lastSlash != std::string::npos && lastDot != std::string::npos && lastDot > lastSlash) {
            clipTag = path.substr(lastSlash + 1, lastDot - lastSlash - 1);
        }
    }
    
    addClip(std::move(motion), clipTag);
    std::cout << "[MotionDatabase] Loaded clip: " << clipTag 
              << " (" << m_clips.back()->getNumFrames() << " frames)" << std::endl;
    return true;
}

void MotionDatabase::preprocess() {
    if (m_clips.empty()) {
        std::cerr << "[MotionDatabase] No clips to preprocess" << std::endl;
        return;
    }
    
    std::cout << "[MotionDatabase] Preprocessing " << m_clips.size() << " clips..." << std::endl;
    
    m_entries.clear();
    
    for (size_t clipIdx = 0; clipIdx < m_clips.size(); ++clipIdx) {
        const auto& clip = m_clips[clipIdx];
        size_t numFrames = clip->getNumFrames();
        
        // 跳过最后几帧（无法计算未来轨迹）
        size_t skipLast = static_cast<size_t>(
            MotionFeature::TRAJECTORY_TIMES[MotionFeature::TRAJECTORY_POINTS - 1] * clip->getFPS()
        ) + 1;
        
        for (size_t frameIdx = 0; frameIdx < numFrames - skipLast; ++frameIdx) {
            MotionDatabaseEntry entry;
            entry.clipIndex = clipIdx;
            entry.frameIndex = frameIdx;
            entry.feature = computeFeature(clipIdx, frameIdx);
            entry.frame = &clip->getFrame(frameIdx);
            m_entries.push_back(entry);
        }
    }
    
    m_preprocessed = true;
    std::cout << "[MotionDatabase] Preprocessed " << m_entries.size() << " entries" << std::endl;
}

MotionFeature MotionDatabase::computeFeature(size_t clipIndex, size_t frameIndex) const {
    const auto& clip = m_clips[clipIndex];
    const auto& currentFrame = clip->getFrame(frameIndex);
    float fps = clip->getFPS();
    
    MotionFeature feature;
    
    // 当前帧的根位置和朝向（作为参考原点）
    Vector3 rootPos = currentFrame.rootPos;
    Quaternion rootQuat = currentFrame.rootQuat;
    
    // 从四元数提取 yaw 角度（使用工具函数）
    float rootYaw = extractYawFromQuaternion(rootQuat);
    
    // 计算轨迹特征（未来帧的位置和朝向，相对于当前帧）
    for (int i = 0; i < MotionFeature::TRAJECTORY_POINTS; ++i) {
        float futureTime = MotionFeature::TRAJECTORY_TIMES[i];
        size_t futureFrame = frameIndex + static_cast<size_t>(futureTime * fps);
        
        if (futureFrame < clip->getNumFrames()) {
            const auto& future = clip->getFrame(futureFrame);
            
            // 相对位置（水平）- 世界坐标系
            Vector3 relPos = Vector3Subtract(future.rootPos, rootPos);
            
            // 转换到局部坐标系（使用工具函数）
            Vector3 localPos = worldToLocal(relPos, rootYaw);
            feature.trajectory[i].position.x = localPos.x;  // right
            feature.trajectory[i].position.y = localPos.z;  // forward
            
            // 相对朝向
            float futureYaw = extractYawFromQuaternion(future.rootQuat);
            feature.trajectory[i].facing = normalizeAngle(futureYaw - rootYaw);
        }
    }
    
    // 计算脚部特征（简化：使用关节角度推算）
    // 真实实现需要正向运动学
    // 这里暂时用占位值
    feature.leftFootPos = {0, 0, 0};
    feature.rightFootPos = {0, 0, 0};
    feature.leftFootVel = {0, 0, 0};
    feature.rightFootVel = {0, 0, 0};
    
    // 计算髋部速度
    if (frameIndex > 0) {
        const auto& prevFrame = clip->getFrame(frameIndex - 1);
        float dt = 1.0f / fps;
        Vector3 worldVel = Vector3Scale(
            Vector3Subtract(currentFrame.rootPos, prevFrame.rootPos),
            1.0f / dt
        );
        
        // 转换到局部坐标系（使用工具函数）
        feature.hipVelocity = worldToLocal(worldVel, rootYaw);
    } else {
        feature.hipVelocity = {0, 0, 0};
    }
    
    return feature;
}

size_t MotionDatabase::findBestMatch(
    const MotionFeature& query,
    const FeatureWeights& weights,
    int excludeClip,
    int excludeFrameStart,
    int excludeFrameEnd,
    float continuityBias
) const {
    if (!m_preprocessed || m_entries.empty()) {
        return 0;
    }
    
    // 收集多个候选匹配（增加多样性）
    constexpr size_t CANDIDATE_COUNT = 5;
    struct Candidate {
        size_t index;
        float distance;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(CANDIDATE_COUNT);
    
    float worstCandidateDist = std::numeric_limits<float>::max();
    
    for (size_t i = 0; i < m_entries.size(); ++i) {
        const auto& entry = m_entries[i];
        
        // 排除当前片段的指定帧范围（只在连续播放时排除）
        if (excludeClip >= 0 && 
            entry.clipIndex == static_cast<size_t>(excludeClip) &&
            excludeFrameStart >= 0 && excludeFrameEnd >= 0 &&
            entry.frameIndex >= static_cast<size_t>(excludeFrameStart) &&
            entry.frameIndex <= static_cast<size_t>(excludeFrameEnd)) {
            continue;
        }
        
        float dist = query.distance(entry.feature, weights);
        
        // 连续性偏置：给当前片段后续帧减分
        if (excludeClip >= 0 && entry.clipIndex == static_cast<size_t>(excludeClip)) {
            if (entry.frameIndex > static_cast<size_t>(excludeFrameEnd)) {
                // 后续帧：大幅减分（例如减 50%）
                dist *= continuityBias;
            } else if (entry.frameIndex >= static_cast<size_t>(excludeFrameStart - 10) &&
                       entry.frameIndex <= static_cast<size_t>(excludeFrameEnd)) {
                // 当前帧附近：小幅减分（例如减 20%）
                dist *= (continuityBias + 0.3f);  // 0.5 + 0.3 = 0.8
            }
        }
        
        // 维护候选列表
        if (candidates.size() < CANDIDATE_COUNT) {
            candidates.push_back({i, dist});
            // 更新最差距离
            if (candidates.size() == 1) {
                worstCandidateDist = dist;
            } else if (dist > worstCandidateDist) {
                worstCandidateDist = dist;
            }
        } else if (dist < worstCandidateDist) {
            // 替换最差的候选
            auto worstIt = std::max_element(candidates.begin(), candidates.end(),
                [](const Candidate& a, const Candidate& b) {
                    return a.distance < b.distance;
                });
            *worstIt = {i, dist};
            // 重新计算最差距离
            worstCandidateDist = std::max_element(candidates.begin(), candidates.end(),
                [](const Candidate& a, const Candidate& b) {
                    return a.distance < b.distance;
                })->distance;
        }
    }
    
    if (candidates.empty()) {
        return 0;
    }
    
    // 从候选中选择：优先选择最佳匹配，但有一定概率选择其他候选（增加多样性）
    // 这里简化：选择距离在最佳距离的120%范围内的随机候选
    std::sort(candidates.begin(), candidates.end(),
        [](const Candidate& a, const Candidate& b) {
            return a.distance < b.distance;
        });
    
    float bestDist = candidates[0].distance;
    float threshold = bestDist * 1.2f;  // 允许20%的误差
    
    // 过滤出在阈值内的候选
    std::vector<Candidate> validCandidates;
    for (const auto& cand : candidates) {
        if (cand.distance <= threshold) {
            validCandidates.push_back(cand);
        }
    }
    
    // 从有效候选中选择（优先选择最佳，但有一定随机性）
    if (validCandidates.size() == 1) {
        return validCandidates[0].index;
    }
    
    // 使用加权随机选择：距离越小，权重越大
    float totalWeight = 0.0f;
    for (const auto& cand : validCandidates) {
        totalWeight += 1.0f / (cand.distance + 0.001f);  // 避免除零
    }
    
    // 简化：选择前3个候选中的一个（增加多样性但保持质量）
    size_t selectCount = std::min(validCandidates.size(), size_t(3));
    size_t selected = (rand() % selectCount);
    return validCandidates[selected].index;
}

// ============================================================================
// MotionMatcher 实现
// ============================================================================

MotionFeature MotionMatcher::computeQueryFeature(Vector3 desiredVelocity, float desiredFacing, bool isFreeMode) const {
    MotionFeature query;
    
    // 使用平滑后的速度和朝向（来自二阶弹簧阻尼器）
    Vector3 smoothVel = m_smoothVelocity.value;
    float smoothFacing = m_smoothFacing.value;
    
    // 将平滑后的世界坐标速度转换到局部坐标系
    Vector3 localVel = worldToLocal(smoothVel, smoothFacing);
    
    if (isFreeMode) {
        // ========== FREE 模式 ==========
        // 在 FREE 模式下，角色始终朝着自己的朝向移动
        // 轨迹是直线，朝向变化为 0
        
        for (int i = 0; i < MotionFeature::TRAJECTORY_POINTS; ++i) {
            float t = MotionFeature::TRAJECTORY_TIMES[i];
            
            // 预测位置（在局部坐标系中）
            query.trajectory[i].position.x = localVel.x * t;   // 横向移动
            query.trajectory[i].position.y = localVel.z * t;  // 前进距离
            
            // 预测朝向变化（FREE 模式下为 0）
            query.trajectory[i].facing = 0.0f;
        }
    } else {
        // ========== LOCK 模式 ==========
        // 在 LOCK 模式下，位移线和朝向线分离
        // 需要匹配移动量与朝向之间的夹角 (Strafing Angle)
        
        // 计算期望朝向（来自平滑后的 facing）
        float targetFacing = smoothFacing;
        
        // 获取当前帧的朝向（用于计算相对朝向变化）
        float currentFacing = smoothFacing;
        if (m_database && m_database->isPreprocessed() && m_currentClip < m_database->getNumClips()) {
            const auto* clip = m_database->getClip(m_currentClip);
            if (clip && m_currentFrame < clip->getNumFrames()) {
                currentFacing = extractYawFromQuaternion(clip->getFrame(m_currentFrame).rootQuat);
            }
        }
        
        float facingDelta = angleDifference(currentFacing, targetFacing);
        
        for (int i = 0; i < MotionFeature::TRAJECTORY_POINTS; ++i) {
            float t = MotionFeature::TRAJECTORY_TIMES[i];
            
            // 预测位置（在局部坐标系中）
            query.trajectory[i].position.x = localVel.x * t;
            query.trajectory[i].position.y = localVel.z * t;
            
            // 预测朝向变化（考虑转向速度）
            float turnRate = facingDelta / 1.0f;  // 假设在1秒内完成转向
            query.trajectory[i].facing = turnRate * t;
            query.trajectory[i].facing = normalizeAngle(query.trajectory[i].facing);
        }
    }
    
    // 脚部特征（暂时不使用）
    query.leftFootPos = {0, 0, 0};
    query.rightFootPos = {0, 0, 0};
    query.leftFootVel = {0, 0, 0};
    query.rightFootVel = {0, 0, 0};
    
    // 髋部速度（局部坐标系）
    query.hipVelocity = localVel;
    
    return query;
}

void MotionMatcher::performSearch(const MotionFeature& query) {
    if (!m_database || !m_database->isPreprocessed()) return;
    
    // 搜索时排除当前帧附近的帧（减少排除范围，增加多样性）
    // 只在连续播放时排除，避免总是匹配到相同的帧
    int excludeStart = -1;
    int excludeEnd = -1;
    
    // 只在当前帧还在合理范围内时排除（避免总是跳回相同位置）
    const auto* currentClip = m_database->getClip(m_currentClip);
    if (currentClip && m_currentFrame < currentClip->getNumFrames()) {
        // 减少排除范围：只排除非常接近的帧（±2帧）
        excludeStart = static_cast<int>(m_currentFrame) - 2;
        excludeEnd = static_cast<int>(m_currentFrame) + 2;
    }
    
    size_t bestIndex = m_database->findBestMatch(
        query, 
        m_weights,
        static_cast<int>(m_currentClip),
        excludeStart,
        excludeEnd,
        continuityBias
    );
    
    const auto& bestEntry = m_database->getEntry(bestIndex);
    
    // 如果找到的是不同的帧，开始惯性化过渡
    // 降低跳转阈值，允许更频繁的匹配
    if (bestEntry.clipIndex != m_currentClip || 
        std::abs(static_cast<int>(bestEntry.frameIndex) - static_cast<int>(m_currentFrame)) > 2) {
        
        // 获取当前帧和目标帧
        const auto* targetClip = m_database->getClip(bestEntry.clipIndex);
        if (!targetClip || bestEntry.frameIndex >= targetClip->getNumFrames()) {
            return;
        }
        
        const auto& currentFrame = currentClip && m_currentFrame < currentClip->getNumFrames() 
            ? currentClip->getFrame(m_currentFrame) 
            : MotionFrame{};
        const auto& targetFrame = targetClip->getFrame(bestEntry.frameIndex);
        
        // 计算速度差异
        Vector3 currentVel = {0, 0, 0};
        Vector3 targetVel = {0, 0, 0};
        if (currentClip && m_currentFrame > 0 && m_currentFrame < currentClip->getNumFrames()) {
            const auto& prevFrame = currentClip->getFrame(m_currentFrame - 1);
            float dt = 1.0f / currentClip->getFPS();
            currentVel = Vector3Scale(
                Vector3Subtract(currentFrame.rootPos, prevFrame.rootPos),
                1.0f / dt
            );
        }
        if (bestEntry.frameIndex > 0) {
            const auto& prevFrame = targetClip->getFrame(bestEntry.frameIndex - 1);
            float dt = 1.0f / targetClip->getFPS();
            targetVel = Vector3Scale(
                Vector3Subtract(targetFrame.rootPos, prevFrame.rootPos),
                1.0f / dt
            );
        }
        
        // 启动惯性化
        m_inertializer.halflife = inertializationHalflife;
        m_inertializer.start(
            currentFrame.rootPos, currentVel, currentFrame.rootQuat,
            targetFrame.rootPos, targetVel, targetFrame.rootQuat
        );
        m_inertializing = true;
        
        // 立即跳转到新位置
        m_currentClip = bestEntry.clipIndex;
        m_currentFrame = bestEntry.frameIndex;
        m_currentRootPos = targetFrame.rootPos;
        m_currentRootRot = targetFrame.rootQuat;
        // 重置上一帧位置，避免位置跳跃
        m_lastFrameRootPos = targetFrame.rootPos;
        m_firstFrame = true;  // 标记为第一帧，下次更新时会正确初始化
        
        std::cout << "[MotionMatcher] Jump to clip " << m_currentClip 
                  << " frame " << m_currentFrame << std::endl;
    }
    
    m_framesSinceLastSearch = 0;
}

MotionFrame MotionMatcher::update(float deltaTime, Vector3 desiredVelocity, float desiredFacing, bool isFreeMode) {
    if (!m_database || !m_database->isPreprocessed()) {
        return MotionFrame{};
    }
    
    // ========== 期望轨迹平滑（二阶弹簧阻尼器）==========
    // 平滑输入速度和朝向
    m_smoothVelocity.update(desiredVelocity, deltaTime);
    m_smoothFacing.update(desiredFacing, deltaTime);
    
    // 检查输入是否有显著变化（包括速度和朝向）
    Vector3 velDiff = Vector3Subtract(desiredVelocity, m_lastDesiredVelocity);
    float velChange = Vector3Length(velDiff);
    
    float facingChange = std::abs(angleDifference(m_lastDesiredFacing, desiredFacing));
    
    // 降低阈值，使搜索更敏感
    bool inputChanged = velChange > (forceSearchThreshold * 0.5f) || 
                        facingChange > 10.0f;  // 朝向变化超过10度
    
    m_lastDesiredVelocity = desiredVelocity;
    m_lastDesiredFacing = desiredFacing;
    
    // 决定是否搜索（更频繁地搜索）
    m_framesSinceLastSearch++;
    // 减少搜索间隔，使搜索更频繁
    int effectiveSearchInterval = std::max(5, searchInterval / 2);
    if (inputChanged || m_framesSinceLastSearch >= effectiveSearchInterval) {
        MotionFeature query = computeQueryFeature(desiredVelocity, desiredFacing, isFreeMode);
        performSearch(query);
    }
    
    // 获取当前帧
    const auto* clip = m_database->getClip(m_currentClip);
    if (!clip) {
        return MotionFrame{};
    }
    
    // 更新时间和帧
    m_currentTime += deltaTime;
    float frameTime = 1.0f / clip->getFPS();
    
    while (m_currentTime >= frameTime) {
        m_currentTime -= frameTime;
        m_currentFrame++;
        
        // 检查是否到达片段末尾
        size_t skipLast = static_cast<size_t>(
            MotionFeature::TRAJECTORY_TIMES[MotionFeature::TRAJECTORY_POINTS - 1] * clip->getFPS()
        ) + 1;
        
        if (m_currentFrame >= clip->getNumFrames() - skipLast) {
            // 强制搜索新的匹配
            MotionFeature query = computeQueryFeature(desiredVelocity, desiredFacing, isFreeMode);
            performSearch(query);
            clip = m_database->getClip(m_currentClip);
            if (!clip) {
                return MotionFrame{};
            }
        }
    }
    
    // 获取当前帧
    MotionFrame result = clip->getFrame(m_currentFrame);
    
    // ========== 计算相对位移（用于 Root Motion）==========
    // 存储上一帧的绝对位置（用于计算相对位移）
    static Vector3 lastFrameRootPos = result.rootPos;
    
    // 保存原始绝对位置
    Vector3 absoluteRootPos = result.rootPos;
    
    // ========== 惯性化插值 ==========
    if (m_inertializing) {
        m_inertializer.update(deltaTime);
        
        // 应用惯性化偏移到绝对位置
        absoluteRootPos = Vector3Add(absoluteRootPos, m_inertializer.position);
        
        // 检查是否完成
        if (m_inertializer.isComplete()) {
            m_inertializing = false;
        }
    }
    
    // ========== Motion Warping ==========
    if (m_warpActive) {
        // 临时设置 rootPos 用于 Motion Warping 计算
        result.rootPos = absoluteRootPos;
        applyMotionWarping(result, deltaTime);
        absoluteRootPos = result.rootPos;
    }
    
    // 计算相对位移（当前帧相对于上一帧）
    Vector3 frameDelta = Vector3Subtract(absoluteRootPos, m_lastFrameRootPos);
    m_lastFrameRootPos = absoluteRootPos;
    
    // 更新 Root Motion 状态（存储绝对位置用于特征匹配）
    m_currentRootPos = absoluteRootPos;
    m_currentRootRot = result.rootQuat;
    
    // 将相对位移存储到 result.rootPos 中（供外部使用）
    // 这样外部可以直接使用 frame.rootPos 作为位移增量
    result.rootPos = frameDelta;
    
    return result;
}

void MotionMatcher::jumpTo(size_t clipIndex, size_t frameIndex) {
    if (!m_database) return;
    
    if (clipIndex < m_database->getNumClips()) {
        const auto* clip = m_database->getClip(clipIndex);
        if (clip && frameIndex < clip->getNumFrames()) {
            m_currentClip = clipIndex;
            m_currentFrame = frameIndex;
            m_currentTime = 0.0f;
            m_blending = false;
            
            // 重置上一帧位置
            const auto& frame = clip->getFrame(frameIndex);
            m_lastFrameRootPos = frame.rootPos;
            m_currentRootPos = frame.rootPos;
            m_currentRootRot = frame.rootQuat;
            m_firstFrame = true;  // 标记为第一帧
        }
    }
}

MotionFrame MotionMatcher::blendFrames(const MotionFrame& from, const MotionFrame& to, float t) const {
    MotionFrame result;
    
    // 插值根位置
    result.rootPos = Vector3Lerp(from.rootPos, to.rootPos, t);
    
    // 插值根旋转（球面线性插值）
    result.rootQuat = QuaternionSlerp(from.rootQuat, to.rootQuat, t);
    
    // 插值关节角度
    size_t numJoints = std::min(from.jointPos.size(), to.jointPos.size());
    result.jointPos.resize(numJoints);
    for (size_t i = 0; i < numJoints; ++i) {
        result.jointPos[i] = from.jointPos[i] + (to.jointPos[i] - from.jointPos[i]) * t;
    }
    
    return result;
}

void MotionMatcher::setWarpTarget(const Vector3& targetPos, float duration) {
    m_warpActive = true;
    m_warpTarget = targetPos;
    m_warpDuration = duration;
    m_warpTime = 0.0f;
    m_warpStartPos = m_currentRootPos;
    m_warpStartFrame = m_currentFrame;
}

void MotionMatcher::applyMotionWarping(MotionFrame& frame, float deltaTime) {
    if (!m_warpActive) return;
    
    m_warpTime += deltaTime;
    float t = std::min(m_warpTime / m_warpDuration, 1.0f);
    
    // 计算期望的位移
    Vector3 desiredDisplacement = Vector3Subtract(m_warpTarget, m_warpStartPos);
    
    // 计算当前帧的原始位移（相对于起始帧）
    Vector3 currentDisplacement = Vector3Subtract(frame.rootPos, m_warpStartPos);
    
    // 计算缩放因子
    float desiredLength = Vector3Length(desiredDisplacement);
    float currentLength = Vector3Length(currentDisplacement);
    
    if (currentLength > 0.001f && desiredLength > 0.001f) {
        float scale = desiredLength / currentLength;
        
        // 应用缩放（使用平滑插值）
        float smoothT = t * t * (3.0f - 2.0f * t);  // smoothstep
        float finalScale = 1.0f + (scale - 1.0f) * smoothT;
        
        // 缩放位移
        Vector3 scaledDisplacement = Vector3Scale(currentDisplacement, finalScale);
        frame.rootPos = Vector3Add(m_warpStartPos, scaledDisplacement);
    }
    
    // 检查是否完成
    if (t >= 1.0f) {
        m_warpActive = false;
    }
}

} // namespace mf
