local animationSequence = {
    "Zeynep Walk",
    "Zeynep Squat"
}

local switchIntervalSeconds = 4.0

local entityAnimationState = {}

local function setAnimationByIndex(entityId, index)
    if animation == nil or animation.setAnimation == nil then
        return false
    end

    local animationCode = animationSequence[index]
    if animationCode == nil then
        return false
    end

    return animation.setAnimation(entityId, animationCode)
end

function onInit(entityId)
    entityAnimationState[entityId] = {
        sequenceIndex = 1,
        nextSwitchTimeSeconds = switchIntervalSeconds
    }
    setAnimationByIndex(entityId, 1)
end

function onTick(entityId, simulationTimeSeconds, deltaTimeSeconds)
    local state = entityAnimationState[entityId]
    if state == nil then
        state = {
            sequenceIndex = 1,
            nextSwitchTimeSeconds = switchIntervalSeconds
        }
        entityAnimationState[entityId] = state
        setAnimationByIndex(entityId, 1)
    end

    if simulationTimeSeconds < state.nextSwitchTimeSeconds then
        return
    end

    state.sequenceIndex = state.sequenceIndex + 1
    if state.sequenceIndex > #animationSequence then
        state.sequenceIndex = 1
    end

    setAnimationByIndex(entityId, state.sequenceIndex)

    while simulationTimeSeconds >= state.nextSwitchTimeSeconds do
        state.nextSwitchTimeSeconds = state.nextSwitchTimeSeconds + switchIntervalSeconds
    end
end

function onShutdown(entityId)
    entityAnimationState[entityId] = nil
end
