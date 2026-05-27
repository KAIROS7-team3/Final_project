"""Blackboard 스키마 및 키 상수 단위 테스트 (rclpy 불필요)."""
from orchestrator.blackboard import (
    KEY_ACTIVE_TOOL_ID,
    KEY_FEASIBILITY_REASON,
    KEY_INTENT,
    KEY_IS_MOVING,
    KEY_STAGING_STATE,
    KEY_TOOL_POSE,
    STAGING_EMPTY,
    STAGING_OCCUPIED,
    BlackboardSchema,
)


class TestKeyConstants:
    def test_all_keys_are_strings(self):
        keys = [
            KEY_ACTIVE_TOOL_ID,
            KEY_TOOL_POSE,
            KEY_STAGING_STATE,
            KEY_INTENT,
            KEY_IS_MOVING,
            KEY_FEASIBILITY_REASON,
        ]
        assert all(isinstance(k, str) for k in keys)

    def test_keys_are_unique(self):
        keys = [
            KEY_ACTIVE_TOOL_ID,
            KEY_TOOL_POSE,
            KEY_STAGING_STATE,
            KEY_INTENT,
            KEY_IS_MOVING,
            KEY_FEASIBILITY_REASON,
        ]
        assert len(keys) == len(set(keys))


class TestBlackboardSchema:
    def test_default_values(self):
        schema = BlackboardSchema()
        assert schema.active_tool_id is None
        assert schema.tool_pose is None
        assert schema.staging_state == STAGING_EMPTY
        assert schema.intent is None
        assert schema.is_moving is False
        assert schema.feasibility_reason == ""

    def test_staging_constants(self):
        assert STAGING_EMPTY != STAGING_OCCUPIED

    def test_intent_assignment(self):
        schema = BlackboardSchema(intent="fetch", active_tool_id="wrench_8mm")
        assert schema.intent == "fetch"
        assert schema.active_tool_id == "wrench_8mm"

    def test_is_moving_flag(self):
        schema = BlackboardSchema(is_moving=True)
        assert schema.is_moving is True
