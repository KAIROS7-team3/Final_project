import pytest

from plc_core.client import PLCClient
from plc_core.states import LEDColor, LEDMode, SystemState


@pytest.fixture
def plc():
    client = PLCClient(port="/dev/null")
    client.connect()
    return client


class TestSetState:
    def test_idle_sets_white_solid(self, plc):
        status = plc.set_state(SystemState.IDLE)
        assert status.led_color == LEDColor.WHITE
        assert status.led_mode == LEDMode.SOLID

    def test_error_sets_red_flash(self, plc):
        status = plc.set_error()
        assert status.led_color == LEDColor.RED
        assert status.led_mode == LEDMode.FLASH

    def test_estop_sets_red_solid(self, plc):
        status = plc.set_estop()
        assert status.led_color == LEDColor.RED
        assert status.led_mode == LEDMode.SOLID
        assert status.system_state == SystemState.E_STOP

    def test_moving_sets_green_solid(self, plc):
        plc.set_state(SystemState.MOVING)
        status = plc.get_status()
        assert status.led_color == LEDColor.GREEN
        assert status.led_mode == LEDMode.SOLID


class TestStateMap:
    def test_all_states_covered(self):
        from plc_core.states import STATE_LED_MAP
        for state in SystemState:
            assert state in STATE_LED_MAP, f"STATE_LED_MAP missing entry for {state}"
