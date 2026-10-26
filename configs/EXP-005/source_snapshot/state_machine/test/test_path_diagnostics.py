"""Check diagnostic additions preserve path holding and transition decisions."""
from types import SimpleNamespace
from builtin_interfaces.msg import Time
from f110_msgs.msg import OTWpntArray, Wpnt
from state_machine.state_machine import StateMachine
from state_machine.state_types import StateType
from state_machine.transitions import SpliniOvertakingTransition


def test_empty_planner_message_keeps_existing_path_and_ttl():
    prior = OTWpntArray(wpnts=[Wpnt(s_m=1.0)])
    fake = SimpleNamespace(avoidance_wpnts=prior, splini_ttl_counter=42,
                           get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
                               to_msg=lambda: Time(sec=10))))
    StateMachine.avoidance_cb(fake, OTWpntArray())
    assert fake.avoidance_wpnts is prior
    assert fake.splini_ttl_counter == 42
    assert fake._diag_input['points'] == 0


def test_blocked_path_forces_trailing_even_with_valid_spline():
    fake = SimpleNamespace(_check_availability_splini_wpts=True,
                           _check_ofree=False, _check_gbfree=True)
    assert SpliniOvertakingTransition(fake) == StateType.TRAILING
    assert fake._diag_reason == 'avoidance_path_blocked'


def test_valid_free_path_keeps_overtaking():
    fake = SimpleNamespace(_check_availability_splini_wpts=True,
                           _check_ofree=True, _check_gbfree=False)
    assert SpliniOvertakingTransition(fake) == StateType.OVERTAKE
    assert fake._diag_reason == 'valid_avoidance_path'


def test_expired_path_rejoins_only_if_global_path_free():
    fake = SimpleNamespace(_check_availability_splini_wpts=False,
                           _check_ofree=True, _check_gbfree=False)
    assert SpliniOvertakingTransition(fake) == StateType.TRAILING
    fake._check_gbfree = True
    assert SpliniOvertakingTransition(fake) == StateType.GB_TRACK
