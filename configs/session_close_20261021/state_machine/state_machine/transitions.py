from __future__ import annotations

from typing import TYPE_CHECKING

from state_machine.state_types import StateType

if TYPE_CHECKING:
    from state_machine.state_machine import StateMachine


def dummy_transition(state_machine: StateMachine)->str:
    match state_machine.state:
        case StateType.GB_TRACK:
            if state_machine._low_bat:
                return StateType.LOW_BAT
            else:
                return StateType.GB_TRACK
        case StateType.LOW_BAT:
            return StateType.LOW_BAT
        case default:
            return StateType.GB_TRACK
        
        
def timetrials_transition(state_machine: StateMachine)->str:
    return StateType.GB_TRACK

def head_to_head_transition(state_machine: StateMachine)->str:
    match state_machine.state:
        case StateType.GB_TRACK:
            return SpliniGlobalTrackingTransition(state_machine)
        case StateType.TRAILING:
            return SpliniTrailingTransition(state_machine)

        case StateType.TRAILING_TO_GBTRACK:
            return SpliniTrailingToGbtrackTransition(state_machine)

        case StateType.OVERTAKE:
            return SpliniOvertakingTransition(state_machine)
        case StateType.FTGONLY:
            return SpliniFTGOnlyTransition(state_machine)
        case default:
            raise ValueError(f"Invalid state {state_machine.state}")


def SpliniGlobalTrackingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.GB_TRACK`"""
    if state_machine._check_gbfree:
        return StateType.GB_TRACK
    return StateType.TRAILING

def SpliniTrailingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.TRAILING`"""
    gb_free = state_machine._check_gbfree

    # If we have been sitting around in TRAILING for a while then FTG
    if state_machine._check_ftg:
        return StateType.FTGONLY

    if gb_free and state_machine._check_close_to_raceline:
        return StateType.TRAILING_TO_GBTRACK
    elif (
        not gb_free
        and state_machine._check_availability_splini_wpts
        and state_machine._check_ofree
    ):
        return StateType.OVERTAKE
    else:
        return StateType.TRAILING

def SpliniTrailingToGbtrackTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.TRAILING_TO_GBTRACK`"""
    # GB_TRACK 이외의 다른 상태로 return 할 때에는 trailing_to_gbtrack_count를 0으로 리셋해주기
    gb_free = state_machine._check_gbfree

    if state_machine._check_ftg:
        state_machine.trailing_to_gbtrack_count = 0
        return StateType.FTGONLY

    if gb_free and state_machine._check_close_to_raceline:
        state_machine.trailing_to_gbtrack_count += 1

        # gb_free의 횟수가 threshold를 넘기면 그때는 진짜로 gbtrack으로 전환
        if state_machine.trailing_to_gbtrack_count >= state_machine.trailing_to_gbtrack_counting_threshold:
            state_machine.trailing_to_gbtrack_count = 0
            return StateType.GB_TRACK
        else:
            return StateType.TRAILING_TO_GBTRACK

    elif (
        not gb_free
        and state_machine._check_availability_splini_wpts
        and state_machine._check_ofree
    ):
        state_machine.trailing_to_gbtrack_count = 0
        return StateType.OVERTAKE
    else:
        state_machine.trailing_to_gbtrack_count = 0
        return StateType.TRAILING



def SpliniOvertakingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.OVERTAKE`"""
    spline_valid = state_machine._check_availability_splini_wpts
    o_free = state_machine._check_ofree

    # If spline is on an obstacle we trail
    if not o_free:
        return StateType.TRAILING
    # Keep overtaking while the evasion spline is still valid
    if spline_valid:
        return StateType.OVERTAKE
    # Overtake finished: rejoin the raceline if it is clear, otherwise trail
    if state_machine._check_gbfree:
        return StateType.GB_TRACK
    return StateType.TRAILING


def SpliniFTGOnlyTransition(state_machine: StateMachine) -> StateType:
    if state_machine._check_close_to_raceline and state_machine._check_gbfree:
        return StateType.GB_TRACK
    return StateType.FTGONLY
        