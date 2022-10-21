from typing import Optional, Tuple, Union

from util import tools
from util.agent import VirxERLU
from util.routines import AerialShot, DoubleJumpShot, GroundShot, JumpShot
from util.utils import Vector

from cutil.utils import get_cap


def find_ground_shot(agent: VirxERLU, target: Tuple[Vector, Vector], cap_: int=6) -> Optional[GroundShot]:
    return find_shot(agent, target, cap_, can_aerial=False, can_double_jump=False, can_jump=False)


def find_any_ground_shot(agent: VirxERLU, cap_: int=6) -> Optional[GroundShot]:
    return find_any_shot(agent, cap_, can_aerial=False, can_double_jump=False, can_jump=False)


def find_jump_shot(agent: VirxERLU, target: Tuple[Vector, Vector], cap_: int=6) -> Optional[JumpShot]:
    return find_shot(agent, target, cap_, can_aerial=False, can_double_jump=False, can_ground=False)


def find_any_jump_shot(agent: VirxERLU, cap_: int=6) -> Optional[JumpShot]:
    return find_any_shot(agent, cap_, can_aerial=False, can_double_jump=False, can_ground=False)


def find_double_jump(agent: VirxERLU, target: Tuple[Vector, Vector], cap_: int=6) -> Optional[DoubleJumpShot]:
    return find_shot(agent, target, cap_, can_aerial=False, can_jump=False, can_ground=False)


def find_any_double_jump(agent: VirxERLU, cap_: int=6) -> Optional[DoubleJumpShot]:
    return find_any_shot(agent, cap_, can_aerial=False, can_jump=False, can_ground=False)


def find_aerial(agent: VirxERLU, target: Tuple[Vector, Vector], cap_: int=6) -> Optional[AerialShot]:
    return find_shot(agent, target, cap_, can_double_jump=False, can_jump=False, can_ground=False)


def find_any_aerial(agent: VirxERLU, cap_: int=6) -> Optional[AerialShot]:
    return find_any_shot(agent, cap_, can_double_jump=False, can_jump=False, can_ground=False)


def find_shot(agent: VirxERLU, target: Tuple[Vector, Vector], cap_: int=6, can_aerial: bool=True, can_double_jump: bool=True, can_jump: bool=True, can_ground: bool=True) -> Optional[Union[GroundShot, JumpShot, DoubleJumpShot, AerialShot]]:
    return tools.find_shot(agent, target, get_cap(agent, cap_), can_aerial, can_double_jump, can_jump, can_ground)


def find_any_shot(agent: VirxERLU, cap_: int=6, can_aerial: bool=True, can_double_jump: bool=True, can_jump: bool=True, can_ground: bool=True) -> Optional[Union[GroundShot, JumpShot, DoubleJumpShot, AerialShot]]:
    return tools.find_any_shot(agent, get_cap(agent, cap_), can_aerial, can_double_jump, can_jump, can_ground)
