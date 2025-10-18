from dataclasses import dataclass
from typing import List


@dataclass
class Action:
    name: str
    args: List[str]


def parse_action(action):
    action = str(action)
    if '(' not in action or ')' not in action:
        return Action(action, [])
    action_name, args_str = action.split('(', 1)
    args_str = args_str.strip().rstrip(')')
    args = [arg.strip() for arg in args_str.split(',') if arg.strip()]
    action_name = action_name.strip()
    return Action(action_name, args)