from mecanum_wand_4ws8_control.operator_console import run_console
from .commissioning import runtime_from_environment


def main(): run_console(runtime_from_environment())
