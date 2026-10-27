import logging
from pathlib import Path

from pyniryo.nate import Nate
from pyniryo.nate.models.programs import ProgramType

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")


def main():
    with Nate() as n:
        project = n.projects.create("Palletizing", description="Boxes on the left pallet")

        program_path = Path(__file__).parent.joinpath("basic_move.py")
        with open(program_path, "rb") as program_file:
            program = n.projects.create_program(project.id, program_file, ProgramType.python(), name=program_path.stem)

        n.projects.activate(project.id)

        for p in n.projects.get_all():
            marker = "*" if p.active else " "
            print(f"{marker} {p.name}: {', '.join(prog.name for prog in p.programs) or 'no program'}")

        n.projects.remove_program(project.id, program.id)
        n.projects.delete(project.id)
        n.programs.delete(program.id)


if __name__ == "__main__":
    main()
